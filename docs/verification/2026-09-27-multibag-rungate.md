# #200 — живая проверка RunGate на нескольких bag в одной ноде

Базовый фикс #200 — `8215fe8`; при работе в ветку также вошёл merge из `main`.
Первый полный прогон собран из `git archive 67d8f65de9d66ac2fc4fedb4058b65a589c093fa`, второй
точечный прогон `--topics` — из `git archive 6cd652860654491508a2ed1f61d570b32546b17a`.
Контейнер: `tram-odom:jury`, сеть отключена, 2 CPU, 512 MiB. Внутри выполнен чистый
`colcon build` (3 пакета). Одна нода живёт на протяжении всех bag внутри каждой группы;
между `bag play` пауза 2 с. Recorder запускается отдельно на каждый bag, чтобы измерения
и `header.stamp` не смешивались.

## Команды и исходный вывод

```bash
git fetch origin && git merge origin/worktree-200-rungate
bash -n tools/submission/multibag_live.sh tools/submission/multibag_live_inside.sh
/home/ir6/my/moscow_transport_hack/.venv/bin/python -m py_compile \
  tools/submission/{make_scenario_bags,check_recording,wait_ready,multibag_report}.py
bash tools/submission/multibag_live.sh
# exit 0, запись: out/submission/multibag-67d8f65de9d66ac2fc4fedb4058b65a589c093fa-20260927T230446/
MULTIBAG_ONLY=no_gnss bash tools/submission/multibag_live.sh
# exit 0, запись: out/submission/multibag-6cd652860654491508a2ed1f61d570b32546b17a-20260927T231043/
```

`a = 30618_bc5e53c2` (26,3 с), `b = 30618_082f1d65` (20,5 с),
`c = 30618_af7496f0` (первые 30 с из 262,9 с). Сценарий `gnss_first`
удаляет входы `/vehicle/*` с `header.stamp` раньше первого master GNSS fix
во втором bag; в ноде GNSS приходит раньше тележек. `b_no_gnss` содержит те же
входы машины и ноль сообщений GNSS. В первом прогоне воспроизводился подготовленный
`b_no_gnss`; в точечном повторе на втором bag использована ровно команда:

```bash
ros2 bag play /bags/b --topics /vehicle/front_bogie_velocity \
  /vehicle/rear_bogie_velocity /vehicle/driver_position_cmd
```

Результат первого прогона (`report.md`, exit 0; ошибки — 2D к master GNSS
исходного bag, ближайший timestamp):

| Группа / bag | Frame map/odom, шт. | 2D median / p95, м | Hz position / velocity | Reset на группу | `input skipped` | Статус |
|---|---:|---:|---:|---:|---:|---|
| одна нода `a` | 1102/0 | 9,87 / 9,87 | 42,05 / 42,16 | 2 | 0 | ok |
| одна нода `b` | 890/0 | 0,86 / 0,87 | 43,65 / 43,69 | 2 | 0 | ok |
| одна нода `c` (30 с) | 1279/0 | 1,16 / 1,82 | 42,82 / 42,92 | 2 | 0 | ok |
| GNSS → без GNSS, `a` | 1102/0 | 9,87 / 9,87 | 42,05 / 42,16 | 1 | 0 | ok |
| GNSS → без GNSS, `b_no_gnss` | 0/718 | — | 39,94 / 43,70 | 1 | 0 | ok |
| ранний GNSS, `a` | 1102/0 | 9,87 / 9,87 | 42,05 / 42,16 | 1 | 0 | ok |
| ранний GNSS, `b_gnss_first` | 886/0 | 0,86 / 0,87 | 43,47 / 43,56 | 1 | 0 | ok |
| свежая нода `a` | 1102/0 | 9,87 / 9,87 | 42,05 / 42,16 | 0 | 0 | ok |
| свежая нода `b` | 889/0 | 0,86 / 0,87 | 43,56 / 43,66 | 0 | 0 | ok |
| свежая нода `c` (30 с) | 1279/0 | 1,16 / 1,82 | 42,82 / 42,91 | 0 | 0 | ok |

`check_recording.py` вернул 0 на каждом bag: типы выходов корректны и каждый
выходной `header.stamp` присутствует во входном bag. `play_exit`,
`record_ready_exit`, `check_exit` — 0 в каждом `exit.tsv`. В логах нод
`new bag: a fresh run` встречается ровно 2/1/1 раза в группах с 3/2/2 bag;
`input skipped` — 0. На свежих нодах reset — 0.

Повтор с настоящим `ros2 bag play --topics` (`6cd6528`, exit 0): `a` —
1102 `map`, 9,87/9,87 м median/p95, 42,05/42,16 Гц; `b` — 718 `odom`, 0 `map`,
39,94/43,70 Гц. В одной ноде один reset, ноль `input skipped`; проверки записи
обоих bag вернули 0.

## Ограничение

Третий bag проверен на первых 30 с, а не на всех 262,9 с. Числа для него относятся
только к этому окну; свежая нода использовала то же окно. Ошибки считают по GNSS
исходного bag, который не передаётся ноде после начального окна и служит только
внешней оценкой.

После прогонов в ветку вошёл merge из `main`: он изменил обработку завершения
`odometry_node.py` (`ExternalShutdownException`) и добавил тест. Эту итоговую
версию `src/` в контейнере отдельно не прогоняли; приведённые результаты
относятся строго к двум SHA из раздела выше.
