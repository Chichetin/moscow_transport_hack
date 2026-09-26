# Доказательства T1, T3, T7, A1 матрицы соответствия (#102)

Коммит кода `0ddcee544d11e6a9094e88761b19520d32e075e3` (`origin/main`, 26.09), чистый
worktree. Все команды ниже завершились с кодом 0.

## T1 — ROS 2 Humble, `colcon build` без интернета

```bash
bash tools/submission/jury_layouts.sh --run
```

Отчёт `out/submission/layouts-0ddcee544d11e6a9094e88761b19520d32e075e3.json`: `dirty=0`,
10/10 раскладок `build=ok` (`readme`, `clone_in_src`, `repo_root`, `jury_msgs_in_src`,
`over_orig_msgs`, `jury_msgs_over`, `up_to`, `merge_install`, `symlink_install`,
`underlay`). Каждая раскладка — `git archive HEAD` в чистом `ros:humble-ros-base`,
`--network=none`, 2 CPU, 512 МБ.

| Исходный bag | Статус | `/result/velocity` | `/result/position` |
|---|---|---:|---:|
| `30618_082f1d65` (с GNSS) | ok, нода жива, типы и stamp из входов | 891 | 891 |
| `30618_5036aa78` (без GNSS) | ok, нода жива, типы и stamp из входов | 1124 | 1124 |

## T3 — GNSS только в окне начальной выставки; T7 — только онлайн

CI-эквивалент в dev-образе (numpy 1.21.5, как у жюри):

```bash
bash docker/dev.sh bash -c 'source /opt/ros/humble/setup.bash; python3 -m pytest -q;
  colcon build; colcon test --return-code-on-test-failure; colcon test-result --verbose'
```

- `pytest`: 418 passed, 8 skipped. Пропуски — тесты на bag (данных в контейнере нет) и
  `matplotlib`.
- `colcon build`: 3 packages finished. `colcon test-result`: 275 tests, 0 errors,
  0 failures, 0 skipped; среди них тесты ноды `test_real_core_ignores_gnss_after_window_through_the_node`
  и `test_gnss_after_window_changes_no_published_bit`.

Вариант на bag, пропущенный в контейнере, прогнан на хосте с данными:

```bash
.venv/bin/python -m pytest tools/eval/tests/test_gnss_after_window.py   # 5 passed, 0 skipped
.venv/bin/python -m pytest src/tram_odometry_core/test/test_pipeline_baseline.py \
  src/tram_odometry_core/test/test_preprocess.py tools/eval/tests/test_bag.py \
  tools/eval/tests/test_gnss_after_window.py -k "gnss or future"      # все прошли
```

Что эти тесты доказывают:

- T3: GNSS после `gnss.init_window_s` не меняет ни одного поля `Estimate` ядра и ни одного
  бита `/result/*` ноды; первый вход со stamp из будущего не держит окно открытым (D-021,
  D-052, D-055).
- T7: выход со всем GNSS равен выходу с GNSS, обрезанным как в `tools/eval`, бит в бит.
  Сверка кода на `0ddcee5`: `OdometryNode.on_input` вызывает `Odometry.step` один раз на
  входное сообщение и публикует сразу, без таймеров и буферов
  (`src/tram_odometry/tram_odometry/odometry_node.py`); ядро держит только прошлое —
  история команд `deque(maxlen=CMD_HISTORY)` читается по `stamp ≤ t − drive.response_delay_s`
  (`pipeline._notch_at`).

## A1 — пакеты ROS 2 Humble с подпиской и `/result/*`

Проверка A1 в матрице — T1–T8. После этого прогона T1–T8 все ✅: T1, T3, T7 — здесь;
T2, T4, T5, T6, T8 — в своих строках (`docs/verification/2026-09-26-issue97-recording-gate.md`).
