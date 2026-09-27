# #194 — README §«Для жюри»: реальный прогон человеком, находка по шагу 3

Проверено 2026-09-27. Исполнитель — участник команды (git-автор коммитов `core-position`,
`core-preprocess`, `core-pipeline`, `core-slip`), то есть **писавший ядро** — это НЕ независимая
репетиция «человеком, не писавшим ноду» из `submission-checklist`; тот пункт остаётся открытым
отдельно. Ценность этого прогона — README §«Для жюри» пройден дословно вручную (не агентом) на
хосте без ROS 2 Humble (Arch Linux, не Ubuntu), через Docker, что и вскрыло находку ниже.

Хост без Remote Control, ассистент (эта сессия) помогал диагностикой в реальном времени через
`docker exec` в тот же контейнер — то есть прогон был не «без подсказок» в смысле
`submission-checklist`. Ниже — только то, что подтверждено командой и выводом.

## Окружение

```bash
git clone https://github.com/Chichetin/moscow_transport_hack tram   # в ~/tram
docker pull ros:humble-ros-base
docker run -it --name tram-rehearsal -v ~/tram:/tram:ro \
  -v <dataset>/data:/data:ro --network=none ros:humble-ros-base bash
```
bag `30618_e9a34502` (длительность 1201,44 с ≈ 20 мин — подтверждено `sqlite3` по
`MIN/MAX(timestamp)` в `.db3`).

## Находка (issue #194) — шаг 3 не говорит, что делать нужно, пока bag играет

Живое воспроизведение: после перезапуска терминала 2 (`ros2 bag play`) участник ушёл проверять
топики позже — bag к этому моменту уже доиграл до конца (процесс `ros2 bag play` завершился
сам, штатно). `ros2 topic hz /result/velocity` и `ros2 topic echo --once /result/position` в
этот момент не печатали вообще ничего и не завершались — участник интерпретировал это как
зависшую ноду.

Диагностика ассистента через `docker exec` в тот же контейнер подтвердила причину командой и
выводом:

```
$ docker exec tram-rehearsal bash -c "ps aux"
root       684  ...  odometry_node --ros-args -r __node:=tram_odometry ...     # нода жива
# --- ros2 bag play в списке процессов ОТСУТСТВУЕТ — не играет ---
```

```
$ docker exec tram-rehearsal bash -c "source ...; timeout 6 ros2 topic hz /result/position"
(пусто, таймаут)
$ docker exec tram-rehearsal bash -c "source ...; timeout 6 ros2 topic hz /result/velocity"
(пусто, таймаут)
$ docker exec tram-rehearsal bash -c "source ...; timeout 6 ros2 topic hz /sensing/gnss/master/fix"
(пусто, таймаут)
```

Все топики без данных одновременно — подтверждает, что причина не в ноде (она жива, см. `ps
aux` выше) и не в конкретном топике, а в отсутствии проигрывания bag. После перезапуска
`ros2 bag play` в терминале 2 и повторной проверки **пока он играет** — `hz`/`echo` вернули
ожидаемый вывод (см. ниже).

Вторая деталь того же эпизода: участник пробовал прервать зависший `ros2 topic echo` через
`Ctrl-Z` — это не завершает процесс (`SIGTSTP`, только приостанавливает в фоне), а не `Ctrl-C`
(`SIGINT`). Подтверждено командой:
```
$ docker exec tram-rehearsal bash -c "ps aux" | grep "topic echo"
root  910  ...  Sl+  ...  ros2 topic echo --once /result/position    # жив после предполагаемого Ctrl-Z
```

## Шаг 4 — некорректный вход

Подтверждено участником вручную («на шаге 4 тоже всё было ок») и отдельно точной командой
ассистентом в том же контейнере — синтаксис `ros2 topic pub` с `NaN` реально работает:
```
$ docker exec tram-rehearsal bash -c "source ...; timeout 3 ros2 topic pub --once \
    /vehicle/front_bogie_velocity tram_vehicle_msgs/msg/VelocitySensor \
    '{header: {stamp: {sec: 0, nanosec: 0}}, velocity: .nan}'"
publisher: beginning loop
publishing #1: tram_vehicle_msgs.msg.VelocitySensor(header=...(sec=0, nanosec=0), ...), velocity=nan)
```
Нода не упала, `/result/*` продолжила публикацию (независимо подтверждено следующими шагами).

## Шаг 5 — метрики без ROS (вывод участника)

```
$ .venv/bin/python tools/eval/run_eval.py --bag 30618_e9a34502
| speed_rmse | 0.028 | ... |
| drift_pct  | 0.008 | ... |
| along_rmse | 0.792 | ... |
1 bag, окно GNSS 5.0 с, 6 с; упали: нет; оценок NaN/inf (вне метрик): 0
-> /home/chichetin/tram/out/eval/255b74b-30618_e9a34502/metrics.json
```

## Шаг 6 — стенд жюри (вывод участника + подтверждение ассистента)

```
$ bash docker/jury-stand.sh 30618_e9a34502
== colcon build (сеть отключена)
Finished <<< tram_odometry [0.60s]
Summary: 3 packages finished [9.06s]
== запуск odometry.launch.py, bag play x1.0
```
На этом месте участник снова спросил, не зависло ли — ассистент проверил напрямую:
```
$ docker inspect dreamy_khorana --format '{{.State.StartedAt}}'
2026-09-27T15:32:06.36078895Z
$ date -u
Вс 27 сен 2026 15:39:30 UTC     # прошло ~7,5 мин из ~20 мин bag
$ tail out/stand/30618_e9a34502/record.log
...Recording... Subscribed to topic '/result/velocity' ...
$ ls -la out/stand/30618_e9a34502/resources.csv
... 12723 байт, обновляется   # снимается раз в секунду — растёт, значит не завис
```
Финальная таблица (доигралось до конца):
```
| задержка p99, мс | 4.4 | ≤ 100 | ✅ |
| задержка max, мс | 19.8 | ≤ 250 | ✅ |
| частота /result/*, Гц | 38.4 | ≥ 10 | ✅ |
| CPU max, ядер | 0.14 | ≤ 2 | ✅ |
| RSS пик, МБ | 63.0 | ≤ 512 | ✅ |
| рост RSS, МБ/мин | -2.33 | ≤ 1 | ✅ |
```

## Фикс (этот PR)

`README.md` §«Для жюри» дополнен: явный переход на Docker, если ROS 2 Humble недоступен
bare-metal (шаг 1); явное предупреждение, что шаги 3–4 нужно успевать делать, пока bag играет,
и что зависший `hz`/`echo` без bag — не поломка ноды (шаг 2); блоки «Ожидается» с конкретными
образцами вывода после каждой команды (шаги 1, 2, 3, 5, 6); готовая команда `ros2 topic pub`
для шага 4 вместо словесного описания; предупреждение про `Ctrl-C` vs `Ctrl-Z`; предупреждение,
что стенд (шаг 6) может идти до нескольких десятков минут в реальном темпе.

## Вывод

Все 6 шагов README пройдены вручную человеком до конца успешно, включая некорректный вход и
стенд жюри (6/6 порогов). Одна находка (эпизод «зависшего» шага 3) — из-за неполноты
инструкции, не из-за кода; исправлена в этом же PR. **Это не закрывает пункт A2
(`submission-checklist`) полностью** — исполнитель писал ядро, прогон был не «без подсказок»
(диагностика — совместно с ассистентом). Независимая репетиция человеком, не писавшим ноду, без
участия ассистента, по исправленному README — по-прежнему нужна отдельно.
