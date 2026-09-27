# #179 — README §2/§3: `source install/setup.bash` в каждом терминале

Проверено 2026-09-27, worktree `worktree-179-readme-source`. Чистый `git archive HEAD src`
в `/tmp` (не рабочее дерево), контейнер `ros:humble-ros-base`, `--network=none`, bag
`30618_01f73500` из `dataset/data`. Каждый «терминал» — отдельная сессия `docker exec`
(своя шелл-сессия, без общих переменных с другими).

## Сборка (шаг 1)

```
$ docker exec <ctr> bash step1_build.sh
Starting >>> tram_odometry_core
Starting >>> tram_vehicle_msgs_vendor
Finished <<< tram_odometry_core [1.13s]
Finished <<< tram_vehicle_msgs_vendor [7.69s]
Starting >>> tram_odometry
Finished <<< tram_odometry [1.07s]

Summary: 3 packages finished [9.02s]
```

## Терминал 1 — нода (README §2, `cd <ws> && source .../setup.bash && source install/setup.bash`)

```
$ docker exec -d <ctr> bash term1_launch.sh
$ docker exec <ctr> ps aux | grep odometry_node
root  703  ... /ws/install/tram_odometry/lib/tram_odometry/odometry_node ...
```

Нода жива.

## Терминал 2 ДО фикса (старый текст README: только `source /opt/ros/humble/setup.bash`, без `cd` и без `install/setup.bash`)

```
$ docker exec <ctr> bash term2_bagplay_before_fix.sh
[INFO] ... rosbag2_storage: Opened database ... READ_ONLY.
[INFO] ... rosbag2_player: Set rate to 1
[WARN] ... rosbag2_player: Ignoring a topic '/vehicle/rear_bogie_velocity', reason: package 'tram_vehicle_msgs' not found, searching: [/opt/ros/humble].
[WARN] ... rosbag2_player: Ignoring a topic '/vehicle/driver_position_cmd', reason: package 'tram_vehicle_msgs' not found, searching: [/opt/ros/humble].
[WARN] ... rosbag2_player: Ignoring a topic '/vehicle/front_bogie_velocity', reason: package 'tram_vehicle_msgs' not found, searching: [/opt/ros/humble].
[INFO] ... rosbag2_player: Adding keyboard callbacks.
```

Все три входных топика молча отброшены плеером. Нода остаётся жива, но без данных.

## Терминал 2 ПОСЛЕ фикса (новый текст README: `cd <ws> && source /opt/ros/humble/setup.bash && source install/setup.bash`, отдельная сессия)

```
$ docker exec <ctr> bash term2_bagplay_after_fix.sh
[INFO] ... rosbag2_storage: Opened database ... READ_ONLY.
[INFO] ... rosbag2_player: Set rate to 1
[INFO] ... rosbag2_player: Adding keyboard callbacks.
```

Без WARN.

## Терминал 3 — `ros2 topic hz` (README §3 после фикса)

```
$ docker exec <ctr> bash term3_hz.sh
WARNING: topic [/result/velocity] does not appear to be published yet
average rate: 41.832
	min: 0.001s max: 0.051s std dev: 0.02319s window: 42
average rate: 40.877
	...
average rate: 40.329
	...
```

`/result/velocity` публикуется ~40–42 Гц (README §3 ожидает ≈ 40 Гц: 2 × 10 Гц тележки + 20 Гц
контроллер). Первое `WARNING: does not appear to be published yet` — обычный старт `topic hz`
до первого сообщения, не имеет отношения к #179.

## Вывод

Без повторного перехода в `<ws>` и `source install/setup.bash` в каждом новом терминале —
тихая потеря всех трёх входных топиков (WARN, без падения ноды). С фиксом README §2/§3
(`cd <ws> && source /opt/ros/humble/setup.bash && source install/setup.bash` в каждом
терминале) — воспроизведено с нуля, из отдельной шелл-сессии: без WARN, `/result/velocity`
на ожидаемой частоте.

Скрипты (`step1_build.sh`, `term1_launch.sh`, `term2_bagplay_before_fix.sh`,
`term2_bagplay_after_fix.sh`, `term3_hz.sh`) не сохранены в репозитории — временные,
только команды внутри них совпадают дословно с блоками README §2/§3 на момент проверки.
