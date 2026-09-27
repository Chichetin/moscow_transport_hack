# #179 — README §2/§3: `cd <ws> && source install/setup.bash` в каждом терминале

Проверено 2026-09-27, worktree `worktree-179-readme-source`, коммит `a2181fb` + доработка
по ревью. Чистый `git archive HEAD src` в `/tmp` (не рабочее дерево), контейнер
`ros:humble-ros-base`, `--network=none`, bag `30618_01f73500` из `dataset/data`. Каждый
«терминал» — **отдельный** вызов `docker exec` (своя шелл-сессия, не подшелл — подшелл
внутри одного `docker exec` наследует уже выставленное окружение родителя и даёт ложный
негативный результат; первая попытка так и ошиблась, ниже — только изолированные прогоны).

## Сборка (шаг 1)

`step1_build.sh`:
```bash
#!/bin/bash
set -e
cd /ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

```
$ docker exec <ctr> bash /ws/step1_build.sh
Starting >>> tram_odometry_core
Starting >>> tram_vehicle_msgs_vendor
Finished <<< tram_odometry_core [0.60s]
Finished <<< tram_vehicle_msgs_vendor [4.12s]
Starting >>> tram_odometry
Finished <<< tram_odometry [0.56s]

Summary: 3 packages finished [4.82s]
```

## Терминал 1 — нода (README §2)

`term1_launch.sh`:
```bash
#!/bin/bash
cd /ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch tram_odometry odometry.launch.py
```

```
$ docker exec -d <ctr> bash /ws/term1_launch.sh
$ docker exec <ctr> ps aux | grep odometry_node
root  703  ... /ws/install/tram_odometry/lib/tram_odometry/odometry_node ...
```

Нода жива.

## Терминал 2 БЕЗ `cd <ws>` — воспроизводит #179, сессия начинается в `$HOME`

`term2_no_cd.sh`:
```bash
#!/bin/bash
cd
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "EXIT of second source: $?"
ros2 bag play /data/30618_01f73500
```

```
$ docker exec <ctr> bash /ws/term2_no_cd.sh
EXIT of second source: 1
/ws/term2_no_cd.sh: line 6: install/setup.bash: No such file or directory
stdin is not a terminal device. Keyboard handling disabled.
[INFO] ... rosbag2_storage: Opened database ... READ_ONLY.
[INFO] ... rosbag2_player: Set rate to 1
[WARN] ... rosbag2_player: Ignoring a topic '/vehicle/rear_bogie_velocity', reason: package 'tram_vehicle_msgs' not found, searching: [/opt/ros/humble].
[WARN] ... rosbag2_player: Ignoring a topic '/vehicle/driver_position_cmd', reason: package 'tram_vehicle_msgs' not found, searching: [/opt/ros/humble].
[WARN] ... rosbag2_player: Ignoring a topic '/vehicle/front_bogie_velocity', reason: package 'tram_vehicle_msgs' not found, searching: [/opt/ros/humble].
[INFO] ... rosbag2_player: Adding keyboard callbacks.
```

Это подтверждает механизм из D-090 буквально: второй `source` падает («No such file or
directory», код 1, относительный путь `install/setup.bash` без `cd`), но это отдельная
команда, а не часть `&&`-цепочки со следующей строкой — `ros2 bag play` всё равно
выполняется и молча отбрасывает все три входных топика (WARN, без падения плеера).

С отдельным терминалом с уже сорсированным `<ws>` (`term_hz_debug.sh`, ниже) в этом
состоянии `/result/velocity` не публикуется вообще — `ros2 topic hz` висит до таймаута без
единой строки вывода (иначе поймали бы контаминацию окружения, см. ниже):

```
$ docker exec <ctr> bash /ws/term_hz_debug.sh   # bag play из term2_no_cd.sh ещё играет
$ cat hz_out.log
hz exit: 124
```

## Терминал 2 С `cd <ws>` (README §2 после фикса), отдельная сессия

`term2_with_cd.sh`:
```bash
#!/bin/bash
cd /ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 bag play /data/30618_01f73500
```

```
$ docker exec <ctr> bash /ws/term2_with_cd.sh
stdin is not a terminal device. Keyboard handling disabled.
[INFO] ... rosbag2_storage: Opened database ... READ_ONLY.
[INFO] ... rosbag2_player: Set rate to 1
[INFO] ... rosbag2_player: Adding keyboard callbacks.
```

Без WARN.

## Терминал 3 — `ros2 topic hz` (README §3 после фикса), отдельная сессия

`term_hz_debug.sh`:
```bash
#!/bin/bash
cd /ws && source /opt/ros/humble/setup.bash && source install/setup.bash
stdbuf -oL -eL timeout 8 ros2 topic hz /result/velocity > /ws/hz_out.log 2>&1
echo "hz exit: $?" >> /ws/hz_out.log
```

```
$ docker exec -d <ctr> bash /ws/term2_with_cd.sh   # терминал 2, свежий bag play
$ docker exec <ctr> bash /ws/term_hz_debug.sh       # терминал 3, отдельная сессия
$ cat hz_out.log
average rate: 39.952
	min: 0.000s max: 0.051s std dev: 0.02430s window: 40
average rate: 38.958
	...
average rate: 39.800
	...
hz exit: 124
```

`/result/velocity` публикуется ~39–40 Гц (README §3 ожидает ≈ 40 Гц: 2 × 10 Гц тележки +
20 Гц контроллер); `hz exit: 124` — это `timeout 8`, ожидаемое завершение измерения, не
ошибка.

## Изоляция окружения между «терминалами» — важное предостережение

Первая попытка проверить сценарий «до фикса» через `topic hz` использовала подшелл
`(cd; source ...; ros2 bag play ...) &` **внутри одного** `docker exec`, началом которого
была строка `cd /ws && source ... && source install/setup.bash` для самого `topic hz`.
Подшелл унаследовал уже экспортированные переменные окружения (`AMENT_PREFIX_PATH` и
другие, которые выставляет `source install/setup.bash`) от родительской сессии — то есть
«второй терминал» в этом случае не был независимым, и `ros2 bag play` внутри него на самом
деле видел `tram_vehicle_msgs`. Результат — ложные ~40 Гц без единого WARN, хотя по
условиям сценария их следовало ожидать. Обнаружено сравнением с параллельно оставшимися
фоновыми процессами `ros2 bag play` (`ps aux` в контейнере показал три висящих процесса от
предыдущих попыток — они и давали данные). Все прогоны выше — только раздельные
`docker exec`, без общего окружения; после каждого фоновые `ros2 bag play` и `odometry_node`
проверены и убиты (`ps aux` / `kill -9`) перед следующим сценарием.

## Вывод

- Без `cd <ws>` в новой сессии — второй `source` падает молча (`No such file or directory`),
  но `ros2 bag play` на следующей строке всё равно запускается и тихо отбрасывает все три
  входных топика (WARN, без падения плеера); `/result/velocity` не публикуется вообще.
- С `cd <ws> && source /opt/ros/humble/setup.bash && source install/setup.bash` в каждом
  терминале (README §2/§3 после фикса), воспроизведено с нуля в изолированных сессиях: без
  WARN, `/result/velocity` на ожидаемой частоте (~39–40 Гц).

Скрипты (`step1_build.sh`, `term1_launch.sh`, `term2_no_cd.sh`, `term2_with_cd.sh`,
`term_hz_debug.sh`) не сохранены в репозитории — временные; их тела приведены выше
целиком и совпадают дословно с блоками README §2/§3 на момент проверки (кроме
`term2_no_cd.sh`, который намеренно воспроизводит текст README §2 **без** правки — старое
поведение для контраста).
