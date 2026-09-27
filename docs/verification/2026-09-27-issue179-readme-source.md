# #179 — README §2/§3: `cd <ws> && source install/setup.bash` в каждом терминале

Проверено 2026-09-27, worktree `worktree-179-readme-source`, `git archive HEAD` на коммите
`ebfa4087d03df95919f24b6e9bec5215d8a6b2ef` (`src/` в последующих правках этого issue не
менялся — годится любой коммит после `a2181fb`). Экспорт в `/tmp` (не рабочее дерево),
контейнер `ros:humble-ros-base`, `--network=none`, bag `30618_01f73500` из `dataset/data`
(длительность 1211,98 с ≈ 20,2 мин — не заканчивается в ходе проверки). Каждый «терминал» —
**отдельный** вызов `docker exec` (своя шелл-сессия). Перед каждым замером `ros2 topic hz`
явно подтверждено командой `ps aux`, что фоновый `ros2 bag play` из предыдущего шага ещё жив
(bag не закончился, доказательство не тривиально).

## Предостережение из первой попытки (не повторять)

Первая попытка проверить сценарий «до фикса» через `topic hz` запускала bag play в подшелле
`(cd; source ...; ros2 bag play ...) &` **внутри одного и того же** `docker exec`, первой
строкой которого было `cd /ws && source ... && source install/setup.bash` — для самого
`topic hz`. Подшелл унаследовал уже экспортированные переменные окружения
(`AMENT_PREFIX_PATH` и другие, которые выставляет `source install/setup.bash`) от
родительской сессии: «второй терминал» в этом случае не был независимым, `ros2 bag play`
внутри него на самом деле видел `tram_vehicle_msgs`, и результат — ложноположительные
~40 Гц без единого WARN. Ниже — только прогоны с полностью раздельными `docker exec`.

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
Finished <<< tram_odometry_core [0.61s]
Finished <<< tram_vehicle_msgs_vendor [4.35s]
Starting >>> tram_odometry
Finished <<< tram_odometry [0.56s]

Summary: 3 packages finished [5.10s]
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
$ sleep 3
$ docker exec <ctr> bash -c "ps aux | grep odometry_node | grep -v grep"
root  703  ... /ws/install/tram_odometry/lib/tram_odometry/odometry_node ...
```

Нода жива.

## Сценарий «до фикса» — терминал 2 без `cd <ws>`, сессия начинается в `$HOME`

`term2_no_cd.sh` (воспроизводит README §2 **без** правки этого PR — старый текст, для
контраста; `exec > .../log 2>&1` в строке 2 нужен только для захвата вывода фонового
процесса в файл, самого сценария не касается):

```bash
#!/bin/bash
exec > /ws/term2_no_cd.log 2>&1
cd
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play /data/30618_01f73500
```

Запуск в фоне (отдельный терминал) и подтверждение, что процесс жив:
```
$ docker exec -d <ctr> bash /ws/term2_no_cd.sh
$ sleep 2
$ docker exec <ctr> bash -c "ps aux | grep 'bag play' | grep -v grep"
root  737  ...  ros2 bag play /data/30618_01f73500
```

Замер в **третьей**, ещё одной отдельной сессии, сразу после подтверждения живого процесса:

`term3_hz.sh`:
```bash
#!/bin/bash
cd /ws && source /opt/ros/humble/setup.bash && source install/setup.bash
timeout 8 ros2 topic hz /result/velocity
```

```
$ docker exec <ctr> bash /ws/term3_hz.sh > before_hz.log 2>&1; echo "EXIT=$?"
EXIT=124
$ cat before_hz.log
(пусто)
$ docker exec <ctr> bash -c "ps aux | grep 'bag play' | grep -v grep"   # процесс всё ещё жив
root  737  ...  ros2 bag play /data/30618_01f73500
```

`/result/velocity` не публикуется вообще: `ros2 topic hz` висит без единой строки вывода до
таймаута (`timeout 8`, код возврата 124) — не потому, что bag уже доиграл (процесс подтверждён
живым и до, и после замера), а потому, что вход не поступает в ноду.

Лог самого `term2_no_cd.sh`, подтверждающий причину и WARN (снят после остановки процесса):
```
$ docker exec <ctr> bash -c "kill -9 737"
$ cat term2_no_cd.log
/ws/term2_no_cd.sh: line 5: install/setup.bash: No such file or directory
stdin is not a terminal device. Keyboard handling disabled.
[INFO] ... rosbag2_storage: Opened database ... READ_ONLY.
[INFO] ... rosbag2_player: Set rate to 1
[WARN] ... rosbag2_player: Ignoring a topic '/vehicle/rear_bogie_velocity', reason: package 'tram_vehicle_msgs' not found, searching: [/opt/ros/humble].
[WARN] ... rosbag2_player: Ignoring a topic '/vehicle/driver_position_cmd', reason: package 'tram_vehicle_msgs' not found, searching: [/opt/ros/humble].
[WARN] ... rosbag2_player: Ignoring a topic '/vehicle/front_bogie_velocity', reason: package 'tram_vehicle_msgs' not found, searching: [/opt/ros/humble].
[INFO] ... rosbag2_player: Adding keyboard callbacks.
...
/ws/term2_no_cd.sh: line 6:   737 Killed   ros2 bag play /data/30618_01f73500
```

Строка ошибки (`line 5: install/setup.bash: No such file or directory`) в точности совпадает
со строкой 5 приведённого выше тела скрипта (`source install/setup.bash`), выполненной без
`cd <ws>` — относительный путь не находится из `$HOME`. `source` завершается с ошибкой, но
это отдельная команда, не звено `&&`-цепочки с `ros2 bag play` на следующей строке — плеер
всё равно запускается и пишет три WARN, не падая.

## Сценарий «после фикса» — терминал 2 с `cd <ws>` (README §2/§3), отдельная сессия

`term2_with_cd.sh`:
```bash
#!/bin/bash
exec > /ws/term2_with_cd.log 2>&1
cd /ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 bag play /data/30618_01f73500
```

```
$ docker exec -d <ctr> bash /ws/term2_with_cd.sh
$ sleep 2
$ docker exec <ctr> bash -c "ps aux | grep 'bag play' | grep -v grep"
root  837  ...  ros2 bag play /data/30618_01f73500
```

Замер `term3_hz.sh` в отдельной сессии сразу после подтверждения живого процесса:
```
$ docker exec <ctr> bash /ws/term3_hz.sh > after_hz.log 2>&1; echo "EXIT=$?"
EXIT=124
$ cat after_hz.log
average rate: 41.966
	min: 0.000s max: 0.051s std dev: 0.02420s window: 42
average rate: 40.498
	...
average rate: 39.720
	...
$ docker exec <ctr> bash -c "ps aux | grep 'bag play' | grep -v grep"   # процесс всё ещё жив
root  837  ...  ros2 bag play /data/30618_01f73500
```

`/result/velocity` публикуется ~40–42 Гц (README §3 ожидает ≈ 40 Гц: 2 × 10 Гц тележки +
20 Гц контроллер). `EXIT=124` здесь — это возврат `timeout 8` после успешного измерения
(ожидаемое завершение, не ошибка).

Лог `term2_with_cd.sh` — без WARN (снят после остановки процесса):
```
$ docker exec <ctr> bash -c "kill -9 837"
$ cat term2_with_cd.log
stdin is not a terminal device. Keyboard handling disabled.
[INFO] ... rosbag2_storage: Opened database ... READ_ONLY.
[INFO] ... rosbag2_player: Set rate to 1
[INFO] ... rosbag2_player: Adding keyboard callbacks.
...
/ws/term2_with_cd.sh: line 4:   837 Killed   ros2 bag play /data/30618_01f73500
```

## Вывод

- Без `cd <ws>` в новой сессии: второй `source` падает отдельной командой (`No such file or
  directory`, относительный путь), но `ros2 bag play` на следующей строке всё равно
  запускается — WARN на всех трёх входных топиках, `/result/velocity` не публикуется вообще
  (`topic hz` висит без вывода до таймаута; фоновый bag play подтверждён живым и до, и после
  замера — значит дело не в том, что bag уже доиграл).
- С `cd <ws> && source /opt/ros/humble/setup.bash && source install/setup.bash` в каждом
  терминале (README §2/§3 после фикса): без WARN, `/result/velocity` на ожидаемой частоте
  (~40–42 Гц), тем же образом подтверждено живым процессом bag play.

Скрипты не сохранены в репозитории — временные; их тела приведены выше целиком, дословно
совпадают с тем, что было выполнено (line-номера в сообщениях об ошибках сверены с телом
скрипта построчно).
