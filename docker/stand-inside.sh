#!/usr/bin/env bash
# Выполняется внутри контейнера стенда (docker/jury-stand.sh). Не запускать на хосте.
set -o pipefail  # без -u: setup.bash ROS обращается к неопределённым переменным
source /opt/ros/humble/setup.bash
mkdir -p /ws && cp -r /src /ws/src && cd /ws
echo "== colcon build (сеть отключена)"
colcon build --event-handlers console_cohesion+ > /out/build.log 2>&1
status=$?; tail -3 /out/build.log
[ $status -eq 0 ] || { echo "СБОРКА УПАЛА, лог: build.log"; exit 1; }
source install/setup.bash

if ! ros2 pkg prefix tram_odometry >/dev/null 2>&1 || [ -z "$(ls install/tram_odometry/share/tram_odometry/launch/*.launch.py 2>/dev/null)" ]; then
  echo "== нода ещё не подключена (нет launch в tram_odometry) — проверена только сборка"
  exit 0
fi

LAUNCH="$(ls install/tram_odometry/share/tram_odometry/launch/*.launch.py | head -1)"
echo "== запуск $(basename "$LAUNCH"), bag play x$RATE"
ros2 launch "$LAUNCH" > /out/node.log 2>&1 &
NODE=$!
sleep 3
ros2 bag record -o /out/record /result/velocity /result/position /result/diagnostics \
  /vehicle/front_bogie_velocity /vehicle/rear_bogie_velocity /vehicle/driver_position_cmd \
  > /out/record.log 2>&1 &
REC=$!
sleep 2
# CPU/RAM всех процессов ноды раз в секунду (замер RT-1 заменит на точный инструмент)
( while kill -0 $NODE 2>/dev/null; do
    ps -eo pid,pcpu,rss,etimes,comm --no-headers | grep -E 'python3|odometry' >> /out/resources.txt
    sleep 1; done ) &
ros2 bag play /bag --rate "$RATE" > /out/play.log 2>&1
sleep 2
kill -INT $REC $NODE 2>/dev/null; wait $REC 2>/dev/null
echo "== запись: /out/record, ресурсы: resources.txt, лог ноды: node.log"
