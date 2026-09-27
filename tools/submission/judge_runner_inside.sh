#!/usr/bin/env bash
# Выполняется внутри контейнера tools/submission/judge_runner.sh. Не запускать на хосте.
# /repo — git archive проверяемого commit, /bags/<name> — сценарные bag, /plan.tsv — сценарии
# в порядке запуска (один контейнер на все), /out — отчёт.
# Строка плана: name  bag  mode  delay_s
#   protocol — свежая нода, ожидание входных подписок, потом единственный bag play
#   late     — bag play, через delay_s запуск ноды (отрицательный сценарий #201)
set -o pipefail  # без -u: setup.bash ROS обращается к неопределённым переменным
source /opt/ros/humble/setup.bash
READY_TIMEOUT_S=20      # с, готовность подписок свежей ноды
STOP_TIMEOUT_S=15       # с, завершение ноды и записи после SIGINT
DRAIN_S=2               # с после конца bag play: последние выходы доходят до записи
INPUTS="/vehicle/front_bogie_velocity /vehicle/rear_bogie_velocity /vehicle/driver_position_cmd
/sensing/gnss/master/fix /sensing/gnss/rover/fix /sensing/gnss/master/vel"
RESULTS="/result/velocity /result/position /result/diagnostics"

now() { date +%s.%N; }

mkdir -p /tmp/ws && cp -r /repo/src /tmp/ws/src && cd /tmp/ws
echo "== colcon build (сеть отключена)"
colcon build --event-handlers console_cohesion+ > /out/build.log 2>&1
status=$?; tail -3 /out/build.log
[ $status -eq 0 ] || { echo "СБОРКА УПАЛА, лог: build.log"; exit 1; }
source install/setup.bash
LAUNCH=install/tram_odometry/share/tram_odometry/launch/odometry.launch.py

stop() { # stop <pid> <имя> <каталог>: SIGINT своему процессу (группе), ждать, записать exit code
  local pid="$1" name="$2" dir="$3" code
  if kill -0 "$pid" 2>/dev/null; then
    echo "$name running" >> "$dir/alive.txt"
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
    for _ in $(seq $((STOP_TIMEOUT_S * 10))); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
    if kill -0 "$pid" 2>/dev/null; then
      echo "$name не завершился за $STOP_TIMEOUT_S с после SIGINT — SIGKILL" >> "$dir/errors.txt"
      kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid"
    fi
  else
    echo "$name exited before stop" >> "$dir/alive.txt"
  fi
  wait "$pid"; code=$?
  printf '%s_exit\t%s\n' "$name" "$code" >> "$dir/exit.tsv"
}

run() { # run <name> <bag> <mode> <delay_s>
  local name="$1" bag="/bags/$2" mode="$3" delay="$4" dir="/out/$1" node rec play
  mkdir -p "$dir"; : > "$dir/exit.tsv"; : > "$dir/times.tsv"; : > "$dir/alive.txt"
  printf '%s\t%s\t%s\t%s\n' "$name" "$2" "$mode" "$delay" > "$dir/plan.tsv"
  echo "== $name: $2, $mode, delay $delay s"
  # свежая нода: ни одного процесса ноды от прошлого сценария
  if pgrep -f odometry_node > /dev/null; then
    echo "перед запуском жив процесс ноды прошлого сценария: $(pgrep -af odometry_node)" >> "$dir/errors.txt"
  fi
  printf 'start\t%s\n' "$(now)" >> "$dir/times.tsv"
  setsid ros2 bag record -o "$dir/record" $RESULTS > "$dir/record.log" 2>&1 & rec=$!
  if [ "$mode" = protocol ]; then
    setsid ros2 launch "$LAUNCH" > "$dir/node.log" 2>&1 & node=$!
    printf 'node_launch\t%s\n' "$(now)" >> "$dir/times.tsv"
    python3 /runner/wait_ready.py "$READY_TIMEOUT_S" $INPUTS > "$dir/ready.log" 2>&1
    printf 'ready_exit\t%s\n' "$?" >> "$dir/exit.tsv"
    python3 /runner/wait_ready.py "$READY_TIMEOUT_S" /result/velocity /result/position \
      > "$dir/record_ready.log" 2>&1
    printf 'record_ready_exit\t%s\n' "$?" >> "$dir/exit.tsv"
    printf 'ready\t%s\n' "$(now)" >> "$dir/times.tsv"
    printf 'play\t%s\n' "$(now)" >> "$dir/times.tsv"
    ros2 bag play "$bag" > "$dir/play.log" 2>&1
    printf 'play_exit\t%s\n' "$?" >> "$dir/exit.tsv"
  else
    printf 'play\t%s\n' "$(now)" >> "$dir/times.tsv"
    ros2 bag play "$bag" > "$dir/play.log" 2>&1 & play=$!
    sleep "$delay"
    setsid ros2 launch "$LAUNCH" > "$dir/node.log" 2>&1 & node=$!
    printf 'node_launch\t%s\n' "$(now)" >> "$dir/times.tsv"
    python3 /runner/wait_ready.py "$READY_TIMEOUT_S" $INPUTS > "$dir/ready.log" 2>&1
    printf 'ready_exit\t%s\n' "$?" >> "$dir/exit.tsv"
    printf 'ready\t%s\n' "$(now)" >> "$dir/times.tsv"
    wait "$play"
    printf 'play_exit\t%s\n' "$?" >> "$dir/exit.tsv"
  fi
  printf 'play_end\t%s\n' "$(now)" >> "$dir/times.tsv"
  sleep "$DRAIN_S"
  stop "$rec" record "$dir"
  stop "$node" node "$dir"
  printf 'stopped\t%s\n' "$(now)" >> "$dir/times.tsv"
  if pgrep -f odometry_node > /dev/null; then
    echo "после остановки жив процесс ноды: $(pgrep -af odometry_node)" >> "$dir/errors.txt"
  fi
  python3 /runner/check_recording.py "$bag" "$dir/record" > "$dir/check_recording.log" 2>&1
  printf 'check_recording_exit\t%s\n' "$?" >> "$dir/exit.tsv"
  cat "$dir/exit.tsv" | tr '\n' ' '; echo
}

while IFS=$'\t' read -r name bag mode delay _expect; do
  [ -z "$name" ] || [ "${name#\#}" != "$name" ] && continue
  run "$name" "$bag" "$mode" "$delay" < /dev/null   # bag play не читает план из stdin
done < /plan.tsv
echo "== сценарии пройдены, отчёт: judge_runner_report.py"
