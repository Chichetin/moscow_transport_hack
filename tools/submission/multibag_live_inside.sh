#!/usr/bin/env bash
# Invoked by multibag_live.sh inside the jury container; all bags in a group use one node.
set -o pipefail
source /opt/ros/humble/setup.bash
mkdir -p /tmp/ws && cp -r /repo/src /tmp/ws/src && cd /tmp/ws
colcon build --event-handlers console_cohesion+ > /out/build.log 2>&1 || { tail -30 /out/build.log; exit 1; }
tail -3 /out/build.log
source install/setup.bash
LAUNCH=install/tram_odometry/share/tram_odometry/launch/odometry.launch.py
INPUTS='/vehicle/front_bogie_velocity /vehicle/rear_bogie_velocity /vehicle/driver_position_cmd /sensing/gnss/master/fix /sensing/gnss/rover/fix /sensing/gnss/master/vel'
RESULTS='/result/velocity /result/position /result/diagnostics'
failed=0
stop_process() {
  local pid="$1" label="$2" dest="$3" i
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
  for i in $(seq 1 150); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
  if kill -0 "$pid" 2>/dev/null; then
    echo "$label did not exit after SIGINT" >> "$dest/errors.txt"
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
    failed=1
  fi
  wait "$pid"
  printf '%s_exit\t%s\n' "$label" "$?" >> "$dest/exit.tsv"
}
run_group() {
  local group="$1" node rec bag expect stage
  shift
  mkdir -p "/out/$group"
  : > "/out/$group/exit.tsv"
  echo "== group $group: one node, $# bags"
  setsid ros2 launch "$LAUNCH" > "/out/$group/node.log" 2>&1 & node=$!
  python3 /runner/wait_ready.py 20 $INPUTS > "/out/$group/ready.log" 2>&1
  printf 'ready_exit\t%s\n' "$?" >> "/out/$group/exit.tsv"
  for spec in "$@"; do
    bag="${spec%%:*}"; expect="${spec#*:}"; stage="/out/$group/$bag"
    mkdir -p "$stage"
    : > "$stage/exit.tsv"
    printf '%s\n' "$expect" > "$stage/expect.txt"
    setsid ros2 bag record -o "$stage/record" $RESULTS > "$stage/record.log" 2>&1 & rec=$!
    python3 /runner/wait_ready.py 20 $RESULTS > "$stage/record_ready.log" 2>&1
    printf 'record_ready_exit\t%s\n' "$?" >> "$stage/exit.tsv"
    echo "== $group/$bag play"
    ros2 bag play "/bags/$bag" > "$stage/play.log" 2>&1
    printf 'play_exit\t%s\n' "$?" >> "$stage/exit.tsv"
    sleep 2
    stop_process "$rec" record "$stage"
    python3 /runner/check_recording.py "/bags/$bag" "$stage/record" > "$stage/check_recording.log" 2>&1
    printf 'check_exit\t%s\n' "$?" >> "$stage/exit.tsv"
    sleep 2
  done
  stop_process "$node" node "/out/$group"
}
run_group three a:map b:map c:map
run_group no_gnss a:map b_no_gnss:odom
run_group gnss_first a:map b_gnss_first:map
run_group fresh_a a:map
run_group fresh_b b:map
run_group fresh_c c:map
exit "$failed"
