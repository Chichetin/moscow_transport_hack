#!/usr/bin/env bash
# Выполняется внутри контейнера tools/submission/jury_layouts.sh. Не запускать на хосте.
# /repo — git archive проверяемого commit (только чтение), /bags/<id> — bag, /out — отчёт.
set -o pipefail  # без -u: setup.bash ROS обращается к неопределённым переменным
source /opt/ros/humble/setup.bash
LOGS="/out/layouts-$COMMIT"; mkdir -p "$LOGS"
RES=/tmp/results.tsv; : > "$RES"

# Пакет сообщений у жюри. Оригинал организаторов в Humble не собирается (нет <maintainer>,
# D-006), значит у жюри рабочая копия — такая же, как вложенная в наш vendor (D-044); оригинал
# нужен для раскладки «поверх».
JURY_MSGS=/tmp/jury_msgs/tram_vehicle_msgs
ORIG_MSGS=/tmp/orig_msgs/tram_vehicle_msgs
mkdir -p "$JURY_MSGS" "$ORIG_MSGS"
for d in "$JURY_MSGS" "$ORIG_MSGS"; do
  cp /repo/src/tram_vehicle_msgs/vendor/* "$d/" && cp -r /repo/src/tram_vehicle_msgs/msg "$d/"
done
sed -i '/<maintainer/d' "$ORIG_MSGS/package.xml"
# Образ судьи организаторов (check-code, #184): tram_vehicle_msgs только с VelocitySensor.
# Наш vendor видит этот пакет и свою копию не собирает, DriverControllerCommand нет — нода
# обязана импортироваться и работать по тележкам, bag play топик контроллера пропустит.
JUDGE_MSGS=/tmp/judge_msgs/tram_vehicle_msgs
mkdir -p "$JUDGE_MSGS/msg"
cp /repo/src/tram_vehicle_msgs/vendor/* "$JUDGE_MSGS/" &&
  cp /repo/src/tram_vehicle_msgs/msg/VelocitySensor.msg "$JUDGE_MSGS/msg/"
sed -i '/msg\/DriverControllerCommand.msg/d' "$JUDGE_MSGS/CMakeLists.txt"

build() { # build <раскладка> <каталог workspace> [аргументы colcon build]
  local name="$1" ws="$2"; shift 2
  (cd "$ws" && colcon build "$@" > "$LOGS/$name.build.log" 2>&1)
}

importable() { # importable <раскладка> <каталог workspace>: сообщения и нода видны после source
  local msgs='VelocitySensor, DriverControllerCommand'
  case "$1" in judge_*) msgs='VelocitySensor' ;; esac   # в пакете судьи контроллера нет
  (source "$2/install/setup.bash" &&
   python3 -c "from tram_vehicle_msgs.msg import $msgs" &&
   python3 -c 'import tram_odometry.odometry_node') > "$LOGS/$1.import.log" 2>&1
}

layout() { # layout <имя> <что имитирует>; готовит /tmp/<имя>/ws, собирает, пишет строку результата
  local name="$1" what="$2" ws="/tmp/$1/ws" status note=""
  mkdir -p "$ws/src"
  case "$name" in
    readme)            cp -r /repo/src/* "$ws/src/" ;;
    clone_in_src)      cp -r /repo "$ws/src/repo" ;;
    repo_root)         rm -rf "$ws" && cp -r /repo "$ws" ;;
    jury_msgs_in_src)  cp -r "$JURY_MSGS" "$ws/src/" && cp -r /repo "$ws/src/repo" ;;
    over_orig_msgs)    cp -r "$ORIG_MSGS" "$ws/src/" && cp -r /repo/src/* "$ws/src/" ;;
    jury_msgs_over)    cp -r /repo/src/* "$ws/src/" && cp -r "$JURY_MSGS" "$ws/src/" ;;
    judge_msgs_in_src) cp -r "$JUDGE_MSGS" "$ws/src/" && cp -r /repo "$ws/src/repo" ;;
    judge_msgs_over)   cp -r /repo/src/* "$ws/src/" && cp -r "$JUDGE_MSGS" "$ws/src/" ;;
    up_to|merge_install|symlink_install) cp -r /repo/src/* "$ws/src/" ;;
    underlay)          mkdir -p "/tmp/$name/under/src" && cp -r "$JURY_MSGS" "/tmp/$name/under/src/" &&
                       build "$name.underlay" "/tmp/$name/under" &&
                       source "/tmp/$name/under/install/setup.bash" && cp -r /repo/src/* "$ws/src/" ;;
  esac
  local args=()
  case "$name" in
    up_to)           args=(--packages-up-to tram_odometry) ;;
    merge_install)   args=(--merge-install) ;;
    symlink_install) args=(--symlink-install) ;;
  esac
  if ! build "$name" "$ws" "${args[@]}"; then
    status=fail; note="$(grep -m1 -E 'Duplicate package|failed|Error' "$LOGS/$name.build.log" | cut -c1-200)"
  elif ! importable "$name" "$ws"; then
    status=fail; note="не импортируется: $(tail -1 "$LOGS/$name.import.log" | cut -c1-180)"
  else status=ok; fi
  printf '%s\t%s\t%s\t%s\n' "$name" "$what" "$status" "$note" >> "$RES"
  echo "== $name: $status ${note:+— $note}"
}

layout readme           "README: cp -r src/* <ws>/src/ && colcon build"
layout clone_in_src     "git clone репозитория в <ws>/src/ пустого workspace"
layout repo_root        "colcon build прямо в корне клона"
layout jury_msgs_in_src "клон в <ws>/src/, где уже лежит tram_vehicle_msgs жюри (README данных, §6.1)"
layout over_orig_msgs   "cp -r src/* поверх оригинала организаторов в <ws>/src/"
layout jury_msgs_over   "cp -r src/*, затем tram_vehicle_msgs жюри поверх в <ws>/src/ (README данных, §6.1)"
layout up_to            "README, но colcon build --packages-up-to tram_odometry"
layout merge_install    "README, но colcon build --merge-install"
layout symlink_install  "README, но colcon build --symlink-install"
layout underlay         "tram_vehicle_msgs жюри собран отдельно (underlay), наш src поверх"
layout judge_msgs_in_src "клон в <ws>/src/ образа судьи: tram_vehicle_msgs только с VelocitySensor (#184)"
layout judge_msgs_over   "cp -r src/*, затем tram_vehicle_msgs судьи (только VelocitySensor) поверх (#184)"

# Запуск: в раскладке README (install-дерево то же во всех, кроме пакета сообщений) каждый bag
# отдельно; первый bag ещё раз в judge_msgs_over — без DriverControllerCommand (#184).
RUN=/tmp/run.tsv; : > "$RUN"
run_bag() { # run_bag <каталог workspace> <bag> <ключ в отчёте>
  local ws="$1" b="$2" key="$3" launch
  launch="$(ls "$ws"/install/tram_odometry/share/tram_odometry/launch/*.launch.py 2>/dev/null | head -1)"
  if [ -z "$launch" ]; then
    printf '%s\t%s\t%s\n' "$key" skipped "нет launch в tram_odometry" >> "$RUN"; return
  fi
  ( # без job control фоновые процессы стартуют с игнорируемым SIGINT и не останавливаются
    set -m
    source "$ws/install/setup.bash"
    ros2 launch "$launch" > "$LOGS/run.$key.node.log" 2>&1 & NODE=$!
    sleep 3
    ros2 bag record -o "/tmp/rec_$key" /result/velocity /result/position > "$LOGS/run.$key.record.log" 2>&1 & REC=$!
    sleep 2
    timeout 25 ros2 bag play "/bags/$b" > "$LOGS/run.$key.play.log" 2>&1
    sleep 2
    alive=yes; kill -0 $NODE 2>/dev/null || alive=no
    kill -INT $REC 2>/dev/null; wait $REC 2>/dev/null
    kill -INT $NODE 2>/dev/null; wait $NODE 2>/dev/null
    info="$(ros2 bag info "/tmp/rec_$key" 2>/dev/null)"
    vel="$(printf '%s\n' "$info" | grep 'Topic: /result/velocity' | grep -oE 'Count: [0-9]+' | grep -oE '[0-9]+')"
    pos="$(printf '%s\n' "$info" | grep 'Topic: /result/position' | grep -oE 'Count: [0-9]+' | grep -oE '[0-9]+')"
    # исключение ядра нода ловит и живёт дальше, но вход пропущен — это поломка (#184)
    errs="$(grep -cE 'input skipped|Traceback' "$LOGS/run.$key.node.log")"
    note="нода жива: $alive; /result/velocity: ${vel:-0}; /result/position: ${pos:-0}; ошибок в логе ноды: $errs"
    if [ "$alive" = yes ] && [ "${vel:-0}" -gt 0 ] && [ "${pos:-0}" -gt 0 ] && [ "$errs" -eq 0 ]; then
      # проверка читает и входной топик контроллера: типы из README-раскладки (полный пакет)
      if checked="$(source /tmp/readme/ws/install/setup.bash &&
                    python3 /repo/tools/submission/check_recording.py "/bags/$b" "/tmp/rec_$key" 2>&1)"; then
        st=ok
      else
        st=fail
      fi
      note="$note; $checked"
    else st=fail; fi
    python3 /repo/tools/submission/layout_report.py "$RUN" "$key" "$st" "$note"
  )
  echo "== run $key: $(tail -1 "$RUN" | cut -f2-)"
}
for b in $BAGS; do run_bag /tmp/readme/ws "$b" "$b"; done
FIRST="${BAGS%% *}"
[ -n "$FIRST" ] && run_bag /tmp/judge_msgs_over/ws "$FIRST" "$FIRST@judge_msgs"

python3 - "$RES" "$RUN" "/out/layouts-$COMMIT.json" <<'PY'
import datetime, json, os, sys
sys.path.insert(0, '/repo/tools/submission')
from layout_report import read_runs
rows = [l.rstrip('\n').split('\t') for l in open(sys.argv[1]) if l.strip()]
report = {
    'commit': os.environ['COMMIT'], 'dirty': int(os.environ['DIRTY']),
    'created': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
    'layouts': [{'name': n, 'what': w, 'build': s, 'note': note} for n, w, s, note in rows],
    'runs': read_runs(sys.argv[2]),
}
json.dump(report, open(sys.argv[3], 'w'), ensure_ascii=False, indent=2)
print('отчёт:', sys.argv[3])
PY
