#!/usr/bin/env bash
# Вход в dev-контейнер ROS 2 Humble с текущим worktree в /ws и данными в /data (только чтение).
#   bash docker/dev.sh                 # интерактивный bash
#   bash docker/dev.sh colcon build    # одна команда
# Данные: TRAM_DATA_DIR из окружения или .env, иначе <основная копия>/dataset/data.
set -euo pipefail
ROOT="$(git rev-parse --show-toplevel)"
MAIN="$(cd "$(git rev-parse --path-format=absolute --git-common-dir)/.." && pwd)"
[ -f "$ROOT/.env" ] && set -a && . "$ROOT/.env" && set +a
DATA="${TRAM_DATA_DIR:-$MAIN/dataset/data}"
case "$DATA" in /*) ;; *) DATA="$ROOT/$DATA" ;; esac
[ -d "$DATA" ] || { echo "Нет данных: $DATA (распаковать по docs/data.md или задать TRAM_DATA_DIR)" >&2; exit 1; }

docker build -q -f "$ROOT/docker/Dockerfile" --target dev -t tram-odom:dev "$ROOT" >/dev/null
TTY=(); [ -t 0 ] && TTY=(-it)
exec docker run --rm "${TTY[@]}" --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e COLCON_DEFAULTS_FILE=/ws/docker/colcon-defaults.yaml \
  -v "$ROOT:/ws" -v "$DATA:/data:ro" -w /ws tram-odom:dev \
  bash -c 'source /opt/ros/humble/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; if [ $# -eq 0 ]; then exec bash; else exec "$@"; fi' _ "$@"
