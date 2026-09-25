#!/usr/bin/env bash
# Стенд жюри: то же ограничение ресурсов, что в критерии 4 (task.md), и никакой сети.
#   bash docker/jury-stand.sh <bag_id> [rate]     # например 30618_0e41eac3 1.0
# Шаги: чистый colcon build без сети -> нода по launch -> ros2 bag play -> запись /result/*
# и входов -> замеры (задержка, частота, CPU, RAM) в $TRAM_OUT_DIR/stand/<bag>/.
# Пока ноды нет в main (план, пакет RT-1), стенд проверяет только офлайн-сборку.
set -euo pipefail
BAG="${1:?bag id, например 30618_0e41eac3}"; RATE="${2:-1.0}"
ROOT="$(git rev-parse --show-toplevel)"
MAIN="$(cd "$(git rev-parse --path-format=absolute --git-common-dir)/.." && pwd)"
[ -f "$ROOT/.env" ] && set -a && . "$ROOT/.env" && set +a
DATA="${TRAM_DATA_DIR:-$MAIN/dataset/data}"; case "$DATA" in /*) ;; *) DATA="$ROOT/$DATA" ;; esac
OUT="${TRAM_OUT_DIR:-$ROOT/out}"; case "$OUT" in /*) ;; *) OUT="$ROOT/$OUT" ;; esac
OUT="$OUT/stand/$BAG"; mkdir -p "$OUT"
[ -d "$DATA/$BAG" ] || { echo "Нет bag: $DATA/$BAG" >&2; exit 1; }

rm -rf "$OUT/record" "$OUT/stand.json"   # запись прошлого прогона не должна попасть в замер
docker build -q -f "$ROOT/docker/Dockerfile" --target jury -t tram-odom:jury "$ROOT" >/dev/null
# src копируется внутрь (read-only монтирование + свой build/), чтобы не пачкать worktree.
docker run --rm --cpus=2 --memory=512m --memory-swap=512m --network=none \
  -v "$ROOT/src:/src:ro" -v "$ROOT/docker:/stand:ro" -v "$ROOT/tools/stand:/stand-tools:ro" -v "$DATA/$BAG:/bag:ro" -v "$OUT:/out" \
  -e RATE="$RATE" tram-odom:jury bash /stand/stand-inside.sh
echo "Результаты: $OUT"
PY="$ROOT/.venv/bin/python"; [ -x "$PY" ] || PY=python3
"$PY" "$ROOT/tools/stand/run_stand.py" "$OUT" || echo "замеры не посчитаны (нет записи ноды или нет rosbags)" >&2
