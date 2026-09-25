#!/usr/bin/env bash
# Собирает и запускает решение так, как его может разложить жюри (D-022), из закоммиченного
# HEAD (git archive), а не из рабочего дерева: незакоммиченное здесь не видно — так и задумано.
#   bash tools/submission/jury_layouts.sh              # только сборка во всех раскладках
#   bash tools/submission/jury_layouts.sh --run        # + нода и bag: с GNSS и без GNSS
#   bash tools/submission/jury_layouts.sh --run <bag>… # свои bag
# Каждая раскладка — чистый ros:humble-ros-base (образ jury), --network=none, 2 CPU, 512 МБ,
# не root. Отчёт — $TRAM_OUT_DIR/submission/layouts-<commit>.json, его читает check_submission.py.
# Упавшая раскладка — это риск «у жюри не собралось»: сразу issue с меткой blocker (CLAUDE.md).
set -euo pipefail
ROOT="$(git rev-parse --show-toplevel)"
MAIN="$(cd "$(git rev-parse --path-format=absolute --git-common-dir)/.." && pwd)"
[ -f "$ROOT/.env" ] && set -a && . "$ROOT/.env" && set +a
DATA="${TRAM_DATA_DIR:-$MAIN/dataset/data}"; case "$DATA" in /*) ;; *) DATA="$ROOT/$DATA" ;; esac
OUT="${TRAM_OUT_DIR:-$ROOT/out}"; case "$OUT" in /*) ;; *) OUT="$ROOT/$OUT" ;; esac
OUT="$OUT/submission"; mkdir -p "$OUT"

BAGS=()
if [ "${1:-}" = --run ]; then
  shift
  # по умолчанию 20 с holdout с GNSS и 98 с без GNSS: нода обязана публиковать в обоих случаях
  if [ $# -gt 0 ]; then BAGS=("$@"); else BAGS=(30618_082f1d65 30618_5036aa78); fi
fi
MOUNTS=()
for b in "${BAGS[@]}"; do
  [ -d "$DATA/$b" ] || { echo "Нет bag: $DATA/$b" >&2; exit 1; }
  MOUNTS+=(-v "$DATA/$b:/bags/$b:ro")
done

COMMIT="$(git -C "$ROOT" rev-parse HEAD)"
DIRTY="$(git -C "$ROOT" status --porcelain --untracked-files=no | wc -l | tr -d ' ')"
[ "$DIRTY" = 0 ] || echo "ВНИМАНИЕ: $DIRTY незакоммиченных изменений не проверяются — проверяется $COMMIT"

TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/repo" && git -C "$ROOT" archive HEAD | tar -x -C "$TMP/repo"
docker build -q -f "$TMP/repo/docker/Dockerfile" --target jury -t tram-odom:jury "$TMP/repo" >/dev/null

docker run --rm --network=none --cpus=2 --memory=512m --memory-swap=512m \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e COMMIT="$COMMIT" -e DIRTY="$DIRTY" \
  -e BAGS="${BAGS[*]}" -v "$TMP/repo:/repo:ro" -v "$OUT:/out" "${MOUNTS[@]}" \
  tram-odom:jury bash /repo/tools/submission/layouts_inside.sh
