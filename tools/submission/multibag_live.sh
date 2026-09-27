#!/usr/bin/env bash
# Live #200 check: one launched node, sequential bag plays, then fresh-node controls.
set -euo pipefail
ROOT="$(git rev-parse --show-toplevel)"
MAIN="$(cd "$(git rev-parse --path-format=absolute --git-common-dir)/.." && pwd)"
[ -f "$MAIN/.env" ] && set -a && . "$MAIN/.env" && set +a
DATA="${TRAM_DATA_DIR:-$MAIN/dataset/data}"
OUT="${TRAM_OUT_DIR:-$ROOT/out}"
case "$DATA" in /*) ;; *) DATA="$ROOT/$DATA";; esac
case "$OUT" in /*) ;; *) OUT="$ROOT/$OUT";; esac
PY="$MAIN/.venv/bin/python"
COMMIT="$(git rev-parse HEAD)"
RUN="$OUT/submission/multibag-$COMMIT-$(date +%Y%m%dT%H%M%S)"
mkdir -p "$RUN/bags"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir "$TMP/repo"
git archive HEAD | tar -x -C "$TMP/repo"
SCEN="$TMP/repo/tools/submission"
for spec in '30618_bc5e53c2 a crop 90' '30618_082f1d65 b crop 90' \
            '30618_af7496f0 c crop 30' '30618_082f1d65 b_no_gnss no_gnss 90' \
            '30618_082f1d65 b_gnss_first gnss_first 90'; do
  read -r source name variant seconds <<< "$spec"
  "$PY" "$SCEN/make_scenario_bags.py" "$DATA/$source" "$RUN/bags" "$name" "$variant" "$seconds"
done
cat > "$RUN/plan.tsv" <<'EOF'
three	a	map
three	b	map
three	c	map
no_gnss	a	map
no_gnss	b_no_gnss	odom
gnss_first	a	map
gnss_first	b_gnss_first	map
fresh_a	a	map
fresh_b	b	map
fresh_c	c	map
EOF
"$PY" - "$DATA" "$RUN" <<'PY'
import json, sys
from pathlib import Path
data, run = map(Path, sys.argv[1:])
names = {'a':'30618_bc5e53c2','b':'30618_082f1d65','c':'30618_af7496f0',
         'b_no_gnss':'30618_082f1d65','b_gnss_first':'30618_082f1d65'}
(run/'sources.json').write_text(json.dumps({k:str(data/v) for k,v in names.items()}))
PY
printf 'commit %s\ncommand bash tools/submission/multibag_live.sh\nstarted %s\n' "$COMMIT" "$(date -Is)" > "$RUN/run.txt"
docker build -q -f "$TMP/repo/docker/Dockerfile" --target jury -t tram-odom:jury "$TMP/repo" >/dev/null
set +e
docker run --rm --network=none --cpus=2 --memory=512m --memory-swap=512m \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$TMP/repo:/repo:ro" -v "$SCEN:/runner:ro" -v "$RUN/bags:/bags:ro" \
  -v "$RUN:/out" tram-odom:jury bash /runner/multibag_live_inside.sh 2>&1 | tee "$RUN/container.log"
container=${PIPESTATUS[0]}
set -e
printf 'container_exit %s\n' "$container" >> "$RUN/run.txt"
set +e
"$PY" "$SCEN/multibag_report.py" "$RUN" 2>&1 | tee "$RUN/report.md"
report=${PIPESTATUS[0]}
set -e
printf 'report_exit %s\n' "$report" >> "$RUN/run.txt"
echo "Report: $RUN"
test "$container" -eq 0 && test "$report" -eq 0
