#!/usr/bin/env bash
# Runner сценариев судьи (#201, #200): проверяет НАШ порядок запуска в контейнере жюри из
# закоммиченного HEAD (git archive) — свежая нода → входные подписки готовы → единственный
# bag play → проверка /result/* → остановка только своих процессов. Порядок у реального стенда
# жюри этим не доказывается: runner подтверждает работу при указанном порядке.
#   bash tools/submission/judge_runner.sh [bag_A bag_B]   # train, по умолчанию 0e41eac3, 68847170
# Сценарии (один контейнер, подряд): A и B со свежей нодой; A с GNSS до первой тележки;
# B без GNSS после bag с GNSS (только odom, без старого якоря); A с GNSS 5 с и нодой через
# LATE_S после bag play — ожидаемый отрицательный результат: вся позиция в odom.
# Отчёт: $TRAM_OUT_DIR/submission/judge-runner-<commit>/ (логи, exit codes, report.json).
set -euo pipefail
A="${1:-30618_0e41eac3}"; B="${2:-30618_68847170}"
CUT_S="${CUT_S:-90}"       # с записи на сценарий
LATE_S="${LATE_S:-8}"      # с между bag play и запуском ноды в отрицательном сценарии
ROOT="$(git rev-parse --show-toplevel)"
MAIN="$(cd "$(git rev-parse --path-format=absolute --git-common-dir)/.." && pwd)"
[ -f "$ROOT/.env" ] && set -a && . "$ROOT/.env" && set +a
DATA="${TRAM_DATA_DIR:-$MAIN/dataset/data}"; case "$DATA" in /*) ;; *) DATA="$ROOT/$DATA" ;; esac
OUT="${TRAM_OUT_DIR:-$ROOT/out}"; case "$OUT" in /*) ;; *) OUT="$ROOT/$OUT" ;; esac
PY="$ROOT/.venv/bin/python"; [ -x "$PY" ] || PY="$MAIN/.venv/bin/python"
for b in "$A" "$B"; do [ -d "$DATA/$b" ] || { echo "Нет bag: $DATA/$b" >&2; exit 1; }; done

COMMIT="$(git -C "$ROOT" rev-parse HEAD)"
DIRTY="$(git -C "$ROOT" status --porcelain --untracked-files=no | wc -l | tr -d ' ')"
[ "$DIRTY" = 0 ] || echo "ВНИМАНИЕ: $DIRTY незакоммиченных изменений не проверяются — проверяется $COMMIT"
RUN="$OUT/submission/judge-runner-$COMMIT"; rm -rf "$RUN"; mkdir -p "$RUN/bags"

TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/repo" && git -C "$ROOT" archive HEAD | tar -x -C "$TMP/repo"
SCEN="$TMP/repo/tools/submission"
for v in crop gnss_first short_gnss; do "$PY" "$SCEN/make_scenario_bags.py" "$DATA/$A" "$RUN/bags" "a_$v" "$v" "$CUT_S"; done
for v in crop no_gnss; do "$PY" "$SCEN/make_scenario_bags.py" "$DATA/$B" "$RUN/bags" "b_$v" "$v" "$CUT_S"; done
printf '{"a_crop": "%s", "a_gnss_first": "%s", "a_short_gnss": "%s", "b_crop": "%s", "b_no_gnss": "%s"}\n' \
  "$DATA/$A" "$DATA/$A" "$DATA/$A" "$DATA/$B" "$DATA/$B" > "$RUN/sources.json"
# name bag mode delay expect
printf '%s\n' \
  $'seq1_a\ta_crop\tprotocol\t0\tmap' \
  $'seq2_b\tb_crop\tprotocol\t0\tmap' \
  $'gnss_first\ta_gnss_first\tprotocol\t0\tmap' \
  $'no_gnss_after_gnss\tb_no_gnss\tprotocol\t0\todom' \
  "late_short_gnss"$'\ta_short_gnss\tlate\t'"$LATE_S"$'\todom' > "$RUN/plan.tsv"
{ echo "commit $COMMIT dirty $DIRTY"; echo "bags A=$A B=$B cut ${CUT_S}s late ${LATE_S}s"
  echo "cmd: bash tools/submission/judge_runner.sh $*"; date -Is; } > "$RUN/run.txt"

docker build -q -f "$TMP/repo/docker/Dockerfile" --target jury -t tram-odom:jury "$TMP/repo" >/dev/null
set +e
docker run --rm --network=none --cpus=2 --memory=512m --memory-swap=512m \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$TMP/repo:/repo:ro" -v "$SCEN:/runner:ro" -v "$RUN/bags:/bags:ro" \
  -v "$RUN/plan.tsv:/plan.tsv:ro" -v "$RUN:/out" \
  tram-odom:jury bash /runner/judge_runner_inside.sh 2>&1 | tee "$RUN/container.log"
CONTAINER=${PIPESTATUS[0]}
set -e
echo "container_exit $CONTAINER" >> "$RUN/run.txt"
[ "$CONTAINER" = 0 ] || { echo "КОНТЕЙНЕР ВЫШЕЛ С КОДОМ $CONTAINER, лог: $RUN/container.log" >&2; exit 1; }
"$PY" "$SCEN/judge_runner_report.py" "$RUN" "$RUN/plan.tsv" "$RUN/sources.json" | tee "$RUN/report.md"
REPORT=${PIPESTATUS[0]}
echo "report_exit $REPORT" >> "$RUN/run.txt"; echo "Отчёт: $RUN"
exit "$REPORT"
