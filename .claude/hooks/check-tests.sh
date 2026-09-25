#!/usr/bin/env bash
# Stop hook: ход не считается завершённым, если тесты проекта красные.
# Сам находит тесты:
#   pytest     — если в корне pyproject.toml с [tool.pytest.ini_options]; интерпретатор —
#                .venv/bin/python, иначе python3 (в dev-контейнере). Код выхода 5
#                («тестов не найдено») — не ошибка: пока тестов нет, хук молчит.
#   colcon test — только если colcon доступен (dev-контейнер с ROS) и в src/*/test есть
#                test_*.py. На хосте без ROS молча пропускается.
# Прогон пропускается, если ничего не менялось с последнего зелёного.
set -uo pipefail

MARKER=".claude/.last-green-test-run"

if [ -f "$MARKER" ]; then
  CHANGED="$(find . -type f -newer "$MARKER" \
    -not -path './.git/*' -not -path './.claude/*' -not -path './dataset/*' -not -path './out/*' \
    -not -path './build/*' -not -path './install/*' -not -path './log/*' \
    -not -path '*/.venv/*' -not -path '*/__pycache__/*' -not -path '*/.pytest_cache/*' 2>/dev/null | head -1)"
  [ -z "$CHANGED" ] && exit 0
fi

RAN=0
OUTPUT=""
STATUS=0

if [ -f pyproject.toml ] && grep -q '^\[tool\.pytest\.ini_options\]' pyproject.toml; then
  PY=python3
  [ -x .venv/bin/python ] && PY=.venv/bin/python
  OUT="$($PY -m pytest -q -p no:cacheprovider 2>&1)"
  code=$?
  if [ "$code" -ne 5 ]; then
    RAN=1
    [ "$code" -ne 0 ] && STATUS=1
    OUTPUT+="== pytest"$'\n'"$OUT"$'\n'
  fi
fi

if command -v colcon >/dev/null 2>&1 && ls src/*/test/test_*.py >/dev/null 2>&1; then
  RAN=1
  OUT="$(colcon build --symlink-install >/dev/null 2>&1 && colcon test --return-code-on-test-failure 2>&1 | tail -20)" || STATUS=1
  OUTPUT+="== colcon test"$'\n'"$OUT"$'\n'
fi

[ "$RAN" -eq 0 ] && exit 0

if [ "$STATUS" -eq 0 ]; then
  touch "$MARKER"
  exit 0
fi

TAIL="$(printf '%s' "$OUTPUT" | tail -n 40)"
jq -n --arg out "$TAIL" '{decision: "block", reason: ("Тесты красные перед завершением хода:\n" + $out)}'
exit 0
