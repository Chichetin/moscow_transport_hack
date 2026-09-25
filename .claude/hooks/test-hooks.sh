#!/usr/bin/env bash
# Проверка Stop-хуков на временном git-репозитории: bash .claude/hooks/test-hooks.sh
set -uo pipefail

HOOKS="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HOOKS/../.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
cd "$TMP" && git init -q . && mkdir -p .claude
FAIL=0

expect() { # expect <block|pass> <описание> <хук>
  local out
  out="$(bash "$HOOKS/$3" 2>&1)"
  if [ "$1" = block ] && ! printf '%s' "$out" | grep -q '"decision": *"block"'; then
    echo "FAIL: $2 — ожидалась блокировка, вывод: $out"; FAIL=1
  elif [ "$1" = pass ] && [ -n "$out" ]; then
    echo "FAIL: $2 — ожидался пустой вывод, вывод: $out"; FAIL=1
  else
    echo "ok:   $2"
  fi
}

# --- check-secrets.sh
expect pass  "чистый репозиторий"                   check-secrets.sh
echo 'TRAM_DATA_DIR=/path/to/data  # your path' > .env.example
expect pass  ".env.example с плейсхолдером"         check-secrets.sh
echo 'const token = "f9a8Xc72kLmQ01zYvB3n4dHs"' > bot.js
expect block "токен в коде"                         check-secrets.sh
echo 'const token = "f9a8Xc72kLmQ01zYvB3n4dHs" // secret-scan: allow' > bot.js
expect pass  "строка с явным secret-scan: allow"    check-secrets.sh
rm bot.js
echo 'client = OpenAI(api_key="sk-3f9a8c72b1d04e6fa9b7c2d5e8f10a4b")' > llm.py
expect block "ключ LLM в коде"                      check-secrets.sh
rm llm.py
echo 'TRAM_DATA_DIR=/data' > .env
expect block ".env не в .gitignore"                 check-secrets.sh
echo '.env' > .gitignore
expect pass  ".env в .gitignore"                    check-secrets.sh
printf -- '-----BEGIN RSA PRIVATE KEY-----\n' > key.pem
expect block "приватный ключ"                       check-secrets.sh
rm key.pem

# --- check-tests.sh (pytest из корневого pyproject.toml; код 5 «нет тестов» — молчать)
PY_OK=0
if [ -x "$REPO/.venv/bin/python" ] && "$REPO/.venv/bin/python" -c 'import pytest' 2>/dev/null; then
  ln -s "$REPO/.venv" .venv; PY_OK=1
elif python3 -c 'import pytest' 2>/dev/null; then PY_OK=1; fi
expect pass  "тестов нет — хук молчит"              check-tests.sh
if [ "$PY_OK" -eq 1 ]; then
  printf '[tool.pytest.ini_options]\ntestpaths = ["t"]\n' > pyproject.toml && mkdir -p t
  expect pass  "pytest без тестов (код 5) — молчит"    check-tests.sh
  printf 'def test_bad():\n    assert 1 == 2\n' > t/test_x.py
  expect block "pytest падает"                        check-tests.sh
  printf 'def test_ok():\n    assert 1 == 1\n' > t/test_x.py
  expect pass  "pytest зелёный"                       check-tests.sh
  printf 'def test_bad():\n    assert 1 == 2\n' > t/test_x.py
  touch -d '-1 min' t/test_x.py
  expect pass  "ничего не менялось с зелёного прогона" check-tests.sh
  rm -rf t pyproject.toml .claude/.last-green-test-run
else
  echo "skip: pytest недоступен (.venv или python3 -m pytest) — случаи pytest не проверены"
fi
[ -L .venv ] && rm .venv

# --- session-context.sh (SessionStart: stdout уходит в контекст)
expect_out() { # expect_out <has|lacks> <описание> <шаблон>
  local out
  out="$(bash "$HOOKS/session-context.sh" 2>&1)"
  if [ "$1" = has ] && ! printf '%s' "$out" | grep -q -- "$3"; then
    echo "FAIL: $2 — нет «$3», вывод: $out"; FAIL=1
  elif [ "$1" = lacks ] && printf '%s' "$out" | grep -q -- "$3"; then
    echo "FAIL: $2 — лишнее «$3», вывод: $out"; FAIL=1
  else
    echo "ok:   $2"
  fi
}
git checkout -q -b main 2>/dev/null
echo init > README.md
git add README.md && git -c user.name=t -c user.email=t@t commit -q -m init
mkdir -p "$TMP/bin"
cat > "$TMP/bin/gh" <<'GH'
#!/usr/bin/env bash
[ -n "${GH_FAIL:-}" ] && { echo "not logged in" >&2; exit 1; }
case "$1 $2" in
  "api user") echo me ;;
  "issue list") cat <<'JSON'
[{"number":3,"title":"Детектор проскальзывания","assignees":[{"login":"me"}],"labels":[{"name":"area:core-slip"}]},
 {"number":5,"title":"Модель привода","assignees":[{"login":"alice"}],"labels":[{"name":"area:core-dynamics"}]},
 {"number":7,"title":"Никем не взята","assignees":[],"labels":[{"name":"area:eval"}]}]
JSON
  ;;
esac
GH
chmod +x "$TMP/bin/gh"
export PATH="$TMP/bin:$PATH"
expect_out has   "основная копия на main — предупреждение" "основной копии"
expect_out has   "мои issues видны"                         "#3 Детектор проскальзывания"
expect_out has   "занятая другим area видна"                "area:core-dynamics: #5 alice"
expect_out lacks "свободная issue не считается занятой"     "area:eval"
GH_FAIL=1 expect_out has "gh без авторизации — предупреждение, без падения" "gh не авторизован"
git worktree add -q .claude/worktrees/wt-test -b worktree-wt-test 2>/dev/null
(cd .claude/worktrees/wt-test && expect_out lacks "worktree — без предупреждения" "основной копии")
(cd .claude/worktrees/wt-test && expect_out has   "worktree — видна ветка"        "worktree-wt-test")

# --- check-worktrees.sh (SubagentStop: секреты во всех worktree)
expect pass  "worktrees: секретов нигде нет"           check-worktrees.sh
echo 'const token = "f9a8Xc72kLmQ01zYvB3n4dHs"' > .claude/worktrees/wt-test/bot.js
expect block "worktrees: секрет в чужом worktree"       check-worktrees.sh
out="$(bash "$HOOKS/check-worktrees.sh" 2>&1)"
if printf '%s' "$out" | grep -q 'worktree .*wt-test'; then echo "ok:   worktrees: в причине назван worktree"
else echo "FAIL: worktrees: worktree не назван, вывод: $out"; FAIL=1; fi
rm .claude/worktrees/wt-test/bot.js
expect pass  "worktrees: после удаления секрета чисто"  check-worktrees.sh

# --- check-upstream.sh: чужие коммиты в origin/main против моей работы
cd "$TMP" && git init -q --bare -b main remote.git
git clone -q remote.git mate 2>/dev/null
mate_push() { # mate_push <файл> <сообщение>
  (cd "$TMP/mate" && echo "$2" >> "$1" && git add "$1" &&
   git -c user.name=mate -c user.email=m@m commit -q -m "$2" && git push -q origin HEAD:main)
}
mate_push README.md "init"
git clone -q remote.git me && cd me && git checkout -q -b worktree-me origin/main
expect_up() { # expect_up <block|pass> <описание> [шаблон]
  local out
  out="$(echo '{}' | bash "$HOOKS/check-upstream.sh" stop 2>&1)"
  if [ "$1" = block ] && ! { printf '%s' "$out" | grep -q '"decision": *"block"' &&
                            printf '%s' "$out" | grep -q -- "$3"; }; then
    echo "FAIL: $2 — ожидалась блокировка с «$3», вывод: $out"; FAIL=1
  elif [ "$1" = pass ] && [ -n "$out" ]; then
    echo "FAIL: $2 — ожидался пустой вывод, вывод: $out"; FAIL=1
  else
    echo "ok:   $2"
  fi
}
expect_up pass  "upstream: новых коммитов нет"
mate_push other.txt "eval: чужой файл"
expect_up pass  "upstream: чужой коммит не пересекается с моими файлами"
echo mine > shared.txt
mate_push shared.txt "core: тот же файл"
expect_up block "upstream: пересечение с незакоммиченным файлом" "shared.txt"
expect_up pass  "upstream: то же состояние origin/main второй раз не блокирует"
echo mine > new.txt && git add new.txt && git -c user.name=me -c user.email=me@me commit -q -m wip
mate_push new.txt "ros: тот же новый файл"
expect_up block "upstream: пересечение с закоммиченным в ветке файлом" "new.txt"
mate_push HANDOFF.md "docs: итог дня"
expect_up block "upstream: изменился HANDOFF — показать всегда" "HANDOFF.md"
mkdir -p "$TMP/mate/docs" && mate_push docs/contracts.md "contracts: новое поле"
expect_up block "upstream: изменился контракт — показать всегда" "docs/contracts.md"
mate_push shared.txt "core: ещё правка"
out="$(echo '{}' | bash "$HOOKS/check-upstream.sh" post 2>&1)"
if printf '%s' "$out" | jq -e '.hookSpecificOutput.additionalContext | contains("shared.txt")' >/dev/null 2>&1; then
  echo "ok:   upstream post: факты в additionalContext без блокировки"
else echo "FAIL: upstream post: нет additionalContext, вывод: $out"; FAIL=1; fi
mate_push shared.txt "core: третья правка"
out="$(echo '{}' | bash "$HOOKS/check-upstream.sh" post 2>&1)"
if [ -z "$out" ]; then echo "ok:   upstream post: не чаще раза в 5 минут"
else echo "FAIL: upstream post: сработал раньше 5 минут, вывод: $out"; FAIL=1; fi
git remote set-url origin "$TMP/nope.git"
expect_up pass  "upstream: сеть недоступна — молчит"

exit $FAIL
