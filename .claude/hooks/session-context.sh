#!/usr/bin/env bash
# SessionStart: stdout попадает в контекст сессии. Показывает, где агент стоит в Git,
# предупреждает о работе в основной копии на main, выводит мои открытые issues и занятые
# области (area:* у issues, взятых другими). Реестр занятости — GitHub Issues, не HANDOFF.md.
# Ничего не блокирует. gh недоступен, не авторизован или нет сети — одна строка предупреждения.
set -uo pipefail

git rev-parse --is-inside-work-tree >/dev/null 2>&1 || exit 0

branch="$(git branch --show-current 2>/dev/null)"
dirty="$(git status --porcelain 2>/dev/null | wc -l | tr -d ' ')"
echo "[git] ветка: ${branch:-detached}; незакоммиченных путей: $dirty; $(git rev-parse --show-toplevel)"

# В связанном worktree git-dir и git-common-dir различаются.
if [ "$(git rev-parse --git-dir)" = "$(git rev-parse --git-common-dir)" ] && [ "$branch" = main ]; then
  echo "[git] ВНИМАНИЕ: ты в основной копии на main. Задачу, меняющую файлы, делай в своём worktree:"
  echo "      claude -w <issue>-<тема>  или  git worktree add .claude/worktrees/<issue>-<тема> -b worktree-<issue>-<тема> origin/main (GIT.md)"
  [ "$dirty" != 0 ] && echo "      Незакоммиченные изменения здесь могут быть чужими — не коммить и не трогай их."
fi

if ! command -v gh >/dev/null 2>&1; then
  echo "[issues] gh не установлен — занятость областей не видна. Поставить gh и выполнить gh auth login."
  exit 0
fi

me="$(timeout 8 gh api user -q .login 2>/dev/null)"
issues="$(timeout 8 gh issue list --state open --limit 200 \
  --json number,title,assignees,labels 2>/dev/null)"
if [ -z "$me" ] || [ -z "$issues" ]; then
  echo "[issues] gh не авторизован или нет сети — занятость областей не видна (gh auth status)."
  exit 0
fi

blockers="$(printf '%s' "$issues" | jq -r \
  '.[] | select(any(.labels[]; .name == "blocker")) | "  #\(.number) \(.title)"')"
if [ -n "$blockers" ]; then
  echo "[blocker] открыты — сдача красная, пока открыт хоть один (D-024). Можешь закрыть — закрой раньше своей задачи:"
  printf '%s\n' "$blockers"
fi

mine="$(printf '%s' "$issues" | jq -r --arg me "$me" \
  '.[] | select(any(.assignees[]; .login == $me))
   | "  #\(.number) \(.title) [\([.labels[].name | select(startswith("area:"))] | join(", "))]"')"
busy="$(printf '%s' "$issues" | jq -r --arg me "$me" \
  '[.[] | select((.assignees | length) > 0 and (any(.assignees[]; .login == $me) | not))
    | {a: ([.labels[].name | select(startswith("area:"))] | join(", ")),
       w: "#\(.number) \([.assignees[].login] | join(","))"}]
   | group_by(.a)[] | "  \(.[0].a // "без area"): \([.[].w] | join("; "))"')"

if [ -n "$mine" ]; then
  echo "[issues] мои открытые ($me):"
  printf '%s\n' "$mine"
else
  echo "[issues] за $me открытых issues нет. Взять: gh issue list --search 'no:assignee' → gh issue edit <N> --add-assignee @me"
fi
if [ -n "$busy" ]; then
  echo "[issues] заняты другими (issue с той же area: не брать — CLAUDE.md, «Команда»):"
  printf '%s\n' "$busy"
fi
exit 0
