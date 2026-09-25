# GIT.md — правила ведения Git

Цель: `main` в любой момент — рабочий кандидат на сдачу (`colcon build` без сети проходит,
тесты зелёные, `tools/eval` считается), а сданную версию можно однозначно восстановить.

Правило: **закончил одно логическое действие — проверь, закоммить и запушь ветку.**

## Сервер

`origin` — `https://github.com/Chichetin/moscow_transport_hack.git`, приватный. Защиты
ветки на бесплатном тарифе нет, поэтому запрет прямого push в `main` мягкий (D-014): в
`.claude/settings.json` он в `ask`, агент спрашивает человека. Без явного слова человека
в `main` попадает только PR. Force-push, rebase, amend, `reset --hard`, `clean`, `add -A`
закрыты в `permissions.deny`.

## Автономность агента

Агент коммитит и пушит **свою ветку сам, без запроса подтверждения**, как только шаг
закончен и проверки зелёные, и создаёт PR в `main`. Разрешение опирается на проверки:

- перед коммитом и push — `git fetch --prune` и просмотр обеих сторон расхождения;
- в коммит — только текущий шаг: `git add <явные пути>`. Никогда `git add -A`, `git add .`,
  `git commit -a`: в дереве может лежать чужая работа;
- `.env`, `dataset/`, `build/`, `install/`, `log/`, `out/`, `.venv` не коммитятся
  (`.gitignore`, Stop-хук `check-secrets.sh`);
- трейлеры соавторства ИИ (`Co-Authored-By`, `Claude-Session`, «Generated with Claude Code»)
  в коммиты и PR не добавляем (`attribution` в `.claude/settings.json` пустая).

## Worktree на задачу

Основная копия — не рабочее место: там могут сидеть человек и другие сессии, и
незаконченная правка одного ломает тесты остальным. **Любая задача, меняющая файлы, — в
своём worktree на своей ветке, одна issue — один worktree.**

```bash
gh issue edit <N> --add-assignee @me          # сначала взять issue
claude -w <N>-<тема>                          # .claude/worktrees/<N>-<тема>, ветка worktree-<N>-<тема>

# уже запущенная сессия или другой агент — то же руками:
git fetch --prune
git worktree add .claude/worktrees/<N>-<тема> -b worktree-<N>-<тема> origin/main
```

`<тема>` — 2–3 слова латиницей: `12-slip-detector`. `claude -w` копирует `.env` из основной
копии (`.worktreeinclude`); при `git worktree add` — скопировать самому. `.venv` в worktree
нет: `python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt` или
`ln -s <основная копия>/.venv .venv`. Данные не копировать: скрипты находят
`<основная копия>/dataset/data` сами.

## Финиш: ветка → PR → main

```bash
# 1. проверки зелёные, коммит только своих путей
git fetch --prune && git merge origin/main   # если main ушёл вперёд — после merge всё заново:
.venv/bin/python -m pytest && bash docker/dev.sh colcon build
# 2. push и PR
git push -u origin HEAD
gh pr create --base main --title "<area>: <что сделано>" --body-file <тело по шаблону>
```

Тело PR — по `.github/pull_request_template.md`: `Closes #<N>`, что сделано, таблица
holdout «было → стало» с командой, затронутый контракт и его потребители. Повторный push
обновляет открытый PR.

**Merge** — `gh pr merge <N> --merge --delete-branch` (обычный merge-коммит, без squash и
rebase, иначе правило уборки «нет коммитов сверх `origin/main`» перестаёт работать).
Мержит автор, когда:

- проверки зелёные после `git merge origin/main`;
- таблица holdout не хуже `origin/main` (D-012) или в PR есть `D-…` с обоснованием;
- PR с кодом, тестами, `params.yaml` или `.claude/` — есть вердикт `reviewer` `merge` на
  последнем commit ветки (D-016); PR только с документами — по желанию;
- PR трогает контракт — ещё и апрув всех потребителей (`gh pr review --approve` от владельцев
  соседних issue).

После merge в основной копии, когда там никто не работает: `git pull --ff-only`.

## Коммиты

Формат: `<area>: <что закончено>`, по-русски. Префикс — `area` из
`docs/project-structure.md` без `area:` (`core-slip: детектор по расхождению тележек`);
`fix:` — если смешанный; `docs:`, `infra:`. Один коммит — один осмысленный шаг.

## Уборка

Ветку или worktree удаляет агент сам, только если **оба** условия выполнены:

1. ветка влита: `git fetch --prune && git rev-list --count origin/main..<ветка>` → `0`;
2. ветка не открыта ни в одном worktree, кроме твоего, и в твоём нет незакоммиченного:
   `git worktree list`, `git -C <путь> status --porcelain` → пусто.

```bash
git worktree remove .claude/worktrees/<N>-<тема>   # без --force, из основной копии
git branch -D worktree-<N>-<тема>                  # после проверки п. 1
git push origin --delete worktree-<N>-<тема>       # если gh не удалил при merge
git worktree prune
```

Чужой worktree или ветку с невлитыми коммитами не трогать никогда, даже если выглядит
заброшенной: спросить человека.

Без worktree, прямо в основной копии, можно только читать, запускать тесты, `jury`, стенд и
`git pull --ff-only`.

## Перед началом работы

```bash
git fetch --prune && git status -sb
git log --oneline HEAD..origin/main   # что пришло от команды
gh issue list --assignee @me          # мои задачи
```

Прежде чем назвать что-то дефектом, убедиться, что ветка не `behind`.

## Сдача и тег

Тег — **после** проверки по скиллу `submission-checklist`, на commit из `origin/main`:

```bash
git fetch --prune
git tag -a submit-final origin/main -m "Сдача: <commit>, стенд пройден <дата>"
git push origin submit-final
```

## Безопасный откат

| Ситуация | Действие |
|---|---|
| Посмотреть историю | `git log --oneline --decorate --graph` |
| Вернуть свой файл | `git restore <path>` после просмотра diff |
| Отменить влитый коммит | `git revert <hash>` в новой ветке → PR |
| Посмотреть старое состояние | `git switch --detach <tag-or-hash>` |
