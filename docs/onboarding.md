# Как подключиться к работе

Два промпта для Claude Code: первый — один раз после клонирования (окружение и выбор
задачи), второй — в начале каждой задачи в своём worktree. Правила, на которые они
опираются, — `CLAUDE.md`, `GIT.md`, `docs/agents.md`.

## Перед первым промптом (руками, ~2 минуты)

1. Принять приглашение в `Chichetin/moscow_transport_hack` (владелец добавляет в
   Settings → Collaborators).
2. Установить `git`, `gh`, Docker; выполнить `gh auth login`.
3. `git clone https://github.com/Chichetin/moscow_transport_hack.git && cd moscow_transport_hack && claude`

## Промпт 1 — после клонирования

```
Ты — мой Claude Code в команде из пяти равноправных участников на хакатоне Московского
транспорта, кейс «Резервная одометрия по модели». Дедлайн — 27.09.2026 23:59 МСК.
Координатора нет: задачи — GitHub Issues, стыковка — контракты, спор решает метрика holdout.
Сейчас ты в основной копии репозитория. Задача этой сессии — подготовить моё рабочее
место и помочь мне выбрать первую issue. Код продукта в этой сессии не пишешь.

1. Прочитай целиком CLAUDE.md, GIT.md, docs/agents.md, docs/data.md (ловушки данных),
   docs/contracts.md и docs/plans/2026-09-25-plan.md. Перескажи мне правила в 8–10 строках
   своими словами: что нельзя (GNSS после окна 5 с, push в main без моего слова, force/rebase/
   add -A, правка контракта молча), единицы (тележки в км/ч), numpy 1.21 в ядре,
   как брать задачу и как PR попадает в main.

2. Проверь окружение и сделай недостающее. Каждый пункт — с командой и её выводом:
   - `gh auth status`; доступ на запись: `gh repo view --json viewerPermission`
     (нужен WRITE или ADMIN). Если не авторизован, попроси меня выполнить `! gh auth login`.
   - `git config user.name` / `user.email` заданы.
   - Docker работает (`docker info`), на диске свободно ≥ 5 ГБ (`df -h .`). Если меньше —
     скажи мне, сам ничего не удаляй.
   - `python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt`.
   - Данные. Если `dataset/data` ещё нет — скачай публичную папку организаторов:
       K='https://disk.yandex.ru/d/DdnscmWTBtzkOQ'
       HREF=$(curl -s "https://cloud-api.yandex.net/v1/disk/public/resources/download?public_key=$K&path=/dataset.zip" | python3 -c 'import json,sys; print(json.load(sys.stdin)["href"])')
       curl -L -o /tmp/dataset.zip "$HREF"   # ~256 МБ
     Распакуй так, чтобы получилось dataset/data.zip, dataset/README.md и
     dataset/tram_vehicle_msgs/. Структуру архива посмотри через `unzip -l`, не угадывай.
     Потом `unzip -q dataset/data.zip -d dataset/data`. Проверка: в `dataset/data` ровно
     122 каталога с metadata.yaml.
   - `cp .env.example .env` и пропиши в TRAM_DATA_DIR абсолютный путь к dataset/data.
   - `bash .claude/hooks/test-hooks.sh` — все ok.
   - `bash docker/dev.sh colcon build` — «3 packages finished».
   - `bash docker/dev.sh python3 -m pytest` — зелёный или «no tests ran» (код 5).
   - `bash docker/jury-stand.sh 30618_e9a34502` — сборка без сети проходит.
   Если что-то падает — две честные попытки починить. Не вышло — стоп и отчёт мне.
   Файлы репозитория в этой сессии не правишь и не коммитишь.

3. Помоги выбрать задачу:
   - `gh issue list --state open --limit 50 --json number,title,labels,assignees`
   - Свободная задача — без assignee, и её `area:` не занята другой открытой issue
     с assignee. Зависимости из строки «Зависит от #…» лучше закрытые; открытая
     зависимость допустима — тогда работа на заглушке (см. план).
   - Предложи мне 2–3 варианта: номер, что даёт в баллах по критериям, размер,
     чего ждёт. Приоритет — вехи checkpoint:0, потом checkpoint:1.
   - Когда я выберу: `gh issue edit <N> --add-assignee @me`. Проверь, что assignee
     встал и что его никто не взял секундой раньше (`gh issue view <N>`). Сразу после
     этого дай мне команду для новой сессии в worktree:
       claude -w <N>-<тема-латиницей-2-3-слова>

4. Итог — таблица «проверка | результат | команда». В конце одна строка: какую issue
   я взял и что вставить в новую сессию (промпт 2 из docs/onboarding.md).

Субагентов в этой сессии не запускай: здесь только настройка. В задачах они штатные
(docs/agents.md, D-016). Модель по умолчанию sonnet.
```

## Промпт 2 — в начале каждой задачи (новая сессия `claude -w <N>-<тема>`)

```
Работаем над issue #<N>. Сессия открыта в своём worktree. Проверь это:
`git status -sb` показывает ветку worktree-<N>-…, а не main.

1. `gh issue view <N>` — перескажи мне задачу, критерий приёмки, area и зависимости.
   Проверь, что assignee — я. Прочитай нужные части docs/contracts.md и код соседей,
   которых касаешься (только через контракт).
2. Если в worktree нет .venv — `ln -s <основная копия>/.venv .venv`. Если нет .env —
   скопируй из основной копии.
3. Предложи план в 5–8 шагах и скажи, кто что делает по конвейеру docs/agents.md:
   код сам (S) или субагент builder (M+). Дождись моего «ок».
4. Работа: тест → красный → код → зелёный. Ядро без rclpy и scipy, только numpy 1.21:
   проверь `bash docker/dev.sh python3 -m pytest`. Для кода с расчётом — 3–5 мутаций
   (знак, км/ч↔м/с, порог). Если код пишет builder — дай ему бриф: номер issue и
   абсолютный путь этого worktree; его отчёт перескажи мне.
5. Перед merge одним сообщением параллельно: субагент evaluator (таблица holdout
   «было → стало» против origin/main, как только tools/eval в main) и субагент
   reviewer (diff против origin/main). Главные метрики хуже больше чем на 2 % — не
   merge, а разговор со мной (D-012). Вердикт reviewer «доработать» — правим и зовём снова.
6. Контракт (docs/contracts.md, types.py, имена в params.yaml, формат метрик,
   splits.yaml) не меняешь. Если нужен — стоп: что, зачем, кто потребители.
7. Документы в том же PR: D-… в docs/decisions.md, строка в docs/tz-compliance.md.
8. `git fetch --prune && git merge origin/main`, проверки заново. Коммит только своих
   путей, `git push -u origin HEAD`, `gh pr create` по .github/pull_request_template.md
   с `Closes #<N>`. Коммит и push ветки — без моего подтверждения. Merge — после того,
   как выполнены условия из GIT.md («Финиш»).

Математику модели и фильтра (уравнения, якобианы, наблюдаемость) делаем на opus.
Если дойдём до неё, скажи мне переключиться командой /model opus, а потом вернуться
на sonnet. Субагенты — по конвейеру docs/agents.md, роли не выдумывай: только
builder, reviewer, evaluator, scribe, jury.
```
