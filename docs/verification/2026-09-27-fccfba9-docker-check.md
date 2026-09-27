# Docker-проверка опубликованного fccfba9 — 27.09.2026

Проверен точный commit `fccfba998f3bbfb3b44e8723ea54bafb458371bd` в новом detached worktree `.claude/worktrees/docker-verify-fccfba9`. Полный Docker pytest, чистая офлайн-сборка и тесты полного архива прошли. Короткий train smoke стенда дал 6/6 по опубликованному анализатору. Готовность к сдаче этим отчётом не подтверждается: #200 и #201 остаются OPEN.

Оркестратор опубликовал этот отчёт после завершения независимого evaluator. [Исходный `stand.json` этого запуска](2026-09-27-fccfba9-docker-stand.json) приложен без изменения чисел. Полные логи и служебные скрипты в `out/` остаются локальными; перечисленные далее абсолютные пути описывают среду измерения, а не требование к расположению проекта у жюри.

## Происхождение и среда

| Объект | Значение |
|---|---|
| HEAD | `fccfba998f3bbfb3b44e8723ea54bafb458371bd` |
| `src/`, git tree | `91332ae819ef7ad959feaa3c96ade08f6f3f7d3a` |
| `docker/`, git tree | `b7ca7c8c9cc235a185ea109bf597c930751a1f87` |
| `tools/stand/`, git tree | `34df2125db99f5b9d6aec145274c627bda62bbc6` |
| `params.yaml`, git blob | `46c96df82ca5acfd54c869df344eabee84fa4c04` |
| `maps/route.csv`, git blob | `6bb506093f495c612c1f4bed1145751383985a8e` |
| `splits.yaml`, git blob | `2ca9d8baed7cbfa319d0684349d5fd229047061a` |
| Хост | Arch Linux x86_64, kernel `7.0.10-zen1-1-zen`, Intel Core i7-13700KF, `nproc=24`, RAM 32 676 340 KiB |
| Docker | client 29.8.1, server 29.8.0 |
| Контейнер dev | Ubuntu 22.04.5, ROS Humble, Python 3.10.12, NumPy 1.21.5, SciPy 1.8.0, pytest 6.2.5, rosbags 0.11.5 |
| Анализатор на хосте | `.venv` основной копии: Python 3.14.7, NumPy 2.5.3, rosbags 0.11.5 |

Исходные image ID перед проверкой: dev `sha256:0145e026fabcd66cb10a76a4835c2b9cea7bf6e88599435a161345b1a4af655c`, jury `sha256:82c176a0f23cce384dc45ad3fb6eea868855e5946245847046101c8130096880`. Оба опубликованных Docker build завершились успешно из кеша; ошибки APT/сети не возникали. После повторного экспорта кешированных образов фактически использованы:

- dev: `sha256:28af3e7e7d23cded1bee8567e7528975d329914605ae2b1dae26858b673d0fcb`;
- jury: `sha256:815c09d194c608cd75a86cc6d2000b0ce27a19e01ea3973daa86d15ccc67fa00`.

Это проверка сборки ROS-пакетов без сети на подготовленном образе; создание образа с пустым кешем и установкой всех apt-зависимостей здесь не проверялось.

У исторического `8626ddef4f321245dcb5e748545cc6bbb07133e2` тот же tree `src/`. Новый 31-минутный прогон не выполнялся; исторические измерения не объявляются измерениями этого запуска. Локальные кандидаты `5b2a583` / `1e36aa0` не использовались.

## Сборка и тесты

| Этап | Exit | Результат |
|---|---:|---|
| Docker build dev | 0 | Все слои из кеша |
| Полный Docker pytest, сеть отключена, dataset отсутствует | 0 | **809 passed, 9 skipped**, 14,82 с |
| Clean offline colcon build, первый workspace только `src/` | 0 | 3 пакета собраны |
| Colcon test, первый workspace только `src/` | 2 | ROS: 32 passed; core: ошибка сбора тестов, `ModuleNotFoundError: build_route` |
| Clean offline colcon build полного `git archive fccfba9` | 0 | 3 пакета собраны |
| Colcon test полного архива | 0 | 569 core + 32 ROS |
| Colcon test-result --verbose полного архива | 0 | **601 tests, 0 errors, 0 failures, 0 skipped** |

В первом workspace отсутствовал `tools/pathgraph/build_route.py`, который импортирует опубликованный `src/tram_odometry_core/test/test_position.py`. Сборка пакетов при этом прошла. Последующий workspace создавался заново в `/tmp` из `git archive` точного commit, включая его `tools/`; ни исходники, ни тесты не исправлялись. В первой попытке `test-result` не был вызван из-за `&&` после упавшего test; в полной проверке его результат записан отдельно.

Оба workspace были новыми, без `build/`, `install/`, `log/`, внутри новых контейнеров с `--network=none`. Для полного pytest заданы `TRAM_DATA_DIR=/tmp/nonexistent-fcc-docker-data`, `-p no:cacheprovider`, `PYTHONDONTWRITEBYTECODE=1`; raw dataset в контейнер не монтировался.

## Короткий стенд

TRAIN bag `30618_af7496f0` выбран из неизменённого `tools/eval/splits.yaml`, проверено отсутствие в holdout/dups. Metadata: **262,948242572 с**, 20 921 сообщение. Rate **1.0**. Оригинальный `docker/jury-stand.sh` запущен без изменений; свежая сборка внутри jury-контейнера: 3 пакета, 4,88 с.

Запуск: **2026-09-27 21:59:21—22:03:57 МСК** (18:59:21—19:03:57 UTC), вся команда 276,328 с. Перед стартом работающих контейнеров не было; других тяжёлых проверок во время воспроизведения не запускали. Docker inspect подтверждает `NanoCpus=2000000000`, `Memory=536870912`, `MemorySwap=536870912`, `NetworkMode=none`.

| Проверка опубликованного анализатора | Значение | Порог | Результат |
|---|---:|---:|---|
| Задержка p99 | 3.125244 мс | ≤100 мс | PASS |
| Задержка max | 29.290709 мс | ≤250 мс | PASS |
| Меньшая средняя частота | 39.769117 Гц | ≥10 Гц | PASS |
| CPU max ноды | 0.109890 ядра | ≤2 | PASS |
| RSS peak ноды | 64.429688 МиБ | ≤512 | PASS |
| Рост RSS после первой трети | -0.00057966 МиБ/мин | ≤1 | PASS |

Итого **6/6**, без `false` и `null`. p50: 0.593302 мс; CPU mean: 0.054528 ядра; отсчётов ресурсов: 265. Ресурсы относятся к процессу ноды (sampler по `odometry_node`), Docker-ограничение применяется ко всему контейнеру. Задержка измерена опубликованным методом по времени получения recorder с сопоставлением `header.stamp`.

| Выход | Сообщений | Средняя частота, Гц | Минимальное полное окно 1 с, Гц | Unmatched |
|---|---:|---:|---:|---:|
| `/result/velocity` | 10456 | 39.782919 | 9 | 0 |
| `/result/position` | 10452 | 39.769117 | 8 | 0 |

Оба выхода содержат измеренную частоту; случай, когда анализатор скрывает `mean_hz=null`, к этому прогону не относится. Однако минимальные секундные окна — **9 и 8 Гц**: опубликованный verdict использует среднюю частоту. Требование ≥10 Гц в каждом секундном окне этим запуском не подтверждено.

В записи полностью присутствуют входы train bag: передняя тележка **2570**, задняя **2580**, контроллер **5306** сообщений — ровно столько же, сколько в исходном bag. Выходы охватывают около 262,8 с; записано также 2056 сообщений диагностики. Четыре непубликации позиции на старте совместимы с контрактом ожидания первого fix; оценки качества положения в этом smoke не вычислялись.

`docker top` во время воспроизведения фиксирует запущенные `odometry_node` (container PID 687), `ros2 bag play`, recorder и sampler. `play.log` подтверждает открытие выбранного bag READ_ONLY и rate 1, `node.log` — запуск ноды; в `node.log`/`play.log`/`record.log` нет ERROR/Traceback/WARN. Recorder записал оставшийся кеш и сообщил `Recording stopped`. Docker events: контейнер `25dbe16d30dd3317f059ad530e4444f3c0aa6392cd273dd693e7473dc4382efd` завершился `exitCode=0`, события OOM отсутствуют.

Оболочка `jury-stand.sh` вернула **0**. После неё отдельно вызван именно опубликованный `tools/stand/run_stand.py`, реальный exit **0**, `stand.json` повторно проверен на шесть явных `true`. Дополнительная проверка записи, обеих частот и полного числа входов вернула **0**.

Исходный wrapper маскирует ошибку анализатора через `|| echo`, анализатор сам всегда возвращает 0 после таблицы, а `stand-inside.sh` не сохраняет отдельные exit player и ноды. Поэтому их индивидуальные exit здесь неизвестны; код 0 внешней оболочки не использован как единственное доказательство. Нормальное воспроизведение подтверждается длительностью, полными входными счётчиками, выходами, снимками процессов и логами. Отдельного доказательства корректного shutdown ноды этот wrapper не даёт.

## Ограничения и открытые задачи

- Это один train smoke примерно на 4,4 минуты. Новый long leak test и повторная holdout/stress-оценка не выполнялись; тестовые результаты не заменяют метрики качества модели.
- Raw holdout, код внешнего судьи/check-code и запрещённые sealed-каталоги не читались.
- [#200](https://github.com/Chichetin/moscow_transport_hack/issues/200) — **OPEN**: одна нода на несколько bag подряд; [#201](https://github.com/Chichetin/moscow_transport_hack/issues/201) — **OPEN**: поздний старт ноды после окна GNSS. Статусы проверены `gh issue view` в 21:58:39—40 МСК. Один bag при старте ноды до player эти сценарии не проверяет.
- `src/`, tests, params, Docker scripts, tools и CI не изменялись. Git diff по этим путям пуст, итоговые SHA256 скриптов и splits совпали с начальными. `.venv` — сохранённая ссылка на основную копию. Commit/push/комментарии GitHub из этой проверки не выполнялись.

## Артефакты и команды

Артефакты находятся в этом worktree под `out/docker-verification-fccfba9/`. Для каждого этапа: `<stage>.command.txt` — точная команда, `<stage>.stdout.log` и `.stderr.log` — раздельные потоки, `<stage>.json` — cwd, argv, timestamps UTC, elapsed и exit. `07-colcon-logs.tar` содержит логи colcon и junit XML, `fccfba9-committed.tar` — проверенный git archive. `stand/30618_af7496f0/` содержит `build.log`, `node.log`, `play.log`, `record.log`, `resources.csv`, `record/`, `stand.json`. Дополнительно сохранены `record-verification.json`, `stand-container-settings.json`, Docker events, metadata выбранного train bag и SHA256-манифест.

| Этап | Начало UTC | Длительность, с | Exit |
|---|---|---:|---:|
| `02-image-dev-build` | 2026-09-27T18:56:14.071956+00:00 | 0.191 | 0 |
| `03-docker-pytest` | 2026-09-27T18:56:34.918369+00:00 | 15.249 | 0 |
| `05a-colcon-build` | 2026-09-27T18:57:12.208206+00:00 | 2.870 | 0 |
| `05b-colcon-test` | 2026-09-27T18:57:15.184090+00:00 | 3.662 | 2 |
| `07a-colcon-build` | 2026-09-27T18:58:22.438115+00:00 | 3.012 | 0 |
| `07b-colcon-test` | 2026-09-27T18:58:25.542903+00:00 | 7.670 | 0 |
| `07c-colcon-test-result` | 2026-09-27T18:58:33.230483+00:00 | 0.094 | 0 |
| `10-jury-stand` | 2026-09-27T18:59:21.023502+00:00 | 276.328 | 0 |
| `16-explicit-analyzer` | 2026-09-27T19:04:10.735442+00:00 | 0.352 | 0 |
| `17-record-verification` | 2026-09-27T19:04:11.121646+00:00 | 0.049 | 0 |

Worktree создан командой из основной копии:

```bash
git worktree add --detach .claude/worktrees/docker-verify-fccfba9 fccfba998f3bbfb3b44e8723ea54bafb458371bd
```

Основные команды ниже выполнялись из корня detached worktree. Полные команды служебных снимков также сохранены в `.command.txt`.

`02-image-dev-build`:

```bash
timeout 120 docker build --progress=plain -f docker/Dockerfile --target dev -t tram-odom:dev .
```

`03-docker-pytest`:

```bash
docker run --rm --name fccfba9-pytest --network=none --user 1000:1000 -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -e TRAM_DATA_DIR=/tmp/nonexistent-fcc-docker-data -v /home/m1hairu/Documents/msk_transport/repo/.claude/worktrees/docker-verify-fccfba9:/ws:ro -w /ws tram-odom:dev bash -c 'source /opt/ros/humble/setup.bash && python3 -m pytest -p no:cacheprovider --basetemp=/tmp/fccfba9-pytest'
```

`05-colcon-container`:

```bash
docker run --rm --name fccfba9-colcon --network=none --user 1000:1000 -e HOME=/tmp -e TRAM_DATA_DIR=/tmp/nonexistent-fcc-docker-data -e PYTHONDONTWRITEBYTECODE=1 -v /home/m1hairu/Documents/msk_transport/repo/.claude/worktrees/docker-verify-fccfba9/src:/src:ro -v /home/m1hairu/Documents/msk_transport/repo/.claude/worktrees/docker-verify-fccfba9/out/docker-verification-fccfba9:/evidence tram-odom:dev bash -c 'source /opt/ros/humble/setup.bash && TRAM_VERIFY_WS=$(mktemp -d /tmp/fccfba9-colcon.XXXXXX) && cd "$TRAM_VERIFY_WS" && cp -a /src src && test ! -e build && test ! -e install && test ! -e log && python3 /evidence/run_stage.py 05a-colcon-build colcon build --event-handlers console_cohesion+ && source install/setup.bash && python3 /evidence/run_stage.py 05b-colcon-test colcon test --event-handlers console_direct+ && python3 /evidence/run_stage.py 05c-colcon-test-result colcon test-result --verbose'
```

`06-published-archive`:

```bash
git archive --format=tar --output=out/docker-verification-fccfba9/fccfba9-committed.tar fccfba998f3bbfb3b44e8723ea54bafb458371bd
```

`07-colcon-archive-container`:

```bash
docker run --rm --name fccfba9-colcon-archive --network=none --user 1000:1000 -e HOME=/tmp -e TRAM_DATA_DIR=/tmp/nonexistent-fcc-docker-data -e PYTHONDONTWRITEBYTECODE=1 -v /home/m1hairu/Documents/msk_transport/repo/.claude/worktrees/docker-verify-fccfba9/out/docker-verification-fccfba9:/evidence tram-odom:dev bash -c 'source /opt/ros/humble/setup.bash && TRAM_VERIFY_WS=$(mktemp -d /tmp/fccfba9-archive.XXXXXX) && cd "$TRAM_VERIFY_WS" && tar -xf /evidence/fccfba9-committed.tar && test ! -e build && test ! -e install && test ! -e log || exit 99; python3 /evidence/run_stage.py 07a-colcon-build colcon build --base-paths src --event-handlers console_cohesion+; BUILD_STATUS=$?; test "$BUILD_STATUS" -eq 0 || exit "$BUILD_STATUS"; source install/setup.bash; python3 /evidence/run_stage.py 07b-colcon-test colcon test --base-paths src --event-handlers console_direct+; TEST_STATUS=$?; python3 /evidence/run_stage.py 07c-colcon-test-result colcon test-result --verbose; RESULT_STATUS=$?; tar -cf /evidence/07-colcon-logs.tar log build/tram_odometry/pytest.xml build/tram_odometry_core/pytest.xml; test "$TEST_STATUS" -eq 0 && test "$RESULT_STATUS" -eq 0'
```

`10-jury-stand`:

```bash
env TRAM_DATA_DIR=/home/m1hairu/Documents/msk_transport/repo/dataset/data TRAM_OUT_DIR=/home/m1hairu/Documents/msk_transport/repo/.claude/worktrees/docker-verify-fccfba9/out/docker-verification-fccfba9 PYTHONDONTWRITEBYTECODE=1 bash -x docker/jury-stand.sh 30618_af7496f0 1.0
```

`16-explicit-analyzer`:

```bash
env PYTHONDONTWRITEBYTECODE=1 .venv/bin/python tools/stand/run_stand.py out/docker-verification-fccfba9/stand/30618_af7496f0
```

`17-record-verification`:

```bash
.venv/bin/python out/docker-verification-fccfba9/verify_record.py
```
