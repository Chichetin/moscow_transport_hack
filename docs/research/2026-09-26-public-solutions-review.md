# Публичные решения резервной одометрии: сверка кода и доказательств

Дата: 26 сентября 2026. Исходный список и предварительную оценку предоставил участник
команды; ключевые утверждения независимо проверены чтением опубликованных исходников.
Это статическая часть исследования. Затем выполнены реальные прогоны:
[результаты, ограничения и задачи](../verification/2026-09-26-public-solutions-benchmark.md).

## Метод и границы вывода

Проверены default branches пяти репозиториев, README, ROS-обёртки, доступные ядра,
отчёты и файлы тестов. Страницы всех пяти ответили HTTP 200 на запросы без токена,
Authorization и cookies. GitHub API также показывает `private=false`.
На этапе этой статической сверки чужие пакеты, Docker-образы, тесты и обработку bag
не запускали. Последующие прогоны ядер описаны в отдельном отчёте выше.
Опубликованные числа ниже — результаты авторов; наличие исходника или лога не
означает независимого воспроизведения.

По полноте артефактов именно для нашего кейса наиболее зрелым из пяти выглядит
**LCM-Solution**. Это оценка видимой готовности. Победитель по точности на общем
holdout на этапе статической сверки не установлен: различаются выборки, калибровка, эталоны, порядок событий
и определения метрик.

## Зафиксированные версии

| Репозиторий | Проверенный commit | Default branch | Без авторизации |
|---|---|---|---|
| [TellSamm/LCM-Solution](https://github.com/TellSamm/LCM-Solution) | `77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3` | `main` | HTTP 200 |
| [b1ank37/mttex-tram-odometry-hackathon](https://github.com/b1ank37/mttex-tram-odometry-hackathon) | `67664a62df3a00fe6132b25a61bc03ed346ed692` | `main` | HTTP 200 |
| [ktoyart/tram_odometry_pkg](https://github.com/ktoyart/tram_odometry_pkg) | `9bc59b1d10b891ba83c6c268ae0600c8ad34a66d` | `main` | HTTP 200 |
| [ilushenssss/odometer-on-model](https://github.com/ilushenssss/odometer-on-model) | `a4b016de373422ff93393f87825668f7ef511fdf` | `claude/autonomous-tram-gnss-free-nav-gqj7kc` | HTTP 200 |
| [Ange1ika/tram_backup_odometry_ws](https://github.com/Ange1ika/tram_backup_odometry_ws) | `4df9d7d7328fe5e5cf22699bb8c020a48304fe0b` | `main` | HTTP 200 |

`pushed_at` репозитория может относиться к другой ветке; для воспроизводимости
оценка привязана к commit прочитанной default branch.

## Сводная оценка

| Решение | Что подтверждено чтением | Ограничение готовности / доказательства |
|---|---|---|
| LCM-Solution | Humble-нода, нужные входы/выходы, карта, калибровки, инструкция жюри, Docker runner, отчёт и лог реального ROS-прогона | Основная оценка на тех же 67 bag, что калибровка. Офлайн меняет порядок событий; персональная калибровка выбирается по имени bag. В паузах есть синтетические stamp. |
| mttex | ROS-пакет, перевод км/ч→м/с, модель, скрипт и таблица скорости против GNSS | Положение интегрируется только по x, публикации получают время часов ноды. Обновление состояния зависит от переднего датчика. |
| ktoyart | Фильтр, карта и нужные названия выходных топиков | Нет выхода до пяти GNSS fix; колёса подаются в фильтр без перевода км/ч→м/с. В просмотренном дереве не найден набор тестов или отчёт по bag. |
| odometer-on-model | Симулятор, модель, карта, тесты, сценарии и численные отчёты | Входные типы/единицы и выходные топики отличаются от контракта кейса. Результаты — симуляционные. |
| tram_backup_odometry_ws | C++-ядро, ROS-оболочка, тесты, Docker, отчёт проверки | Авторы обозначают демонстрационный статус. Jazzy/Ubuntu 24.04, собственные сообщения; Humble и точность на реальном трамвае не подтверждены. |

## 1. TellSamm/LCM-Solution

### Что реализовано и измерено авторами

Нода подписана на два `VelocitySensor`, команду контроллера и GNSS fix двух антенн;
публикует `/result/velocity`, `/result/position` и диагностику. GNSS ограничен
начальной выставкой; перевод единиц выполняет ядро. Есть инструкция жюри,
Docker-конфигурация и скрипты запуска/оценки.
Источники: [нода](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/ros2_ws/src/tram_odometry/tram_odometry/node.py#L37),
[обработка колеса](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/ros2_ws/src/tram_odometry/tram_odometry/estimator.py#L66),
[инструкция](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/docs/JURY_INSTRUCTIONS.md), [Docker](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/docker/Dockerfile).

В отчёте основной набор — 67 уникальных прогонов длиннее 10 минут с полным GNSS;
авторы прямо указывают, что калибровали на этих же данных. Опубликованы медиана
RMSE скорости 0,05 м/с, медиана RMSE положения 15,6 м и медиана дрейфа 0,43%.
Это не оценка на отложенных от калибровки данных. Утверждение авторов о слабом
переобучении отдельно здесь не подтверждено.
Источник: [методика и таблица](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/docs/ACCURACY_AND_PERFORMANCE.md#L17).

В сохранённом логе одного ROS-прогона `30618_0e41eac3`: 51 303 сообщения каждого
выхода, 38,7 Гц; RMSE скорости 0,042 м/с; CPU в среднем 9,6%, максимальный RSS
58,7 МБ. В отчёте прогон описан как Docker с лимитами 2 CPU / 512 МБ.
Это полезное доказательство уровня готовности, но не наш повторный замер.
Источники: [лог](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/docs/realtime_run_30618_0e41eac3.log),
[условия прогона](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/docs/ACCURACY_AND_PERFORMANCE.md#L93).

### Уточнения по коду, важные для сравнения

| Место | Наблюдение | Следствие для нашей проверки |
|---|---|---|
| `tools/evaluate.py:55–72` | Вагон берётся из первых пяти символов имени bag; выбирается его файл калибровки. События сортируются по stamp и типу. | Нужен отдельный режим с доступной ноде информацией и реальным порядком прихода. Такая офлайн-оценка не проверяет все задержки/rollback потока. |
| `tools/evaluate.py:87–99` | Скорость сравнивается с интерполированным GNSS; позиция — с ближайшим stamp, допуск 0,05 с. | Одна общая формулировка о nearest-stamp не описывает обе метрики; перенести определения при сравнении явно. |
| `node.py:43–57` | Нода выбирает персональную калибровку только при известном `tram_id`; иначе `default`. | Результаты `--calib auto` офлайн нельзя автоматически приписать запуску жюри без номера вагона. |
| `node.py:107–120` | `gap_fill` добавляет к последнему stamp прошедшее `time.monotonic()` и публикует прогноз/удержание. | В этой ветке stamp не обязан совпадать с входным. Частота выхода сама по себе не доказывает выполнение нашего D-015. |
| `tools/evaluate.py:7–9,109` | Загрузчик `cache` и `notes/bags_summary.csv` ищутся во внешнем родительском дереве. | Для независимого повторения офлайн-оценки нужны дополнительные файлы/адаптер; одного клона недостаточно для указанного пути данных. |

Ссылки на код: [оценка и выбор калибровки](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/tools/evaluate.py#L55),
[метрики](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/tools/evaluate.py#L87),
[внешние зависимости](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/tools/evaluate.py#L7),
[выбор калибровки нодой](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/ros2_ws/src/tram_odometry/tram_odometry/node.py#L43),
[gap_fill](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/ros2_ws/src/tram_odometry/tram_odometry/node.py#L107).

В дереве commit не найден отдельный набор unit/integration-тестов; есть
`tools/ros_smoke_test.sh` и сценарии внесения аномалий в evaluator. Поэтому корректно
говорить об отсутствии найденного набора тестов, а не об отсутствии любых проверок.
Отчёт ссылается также на `tools/build_cache.py`, которого в проверенном дереве нет.
Источники: [smoke-скрипт](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/tools/ros_smoke_test.sh),
[аномалии](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/tools/evaluate.py#L35),
[команды воспроизведения](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/docs/ACCURACY_AND_PERFORMANCE.md#L113).

## 2. b1ank37/mttex-tram-odometry-hackathon

Перевод `/3.6` есть у обеих тележек. Оценка объединяет среднее колёс с моделью,
а при расхождении увеличивает вес модели. README приводит RMSE/MAE скорости
по трём bag; это опубликованные авторами измерения.
Источники: [код оценивания](https://github.com/b1ank37/mttex-tram-odometry-hackathon/blob/67664a62df3a00fe6132b25a61bc03ed346ed692/src/tram_reserve_odometry/tram_reserve_odometry/reserve_odometry_node.py#L62),
[таблица](https://github.com/b1ank37/mttex-tram-odometry-hackathon/blob/67664a62df3a00fe6132b25a61bc03ed346ed692/README.md#L211).

Конкретные ограничения:

- `reserve_odometry_node.py:102–103,136`: интегрируется только `position_x`;
  карты и поворотов в этом пути нет, y/z остаются значениями по умолчанию.
- `:120–140`: публикация по таймеру получает `get_clock().now()`, а не stamp
  обработанного измерения. Даже при sim time это не доказывает равенство stamp входа.
- `:62–65,75–107`: заднее колесо лишь обновляет кэш; расчёт выполняется в `on_front`.
  При молчании переднего датчика таймер продолжает публиковать последнее состояние,
  но прогноз по командам в этом пути не продвигается.

Источник: [нода целиком](https://github.com/b1ank37/mttex-tram-odometry-hackathon/blob/67664a62df3a00fe6132b25a61bc03ed346ed692/src/tram_reserve_odometry/tram_reserve_odometry/reserve_odometry_node.py).
В `test/` присутствуют шаблонные проверки copyright/flake8/pep257; их наличие
не является покрытием математики или сценария потери передней тележки.

## 3. ktoyart/tram_odometry_pkg

Нода содержит фильтр состояния пути/скорости и проход по карте, но до
`map_locked` возвращается из `update_filter`. Блокировка снимается после накопления
как минимум пяти GNSS fix. При отсутствии GNSS этот путь не выдаёт результат;
тайм-аутного перехода к публикации без выставки здесь нет.
Источник: [выставка и гейт публикации](https://github.com/ktoyart/tram_odometry_pkg/blob/9bc59b1d10b891ba83c6c268ae0600c8ad34a66d/tram_odometry_pkg/odometry_node.py#L79).

`v_front_cb`/`v_rear_cb` сохраняют `msg.velocity` напрямую; обновление фильтра
использует их среднее. При наших входах в км/ч это несовместимо с состоянием
скорости в м/с. В просмотренном коде нет пересчёта между этими шагами.
Источник: [callbacks и update_filter](https://github.com/ktoyart/tram_odometry_pkg/blob/9bc59b1d10b891ba83c6c268ae0600c8ad34a66d/tram_odometry_pkg/odometry_node.py#L138).

В проверенном дереве не найдены отдельные тесты и результаты на bag. Оценка
«ранний прототип» основана на этих ограничениях, а не на измеренном проигрыше точности.

## 4. ilushenssss/odometer-on-model

Развиты симулятор, сценарии отказов, карта, ядро и тесты. Численные таблицы включают
разные покрытия/погоду, blackout, изменение параметров и Монте-Карло; отдельный
отчёт о траекториях сравнивает оценку с истиной симулятора. Эти числа не заменяют
проверку на реальных bag.
Источники: [численные результаты](https://github.com/ilushenssss/odometer-on-model/blob/a4b016de373422ff93393f87825668f7ef511fdf/docs/results.md),
[источник эталона траекторий](https://github.com/ilushenssss/odometer-on-model/blob/a4b016de373422ff93393f87825668f7ef511fdf/docs/trajectory_tests.md#L1),
[сценарии](https://github.com/ilushenssss/odometer-on-model/blob/a4b016de373422ff93393f87825668f7ef511fdf/tram_nav/tram_nav/core/scenarios.py).

`navigator_node.py:96–105` принимает `Float64` и `JointState`; скорости колёс
представлены как угловые. Выходы — `odometry`, `pose`, `speed`, `distance`, `state`,
с remapping под `/tram/...`, а не требуемый набор `/result/*`.
Потребуется адаптация типов, единиц, времени и контрактов; простого изменения имён
топиков недостаточно. Источник: [ROS-нода](https://github.com/ilushenssss/odometer-on-model/blob/a4b016de373422ff93393f87825668f7ef511fdf/tram_nav/tram_nav/nodes/navigator_node.py#L96).

## 5. Ange1ika/tram_backup_odometry_ws

README явно описывает прототип с демонстрационной картой тяги и параметрами.
Базовая среда — ROS 2 Jazzy / Ubuntu 24.04; совместимость с Humble авторы не
подтверждают. Docker действительно использует образ Jazzy.
Источники: [статус проекта](https://github.com/Ange1ika/tram_backup_odometry_ws/blob/4df9d7d7328fe5e5cf22699bb8c020a48304fe0b/README.md#L1),
[Dockerfile](https://github.com/Ange1ika/tram_backup_odometry_ws/blob/4df9d7d7328fe5e5cf22699bb8c020a48304fe0b/docker/Dockerfile).

В репозитории есть C++-ядро и его тест, ROS integration-тест и отчёт об офлайн-сборке
в контейнере. Отчёт отделяет проверку интеграции от физической точности и прямо
указывает отсутствие проверки на реальном трамвае.
Источники: [верификация](https://github.com/Ange1ika/tram_backup_odometry_ws/blob/4df9d7d7328fe5e5cf22699bb8c020a48304fe0b/docs/VERIFICATION.md),
[тест ядра](https://github.com/Ange1ika/tram_backup_odometry_ws/blob/4df9d7d7328fe5e5cf22699bb8c020a48304fe0b/src/tram_model_core/test/test_model.cpp),
[тест pipeline](https://github.com/Ange1ika/tram_backup_odometry_ws/blob/4df9d7d7328fe5e5cf22699bb8c020a48304fe0b/src/tram_tools/test/test_pipeline.py).

Нода принимает собственные `ControllerCommand`/`WheelSpeeds` на `/tram/*` и
публикует собственные `TrackOdometry`/`ModelEstimate` на `/backup/*`.
Источник: [подписки и публикации](https://github.com/Ange1ika/tram_backup_odometry_ws/blob/4df9d7d7328fe5e5cf22699bb8c020a48304fe0b/src/tram_backup_odometry/src/backup_odometry_node.cpp#L34).
Это полезный инженерный образец проверки деградации, но совместимость с входами
и выходами нашего кейса не установлена.

## Старый источник MikeEgorshev/FunnyRepo

И публичная страница без авторизации, и GitHub API вернули HTTP 404.
Из этого нельзя установить, удалён ли репозиторий, переименован или стал приватным.
Текущее содержимое проверить нельзя; в сравнение он не включён.

[Исследование от 25 сентября](2026-09-25-desk-research.md) использовало его README
и само отмечало отсутствие `task.md` и данных при подготовке. Это исторический
источник, не актуальное доказательство правил или состава конкурентов. Нормативные
факты проверять по нашим [условиям](../../task.md), [контрактам](../contracts.md)
и [данным](../data.md).

## Доработки для команды по результатам обзора

Таблица ниже предлагает проверки, не назначает исполнителей и не создаёт новые
issues. Перед работой проверить занятость `area:` и уже открытые задачи. Общие
документы модели остаются в issue #20 / PR #89 у GG-crypto34 до явной передачи.

| ID | Приоритет / область | Проверка | Артефакт и критерий завершения |
|---|---|---|---|
| PUB-R1 | P1, eval / core-pipeline | Сопоставить текущие результаты ядра и ROS-записи на одинаковых входах в порядке доставки, включая задержанные stamp | Отчёт на зафиксированном main: одинаковые режимы публикации, идентичность или объяснённое расхождение; сортировка входов не скрывает rollback. |
| PUB-R2 | P1, ros / stand | Проверить stamp, частоту и продвижение состояния при молчании передней/задней/обеих тележек и при продолжающихся командах | Запись `/result/*`, соответствие stamp входам, длительность паузы и время восстановления. Связать с существующими проверками #19, а не заводить дублирующий стенд. |
| PUB-R3 | P2, eval / research | Подготовить воспроизводимое сравнение ядра LCM с нашим frozen baseline | Зафиксировать SHA, параметры и происхождение их калибровки, одинаковые bag/порядок/эталон/метрики. Результаты LCM с обучением на наших holdout помечать как пересекающиеся с калибровкой; независимая оценка потребует переобучения на нашем train. Восстановить отсутствующий загрузчик/адаптер. |
| PUB-R4 | P2, core-slip / core-estimator | Оценить идею обнаружения зависшего ненулевого показания по повторению значения и расхождению с моделью | Сначала проверить текущее покрытие и воспроизвести случай; добавлять логику только при измеримом пропуске детектора. Train — для выбора порога, holdout/stress — для оценки эффекта, контроль ложных флагов на равномерном ходе. |
| PUB-R5 | P2, docs / pitch | Сделать собственные доказательства столь же прослеживаемыми: команды → commit → сырые логи → таблицы | Чётко разделить калибровку/train/holdout, симуляцию и bag, callback latency и полный вход→публикация. Обновления model/parameters — через существующий PR #89. |

Для PUB-R3 нельзя напрямую сравнивать чужой 2D position RMSE с нашим along-track
RMSE; сначала зафиксировать одно определение метрики и начала координат. Номер вагона
и правильная стартовая конечная должны либо быть доступны обоим решениям по контракту,
либо оцениваться отдельным явно обозначенным экспериментом. Если чужая калибровка
видела наш holdout, единый evaluator сам по себе не устраняет это пересечение.

Идея PUB-R4 опирается на [детектор повторных показаний LCM](https://github.com/TellSamm/LCM-Solution/blob/77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3/ros2_ws/src/tram_odometry/tram_odometry/estimator.py#L73).
Её полезность для нашего алгоритма пока гипотеза; готового вывода об улучшении нет.

## Как обновить сверку

Для каждого репозитория получить метаданные `gh api repos/OWNER/REPO`, скачать
выбранную ветку, записать `git rev-parse HEAD` и прочитать изменённые пути из ссылок.
Публичность проверять отдельным HTTP-запросом без токена и cookies; авторизованный
`gh` сам по себе не доказывает доступ без входа.

Для утверждения «не найдено» просмотреть `git ls-tree -r --name-only HEAD` целиком,
включая скрытые каталоги. Не считать README, наличие теста и собственный успешный
прогон одним уровнем доказательства. Результаты повторного исполнения добавлять
отдельно с окружением и командами; этот документ фиксирует только чтение кода.
