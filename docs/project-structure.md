# Структура репозитория и карта областей

Потоков-людей нет. Есть **области** (`area:`): каталог или модуль, который в каждый момент
правит не больше одной issue в работе. Метка `area:` у issue ставится по этой таблице.
Правило: не брать issue с той же `area:`, если на ней уже есть assignee и она открыта
(`session-context.sh` показывает занятые области в начале сессии).

## Дерево

```text
.
├── src/                                  # colcon workspace (жюри копирует это в свой ws)
│   ├── tram_vehicle_msgs/                # tram_vehicle_msgs_vendor: копия сообщений организаторов, если у жюри нет своей (D-006, D-043)
│   ├── tram_odometry_core/               # ядро: чистый Python + numpy, без rclpy (D-001, D-004)
│   │   ├── tram_odometry_core/
│   │   │   ├── types.py                  # КОНТРАКТ: dataclass-типы, Params, load_params
│   │   │   ├── pipeline.py               # Odometry.step(raw) -> Estimate: сборка модулей
│   │   │   ├── preprocess/               # единицы, stamp, дыры, выбросы, молчащий датчик
│   │   │   ├── dynamics/                 # модель привода и продольной динамики
│   │   │   ├── estimator/                # EKF скорости, масштабы колёс
│   │   │   ├── slip/                     # проскальзывание/юз, доверие к тележкам, сцепление
│   │   │   └── position/                 # выставка по GNSS, карта, x/y/yaw, ковариация
│   │   └── test/                         # pytest ядра (без ROS)
│   └── tram_odometry/                    # ROS 2 ноды: тонкие обёртки
│       ├── tram_odometry/                # odometry_node.py, diagnostics
│       ├── launch/                       # odometry.launch.py — точка входа жюри
│       ├── config/params.yaml            # КОНТРАКТ по именам/единицам
│       ├── maps/route.csv                # карта маршрута (строится tools/pathgraph)
│       └── test/                         # тесты ноды (colcon test, в контейнере)
├── tools/
│   ├── eval/                             # метрики без ROS: rosbags → ядро → таблица
│   │   ├── splits.yaml                   # КОНТРАКТ: train / holdout / quick / dups
│   │   └── tests/
│   ├── stand/                            # замер задержки/частоты/ресурсов по записи стенда
│   ├── pathgraph/                        # сборка maps/route.csv из GNSS train
│   └── survey/                           # обзор bag → docs/data/
├── notebooks/identification/             # идентификация модели: скрипты, графики, отчёт
├── docker/                               # образ jury/dev, dev.sh, jury-stand.sh
├── docs/
│   ├── contracts.md                      # КОНТРАКТ
│   ├── data.md, data/                    # данные и ловушки
│   ├── decisions.md, tz-compliance.md    # журнал решений, матрица условий
│   ├── model.md, parameters.md,          # документы сдачи (создаются по плану)
│   │   accuracy.md, roadmap.md
│   ├── onboarding.md                     # промпты для подключения участника
│   ├── pitch/                            # материалы питча
│   ├── plans/                            # план и пакеты задач
│   ├── verification/                     # прогоны eval и стенда: команды и числа
│   └── research/                         # заметки по литературе и вариантам модели
├── .claude/                              # настройки, хуки, роли, скиллы Claude Code
├── .github/                              # шаблоны PR и issue
├── README.md                             # инструкция жюри (артефакт сдачи 2)
├── task.md                               # условия дословно
├── CLAUDE.md, GIT.md, HANDOFF.md
└── pyproject.toml, requirements-dev.txt  # pytest и зависимости инструментов
```

## Области

| `area:` | Пути | Что внутри | Потребители (чей код править и кому оставить комментарий при смене контракта) |
|---|---|---|---|
| `area:contracts` | `docs/contracts.md`, `core/types.py`, имена в `params.yaml`, `tools/eval/splits.yaml` | контракт | все |
| `area:core-preprocess` | `core/preprocess/` | км/ч→м/с, stamp, дыры, выбросы | pipeline, eval |
| `area:core-dynamics` | `core/dynamics/` | модель привода и динамики | estimator, slip, ident |
| `area:core-estimator` | `core/estimator/` | EKF скорости, масштаб колёс | pipeline |
| `area:core-slip` | `core/slip/` | детектор, доверие, сцепление | estimator, ros (diagnostics) |
| `area:core-position` | `core/position/` | выставка, карта, x/y | pipeline, eval |
| `area:core-pipeline` | `core/pipeline.py` | сборка модулей | ros, eval |
| `area:ros` | `src/tram_odometry/` кроме `maps/`, значений `params.yaml` | нода, launch, diagnostics | стенд |
| `area:map` | `tools/pathgraph/`, `src/tram_odometry/maps/` | карта маршрута | position, eval |
| `area:ident` | `notebooks/identification/`, значения в `params.yaml` | идентификация параметров | dynamics, estimator |
| `area:eval` | `tools/eval/` кроме `splits.yaml` | метрики, стресс-генератор | все PR (таблица) |
| `area:stand` | `docker/`, `tools/stand/` | образ, стенд жюри, замеры RT | ros |
| `area:docs` | `README.md`, `docs/accuracy.md` | инструкция жюри, точность и быстродействие | — |
| `area:docs-model` | `docs/model.md`, `docs/parameters.md`, `docs/roadmap.md` | модель, допущения и параметры, план развития | — |
| `area:pitch` | `docs/pitch/` | материалы питча, сценарий демо | — |
| `area:infra` | `.claude/`, `.github/`, `CLAUDE.md`, `GIT.md`, `pyproject.toml`, `requirements-dev.txt`, `tools/submission/` | правила и tooling, гейт сдачи и раскладки жюри | все |

`core/` = `src/tram_odometry_core/tram_odometry_core/`.

Исключение из правила областей: PR модуля ядра подключает свой модуль в `pipeline.py`
(несколько строк вызова) без метки `area:core-pipeline` — иначе каждый модуль ждал бы
чужую issue. Конфликты в `pipeline.py` при merge разрешает тот, кто вливается вторым.

Общие файлы, которые правит любой PR и где конфликт разрешается сохранением обеих сторон:
`docs/decisions.md` (строка D-…), `docs/tz-compliance.md` (строка), `HANDOFF.md`.

## Зависимости модулей

```
types.py ← preprocess ← pipeline → ros node
         ← dynamics  ←┤
         ← slip      ←┤
         ← estimator ←┤
         ← position  ←┘ ← maps/route.csv ← tools/pathgraph
pipeline ← tools/eval ← splits.yaml
```

Модуль ядра импортирует только `types.py` и numpy; соседей — только через `pipeline`.
`rclpy` импортируется только в `src/tram_odometry/`.
