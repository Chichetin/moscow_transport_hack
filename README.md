# Резервная одометрия трамвая по модели

Пакеты ROS 2 Humble. Они оценивают продольную скорость и положение трамвая по позиции
контроллера водителя и скоростям двух тележек, без GNSS и IMU, и публикуют
`/result/velocity` и `/result/position` в реальном времени. Хакатон Московского
транспорта, кейс «Резервная одометрия по модели» (`task.md`).

> Ни одно число в документах не пишется без команды, которой оно получено: источник
> чисел — `docs/verification/`, сводка — `docs/accuracy.md`.

## Для жюри: проверка за 5 минут

Нужны: ROS 2 Humble (`ros-humble-ros-base` достаточно), `python3-numpy`, `python3-yaml`
из apt Humble. Шаги 1–4 (сборка, запуск, выходы, логи) работают без интернета. Для шагов 5–6
сеть нужна один раз: `pip install` зависимостей `tools/eval` и `docker build` образа стенда
(`ros:humble-ros-base` + `apt-get install procps time`); сам стенд идёт с `--network=none`.

### 1. Сборка (без интернета)

```bash
# в своём workspace ROS 2 Humble
git clone <репозиторий> tram && cp -r tram/src/* <ws>/src/
cd <ws> && source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Собираются три пакета: `tram_vehicle_msgs_vendor`, `tram_odometry_core`, `tram_odometry`.
Свой `tram_vehicle_msgs` ничего исключать не требует. `tram_vehicle_msgs_vendor` (каталог
`src/tram_vehicle_msgs/`) сначала ищет ваш `tram_vehicle_msgs` — в том же workspace или в
underlay — и берёт его. Только если его нет, он собирает вложенную копию пакета организаторов
(`.msg` без изменений, в `package.xml` добавлен `<maintainer>`, без него Humble пакет не
собирает). Копия вложена в другой пакет, поэтому `colcon` не видит два пакета с одним
именем ни в какой раскладке (D-044). В `--packages-select` пакет сообщений называется
`tram_vehicle_msgs_vendor`; `--packages-up-to tram_odometry` включает его сам.

Та же сборка проверяется у нас в десяти раскладках workspace, которые может выбрать жюри
(клон целиком в `src/`, только `src/*`, свой `tram_vehicle_msgs` рядом или поверх, underlay,
`--packages-up-to`, `--merge-install`, `--symlink-install`), из `git archive`, без сети,
2 CPU, 512 МБ: `bash tools/submission/jury_layouts.sh`. После сборки в каждой раскладке проверяется,
что после `source install/setup.bash` импортируются сообщения и нода.

### 2. Запуск

В **каждом** новом терминале сначала перейти в `<ws>` и повторить `source` из шага 1
(`tram_vehicle_msgs` и нода видны только в сессии, где из `<ws>` выполнен
`source install/setup.bash`; без этого `ros2 bag play` во втором терминале молча
проигнорирует все три входных топика с WARN `package 'tram_vehicle_msgs' not found` —
нода останется жива, но без данных):

```bash
# терминал 1: нода
cd <ws> && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch tram_odometry odometry.launch.py

# терминал 2: bag организаторов
cd <ws> && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 bag play <абсолютный путь к каталогу bag>
```

`odometry.launch.py` — единственная точка входа. Параметры — `params_file:=<yaml>`,
по умолчанию `share/tram_odometry/config/params.yaml` (описание каждого ключа —
`docs/parameters.md`). Карта маршрута `share/tram_odometry/maps/route.csv` ставится
вместе с пакетом; отдельно ничего копировать не нужно. `ros2 run tram_odometry odometry_node`
тоже работает: без `params_file` нода берёт установленный `params.yaml`.

Нода подписана на `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity`,
`/vehicle/driver_position_cmd` и на GNSS `/sensing/gnss/master/{fix,vel}`,
`/sensing/gnss/rover/fix`. GNSS используется только первые `gnss.init_window_s` = 5 с
от первого сообщения bag — для начальной выставки на карте; дальше сообщения GNSS
отбрасываются в ядре (тест `test_real_core_ignores_gnss_after_window_through_the_node`).
Подписки best-effort, поэтому нода соединяется с `ros2 bag play` при любом QoS издателя.
Если в workspace стоит `tram_vehicle_msgs` без `DriverControllerCommand` (так в образе судьи
организаторов `check-code`: там только `VelocitySensor`), нода всё равно запускается и считает
по двум тележкам без модели привода (выход тогда ~20 Гц — только на входы тележек);
`ros2 bag play` в таком окружении топик контроллера
пропускает с WARN о typesupport (D-093, раскладки `judge_msgs_*` в `jury_layouts.sh`).

### 3. Что ожидать на выходе

| Топик | Тип | Частота | Содержимое |
|---|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | на каждом принятом сообщении тележек и контроллера, ~40 Гц (2 × 10 Гц + 20 Гц); GNSS и отброшенный вход выхода не дают | `velocity` — продольная скорость, **м/с** (вход тележек — км/ч, перевод в ядре), на момент stamp − 0,09 с (`output.velocity_delay_s`): эталон судьи `/localization/kinematic_state` отстаёт от датчиков на ~0,09 с (D-095) |
| `/result/position` | `nav_msgs/msg/Odometry` | та же, начиная с первого валидного fix master (до него, в первые ≤ 1 с bag, позиции ещё нет — выходит только скорость); bag без GNSS — с конца окна `gnss.init_window_s`, frame `odom` (D-086) | `pose.pose.position` — x, y — плоская сетка MGRS `37UCB` (UTM 37N − (300 000, 6 100 000)), z — высота над эллипсоидом WGS84, м, frame `map`; `pose.pose.orientation` — курс; `twist.twist.linear.x` — скорость на stamp входа, без сдвига `/result/velocity` (D-095); ковариации заполнены, неоцениваемые компоненты 1e6 |
| `/result/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1–10 Гц | флаги проскальзывания, возраст входов и состояние обеих тележек |

`header.stamp` на обоих выходах — время входного сообщения из bag (не wall clock).
`frame_id`: у `/result/velocity` — `base_link`, у `/result/position` — `map` с
`child_frame_id` = `base_link` (у bag без GNSS — `odom`: локальные метры от старта, привязки к
сетке нет, D-084, D-086). Frame `map` — плоская сетка MGRS, как в примере организаторов
(lat 55,8088325462547, lon 37,4602768500852 → x 103 501,6309, y 85 876,1201): начало не зависит
от прогона, курс — от оси `x` сетки (D-083). Ядро считает в локальной ENU от первого fix master
и переводит на выходе (`position/geo.py`).
`pose.pose.position` — точка `base_link` по tf организаторов: ось передней тележки на уровне
касания колеса и рельса, 9,873 м впереди антенны master и 3,0 м ниже антенн
(`position.base_ahead_m`, `position.antenna_height_m`; D-077). Выход в точке самой антенны
master — оба ключа `0` в `params.yaml`.

```bash
# терминал 3: те же source, что в шаге 2
cd <ws> && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic hz /result/velocity                       # ≈ 40 Гц
ros2 topic echo --once /result/position
```

### 4. Логи

- Нода пишет в stdout терминала `ros2 launch` (`output='screen'`). Некорректный вход (NaN,
  stamp из прошлого, молчащая тележка) пропускается без сообщения: на каждом сообщении нода
  не логирует. Исключение ядра — одна строка `error` с троттлингом 5 с,
  нода живёт дальше.
- На стенде (п. 6) всё складывается в `out/stand/<bag>/`: `build.log` (colcon), `node.log`
  (нода), `play.log`, `record/` (rosbag2 с входами и `/result/*`), `resources.csv`, `stand.json`.

### 5. Метрики против GNSS-эталона (без ROS)

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt   # rosbags, numpy, pyyaml, scipy, pandas, matplotlib, pytest
cp .env.example .env                                  # TRAM_DATA_DIR = каталог с распакованными bag
.venv/bin/python tools/eval/run_eval.py --bag 30618_e9a34502              # один bag
.venv/bin/python tools/eval/run_eval.py --split holdout                   # 26 bag отложенных дней
```

`tools/eval` подаёт сообщения bag в то же ядро `Odometry.step`, что и нода, в порядке записи
(скорость в метриках — `Estimate.speed` без сдвига `output.velocity_delay_s`: эталон eval, GNSS
`master/vel`, не отстаёт; сдвиг против эталона судьи — `tools/research/judge_speed_delay.py`, D-095),
обрезает GNSS после окна выставки и сравнивает выход с эталоном из GNSS master всего прогона:
скорость (RMSE/MAE, bias на разгоне, торможении, стоянке), along-track, cross-track,
дрейф в конце в % пути, pos3d. Определения — `docs/contracts.md` §4 и `tools/eval/README.md`,
методика и таблицы — `docs/accuracy.md`. Результат печатается в stdout и пишется в
`out/eval/<commit>-<набор>/metrics.json`.

### 6. Задержка, частота, ресурсы — стенд жюри

```bash
bash docker/jury-stand.sh <bag_id>            # 2 CPU, 512 МБ, --network=none; ~ длительность bag; в конце сам печатает таблицу
.venv/bin/python tools/stand/run_stand.py out/stand/<bag_id>     # пересчитать отдельно: таблица + stand.json
```

Стенд собирает `src/` в чистом контейнере `ros:humble-ros-base` без сети, запускает ноду по
launch, играет bag в реальном темпе, пишет входы и `/result/*` и раз в секунду снимает CPU и
RSS процессов ноды. `run_stand.py` считает задержку вход → выход по совпадению `header.stamp`
(p50/p99/max), частоту каждого `/result/*`, CPU в ядрах, пик RSS и рост RSS за прогон и
сравнивает с порогами критерия 4 (p99 ≤ 100 мс, max ≤ 250 мс, ≥ 10 Гц, ≤ 2 ядра, ≤ 512 МБ).
Методика — `tools/stand/README.md`, числа — `docs/accuracy.md`.

## Документы сдачи

| Что | Где |
|---|---|
| Описание модели: уравнения, структура, входы/выходы | `docs/model.md` |
| Допущения, ограничения, параметры конфигурации | `docs/parameters.md` |
| Точность и быстродействие: методика, таблицы, графики, замеры | `docs/accuracy.md`, прогоны — `docs/verification/` |
| Ограничения и план развития | `docs/roadmap.md` |
| Данные организаторов и их особенности | `docs/data.md` |
| Журнал решений | `docs/decisions.md` |
| Матрица условий задачи | `docs/tz-compliance.md` |

## Для команды

Подключение нового участника — `docs/onboarding.md` (два промпта для Claude Code). Правила работы — `CLAUDE.md`, Git — `GIT.md`, формат работы агентов — `docs/agents.md`,
контракты — `docs/contracts.md`, данные — `docs/data.md`.

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt
.venv/bin/python -m pytest                     # ядро и tools, без ROS
bash docker/dev.sh python3 -m pytest           # то же на numpy 1.21, как у жюри
bash docker/dev.sh colcon build                # сборка в контейнере
bash tools/submission/jury_layouts.sh --run    # раскладки жюри + нода на bag
.venv/bin/python tools/submission/check_submission.py   # машинный гейт сдачи
```
