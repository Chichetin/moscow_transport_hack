# Резервная одометрия трамвая по модели

Пакеты ROS 2 Humble. Они оценивают продольную скорость и положение трамвая по позиции
контроллера водителя и скоростям двух тележек, без GNSS и IMU, и публикуют
`/result/velocity` и `/result/position` в реальном времени. Хакатон Московского
транспорта, кейс «Резервная одометрия по модели» (`task.md`).

> **Статус: каркас.** Разделы с пометкой `TBD` заполняются по плану
> (`docs/plans/2026-09-25-plan.md`). Ни одно число ниже не пишется без команды, которой оно получено.

## Для жюри: проверка за 5 минут

### 1. Сборка (без интернета)

```bash
# в своём workspace ROS 2 Humble
cp -r <репозиторий>/src/* <ws>/src/
cd <ws> && source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

В `src/` лежит копия пакета сообщений организаторов `tram_vehicle_msgs`. В ней добавлена
одна строка `<maintainer>`: без неё Humble пакет не собирает. Если в вашем workspace уже
есть свой `tram_vehicle_msgs`, исключите наш: `touch <ws>/src/tram_vehicle_msgs/COLCON_IGNORE`.

Та же проверка в изолированном окружении с ограничениями критерия 4 (2 CPU, 512 МБ, без сети):

```bash
bash docker/jury-stand.sh <bag_id>      # сборка + нода + bag play + запись + ресурсы
```

### 2. Запуск

```bash
ros2 launch tram_odometry odometry.launch.py          # params_file:=<yaml> — свой конфиг
ros2 bag play <путь к bag>                            # в другом терминале
```

### 3. Что ожидать на выходе

| Топик | Тип | Частота | Содержимое |
|---|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | на каждом входном сообщении, ~40 Гц | `velocity` — продольная скорость, м/с |
| `/result/position` | `nav_msgs/msg/Odometry` | на каждом входном сообщении, ~40 Гц | `pose.pose.position` — x (восток), y (север), м, frame `map`; `twist.twist.linear.x` — скорость; ковариации |
| `/result/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | ещё не публикуется (пакет R2) | флаги проскальзывания, оценка сцепления, состояние входов |

`header.stamp` на обоих выходах — время входного сообщения из bag.

### 4. Логи, метрики, задержка

TBD: где лог ноды, как посчитать метрики против GNSS (`tools/eval`), как измерить задержку
(`tools/stand`).

## Документы сдачи

| Что | Где |
|---|---|
| Описание модели: уравнения, структура, входы/выходы | `docs/model.md` — TBD |
| Допущения, ограничения, параметры конфигурации | `docs/parameters.md` — TBD |
| Точность и быстродействие: методика, таблицы, графики, замеры | `docs/accuracy.md` — TBD |
| Ограничения и план развития | `docs/roadmap.md` — TBD |
| Данные организаторов и их особенности | `docs/data.md` |
| Журнал решений | `docs/decisions.md` |

## Для команды

Подключение нового участника — `docs/onboarding.md` (два промпта для Claude Code). Правила работы — `CLAUDE.md`, Git — `GIT.md`, формат работы агентов — `docs/agents.md`,
контракты — `docs/contracts.md`, данные — `docs/data.md`.

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt
.venv/bin/python -m pytest
bash docker/dev.sh            # ROS 2 Humble в контейнере, данные в /data
```
