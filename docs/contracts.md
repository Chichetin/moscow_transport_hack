# Контракты — v1

Единственная точка стыковки модулей. Всё, что здесь описано, меняется **только отдельным
PR** с перечнем потребителей в описании. В том же PR код потребителей, уже лежащий в `main`,
приводится к новому контракту, и тесты зелёные. Merge — по вердикту `reviewer` `merge`, апрув
людей не нужен (D-018); владельцам issue соседних `area:` — комментарий со ссылкой на PR.
Владельца у контракта нет. Статус v1: типы и загрузка параметров реализованы в
`tram_odometry_core/types.py`, сигнатура `step` — в `pipeline.py`.

Состав контракта:

1. ROS-топики, типы, время и системы координат.
2. Типы и сигнатуры ядра `tram_odometry_core` (`types.py` + функции модулей).
3. `src/tram_odometry/config/params.yaml` — имена, единицы, смысл (значения — не контракт).
4. Формат вывода метрик `tools/eval`.
5. Формат карты маршрута `maps/route.csv`.
6. Разбиение прогонов `tools/eval/splits.yaml` (D-011).

## 1. ROS

| Топик | Тип | Направление | Частота | Примечание |
|---|---|---|---|---|
| `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | вход | ~10 Гц | **км/ч** (D-003) |
| `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | вход | ~10 Гц | **км/ч** |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/msg/DriverControllerCommand` | вход | 20 Гц | `position` −15…+15 |
| `/sensing/gnss/master/fix`, `/rover/fix` | `sensor_msgs/msg/NavSatFix` | вход | 10 Гц | только первые `gnss.init_window_s` с (D-005) |
| `/sensing/gnss/master/vel` | `geometry_msgs/msg/TwistStamped` | вход | 10 Гц | ENU, только окно выставки |
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | **выход** | на каждый вход, ~40 Гц | `velocity` — продольная скорость, **м/с**, ≥ 0 |
| `/result/position` | `nav_msgs/msg/Odometry` | **выход** | на каждый вход | см. ниже |
| `/result/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | выход | 1–10 Гц | статус проскальзывания и входов (критерий 3) |

Правила для обоих `/result/*`:

- `header.stamp` = `header.stamp` входного сообщения, вызвавшего публикацию (D-015). Не wall
  clock, не ноль, не время записи bag.
- Публикация только онлайн, без задержки «до следующего сообщения» и без сглаживания назад.
- QoS выходов — по умолчанию (reliable, depth 10); судья подписан best-effort, это совместимо.
  QoS входов ноды — best-effort, depth 100: соединяется с издателем любой надёжности, в том
  числе с best-effort `ros2 bag play` (D-028).

`/result/position` (`nav_msgs/msg/Odometry`):

| Поле | Значение |
|---|---|
| `header.frame_id` | `map` (`frames.map`) — локальная ENU, метры, начало — первая валидная точка GNSS master в окне выставки (**уточнить у организаторов**, `HANDOFF.md`, #23) |
| `child_frame_id` | `base_link` (`frames.base`) |
| `pose.pose.position` | точка `base_link` — ось передней тележки на уровне касания колеса и рельса (tf организаторов 27.09: master x = −9,873, rover x = +2,563, обе z = 3,0 м; D-077): `x` — восток, `y` — север, `z` — вверх, м. По карте — на `position.base_ahead_m` впереди трека master по дуге и на `position.antenna_height_m` ниже высоты антенн. Исключение: до первого принятого fix (запасная прямая D-021) — точка master, `z = 0` |
| `pose.pose.orientation` | курс по касательной карты (yaw), кватернион |
| `pose.covariance` | 6×6 row-major; `[0]`,`[7]` — дисперсии x/y, м²; `[35]` — yaw; неизвестные — `-1` не ставить, ставить большое число |
| `twist.twist.linear.x` | продольная скорость, м/с (= `/result/velocity`) |
| `twist.covariance[0]` | дисперсия скорости, (м/с)² |

`/result/velocity`: `header.frame_id = base_link`, `velocity` в м/с.

`/result/diagnostics`: один `DiagnosticStatus` `tram_odometry: slip` (level OK/WARN/ERROR,
`values`: `slip_front`, `slip_rear`, `adhesion_est`, `wheel_scale_front`, `wheel_scale_rear`)
и один `tram_odometry: inputs` (`front_age_s`, `rear_age_s`, `cmd_age_s`, `gnss_used`).

## 2. Ядро `tram_odometry_core`

Время везде — секунды `float` из `header.stamp`; скорости — м/с после preprocess; СИ.
Код — `tram_odometry_core/types.py` (dataclass, `frozen=True` для входов). Ниже — псевдокод
в синтаксисе Python 3.10; в коде тот же смысл через `typing` (`Union`, `Optional`, `Tuple`). Целое
значение для float-параметра (ROS может прислать `int`) `load_params` принимает.

```python
@dataclass(frozen=True)
class WheelSample:            # после preprocess: м/с
    t: float
    bogie: Literal['front', 'rear']
    speed: float

@dataclass(frozen=True)
class CommandSample:
    t: float
    notch: int                # -15..15

@dataclass(frozen=True)
class GnssFix:
    t: float
    antenna: Literal['master', 'rover']
    lat: float; lon: float; alt: float
    status: int

@dataclass(frozen=True)
class GnssVel:
    t: float
    ve: float; vn: float      # ENU, м/с

Sample = WheelSample | CommandSample | GnssFix | GnssVel

@dataclass(frozen=True)
class SlipState:
    front_trust: float        # 0..1 — вес измерения тележки в фильтре
    rear_trust: float
    slip_front: bool
    slip_rear: bool
    adhesion_est: float | None

@dataclass(frozen=True)
class FilterDiagnostics:
    t: float
    bogie: Literal['front', 'rear']
    nis: float
    accepted: bool

@dataclass(frozen=True)
class Estimate:
    t: float                  # = t входа, вызвавшего обновление
    speed: float              # м/с, >= 0
    speed_var: float          # (м/с)^2
    accel: float              # м/с^2, оценка
    accel_model: float        # м/с^2, прогноз модели привода
    distance: float           # м, путь от начала прогона
    x: float; y: float        # м, frame map
    z: float                  # м, frame map, ENU up (высота карты + смещение прогона, D-024)
    yaw: float                # рад, ENU
    pos_cov: tuple[float, float, float]   # var_x, var_y, cov_xy
    slip: SlipState
    gnss_used: bool
    filter_diagnostics: FilterDiagnostics | None = None  # NIS нового измерения тележки

@dataclass(frozen=True)
class Branch:                 # одна направленная ветка maps/route.csv (§5), массивы numpy (N,)
    s: ndarray                # м, дуга, равномерный шаг, с 0
    x: ndarray; y: ndarray; z: ndarray    # м, ENU от начала карты

@dataclass(frozen=True)
class Route:
    origin: tuple[float, float, float]    # lat °, lon °, alt м — начало ENU карты
    branches: tuple[Branch, ...]          # в порядке файла: индекс = `branch`
    stops: tuple[tuple[int, float], ...] = ()   # (branch, s) места остановок, `maps/stops.csv` (§5), D-034
```

`load_route(path) -> Route` в `types.py` — единственное место, где читается `route.csv` (и `stops.csv` рядом с ним, если он есть)
(как `load_params` для yaml). Нода берёт файл из `share/tram_odometry/maps/<position.map_file>`,
`tools/eval` — из `src/tram_odometry/maps/` того же worktree; оба передают его в `Odometry(params, route=)`.

`FilterDiagnostics(t, bogie, nis, accepted)` — неизменяемая запись нового измерения:
`t` — stamp, `bogie` — `front`/`rear`, `nis` — квадрат инновации, делённый на её
дисперсию (безразмерно), `accepted` — прошло ли измерение порог. На входе
контроллера и без нового измерения `Estimate.filter_diagnostics` равен `None`.
Обязательные ROS-топики и формат `metrics.json` не меняются.

Модули и их публичные функции (одна `area:` — один модуль):

| Модуль | Публичное | Контракт поведения |
|---|---|---|
| `preprocess` | `Preprocessor(params).accept(raw) -> Sample \| None` | `raw` — сырой вход: пара `(topic, ROS-сообщение)` (км/ч, notch как есть; поля сообщения — как в ROS, у bag и rclpy одинаковые); возвращает нормализованный `Sample` или `None` (выброс, NaN, stamp из прошлого сверх допуска, GNSS вне окна); fix `gnss.topic_fix` — `GnssFix(antenna='master')`, fix `/sensing/gnss/rover/fix` — `GnssFix(antenna='rover')` |
| `dynamics` | `model_accel(notch: int, speed: float, params) -> float` | чистая функция, м/с²; без состояния: интерполяция таблиц D-029 по `speed_grid_mps`, пределы сцепления и мощности, минус сопротивление Дэвиса. Задержка отклика `drive.response_delay_s` — состояние `pipeline` (буфер команд): в модель идёт позиция контроллера на момент `t − delay`. При `drive.use_model` pipeline передаёт `accel_model` детектору и прогнозирует скорость на паузе обеих тележек (`v ≥ 0`); `Estimate.accel_model` — это значение |
| `slip` | `SlipDetector(params).update(front, rear, accel_model, est) -> SlipState` | `front`/`rear` — последний `WheelSample` или `None` (молчит); `est` — сглаженная скорость фильтра, м/с, до этого обновления (`None`, пока её нет); `accel_model` — м/с². Возвращает доверие 0..1 и флаги «тележке не доверяем» (аномалия или отказ), D-027. Состояние детектора — последний сэмпл каждой тележки и stamp её последних скачков вверх и вниз (`slip.noise_*`, D-054): при расхождении противофазные недавние скачки обеих — доверие 0,5/0,5, синфазные — 0/0 и оба флага. Ещё — текущий повтор значения каждой тележки: `slip.freeze_min_samples` подряд новых stamp с тем же ненулевым значением при изменении скорости по модели больше `slip.freeze_dv_mps` — тележка залипла, она выбывает как молчащая: доверие 0 и флаг (D-078) |
| `estimator` | `SpeedFilter(params).predict(t, accel_model)`, `.update(sample: WheelSample, trust: float)`, `.state() -> (speed, speed_var, accel)`, `.diagnostics() -> FilterDiagnostics \| None` | монотонное время состояния внутри; новое колесо допускается, если его stamp отстаёт от последнего принятого stamp **колёс** не более чем на `input.stale_timeout_s`, даже когда контроллер опережает оба колеса; более старое колесо отбрасывается. Запоздалое измерение учитывается без отката состояния, с поправкой на возраст; диагностика относится только к новому измерению тележки |
| `position` | `PathTracker(params, route)`; `.on_fix(lat, lon, alt, status, distance)` — каждый fix master в окне `gnss.init_window_s`; `.on_rover(lat, lon, alt, status)` — каждый fix rover в том же окне (только курс); `.ready -> bool`; `.advance(distance, speed=0.0) -> (x, y, z, yaw, pos_cov) \| None` (speed — скорость фильтра, м/с; x, y, z — точка `base_link`, D-077) | frame `map` прогона — ENU первого fix статуса 2 (иначе первого валидного), как эталон `tools/eval`; карта переводится в него один раз через ECEF; якорь — ближайшая точка ближайшей ветки по последнему fix окна; когда есть курс master → rover (база в [`position.heading_min_base_m`, `position.heading_max_base_m`], ловушка 15; rover без GBAS при GBAS-начале курса не даёт), — ближайшая точка ветки, идущей по курсу (при её наличии в `fix_gate_m`); rover-fix меняет уже поставленный якорь, только если тот идёт против курса, а ветка по курсу — другая; дальше только вперёд по дуге `s = s₀ + distance − distance₀` (якорь, остановки и масштаб — в дуге трека master; выход — в точке `s + position.base_ahead_m`, за тупиком — по касательной не дальше `base_ahead_m`, высота минус `position.antenna_height_m`, D-077), конец ветки продолжается на ближайшей ветке не дальше `position.join_m`; тупиковая боковая ветка, отходящая от ветки не по пути по умолчанию, берётся, если в окне [`position.side_min_m`, `position.side_max_m`] после её начала `speed > position.side_speed_mps`, а если путь за её тупиком превысил `position.side_overrun_m` — возврат на путь по умолчанию (D-074); `pos_cov` — `cross_std_m` поперёк, вдоль растёт как `along_drift_frac · путь`; fix окна дальше `position.fix_gate_m` от карты (в ENU карты) — выброс, не участвует ни в начале frame, ни в якоре, ни в высоте; до первого принятого fix `advance` даёт `None`: pipeline публикует прямую D-021 (начало — первый fix, курс — по GNSS vel окна), `z = 0` |
| `pipeline` | `Odometry(params, route=None).step(raw) -> Estimate \| None` | единственная точка, которую зовут нода и `tools/eval`; `None` — вход отброшен, публиковать нечего |

`params` — `Params` из `types.py`: неизменяемый dataclass, по одному вложенному dataclass на
секцию `params.yaml` (`params.gnss.init_window_s`, `params.drive.notch_max`), имена полей =
ключи yaml. Списки читаются как `tuple[float, ...]`. Загрузка yaml → `Params` — одна функция
`load_params(path)` в `types.py`; пропущенный или лишний ключ — `KeyError`, неверный тип —
`TypeError` (нода не должна стартовать на кривом конфиге молча). Нода передаёт те же значения
через ROS-параметры, eval читает yaml той же функцией.

## 3. `params.yaml`

Файл — `src/tram_odometry/config/params.yaml`. Контракт — **имена ключей, единицы и смысл**,
записанные комментарием у каждого ключа. Значения меняются обычным PR с таблицей метрик.
Новый ключ — тоже контракт (ядро и нода читают его одинаково). Пустые списки `[]` запрещены:
ROS 2 не выводит их тип и нода не стартует.

### Шумы фильтра и NIS (D-032)

`filter.q_accel` — спектральная плотность белого шума ускорения, единицы
`(м/с²)²·с`: добавка к дисперсии скорости за `dt` равна `q_accel·dt`.
`filter.r_wheel` — дисперсия скорости одной исправной тележки, `(м/с)²`;
при доверии `trust > 0` эффективная дисперсия измерения равна `r_wheel/trust`.
`filter.q_bias` — спектральная плотность случайного хода медленного смещения
ускорения, `(м/с²)²/с`; `filter.initial_bias_var` — его начальная дисперсия,
`(м/с²)²`. `filter.nis_gate` — безразмерный включительный порог NIS; при NIS
выше него измерение не обновляет состояние. Принятые и отвергнутые измерения
дают отдельную запись `FilterDiagnostics`.

Пары тележек и отъезд (D-073, значения — прежние константы `SpeedFilter`):
`filter.pair_window_s` — с, отсчёты двух тележек не дальше по stamp считаются
одновременной парой (нули остановки, относительный масштаб).
`filter.bias_release_speed_mps` — м/с, ниже неё (или ниже `position.stop_speed_mps`,
что больше) смещение ускорения при торможении сбрасывается.
`filter.departure_slack_mps` — м/с, допуск к `input.max_wheel_accel_mps2·(t − остановка)`
для правдоподобного отъезда; `filter.departure_var_factor` — безразмерно, дисперсия
скорости на отъезде не ниже `factor·` дисперсии измерения (не меньше 1).
Относительный масштаб тележек обновляется только по паре, где доверие обеих не ниже
`filter.scale_min_trust`, обе быстрее `filter.scale_min_speed_mps` (м/с) и расходятся
не больше `filter.scale_max_diff_mps` (м/с); оценка ограничена `±filter.scale_max_rel`
и сглаживается шагом `filter.scale_gain` (оба безразмерны).

### Скачки тележек (D-054)

`slip.noise_accel_mps2` — порог скачка тележки, м/с²: модуль разности собственного ускорения
шага (`Δv/Δt` двух соседних сэмплов одной тележки) и `accel_model` больше него. `slip.noise_hold_s`
— с, сколько скачок считается недавним. Правила применения — §2, модуль `slip`.

### Залипшая тележка (D-078)

`slip.freeze_min_samples` — целое, сколько подряд новых отсчётов тележки (stamp строго больше
прежнего) с точно тем же ненулевым значением делают её кандидатом в залипание.
`slip.freeze_dv_mps` — м/с, модуль суммы `accel_model·Δt` за этот повтор больше него — тележка
залипла. Ровно 0 не залипание (стоянка, мёртвая зона — ловушка 8). Правила применения — §2,
модуль `slip`.

### Пределы входа тележек (D-056)

`input.max_wheel_speed_mps` — м/с, положительный: отсчёт тележки выше него (после перевода км/ч → м/с)
отбрасывается всегда, в том числе первый отсчёт потока. Скачок с ровно 0 быстрее
`input.max_wheel_accel_mps2` проходит, только если вторая тележка свежая и показывает ту же скорость
(`slip.front_rear_threshold_mps`).

### Таблицы привода (D-029)

`drive.speed_grid_mps` — узлы скорости в м/с: не меньше двух, первый равен 0, дальше строго
возрастают. `drive.notch_max` — положительный целый максимум модуля позиции контроллера.
`drive.traction_accel_table` и `drive.brake_accel_table` — неотрицательные модули ускорения
в м/с², по `(notch_max + 1) * len(speed_grid_mps)` чисел каждый. Порядок row-major: сначала
все скорости для позиции 0, затем для 1, ..., `notch_max`; нулевая строка — нули. При
скорости между узлами — линейная интерполяция, вне сетки — крайнее значение. Положительная
позиция выбирает тягу, отрицательная — торможение; значение позиции ограничивается
`[-notch_max, notch_max]`.

`drive.adhesion_accel_mps2` — положительная верхняя граница модуля ускорения привода по
сцеплению (м/с²). `drive.traction_power_w_per_kg` — положительный предел удельной тяговой
мощности `P/m` (Вт/кг = м²/с³): при `v > 0` тяга ограничена `P/(m*v)`, при `v = 0` этот
предел не действует. Торможение ограничено сцеплением, но не тяговой мощностью. К результату
привода отдельно применяется сопротивление `c0 + c1*v + c2*v²` из секции `resistance`.
`types.load_params` отвергает неверную форму таблиц, отрицательные значения, неверную сетку
и неположительные пределы до старта ноды.

## 4. Формат метрик `tools/eval`

Одна команда → одна таблица (markdown в stdout) + `out/eval/<run>/metrics.json`:

```json
{
  "commit": "abc1234", "split": "holdout", "gnss_window_s": 5.0, "ref_point": "base_link", "created": "2026-09-25T20:00:00+03:00",
  "bags": {
    "30618_01f73500": {
      "duration_s": 1212.0, "distance_m": 5400.0, "n_matched": 11800,
      "speed_rmse": 0.0, "speed_mae": 0.0,
      "speed_bias_accel": 0.0, "speed_bias_brake": 0.0, "speed_bias_stop": 0.0, "speed_bias_cruise": 0.0,
      "drift_pct": 0.0,
      "along_mean": 0.0, "along_max": 0.0, "along_rmse": 0.0,
      "cross_mean": 0.0, "cross_max": 0.0, "cross_rmse": 0.0,
      "pos3d_rmse": 0.0, "pos3d_max": 0.0,
      "slip_flag_frac": 0.0, "crashed": false
    }
  },
  "summary": {"median": {"...": 0.0}, "worst_bag": {"speed_rmse": "30639_...", "drift_pct": "..."}}
}
```

Определения (эталон — GNSS master после фильтра выбросов; сопоставление по ближайшему
stamp, допуск 0,05 с, как у судьи):

- `ref_point` — точка эталона положения (D-077). `base_link` (по умолчанию) — по tf
  организаторов: на прямой master → rover в 9,873 м от master, если rover-fix того же момента
  (±0,06 с) даёт базу 12,436 ± 0,5 м, иначе — по курсу ближайшей пары в пределах 1 м дуги, иначе — на 9,873 м дуги дальше по треку master; высота —
  минус 3,0 м. `master` (`--ref-point master`) — сама антенна, как до D-077. Начало frame — fix
  master в обоих случаях; скорость от точки не зависит.

- `speed_*` — м/с; эталон — `hypot(ve, vn)` master/vel. Режимы по эталонному ускорению
  (сглаженному ±0,5 с — только в eval, не в ноде): разгон a > 0,2 м/с², торможение a < −0,2,
  стоянка v < 0,3 м/с, иначе — ход. `bias` — среднее (оценка − эталон) в режиме.
- `drift_pct` — |позиция в конце − эталон в конце| / пройденный эталонный путь × 100.
- `along_*`, `cross_*` — м; ошибка в проекции на карту маршрута: вдоль — разность дуг `s`,
  поперёк — расстояние оценённой точки до полилинии.
- `pos3d_*` — м, евклидово расстояние x/y/z (так сравнивает судья).
- Прогон короче 50 м пути — `drift_pct = null` (ловушка 13 `docs/data.md`).
- Главные метрики для merge (D-012): медианы `speed_rmse`, `along_rmse`, `drift_pct`.

## 5. Карта маршрута `maps/route.csv`

```
# frame: ENU, origin_lat=<>, origin_lon=<>, origin_alt=<>, source=<train bags, commit>; <описание веток>
branch,s_m,x_m,y_m,z_m
0,0.0,0.0,0.0,0.0
```

`branch` — 0 — на запад, 1 — на восток; ≥ 2 — пути конечных (петля, пути отстоя), каждая ответвляется от другой ветки или вливается в неё, стык — ближайшая точка другой ветки (`tools/pathgraph/README.md`); номера веток ≥ 2 при перестроении карты могут меняться. `s_m` строго растёт внутри
ветки; шаг ≤ 2 м. `z_m` — ENU up (м) в той же системе, что x/y: судья сравнивает x/y/z, высота на маршруте меняется на 28 м (D-024). Начало ENU карты — фиксированная точка, не зависит от прогона; перевод в
frame `map` прогона — сдвиг в `position`. GNSS → ENU — только WGS84 ECEF → ENU от начала из заголовка (`tools/pathgraph/build_route.lla_to_enu`): сферическая равнопрямоугольная проекция расходится с ней на 12 м к западному концу маршрута (D-022).

### Места остановок `maps/stops.csv` (D-034)

```
# stop places: ..., tools/pathgraph/build_stops.py
branch,s_m,n_bags
0,146.4,17
```

`branch`, `s_m` — место на карте `route.csv` (тот же `s`); `n_bags` — сколько train bag там
останавливались (справочно, ядро не читает). Файл строит `tools/pathgraph/build_stops.py` из
train, лежит рядом с `route.csv` и ставится в `share/tram_odometry/maps/` тем же `glob('maps/*.csv')`.
Отсутствие файла — не ошибка (мест нет, привязки нет). Ключи `position.stop_*`, `position.scale_*`,
`position.anchor_std_m`, `position.heading_min_base_m`, `position.heading_max_base_m`, `position.side_speed_mps`, `position.side_min_m`, `position.side_max_m`, `position.side_overrun_m` (`params.yaml`, §3) — часть контракта.
