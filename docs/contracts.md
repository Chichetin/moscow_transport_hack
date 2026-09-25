# Контракты — v1

Единственная точка стыковки модулей. Всё, что здесь описано, меняется **только отдельным
PR** с перечнем потребителей в описании; merge — после апрува каждого потребителя (владельца
issue соседней `area:`). Владельца у контракта нет. Статус v1: типы и загрузка параметров
реализованы в `tram_odometry_core/types.py`, сигнатура `step` — в `pipeline.py`.

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
- QoS — по умолчанию (reliable, depth 10); судья подписан best-effort, это совместимо.

`/result/position` (`nav_msgs/msg/Odometry`):

| Поле | Значение |
|---|---|
| `header.frame_id` | `map` (`frames.map`) — локальная ENU, метры, начало — первая валидная точка GNSS master в окне выставки (**уточнить у организаторов**, `HANDOFF.md`) |
| `child_frame_id` | `base_link` (`frames.base`) |
| `pose.pose.position` | `x` — восток, `y` — север, `z` = 0, м |
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
class Estimate:
    t: float                  # = t входа, вызвавшего обновление
    speed: float              # м/с, >= 0
    speed_var: float          # (м/с)^2
    accel: float              # м/с^2, оценка
    accel_model: float        # м/с^2, прогноз модели привода
    distance: float           # м, путь от начала прогона
    x: float; y: float        # м, frame map
    yaw: float                # рад, ENU
    pos_cov: tuple[float, float, float]   # var_x, var_y, cov_xy
    slip: SlipState
    gnss_used: bool
```

Модули и их публичные функции (одна `area:` — один модуль):

| Модуль | Публичное | Контракт поведения |
|---|---|---|
| `preprocess` | `Preprocessor(params).accept(raw) -> Sample \| None` | `raw` — сырой вход (км/ч, notch как есть); возвращает нормализованный `Sample` или `None` (выброс, NaN, stamp из прошлого сверх допуска, GNSS вне окна) |
| `dynamics` | `model_accel(notch: int, speed: float, params) -> float` | чистая функция, м/с²; без состояния |
| `slip` | `SlipDetector(params).update(front, rear, accel_model, est) -> SlipState` | `front`/`rear` — последний `WheelSample` или `None` (молчит) |
| `estimator` | `SpeedFilter(params).predict(t, accel_model)`, `.update(sample: WheelSample, trust: float)`, `.state() -> (speed, speed_var, accel)` | монотонное время внутри; `t` меньше текущего — без отката |
| `position` | `PathTracker(params, route).init(fixes, vel) -> bool`, `.advance(distance) -> (x, y, yaw, pos_cov)` | до успешного `init` — начало координат и курс 0, `gnss_used=False` |
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

## 4. Формат метрик `tools/eval`

Одна команда → одна таблица (markdown в stdout) + `out/eval/<run>/metrics.json`:

```json
{
  "commit": "abc1234", "split": "holdout", "gnss_window_s": 5.0, "created": "2026-09-25T20:00:00+03:00",
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
# frame: ENU, origin_lat=<>, origin_lon=<>, source=<train bags, commit>
branch,s_m,x_m,y_m
0,0.0,0.0,0.0
```

`branch` — 0/1 для двух направлений (или отдельная ветка петли); `s_m` строго растёт внутри
ветки; шаг ≤ 2 м. Начало ENU карты — фиксированная точка, не зависит от прогона; перевод в
frame `map` прогона — сдвиг в `position`.
