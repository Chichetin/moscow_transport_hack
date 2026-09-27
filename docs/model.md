# Модель резервной одометрии

Состояние документа: реализация `origin/main` после #9, #12, #13, #14, #56 и
контракта фильтра D-032. Модель привода #10 и оценщик EKF #11 ещё открыты. Их проектные намерения перечислены отдельно, без
уравнений, которые можно было бы принять за работающий код.

## Входы, выходы и единицы

Единственная точка расчёта — [`Odometry.step`](../src/tram_odometry_core/tram_odometry_core/pipeline.py).
Она передаёт пару `(topic, ROS message)` от ноды или `tools/eval` в
[`Preprocessor.accept`](../src/tram_odometry_core/tram_odometry_core/preprocess/__init__.py),
а затем обрабатывает нормализованный `Sample`. Метод возвращает `Estimate` либо
`None` для отклонённого или служебного входа. В движении используются скорости двух
тележек и позиция контроллера; GNSS master fix/vel допускается только в первые
`gnss.init_window_s` секунд по `header.stamp`. Позиция контроллера уже проходит проверку
и попадает в `CommandSample`, но пока не участвует в расчёте скорости. GNSS rover в ноде
подписан, но ядро его не использует. Выходы ноды — `/result/velocity` в м/с и `/result/position` в метрах,
ENU прогона; их заполнение выполняют
[`velocity_msg` и `position_msg`](../src/tram_odometry/tram_odometry/odometry_node.py).

Внутренние величины `Estimate`: скорость `speed` (м/с), путь `distance` (м), положение
`x/y/z` (м), курс `yaw` (рад), дисперсия скорости и ковариация плоскости. Поля
`accel` и `accel_model` пока всегда равны нулю; это видно в
[`Odometry._estimate`](../src/tram_odometry_core/tram_odometry_core/pipeline.py).

## Скорость и путь: работающий бейзлайн

[`Preprocessor._wheel`](../src/tram_odometry_core/tram_odometry_core/preprocess/__init__.py)
единожды переводит скорость `u_i` тележки из км/ч в м/с и применяет её масштаб:

```text
v_i = u_i · input.wheel_speed_scale · vehicle.wheel_scale_i.
```

До оценщика предобработчик пропускает только возрастающие stamp каждой тележки
и отсекает выбросы по пределу ускорения колеса
`input.max_wheel_accel_mps2`; деталь поведения описана в решении D-031.
Порог `8,0 м/с²` отсекает грубый одиночный выброс, но допускает умеренные
изменения обоих колёс к совместной проверке (D-054). Если колёса с одинаковым
stamp расходятся после согласованной пары в противоположные стороны, а середина
пары движется в пределах ускорения по сцеплению, детектор временно доверяет обоим.
При подозрительном первом сообщении новой пары pipeline ждёт вторую тележку не
дольше измеренного периода пары; при её пропуске снова использует доступное колесо.
После проверки доверия [`Odometry._advance`](../src/tram_odometry_core/tram_odometry_core/pipeline.py)
считает взвешенное среднее; при отсутствии доверенных тележек оставляет прежнюю
скорость. Время состояния не откатывается при запоздавшем сообщении:

```text
v = Σ(w_i v_i) / Σw_i,  если Σw_i > 0; иначе v = v_prev;
dt = max(t, t_prev) − t_prev;  ds = v · dt;  distance += ds.
```

Повторные и не возрастающие stamp потоков тележек и контроллера отклоняются в
[`Preprocessor._fresh`](../src/tram_odometry_core/tram_odometry_core/preprocess/__init__.py).
При молчании обеих тележек дольше `input.stale_timeout_s` (с `drive.use_model: true`, #10, D-036)
скорость **прогнозируется моделью привода**: `v = max(0, v_prev + a_model·dt)`, где
`a_model = model_accel(notch(t − response_delay_s), v_prev)` — таблица D-033 с пределами
сцепления и мощности минус сопротивление Дэвиса; то же `a_model` идёт в `SlipDetector` как
прогноз и публикуется в `Estimate.accel_model`. Пока тележки живы, скорость берётся из них;
модель входит в оценку только через детектор (сведение прогноза и измерений — EKF #11).
`Estimate.accel` по-прежнему 0.

## Проскальзывание и отказ тележки

[`SlipDetector.update`](../src/tram_odometry_core/tram_odometry_core/slip/detector.py)
сравнивает две свежие тележки с допуском и прогнозом; в текущем pipeline
`accel_model = 0`:

```text
tol = slip.front_rear_threshold_mps + slip.model_residual_threshold_mps2 · dt;
v_pred = v_prev + accel_model · dt;
dev_i = |v_i − v_pred|.
```

Если обе скорости расходятся не более чем на `tol`, обе получают доверие 1. При
большем расхождении и ровно нулевой скорости одной тележки нулевая получает
доверие 0 и флаг отказа. Если прогноза `pred` ещё нет, обе получают 0,5;
иначе отбрасывается тележка с большим `dev_i`, при равенстве — 0,5/0,5.
Возраст тележки считается относительно **последнего stamp тележек**, а не
текущего входа контроллера: при возрасте больше `input.stale_timeout_s` доверие
становится 0; флаг отказа появляется только если другая тележка продолжает
сообщать. `adhesion_est` пока `None`. Это эвристика на
расхождение и застревание датчика; согласное проскальзывание обеих тележек она
не обнаруживает.

## Положение: работающая карта и запасной режим

[`Odometry._on_fix`](../src/tram_odometry_core/tram_odometry_core/pipeline.py)
передаёт разрешённые фиксы в
[`PathTracker.on_fix`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py).
После фильтра расстояния до карты `position.fix_gate_m` начало ENU прогона берётся
по первому GBAS fix (если он есть в окне), иначе по первому валидному fix. Карта
переводится между двумя ENU через WGS84 ECEF в
[`PathTracker._set_origin`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py).
Принятый fix окна обновляет якорь `(ветка, s_anchor, distance_anchor)` по
ближайшему сегменту карты в
[`PathTracker._nearest`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py).
Выбросы за `fix_gate_m` и fix обычного статуса после GBAS не обновляют якорь.

После окна [`PathTracker.advance`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py)
двигается только вперёд по выбранной ветке с текущей оценкой масштаба `scale` и
интерполирует `x/y/z/yaw` в
[`PathTracker._at`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py):

```text
s = s_anchor + scale · (distance − distance_anchor);
σ_cross² = position.cross_std_m²;
σ_along² = σ_cross² + var0 + [position.along_drift_frac · scale · (distance − distance_anchor)]²;
Σ_xy = R(yaw) · diag(σ_along², σ_cross²) · R(yaw)ᵀ.
```

Высота карты сдвигается на медиану разности высот fix и карты за окно выставки.
При достижении конца ветки продолжение ищется в пределах `position.join_m`.
После окна GNSS и стоянки не короче `position.stop_min_s` со скоростью ниже
`position.stop_speed_mps`, pipeline один раз предлагает карте из
[`stops.csv`](../src/tram_odometry/maps/stops.csv) поправить `s` к ближайшему
месту, если оно не дальше `position.stop_snap_max_m`.
Поправка взвешенная: `gain = var/(var + stop_std_m²)`, затем `s += gain·(s_stop-s)`;
остаточная дисперсия `var` умножается на `1-gain`. Если две принятые стоянки
находятся на одной ветке и расстояние между ними не меньше
`position.scale_min_arc_m`, отношение пути карты к пути колёс обновляет масштаб:

```text
ratio = clip((s_stop − s_stop_prev) / (distance − distance_prev), 1 ± scale_max_dev);
scale += scale_alpha · (ratio − scale).
```

Реализуют
[`Odometry._on_standstill`](../src/tram_odometry_core/tram_odometry_core/pipeline.py),
[`PathTracker.on_stop`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py)
и [`PathTracker._update_scale`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py).
Если карта отключена либо ещё нет якоря, [`Odometry._advance`](../src/tram_odometry_core/tram_odometry_core/pipeline.py)
интегрирует прямую по курсу из наибольшей скорости GNSS master в окне:

```text
x += ds · cos(yaw);  y += ds · sin(yaw).
```

Курс выбирает [`Odometry._on_vel`](../src/tram_odometry_core/tram_odometry_core/pipeline.py)
через `atan2(v_north, v_east)` после валидного fix. Он не задаёт положение
на карте: там курс берётся из касательной ветки.

## Привод, продольная динамика и EKF: состояние реализации

Схема параметров уже задана в [`DriveParams`, `ResistanceParams` и `FilterParams`](../src/tram_odometry_core/tram_odometry_core/types.py):
таблицы неотрицательных модулей ускорения по позиции контроллера и скорости,
пределы сцепления/удельной мощности, коэффициенты сопротивления и шумы фильтра.
[`_validate_drive`](../src/tram_odometry_core/tram_odometry_core/types.py) проверяет форму
таблиц, нулевую строку и допустимость сетки. В `params.yaml` таблицы заполнены по
train в #9 (D-033), но ещё не участвуют в `Odometry.step`. Модули
[`dynamics`](../src/tram_odometry_core/tram_odometry_core/dynamics/__init__.py) и
[`estimator`](../src/tram_odometry_core/tram_odometry_core/estimator/__init__.py) ещё
не содержат расчётных функций. Поэтому в текущем ядре нет ни модели момента,
ни зависимости момента от скорости вала, ни уравнения продольной динамики с
массой/уклоном, ни EKF с состоянием, якобианом и обновлением измерениями.
`mass_kg`, таблицы и коэффициенты сопротивления сейчас не участвуют в `Odometry.step`.

Контракт D-032 уже задаёт смысл `q_accel`, `r_wheel`, `q_bias`,
`initial_bias_var` и `nis_gate`, а тип `Estimate` содержит необязательное поле
`filter_diagnostics`. До реализации #11 поле остаётся `None`, а значения шумов
не участвуют в расчёте.

#10 должен реализовать преобразование табличного ускорения от тяги/тормоза и
сопротивления в `accel_model`; #11 — прогноз скорости и обновление по двум
тележкам с доверием `SlipState`. Документ нужно дополнить ссылками на **их реальные
функции и уравнения** после merge этих issue. Уклон и обратное вычисление момента
не определены текущими входами: без IMU и данных о передаче их нельзя честно
назвать оцененными величинами. На выходе сейчас оцениваются только скорость,
путь и положение, а не момент.
