# Модель резервной одометрии

Состояние документа: реализация `origin/main` после #12 и #14. Модель привода #10 и
оценщик EKF #11 ещё открыты. Их проектные намерения перечислены отдельно, без
уравнений, которые можно было бы принять за работающий код.

## Входы, выходы и единицы

Единственная точка расчёта — [`Odometry.step`](../src/tram_odometry_core/tram_odometry_core/pipeline.py).
Она принимает пару `(topic, ROS message)` от ноды или `tools/eval` и возвращает
`Estimate` либо `None` для отклонённого входа. В движении используются скорости двух
тележек и позиция контроллера; GNSS master fix/vel допускается только в первые
`gnss.init_window_s` секунд по `header.stamp`. Сейчас позиция контроллера вызывает
шаг расчёта, но её значение **не читается**. GNSS rover в ноде подписан, но ядро его
не использует. Выходы ноды — `/result/velocity` в м/с и `/result/position` в метрах,
ENU прогона; их заполнение выполняют
[`velocity_msg` и `position_msg`](../src/tram_odometry/tram_odometry/odometry_node.py).

Внутренние величины `Estimate`: скорость `speed` (м/с), путь `distance` (м), положение
`x/y/z` (м), курс `yaw` (рад), дисперсия скорости и ковариация плоскости. Поля
`accel` и `accel_model` пока всегда равны нулю; это видно в
[`Odometry._estimate`](../src/tram_odometry_core/tram_odometry_core/pipeline.py).

## Скорость и путь: работающий бейзлайн

[`Odometry._on_wheel`](../src/tram_odometry_core/tram_odometry_core/pipeline.py)
единожды переводит скорость `u_i` тележки из км/ч в м/с и применяет её масштаб:

```text
v_i = u_i · input.wheel_speed_scale · vehicle.wheel_scale_i.
```

После проверки свежести и доверия [`Odometry._advance`](../src/tram_odometry_core/tram_odometry_core/pipeline.py)
считает взвешенное среднее; при отсутствии доверенных тележек оставляет прежнюю
скорость. Время состояния не откатывается при запоздавшем сообщении:

```text
v = Σ(w_i v_i) / Σw_i,  если Σw_i > 0; иначе v = v_prev;
dt = max(t, t_prev) − t_prev;  ds = v · dt;  distance += ds.
```

Потоковые повторы и не возрастающий stamp отклоняются в
[`Odometry._fresh`](../src/tram_odometry_core/tram_odometry_core/pipeline.py).
При молчании обеих тележек это **удержание скорости**, а не прогноз от тяги.
Ускорение ни из разности скоростей, ни из команды контроллера сейчас не вычисляется.

## Проскальзывание и отказ тележки

[`SlipDetector.update`](../src/tram_odometry_core/tram_odometry_core/slip/detector.py)
сравнивает две свежие тележки с допуском и прогнозом; в текущем pipeline
`accel_model = 0`:

```text
tol = slip.front_rear_threshold_mps + slip.model_residual_threshold_mps2 · dt;
v_pred = v_prev + accel_model · dt;
dev_i = |v_i − v_pred|.
```

Если обе скорости расходятся не более чем на `tol`, обе получают доверие 1. Если
одна ровно нулевая, а другая движется, нулевой даётся 0 и флаг отказа. Иначе
отбрасывается тележка с большим `dev_i`; при равенстве — 0,5/0,5. Тележка старше
`input.stale_timeout_s` получает доверие 0; флаг отказа появляется только если
другая продолжает сообщать. `adhesion_est` пока `None`. Это эвристика на
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
Каждый fix окна обновляет якорь `(ветка, s_anchor, distance_anchor)` по ближайшему
сегменту карты в [`PathTracker._nearest`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py).

После окна [`PathTracker.advance`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py)
двигается только вперёд по выбранной ветке и интерполирует `x/y/z/yaw` в
[`PathTracker._at`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py):

```text
s = s_anchor + (distance − distance_anchor);
σ_cross² = position.cross_std_m²;
σ_along² = σ_cross² + [position.along_drift_frac · (distance − distance_anchor)]²;
Σ_xy = R(yaw) · diag(σ_along², σ_cross²) · R(yaw)ᵀ.
```

Высота карты сдвигается на медиану разности высот fix и карты за окно выставки.
При достижении конца ветки продолжение ищется в пределах `position.join_m`.
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
таблиц, нулевую строку и допустимость сетки. В `params.yaml` обе таблицы пока
содержат только нули. Модули
[`dynamics`](../src/tram_odometry_core/tram_odometry_core/dynamics/__init__.py) и
[`estimator`](../src/tram_odometry_core/tram_odometry_core/estimator/__init__.py) ещё
не содержат расчётных функций. Поэтому в текущем ядре нет ни модели момента,
ни зависимости момента от скорости вала, ни уравнения продольной динамики с
массой/уклоном, ни EKF с состоянием, якобианом и обновлением измерениями.
`mass_kg` и таблицы сейчас не участвуют в `Odometry.step`.

#10 должен реализовать преобразование табличного ускорения от тяги/тормоза и
сопротивления в `accel_model`; #11 — прогноз скорости и обновление по двум
тележкам с доверием `SlipState`. Документ нужно дополнить ссылками на **их реальные
функции и уравнения** после merge этих issue. Уклон и обратное вычисление момента
не определены текущими входами: без IMU и данных о передаче их нельзя честно
назвать оцененными величинами. На выходе сейчас оцениваются только скорость,
путь и положение, а не момент.
