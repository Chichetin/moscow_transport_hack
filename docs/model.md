# Модель резервной одометрии

Состояние документа: реализация после #9, #10, #11, #12, #13, #14, #56 и
контракта фильтра D-032. Привод и фильтр связаны в `Odometry.step` (D-036, D-057).

## Входы, выходы и единицы

Единственная точка расчёта — [`Odometry.step`](../src/tram_odometry_core/tram_odometry_core/pipeline.py).
Она передаёт пару `(topic, ROS message)` от ноды или `tools/eval` в
[`Preprocessor.accept`](../src/tram_odometry_core/tram_odometry_core/preprocess/__init__.py),
а затем обрабатывает нормализованный `Sample`. Метод возвращает `Estimate` либо
`None` для отклонённого или служебного входа. В движении используются скорости двух
тележек и позиция контроллера; GNSS master fix/vel допускается только в первые
`gnss.init_window_s` секунд по `header.stamp`. Позиция контроллера проходит проверку
и задаёт ускорение модели после задержки `drive.response_delay_s`. GNSS rover в ноде
подписан, но ядро его не использует. Выходы ноды — `/result/velocity` в м/с и `/result/position` в метрах,
ENU прогона; их заполнение выполняют
[`velocity_msg` и `position_msg`](../src/tram_odometry/tram_odometry/odometry_node.py).

Внутренние величины `Estimate`: скорость `speed` (м/с), путь `distance` (м), положение
`x/y/z` (м), курс `yaw` (рад), дисперсия скорости и ковариация плоскости. Поля
`accel` — оценка ускорения фильтра (модель плюс смещение),
`accel_model` — прогноз модели привода; это видно в
[`Odometry._estimate`](../src/tram_odometry_core/tram_odometry_core/pipeline.py).

## Скорость и путь: работающий фильтр

[`Preprocessor._wheel`](../src/tram_odometry_core/tram_odometry_core/preprocess/__init__.py)
единожды переводит скорость `u_i` тележки из км/ч в м/с и применяет её масштаб:

```text
v_i = u_i · input.wheel_speed_scale · vehicle.wheel_scale_i.
```

До оценщика предобработчик пропускает только возрастающие stamp каждой тележки
и отсекает выбросы по пределу ускорения колеса
`input.max_wheel_accel_mps2`; деталь поведения описана в решении D-031.
[`SpeedFilter`](../src/tram_odometry_core/tram_odometry_core/estimator/__init__.py)
хранит состояние `x = [v, b]`: скорость и смещение ускорения от модели.
Прогноз на `dt > 0` и его ковариация:

```text
v⁻ = max(0, v + (a_model + b) · dt);  b⁻ = b;
F = [[1, dt], [0, 1]];
Q = [[q_accel·dt + q_bias·dt³/3, q_bias·dt²/2],
     [q_bias·dt²/2,            q_bias·dt]];
P⁻ = F P Fᵀ + Q.
```

Для нового колеса дисперсия `R = r_wheel/trust` (с поправкой относительного масштаба
тележки и дополнительным `q_accel·age` при задержке). Свежесть нового измерения определяется относительно последнего stamp колёс
(`stale_timeout_s`), поэтому опережающий контроллер не выбрасывает живые тележки.
Запоздалое измерение сравнивается с состоянием на старом stamp через `H = [1, −age]`, но состояние не откатывается.
`innovation = z − Hx`, `S = HPHᵀ + R`, `NIS = innovation²/S`;
при `NIS ≤ nis_gate` используется усиление `K = PHᵀ/S` и форма Джозефа
`P = (I−KH)P(I−KH)ᵀ + KRKᵀ`. Два согласованных нуля тележек подтверждают
резкую остановку; при малой скорости отрицательное смещение сбрасывается,
а первый допустимый разгон начинает новый участок движения. Фильтр медленно
оценивает относительный масштаб согласованной пары, общий масштаб уточняет
привязка к остановкам D-034. NIS и решение по новому колесу доступны в
`Estimate.filter_diagnostics` (D-032, D-057).

[`Odometry._advance`](../src/tram_odometry_core/tram_odometry_core/pipeline.py)
передаёт доверие `SlipState` фильтру и интегрирует его скорость:
`dt = max(t, t_prev) − t_prev; ds = v · dt; distance += ds`.
Повторные и не возрастающие stamp потоков отклоняются в
[`Preprocessor._fresh`](../src/tram_odometry_core/tram_odometry_core/preprocess/__init__.py).
При молчании колёс остаётся прогноз `a_model + b`. `a_model` вычисляет
[`model_accel`](../src/tram_odometry_core/tram_odometry_core/dynamics/__init__.py)
по позиции контроллера с задержкой `drive.response_delay_s`, таблицам D-033,
пределам сцепления и мощности и сопротивлению Дэвиса (D-036). Прогноз подаётся
в `SpeedFilter.predict` и `SlipDetector.update`, а в `Estimate.accel_model`
публикуется его значение до поправки смещения.

## Проскальзывание и отказ тележки

[`SlipDetector.update`](../src/tram_odometry_core/tram_odometry_core/slip/detector.py)
сравнивает две свежие тележки с допуском и прогнозом модели привода:

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

## Привод и продольная динамика

[`model_accel`](../src/tram_odometry_core/tram_odometry_core/dynamics/__init__.py)
интерполирует идентифицированные на train таблицы ускорения по позиции
контроллера и скорости (D-033, D-036). При тяге и торможении знак ускорения
задаёт команда; от результата вычитается сопротивление Дэвиса
`c0 + c1·v + c2·v²`, а модуль ограничен сцеплением и удельной мощностью.
`Odometry._notch_at` берёт последнюю позицию, задержанную на
`drive.response_delay_s = 0,3 с`. Если `drive.use_model=false`, прогноз модели
равен нулю; фильтр всё равно обновляется колёсами. При скачке часов входа
более `input.max_stamp_jump_s` фильтр переносит время состояния без
интегрирования искусственного участка (D-043).

`mass_kg` пока не участвует в расчёте: таблицы задают ускорение, а не силу.
Уклон и обратное вычисление момента не определены текущими входами: без IMU,
данных о передаче и токе их нельзя честно назвать оцененными величинами.
На выходе оцениваются скорость, ускорение фильтра, путь и положение, но не момент.
