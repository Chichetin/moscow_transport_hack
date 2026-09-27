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
и задаёт ускорение модели после задержки `drive.response_delay_s`. GNSS rover fix в том же окне
даёт только курс master → rover для выбора ветки выставки (D-062). Выходы ноды — `/result/velocity` в м/с и `/result/position` в метрах,
плоская сетка MGRS `37UCB` и высота над эллипсоидом (ядро считает в ENU прогона и переводит
на выходе, D-082); их заполнение выполняют
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
и конечные числа (D-031). Отрицательная скорость становится ровно 0 (откат или шум
у стоянки, D-064). Отсчёт выше `input.max_wheel_speed_mps` отбрасывается всегда,
в том числе первый в потоке (D-056). Шаг быстрее `input.max_wheel_accel_mps2`
отбрасывается как сбой датчика. Исключений два, оба ради залипания в 0 (ловушка 8):
(а) падение ровно в 0 гейт не проверяет — его разбирает детектор проскальзывания (D-056);
(б) выход из ровно 0 принимается, только если вторая тележка свежая (не старше
`input.stale_timeout_s`) и показывает ту же скорость в пределах
`slip.front_rear_threshold_mps` (D-056), иначе это выброс на стоянке. Отрицательный
отсчёт, обнулённый в 0, исключениями не пользуется: гейт к нему применяется (D-064).
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

Измерение тележки `i` с возрастом `age = t_filter − t_i ≥ 0` и относительным
масштабом `k_front = 1 − δ`, `k_rear = 1 + δ`:

```text
z = v_i / k_i + a_model · age;   R = r_wheel / (trust · k_i²) + q_accel · age.
```

Доверие `trust = 0` пропускает отсчёт. Свежесть нового измерения определяется относительно последнего stamp колёс
(`stale_timeout_s`), поэтому опережающий контроллер не выбрасывает живые тележки.
Запоздалое измерение сравнивается с состоянием на старом stamp через `H = [1, −age]`, но состояние не откатывается.
`innovation = z − Hx`, `S = HPHᵀ + R`, `NIS = innovation²/S`;
при `NIS ≤ nis_gate` используется усиление `K = PHᵀ/S` и форма Джозефа
`P = (I−KH)P(I−KH)ᵀ + KRKᵀ`. Два нуля тележек со stamp в пределах 0,02 с
подтверждают резкую остановку: дисперсия скорости раздувается ровно настолько, чтобы
отсчёт прошёл `nis_gate`. Если на принятом отсчёте и сам отсчёт, и оценка ниже `max(position.stop_speed_mps,
0,5 м/с)`, а `a_model ≤ 0`, отрицательное смещение `b` сбрасывается в 0. Первый допустимый разгон после подтверждённой остановки
начинает новый участок движения. Фильтр медленно оценивает относительный масштаб пары
`δ`: `δ += 0,02 · (clip((v_r − v_f)/(v_r + v_f), ±0,03) − δ)`, но только на
согласованных парах: stamp в пределах 0,02 с, обе скорости не ниже 2 м/с, разница не
больше 0,5 м/с, доверие не ниже 0,8.
Общий масштаб уточняет привязка к остановкам D-034. NIS и решение по новому колесу доступны в
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

Здесь `v_prev` — скорость фильтра после последнего принятого отсчёта колеса, без
прогноза по модели (иначе ускорение учлось бы дважды, `Odometry._advance`), а `dt` —
собственный шаг детектора между последними stamp тележек. Кроме того, у каждой тележки
отмечаются **скачки** (D-054): шаг, собственное ускорение которого отличается от
прогноза модели больше, чем на `slip.noise_accel_mps2`:

```text
jump_i:  |(v_i − v_i,prev)/(t_i − t_i,prev) − accel_model| > slip.noise_accel_mps2.
```

Скачок считается недавним `slip.noise_hold_s`, направление (вверх или вниз)
запоминается. Правила проверяются по порядку, срабатывает первое:

1. Обе тележки расходятся не больше чем на `tol` — обе получают доверие 1.
2. Одна тележка ровно 0, другая движется. Нулевая винится (доверие 0, флаг отказа —
   залипание, ловушка 8), если прогноза ещё нет, привод тянет (`accel_model > 0`) или
   движущаяся согласна с `v_pred` в пределах `tol` (D-027, D-045). Иначе ноль честный,
   а движущаяся тележка — выброс; он уходит в правила ниже.
3. Недавние скачки обеих тележек в **одну** сторону и ни одного встречного —
   юз или боксование всего вагона. Доверие 0/0, оба флага; фильтр едет по модели
   привода (D-054).
4. Прогноза ещё нет или есть недавние скачки обеих в **противоположные** стороны —
   противофазный шум, среднее верно: доверие 0,5/0,5.
5. Иначе доверие 1 получает тележка с меньшим `dev_i`. Другой ставится флаг
   проскальзывания, если её `dev_i > tol`. При равенстве — 0,5/0,5.

Возраст тележки считается относительно **последнего stamp тележек**, а не
текущего входа контроллера: при возрасте больше `input.stale_timeout_s` доверие
становится 0; флаг отказа появляется только если другая тележка продолжает
сообщать. Скачок stamp тележки назад больше `input.max_stamp_jump_s` —
ресинхронизация часов (D-043): её история и скачки забываются. `adhesion_est` пока
`None`. Синфазный юз обеих тележек распознаётся только по скачкам (правило 3). Плавное
согласное проскальзывание обеих, медленнее `slip.noise_accel_mps2`, от хода вагона
не отличить без IMU. Это ограничение записано в `docs/roadmap.md`.

## Положение: работающая карта и запасной режим

[`Odometry._on_fix`](../src/tram_odometry_core/tram_odometry_core/pipeline.py)
передаёт разрешённые фиксы в
[`PathTracker.on_fix`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py).
После фильтра расстояния до карты `position.fix_gate_m` начало ENU прогона берётся
по первому GBAS fix (если он есть в окне), иначе по первому валидному fix. Карта
переводится между двумя ENU через WGS84 ECEF в
[`PathTracker._set_origin`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py).
Принятый fix master окна обновляет якорь `(ветка, s_anchor, distance_anchor)` в
[`PathTracker._locate`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py).
Rover стоит впереди master по ходу (ловушка 15), поэтому, если в окне есть оба fix,
курс `h = rover − master` в ENU прогона выбирает ветку: якорь — ближайшая точка
сегмента с `t·h > 0`, то есть ветки, идущей по курсу, в пределах `fix_gate_m`
([`PathTracker._nearest`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py)
с `heading`). Так на стоянке различаются пути двух направлений и тупик у конечной.
Курса нет, если база `|h|` вне [`position.heading_min_base_m`,
`position.heading_max_base_m`] или rover обычного статуса при GBAS-начале. Тогда, как и
при курсе без ветки по нему в пределах `fix_gate_m`, якорь — просто ближайший сегмент.
Rover-fix, пришедший после якоря, переносит якорь, только если текущая ветка идёт
против курса и ближайшая ветка по курсу — другая
([`PathTracker.on_rover`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py),
D-062). Rover никогда не задаёт начало ENU и точку якоря.
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
При достижении конца ветки продолжение ищется в пределах `position.join_m`; если конец —
тупик, а до него от ветки отходит путь с продолжением, по умолчанию берётся он (петля
западной конечной). Второй, тупиковый, путь, отходящий от ветки, — «боковой»: въезд в
депо, ветка 4 от ветки 0. На него
[`PathTracker._take_side`](../src/tram_odometry_core/tram_odometry_core/position/tracker.py)
переводит трамвай, если в окне [`position.side_min_m`, `position.side_max_m`] пути после
его начала скорость фильтра выше `position.side_speed_mps`: на петлю с остановкой
трамваи идут медленно, в депо разгоняются (D-074). Решение — по текущей скорости, без
будущего; путь и дисперсия вдоль пути сохраняются. Если путь за тупиком боковой ветки
превысил `position.side_overrun_m`, переход был ошибкой: трекер возвращает якорь, дисперсию и
последнюю привязку к остановке, как будто перехода не было.
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

Полная формула прогноза с задержкой, сцеплением и мощностью:

```text
n = notch(t − drive.response_delay_s);
A = interp(v, speed_grid_mps, table[min(|n|, notch_max)])      # traction_ при n ≥ 0, brake_ при n < 0
A = min(A, adhesion_accel_mps2);  A = min(A, traction_power_w_per_kg / v)   # второе — только n > 0, v > 0
a_model = sign(n) · A − (c0 + c1·v + c2·v²)     # при n = 0 строка таблицы нулевая: остаётся сопротивление
```

### Момент → ускорение → скорость

Цепочка физики привода и то, что из неё наблюдаемо:

```text
M_motor(n, v)  →  F_wheel = M_motor · i · η / r_wheel  →  a = F_wheel / m − a_res(v) − g·sin θ
              →  v(t) = ∫ a dt  →  s(t) = ∫ v dt.
```

Здесь `i` — передаточное число, `η` — КПД, `r_wheel` — радиус колеса, `m` — масса,
`θ` — уклон. В данных нет ни одной из этих величин, нет тока двигателя и нет IMU.
Поэтому звено «момент → сила» свёрнуто в одну идентифицируемую функцию

```text
A(|n|, v) = M_motor(n, v) · i · η / (r_wheel · m),
```

то есть момент, приведённый к ускорению вагона. Её таблицы `traction_accel_table` и
`brake_accel_table` подогнаны офлайн по train к ускорению колёс, масштаб которых
выверен по GNSS (D-033, `notebooks/identification/identify.py`). Предел сцепления
`adhesion_accel_mps2` физически равен `μ·g`; в `params.yaml` это максимум таблиц (D-070).
Предел мощности даёт `P/(m·v)`. Звено «ускорение → скорость → путь» — это прогноз фильтра
`v⁻ = max(0, v + (a_model + b)·dt)` и интеграл `ds = v·dt` (раздел
«Скорость и путь»). Смещение `b` впитывает то, что таблица не видит: уклон, загрузку
вагона и отличие конкретного трамвая.

Допущение: момент в Н·м по этим входам не наблюдаем. `i`, `η`, `r_wheel` и `m` входят
только комбинацией `i·η/(r_wheel·m)`. Поэтому `mass_kg` в расчёте не участвует и помечен в
`params.yaml` как заглушка, а модель выдаёт приведённое ускорение, а не момент.
Если появятся паспорт передачи (`i`, `η`), радиус колеса и масса с загрузкой, `A(|n|, v)`
пересчитывается в `M_motor(n, v) = A · r_wheel · m / (i · η)` без изменения фильтра; ток
двигателя позволил бы проверить эту оценку независимо. На выходе оцениваются скорость, ускорение
фильтра, путь и положение.
