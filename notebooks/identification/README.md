# Идентификация модели

Офлайн, только **train** и `no_gnss_train` (`tools/eval/splits.yaml`). Результат — числа в
`src/tram_odometry/config/params.yaml` и графики в `docs/` для `docs/model.md` и питча.
Каждое число в `params.yaml` сопровождается командой, которой получено.

Что идентифицируется (план, пакеты ID-*):

- масштаб колеса по трамваю (км/ч → м/с × поправка износа) — по GNSS vel на движении;
- ускорение по позиции контроллера и скорости `a_drive(notch, v)`: тяга и торможение, вне
  проскальзывания (тележки согласны), с учётом задержки отклика привода;
- сопротивление движению `c0 + c1 v + c2 v²` — выбег на нейтрали;
- шум измерений тележек, шум процесса — для фильтра.

Скрипты — `*.py` с `argparse` (воспроизводимо из командной строки), ноутбуки — только для
просмотра. Зависимости — `requirements-dev.txt` (scipy разрешён здесь, не в ядре).

## Модель привода (#9, D-033)

```bash
.venv/bin/python notebooks/identification/identify.py --check-split holdout      # отчёт, абляция уклона, графики, ~2 мин
.venv/bin/python notebooks/identification/identify.py --write-params             # + числа в params.yaml (без уклона)
.venv/bin/python notebooks/identification/identify.py --write-params --grade     # таблицы для модели с уклоном онлайн
.venv/bin/python -m pytest notebooks/identification/tests                        # синтетика с известными параметрами
```

- `drive_ident.py` — извлечение по bag и подгонки: скорость тележек в м/с на сетке 10 Гц, ускорение
  центральной разностью ±0,2 с, позиция контроллера с задержкой, уклон dz/ds по высоте GNSS вдоль
  доплеровской дуги (±30 м). Модель — схема D-029:
  `a = sign(n)·A(|n|, v) − (c0 + c1 v + c2 v²)` (с `--grade` ещё `− g·уклон`); тяга ограничена
  `P/(m v)` и сцеплением.
- `identify.py` — по train: масштаб колёс, задержка (скоринг по сетке 0–1 с), сопротивление по выбегу,
  таблицы позиция × v, пределы; приёмка на окнах 10 с; `--write-params` меняет только значения ключей.
- Числа и приёмка — `docs/verification/2026-09-25-drive-ident.md`; графики — `figures/drive_accel_table.png`
  (a(позиция, v): таблица и медианы train) и `figures/resistance.png` (сопротивление на выбеге).

Главное: модель по позиции контроллера в окне 10 с даёт медианный RMSE скорости 0,27 м/с против
1,70 у «a = 0» (train; на holdout 0,29). Задержка привода 0,3 с. В `params.yaml` — подгонка без уклона,
потому что контракт `model_accel(notch, v)` уклона не получает. Если ядро получит уклон из карты,
`--grade` даст 0,20.

## Остановки по колёсам против карты остановок (#58, D-039)

```bash
.venv/bin/python notebooks/identification/stops_vs_speed.py --split train
.venv/bin/python notebooks/identification/stops_vs_speed.py --split holdout      # только отчёт
.venv/bin/python -m pytest notebooks/identification/tests/test_stops_vs_speed.py
```

Остановки по `v < 0,1 м/с` ≥ 3 с (без GNSS в детекции) сравнены с местами `stops.csv`
(#56/D-034): 84,3 % train / 85,8 % holdout ближе 20 м — порядок величины `stop_snap_max_m`
не противоречит числу. Отчёт — `docs/research/2026-09-25-stops-vs-speed.md`, график —
`docs/data/stops_vs_speed_hist.png`.
