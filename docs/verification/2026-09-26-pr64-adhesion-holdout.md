# Сцепление 1,60 и запас от уклона после модели привода в pipeline (PR #64, D-070)

- Ветка `worktree-9-ident-nits`, commit `5bf86b0` (merge `origin/main` `5a4b2fd` в PR #64); база — `origin/main` `5a4b2fd`
- Машина: Windows 10 (`MINGW64_NT-10.0-17763`), nproc = 2, Python 3.14.6, numpy 2.5.3
- Данные: `tools/eval/splits.yaml` — train 51 bag, holdout 26 bag

## 1. Идентификация заново: таблицы те же

```bash
.venv/Scripts/python notebooks/identification/identify.py                           # train, без записи params
.venv/Scripts/python notebooks/identification/identify.py --check-split holdout      # только проверка
```

Таблицы `traction_accel_table`, `brake_accel_table` из `out/ident/<commit>/ident.json` совпадают с
`params.yaml` в `origin/main` поэлементно. Максимум тяги 0,97, торможения 1,592 м/с². `identify.py` дал
`adhesion_accel_mps2 = 1.6`. Задержка 0,30 с, сопротивление 0,0053 м/с², P/m 8,5 Вт/кг — как в D-033.

Медиана RMSE скорости в окне 10 с, м/с. Подгонка без уклона, симуляция с уклоном GNSS и без него
на одних и тех же окнах:

| Набор | окон | без уклона | с уклоном (`--grade`) |
|---|---|---|---|
| train | 8455 | 0,264 | 0,202 |
| holdout (проверка) | 4387 | 0,254 | 0,201 |

Без уклона в симуляции (режим `params.yaml`): train 0,272 (8764 окна), holdout 0,293 (4638 окон).
Прежнее «0,20 против 0,27» сравнивало эти разные наборы окон.

## 2. Где предел срезает таблицу

`model_accel` (D-036) берёт `min(|A(n, v)|, adhesion_accel_mps2)`. При 1,59 срезаются ячейки торможения
1,592 — позиции −14…−15 (−16 ограничивается `notch_max` = 15) при v ≥ 8 м/с, между 6 и 8 м/с —
интерполяция выше 1,59. По данным (позиция с задержкой 0,3 с, v > 7 м/с): holdout — 15 отсчётов
10 Гц в 2 bag из 26, train — 123 отсчёта в 10 bag.

## 3. Выход pipeline: 1,59 против 1,60

Прямое сравнение `Estimate` на каждом bag holdout. Тот же `pipeline.Odometry`, отличается только
`drive.adhesion_accel_mps2`. Скрипт вне репозитория — цикл по `tram_eval.bag.read_bag` /
`default_odometry` / `run_pipeline` с `dataclasses.replace` на `params.drive`.

| bag | max \|Δv\|, м/с | max \|Δx,y,z\|, м |
|---|---|---|
| `30618_88548b02` | 5,5·10⁻⁴ | 2,6·10⁻⁴ |
| остальные 25 | 0 | 0 |

## 4. Holdout против `origin/main` (D-012)

```bash
# база: отдельный worktree origin/main (5a4b2fd)
.venv/Scripts/python tools/eval/run_eval.py --split holdout --out out/eval/5a4b2fd-holdout
# ветка
.venv/Scripts/python tools/eval/run_eval.py --split holdout --compare out/eval/5a4b2fd-holdout/metrics.json
```

| Метрика (медиана по bag) | было `5a4b2fd` | стало `5bf86b0` | Δ, % |
|---|---|---|---|
| **speed_rmse** | 0.035 | 0.035 | 0.000 |
| speed_mae | 0.024 | 0.024 | 0.000 |
| speed_bias_accel | -0.009 | -0.009 | 0.000 |
| speed_bias_brake | -0.017 | -0.017 | 0.000 |
| speed_bias_stop | -0.006 | -0.006 | 0.000 |
| speed_bias_cruise | -0.018 | -0.018 | 0.000 |
| **drift_pct** | 0.019 | 0.019 | 0.000 |
| along_mean | 1.591 | 1.591 | 0.000 |
| along_max | 8.857 | 8.857 | 0.000 |
| **along_rmse** | 2.353 | 2.353 | 0.000 |
| cross_rmse | 1.062 | 1.062 | 0.000 |
| pos3d_rmse | 3.008 | 3.008 | 0.000 |

`D-012: главные метрики не хуже`. Все 481 число `metrics.json` (4 знака) по всем bag совпадают:
изменение из п. 3 меньше точности записи.

## Вывод

Правка корректна: предел больше не срезает таблицу, из которой выведен. На метрики она не влияет.
Pipeline с 1,60 на holdout отличается от 1,59 только в одном bag, в четвёртом знаке скорости.
