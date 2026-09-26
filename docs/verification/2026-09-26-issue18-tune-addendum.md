# #18, дополнение: масштаб шумов фильтра и пороги slip на bag с расхождением тележек

Дополняет `2026-09-26-issue18-train-tune.md` (PR #120, D-066) двумя фактами из отдельного прогона
того же `tools/eval/tune_estimator.py` (сетка `GRID` та же, что в `main`). Прогон — ветка
`worktree-18-ident-train-tune`, `main` `ed9e51a` + скрипт; проверка после merge — `ff87f17`
(`main` `cd2fa74` + скрипт); ядро и `params.yaml` = `main`. Windows 10, 2 ядра, numpy 2.5.3,
релиз данных `dataset-2026-09-25`. Только train (D-011).

```bash
.venv/bin/python tools/eval/tune_estimator.py --split train --jobs 2 --rounds 0 --random 16 --seed 18 --report out/tune/train-random-report.md
.venv/bin/python tools/eval/tune_estimator.py --split train --bag 30639_3b3d9eb8 --bag 30639_44226bde --rounds 1 --jobs 2 --report out/tune/disagree-report.md
```

## 1. Общий масштаб четырёх ковариаций метрик не меняет

Случайная проба 14 из 16 (seed 18; в PR #120 было 12 проб, поэтому её там нет): все четыре
ковариации ×10 от `params.yaml` (`r_wheel` 0,05 → 0,5, `q_accel` 0,5 → 5, `q_bias` 0,2 → 2,
`initial_bias_var` 0,25 → 2,5), плюс `nis_gate` 4 и пороги `slip.*` — медианы по 51 bag совпадают
со стартом до 4-го знака.

| | speed_rmse | speed_mae | speed_bias_accel | speed_bias_brake | speed_bias_stop | speed_bias_cruise | along_rmse | drift_pct | cross_rmse | score |
|---|---|---|---|---|---|---|---|---|---|---|
| start (params.yaml) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 1.0000 |
| random 14 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 1.0000 |

Почему (`estimator/__init__.py`): при умножении `r_wheel`, `q_accel`, `q_bias`, `initial_bias_var`
на одно k ковариация и дисперсия инновации умножаются на k, усиление Калмана и оценка — нет.
NIS = инновация² / дисперсия делится на k, то есть эффективный порог `nis_gate` умножается на k;
на train гейт не срабатывает (D-066), поэтому метрики те же. Следствие: у шумов фильтра три
свободные степени — отношения `q_accel`, `q_bias`, `initial_bias_var` к `r_wheel`; абсолютный
масштаб задаёт только смысл `nis_gate`.

## 2. Пороги `slip.*` на двух bag train с расхождением тележек

Ловушка 8 (`docs/data.md`): `30639_3b3d9eb8` (10,6 %), `30639_44226bde` (1,8 %). Проверка, не
прячет ли медиана по 51 bag влияние порогов. Только для сведения, выбор по двум bag не делается.
Медианы по 2 bag, прогон на `ff87f17`:

| значение | speed_rmse | speed_bias_accel | along_rmse |
|---|---|---|---|
| start (params.yaml) | 0.1408 | 0.1116 | 14.8101 |
| slip.front_rear_threshold_mps=0.25 | 0.1407 | 0.1117 | 14.8051 |
| slip.front_rear_threshold_mps=0.35 | 0.1407 | 0.1116 | 14.8067 |
| slip.front_rear_threshold_mps=0.7 | 0.1408 | 0.1115 | 14.8133 |
| slip.front_rear_threshold_mps=1 | 0.1408 | 0.1115 | 14.8133 |
| slip.model_residual_threshold_mps2=0.5 | 0.1408 | 0.1116 | 14.8101 |
| slip.model_residual_threshold_mps2=2 | 0.1408 | 0.1116 | 14.8113 |
| slip.noise_accel_mps2=2.5 / 4 / 5 | 0.1408 | 0.1116 | 14.8101 |
| slip.noise_hold_s=0.5 / 2 | 0.1408 | 0.1116 | 14.8101 |

Остальные метрики (speed_mae 0,0778, drift_pct 0,3226, cross_rmse 4,0473, bias brake/stop/cruise)
на всей сетке не меняются. Разброс: speed RMSE ≤ 0,0001 м/с, along RMSE ≤ 0,005 м.
