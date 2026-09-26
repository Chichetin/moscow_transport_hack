# Issue #18 — перебор шумов фильтра и порогов проскальзывания на train

Команда (ветка `worktree-18-train-tune`, код ядра и `params.yaml` = `origin/main` 22d861a):

```bash
.venv/bin/python tools/eval/tune_estimator.py --split train --jobs 4 --rounds 2 --random 12 --seed 18 \
    --report docs/verification/2026-09-26-issue18-train-tune.md
```

Итог: **ни один ключ не меняется.** Покоординатный спуск (9 ключей `filter.*`/`slip.*`, сетка
`tune_estimator.GRID`) закончился на первом проходе без принятого шага; из 12 случайных совместных
точек ни одна не прошла правило «score ниже на 0,005 и ни одна главная метрика D-012 не хуже на 2 %».
Лучшие по score кандидаты (random 1: 0,9856; random 10: 0,9907) снижают speed_rmse, но
along_rmse растёт на 3 % — правило D-012 их отсекает. Четыре ключа `slip.*` на train метрик не меняют
вовсе (доля флага проскальзывания — медиана 0,001): юза в train почти нет, и их значения
этим перебором не подтверждены и не опровергнуты; они остаются ручными (D-066).

Holdout — один прогон после выбора: `.venv/bin/python tools/eval/run_eval.py --split holdout`
на ветке; `src/` совпадает с `origin/main` (`git diff --quiet origin/main HEAD -- src`), поэтому
медианы те же, что у main: speed_rmse 0,035, along_rmse 2,353, drift_pct 0,019, cross_rmse 1,062.

## Отчёт перебора

split `train`, 51 bag, окно GNSS 5.0 с; поиск 41 конфигураций за 1566 с; margin 0.005, rounds 2; random 12, seed 18

Выбрано (отличия от params.yaml): нет

| | speed_rmse | speed_mae | speed_bias_accel | speed_bias_brake | speed_bias_stop | speed_bias_cruise | along_rmse | drift_pct | cross_rmse | упало bag | score | D-012 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| start (params.yaml) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| выбрано | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |

### Все шаги

| проход | значение | speed_rmse | speed_mae | speed_bias_accel | speed_bias_brake | speed_bias_stop | speed_bias_cruise | along_rmse | drift_pct | cross_rmse | упало bag | score | D-012 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | filter.r_wheel=0.005 | 0.0328 | 0.0229 | 0.0077 | -0.0081 | -0.0078 | -0.0068 | 2.1702 | 0.0272 | 0.4441 | 0 | 1.0016 | да |
| 1 | filter.r_wheel=0.015 | 0.0328 | 0.0228 | 0.0074 | -0.0078 | -0.0077 | -0.0067 | 2.1637 | 0.0272 | 0.4441 | 0 | 1.0006 | да |
| 1 | filter.r_wheel=0.05 (текущее) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | filter.r_wheel=0.15 | 0.0340 | 0.0229 | 0.0060 | -0.0063 | -0.0076 | -0.0069 | 2.0913 | 0.0273 | 0.4441 | 0 | 1.0033 | нет |
| 1 | filter.r_wheel=0.5 | 0.0382 | 0.0243 | 0.0046 | -0.0042 | -0.0072 | -0.0060 | 2.0788 | 0.0275 | 0.4443 | 0 | 1.0462 | нет |
| 1 | → filter.r_wheel=0.05 | | | | | | | | | | | | |
| 1 | filter.q_accel=0.05 | 0.0335 | 0.0226 | 0.0061 | -0.0071 | -0.0078 | -0.0066 | 2.1749 | 0.0270 | 0.4441 | 0 | 1.0076 | да |
| 1 | filter.q_accel=0.15 | 0.0329 | 0.0225 | 0.0064 | -0.0072 | -0.0078 | -0.0069 | 2.1643 | 0.0270 | 0.4441 | 0 | 1.0005 | да |
| 1 | filter.q_accel=0.5 (текущее) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | filter.q_accel=1.5 | 0.0334 | 0.0230 | 0.0072 | -0.0072 | -0.0076 | -0.0066 | 2.1189 | 0.0275 | 0.4441 | 0 | 1.0046 | да |
| 1 | filter.q_accel=5 | 0.0339 | 0.0234 | 0.0075 | -0.0071 | -0.0073 | -0.0067 | 2.1229 | 0.0280 | 0.4442 | 0 | 1.0158 | нет |
| 1 | → filter.q_accel=0.5 | | | | | | | | | | | | |
| 1 | filter.q_bias=0.0004 | 0.0378 | 0.0247 | 0.0080 | -0.0046 | -0.0055 | -0.0074 | 2.2257 | 0.0306 | 0.4455 | 0 | 1.1030 | нет |
| 1 | filter.q_bias=0.02 | 0.0346 | 0.0235 | 0.0065 | -0.0059 | -0.0069 | -0.0065 | 2.0953 | 0.0282 | 0.4443 | 0 | 1.0204 | нет |
| 1 | filter.q_bias=0.2 (текущее) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | filter.q_bias=2 | 0.0315 | 0.0225 | 0.0076 | -0.0085 | -0.0079 | -0.0067 | 2.2061 | 0.0269 | 0.4441 | 0 | 0.9910 | нет |
| 1 | → filter.q_bias=0.2 | | | | | | | | | | | | |
| 1 | filter.initial_bias_var=0.025 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | filter.initial_bias_var=0.25 (текущее) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | filter.initial_bias_var=2.5 | 0.0328 | 0.0227 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1524 | 0.0274 | 0.4442 | 0 | 1.0019 | да |
| 1 | → filter.initial_bias_var=0.25 | | | | | | | | | | | | |
| 1 | filter.nis_gate=4 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | filter.nis_gate=6.25 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | filter.nis_gate=9 (текущее) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | filter.nis_gate=16 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | filter.nis_gate=25 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | → filter.nis_gate=9 | | | | | | | | | | | | |
| 1 | slip.front_rear_threshold_mps=0.25 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.front_rear_threshold_mps=0.35 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.front_rear_threshold_mps=0.5 (текущее) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.front_rear_threshold_mps=0.7 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.front_rear_threshold_mps=1 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | → slip.front_rear_threshold_mps=0.5 | | | | | | | | | | | | |
| 1 | slip.model_residual_threshold_mps2=0.5 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.model_residual_threshold_mps2=1 (текущее) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.model_residual_threshold_mps2=2 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | → slip.model_residual_threshold_mps2=1 | | | | | | | | | | | | |
| 1 | slip.noise_accel_mps2=2.5 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.noise_accel_mps2=3 (текущее) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.noise_accel_mps2=4 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.noise_accel_mps2=5 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | → slip.noise_accel_mps2=3 | | | | | | | | | | | | |
| 1 | slip.noise_hold_s=0.5 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.noise_hold_s=1 (текущее) | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | slip.noise_hold_s=2 | 0.0330 | 0.0226 | 0.0068 | -0.0074 | -0.0076 | -0.0068 | 2.1430 | 0.0272 | 0.4441 | 0 | 1.0000 | да |
| 1 | → slip.noise_hold_s=1 | | | | | | | | | | | | |
| random 1 | filter.r_wheel=0.015, filter.q_accel=0.05, filter.q_bias=2, filter.nis_gate=6.25, slip.front_rear_threshold_mps=0.35, slip.noise_accel_mps2=5, slip.noise_hold_s=0.5 | 0.0309 | 0.0217 | 0.0080 | -0.0090 | -0.0080 | -0.0067 | 2.2062 | 0.0270 | 0.4441 | 0 | 0.9856 | нет |
| random 2 | filter.r_wheel=0.15, filter.q_bias=2, filter.nis_gate=6.25, slip.model_residual_threshold_mps2=2, slip.noise_accel_mps2=2.5 | 0.0318 | 0.0224 | 0.0070 | -0.0079 | -0.0079 | -0.0067 | 2.2062 | 0.0270 | 0.4441 | 0 | 0.9946 | нет |
| random 3 | filter.r_wheel=0.5, filter.q_accel=0.15, filter.q_bias=0.02, filter.initial_bias_var=0.025, filter.nis_gate=6.25, slip.front_rear_threshold_mps=0.35, slip.model_residual_threshold_mps2=2, slip.noise_accel_mps2=4, slip.noise_hold_s=2 | 0.0571 | 0.0296 | 0.0022 | 0.0017 | -0.0055 | -0.0055 | 2.1488 | 0.0222 | 0.4447 | 0 | 1.1831 | нет |
| random 4 | filter.r_wheel=0.5, filter.q_accel=0.15, filter.q_bias=2, filter.initial_bias_var=0.025, slip.model_residual_threshold_mps2=0.5, slip.noise_accel_mps2=4 | 0.0337 | 0.0228 | 0.0060 | -0.0070 | -0.0079 | -0.0065 | 2.1768 | 0.0270 | 0.4441 | 0 | 1.0099 | нет |
| random 5 | filter.r_wheel=0.015, filter.q_accel=1.5, filter.initial_bias_var=2.5, filter.nis_gate=25, slip.front_rear_threshold_mps=1, slip.noise_accel_mps2=4, slip.noise_hold_s=2 | 0.0331 | 0.0231 | 0.0075 | -0.0077 | -0.0076 | -0.0066 | 2.1075 | 0.0276 | 0.4441 | 0 | 1.0004 | да |
| random 6 | filter.q_accel=5, filter.q_bias=0.0004, filter.nis_gate=16, slip.front_rear_threshold_mps=1, slip.model_residual_threshold_mps2=2 | 0.0360 | 0.0242 | 0.0090 | -0.0058 | -0.0058 | -0.0069 | 2.2667 | 0.0319 | 0.4452 | 0 | 1.1071 | нет |
| random 7 | filter.r_wheel=0.015, filter.q_accel=0.15, filter.q_bias=0.02, filter.initial_bias_var=2.5, filter.nis_gate=16, slip.front_rear_threshold_mps=0.35, slip.noise_accel_mps2=5, slip.noise_hold_s=2 | 0.0333 | 0.0230 | 0.0065 | -0.0067 | -0.0071 | -0.0067 | 2.0981 | 0.0282 | 0.4442 | 0 | 1.0077 | нет |
| random 8 | filter.r_wheel=0.005, filter.q_accel=0.15, filter.initial_bias_var=2.5, slip.front_rear_threshold_mps=1, slip.noise_accel_mps2=2.5 | 0.0324 | 0.0226 | 0.0077 | -0.0084 | -0.0078 | -0.0068 | 2.2078 | 0.0272 | 0.4441 | 0 | 1.0040 | нет |
| random 9 | filter.r_wheel=0.015, filter.q_accel=1.5, filter.q_bias=0.02, filter.nis_gate=4, slip.front_rear_threshold_mps=0.35, slip.model_residual_threshold_mps2=2, slip.noise_accel_mps2=2.5, slip.noise_hold_s=2 | 0.0345 | 0.0237 | 0.0075 | -0.0068 | -0.0070 | -0.0069 | 2.2057 | 0.0285 | 0.4443 | 0 | 1.0402 | нет |
| random 10 | filter.q_accel=0.15, filter.q_bias=2, filter.initial_bias_var=2.5, filter.nis_gate=25, slip.front_rear_threshold_mps=1, slip.model_residual_threshold_mps2=0.5, slip.noise_hold_s=2 | 0.0314 | 0.0224 | 0.0076 | -0.0085 | -0.0079 | -0.0066 | 2.2068 | 0.0270 | 0.4441 | 0 | 0.9907 | нет |
| random 11 | filter.r_wheel=0.005, filter.q_accel=1.5, filter.q_bias=2, filter.nis_gate=6.25, slip.front_rear_threshold_mps=0.7 | 0.0323 | 0.0229 | 0.0081 | -0.0088 | -0.0077 | -0.0069 | 2.2052 | 0.0270 | 0.4440 | 0 | 1.0008 | нет |
| random 12 | filter.r_wheel=0.15, filter.q_bias=0.02, slip.front_rear_threshold_mps=1 | 0.0389 | 0.0238 | 0.0056 | -0.0043 | -0.0064 | -0.0065 | 2.1363 | 0.0247 | 0.4445 | 0 | 1.0279 | нет |
