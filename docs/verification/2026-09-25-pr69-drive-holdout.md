# Holdout: модель привода в pipeline (#10, PR #69, D-036), base vs head

- base: `e6d12fe` (origin/main), head: `6ee32cd` (worktree-10-drive-dynamics, дерево чистое)
- Машина: `Linux 7.0.10-zen1-1-zen x86_64`, нативный `.venv` (Python 3.14), набор `--split holdout`
  (26 bag, окно GNSS 5,0 с); данные `dataset/data` (122 каталога); стенд и stress не запускались
- Время: база 4 с, ветка 5 с

## Команды

```bash
git worktree add --detach <scratchpad>/base-e6d12fe e6d12fe
cd <scratchpad>/base-e6d12fe && .venv/bin/python tools/eval/run_eval.py --split holdout
      # -> out/eval/e6d12fe-holdout/metrics.json
cd <wt> && .venv/bin/python tools/eval/run_eval.py --split holdout \
      --compare out/eval/e6d12fe-holdout/metrics.json      # -> out/eval/6ee32cd-holdout/metrics.json
```

## Было → стало

| Метрика (медиана по bag) | было | стало | Δ, % |
|---|---|---|---|
| **speed_rmse** | 0.062 | 0.062 | 0.000 |
| speed_mae | 0.039 | 0.039 | 0.000 |
| speed_bias_accel | -0.059 | -0.059 | 0.000 |
| speed_bias_brake | 0.039 | 0.039 | 0.000 |
| speed_bias_stop | -0.006 | -0.006 | 0.000 |
| speed_bias_cruise | -0.025 | -0.025 | 0.000 |
| **drift_pct** | 0.028 | 0.028 | 0.000 |
| along_mean | 1.889 | 1.889 | 0.000 |
| along_max | 9.443 | 9.443 | 0.000 |
| **along_rmse** | 2.610 | 2.610 | 0.000 |
| cross_mean | 0.329 | 0.329 | 0.000 |
| cross_max | 5.003 | 5.003 | 0.000 |
| cross_rmse | 1.075 | 1.075 | 0.000 |
| pos3d_rmse | 3.219 | 3.219 | 0.000 |
| pos3d_max | 11.2 | 11.2 | 0.000 |
| slip_flag_frac | 0.001 | 0.001 | 0.000 |

Вердикт `--compare`: «D-012: главные метрики не хуже». 26/26 bag, упали: нет; NaN/inf: 0;
`t != stamp`: 0.

## Что это значит

Δ = 0,000 по всем метрикам до третьего знака: прогноз по модели срабатывает только когда обе
тележки не приходят дольше `input.stale_timeout_s` = 0,5 с по времени состояния, а такие
паузы в holdout идут вместе с паузой контроллера (ловушка 7), то есть событий для прогноза нет;
у 30639 молчит одна тележка, и скорость берётся из второй. `accel_model` в детекторе не
изменил ни одного решения о доверии на holdout (`slip_flag_frac` тот же). Это ожидалось
(D-036, «Почему»): ценность шага — готовый прогноз и `accel_model` для EKF (#11), где модель
на окнах 10 с даёт 0,29 м/с против 1,67 у «a = 0» (`2026-09-25-drive-ident.md`).
