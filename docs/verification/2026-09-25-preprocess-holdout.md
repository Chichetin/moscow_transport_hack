# Issue #13 (`core-preprocess`, `Preprocessor`) — holdout, было → стало

- Ветка: `worktree-13-preprocess`, база `origin/main` = `ada6a17` (после merge origin/main, включая слитый в main `SlipDetector` #12/D-027)
- Данные: `<основная копия>/dataset/data`, `TRAM_DATA_DIR` не задан
- Команда (из корня worktree): `.venv/bin/python tools/eval/run_eval.py --split holdout`
  (26 bag, окно GNSS 5,0 с)
- «Было» посчитан на коммите `ada6a17` (= `origin/main`) во временном `git worktree`;
  «стало» — эта ветка (`Preprocessor` подключён в `pipeline.Odometry` вместо инлайн-разбора
  km/ч/stamp/окна GNSS, `SlipDetector` не тронут).

## Было → стало

```
.venv/bin/python tools/eval/run_eval.py --split holdout --compare out/eval/ada6a17-holdout/metrics.json
```

| Метрика (медиана по bag) | было | стало | Δ, % |
|---|---|---|---|
| **speed_rmse** | 0.064 | 0.062 | -2.280 |
| speed_mae | 0.039 | 0.039 | 0.000 |
| speed_bias_accel | -0.059 | -0.059 | 0.000 |
| speed_bias_brake | 0.038 | 0.039 | 0.260 |
| speed_bias_stop | -0.006 | -0.006 | 0.787 |
| speed_bias_cruise | -0.025 | -0.025 | 0.000 |
| **drift_pct** | 153.4 | 153.4 | 0.003 |
| along_mean | 2763.1 | 2763.1 | 0.000 |
| along_max | 5394.1 | 5394.1 | 0.000 |
| **along_rmse** | 3263.7 | 3263.7 | 0.000 |
| cross_mean | 2354.8 | 2354.8 | 0.000 |
| cross_max | 5040.8 | 5040.8 | -0.000 |
| cross_rmse | 2905.4 | 2905.4 | 0.000 |
| pos3d_rmse | 4522.3 | 4522.3 | 0.000 |
| pos3d_max | 7433.4 | 7433.4 | 0.000 |
| slip_flag_frac | 0.001 | 0.001 | 0.000 |

D-012: главные метрики не хуже (speed_rmse даже немного лучше). 26/26 bag, упали: нет.

Метрики почти неизменны на holdout — ни один из 26 прогонов бьёт `input.max_wheel_accel_mps2`
достаточно часто, чтобы сильно сдвинуть медиану (выбросы бьют по хвостам, не по медиане;
специально под них — модульные тесты `test_preprocess.py`, не holdout). Небольшое улучшение
`speed_rmse` — единичные глюки датчика, отфильтрованные на нескольких bag, больше не искажают
среднее тележек. Отдельная стресс-проверка «выбросы»/«откаты stamp» из критерия приёмки #13 —
не через holdout, а через `src/tram_odometry_core/test/test_preprocess.py` (31 тест на traps
5–7, 9 `docs/data.md`, включая исключение «застревание на 0», trap 8, — совместимость со
`SlipDetector` #12): генератор `--stress` (#15) ещё не готов.
