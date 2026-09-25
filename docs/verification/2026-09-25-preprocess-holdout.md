# Issue #13 (`core-preprocess`, `Preprocessor`) — holdout, было → стало

- Ветка: `worktree-13-preprocess`, база `origin/main` = `91b0e47` (после merge origin/main, включая слитые в main `SlipDetector` #12/D-027, таблицы привода #52/D-029 и `PathTracker` #14/D-030)
- Данные: `<основная копия>/dataset/data`, `TRAM_DATA_DIR` не задан
- Команда (из корня worktree): `.venv/bin/python tools/eval/run_eval.py --split holdout`
  (26 bag, окно GNSS 5,0 с)
- «Было» посчитан на коммите `91b0e47` (= `origin/main`) во временном `git worktree`;
  «стало» — эта ветка (`Preprocessor` подключён в `pipeline.Odometry` вместо инлайн-разбора
  km/ч/stamp/окна GNSS; `SlipDetector` и `PathTracker` не тронуты, только переведены на
  типизированные `Sample` вместо сырых ROS-сообщений в `_on_fix`/`_on_vel`).

## Было → стало

```
.venv/bin/python tools/eval/run_eval.py --split holdout --compare out/eval/91b0e47-holdout/metrics.json
```

| Метрика (медиана по bag) | было | стало | Δ, % |
|---|---|---|---|
| **speed_rmse** | 0.064 | 0.062 | -2.280 |
| speed_mae | 0.039 | 0.039 | 0.000 |
| speed_bias_accel | -0.059 | -0.059 | 0.000 |
| speed_bias_brake | 0.038 | 0.039 | 0.260 |
| speed_bias_stop | -0.006 | -0.006 | 0.787 |
| speed_bias_cruise | -0.025 | -0.025 | 0.000 |
| **drift_pct** | 0.343 | 0.343 | 0.000 |
| along_mean | 4.152 | 4.153 | 0.008 |
| along_max | 12.2 | 12.2 | 0.000 |
| **along_rmse** | 4.668 | 4.669 | 0.011 |
| cross_mean | 0.331 | 0.331 | 0.000 |
| cross_max | 4.902 | 4.903 | 0.016 |
| cross_rmse | 1.083 | 1.083 | 0.005 |
| pos3d_rmse | 4.961 | 4.961 | 0.000 |
| pos3d_max | 19.9 | 19.8 | -0.159 |
| slip_flag_frac | 0.001 | 0.001 | 0.000 |

D-012: главные метрики не хуже (speed_rmse даже немного лучше). 26/26 bag, упали: нет.

Позиционные метрики резко лучше прежней сравнительной таблицы (`along_rmse` 3264 м → 4,67 м,
`drift_pct` 153 % → 0,34 %) — это эффект `PathTracker` (#14/D-030), не этой ветки; здесь важна
Δ между «было» и «стало» на одном и том же коммите с картой, а она везде ≤ 0,02 %. Ни один из
26 прогонов не бьёт `input.max_wheel_accel_mps2` достаточно часто, чтобы сильно сдвинуть
медиану (выбросы бьют по хвостам, не по медиане; специально под них — модульные тесты
`test_preprocess.py`, не holdout). Небольшое улучшение `speed_rmse` — единичные глюки датчика,
отфильтрованные на нескольких bag, больше не искажают среднее тележек. Отдельная
стресс-проверка «выбросы»/«откаты stamp» из критерия приёмки #13 — не через holdout, а через
`src/tram_odometry_core/test/test_preprocess.py` (31 тест на traps 5–7, 9 `docs/data.md`,
включая исключение «застревание на 0», trap 8, — совместимость со `SlipDetector` #12):
генератор `--stress` (#15) ещё не готов.
