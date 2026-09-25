# Holdout: детектор проскальзывания (#12), base vs head

- base: `5f446ad` (origin/main), head: `1fc6a88` (worktree-12-slip-detector, дерево чистое)
- машина: `Linux chichetin 7.2.6-arch2-1 #1 SMP PREEMPT_DYNAMIC Mon, 14 Sep 2026 22:41:30 +0000 x86_64`, nproc 8; прогон нативный `.venv` (не docker/numpy 1.21)
- набор: `--split holdout` (26 bag, окно GNSS 5.0 с); стенд жюри не запускался; stress не запускался

## Команды

```bash
git worktree add --detach <scratchpad>/base origin/main
cd <scratchpad>/base && <wt>/.venv/bin/python tools/eval/run_eval.py --split holdout
      # -> <scratchpad>/base/out/eval/5f446ad-holdout/metrics.json
cd <wt> && .venv/bin/python tools/eval/run_eval.py --split holdout \
      --compare <scratchpad>/base/out/eval/5f446ad-holdout/metrics.json
      # -> out/eval/1fc6a88-holdout/metrics.json
```

## Медианы по 26 bag (было -> стало, Δ %)

| метрика | было | стало | Δ % | худший bag было -> стало |
|---|---|---|---|---|
| speed_rmse | 0.0636 | 0.0636 | 0.000 | 30618_27e994fc -> 30618_27e994fc (0.1671 -> 0.1671) |
| along_rmse | 3263.67 | 3263.67 | 0.000 | 30618_dd8e6395 -> 30618_dd8e6395 (4974.7) |
| drift_pct | 153.41 | 153.38 | -0.02 | 30639_2b4a6347 -> 30639_2b4a6347 (296.9) |
| slip_flag_frac | 0.000 | 0.0012 | n/a | 30618_01f73500 -> 30618_082f1d65 (0.0505) |

Остальные строки таблицы `--compare` (speed_mae -1.14 %, bias accel/brake/stop/cruise, along_mean/max, cross_*, pos3d_*) — в stdout прогона; все в пределах +-1.2 %, кроме нулевых.
Вердикт `--compare`: «D-012: главные метрики не хуже». crashed: 0 из 26 и в base, и в head.

Положение: along/drift/cross в обоих прогонах сотни-тысячи метров (drift ~153 %) — карты (D-007) в pipeline на этих commit ещё нет; детектор на положение не влияет.

## speed_rmse на 4 bag 30639 (holdout)

| bag | было | стало | Δ % | slip_flag_frac (стало) |
|---|---|---|---|---|
| 30639_4285f2bc | 0.0829 | 0.0829 | 0.00 | 0.0268 |
| 30639_584b6e32 | 0.0937 | 0.0935 | -0.21 | 0.0113 |
| 30639_927002c2 | 0.0748 | 0.0745 | -0.40 | 0.0186 |
| 30639_d927f360 | 0.0571 | 0.0563 | -1.40 | 0.0274 |

Медиана по этим 4: 0.07885 -> 0.07870 (-0.19 %).

## Где детектор что-то изменил (speed_rmse по bag, было -> стало)

- 30618_2050d396: 0.0950 -> 0.0652 (-31 %); slip_flag_frac 0.0039
- 30618_33bec73f: 0.0929 -> 0.0647 (-30 %); 0.0047
- 30639_50956d6e: 0.1395 -> 0.1279 (-8 %); 0.0043
- 30639_9f0b519f: 0.0398 -> 0.0392; остальные bag 0.0 Δ

slip_flag_frac в base = 0 на всех bag (детектора нет); в head 0.0005–0.0505, максимум у 30618_082f1d65 (0.0505; 0.445 м пути, 230 точек — почти стоящий bag) и 30639_4285f2bc/d927f360 (0.027).

## Аномалии и ограничения

- Худший bag по скорости (30618_27e994fc, 0.167) и остальные крупные (0686195f 0.110, defd0170 0.129) детектор не изменил.
- Считался только holdout, native numpy из `.venv`, не docker. Train не использовался. Медианы «стало» по slip_flag_frac сравнивать с 0 некорректно (в base метрики нет по смыслу).
