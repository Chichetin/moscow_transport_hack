# Holdout: гейт скачка stamp вперёд в preprocess (#77, D-043), base vs head

- base: `33fd556` (origin/main), head: `d6f7a87` — первая версия гейта; финальная версия с ресинхронизацией назад перепроверена ниже
- Машина: `Linux 7.0.10-zen1-1-zen x86_64`, nproc 24, нативный `.venv`; `--split holdout` (26 bag, окно GNSS 5 с)

```bash
git worktree add --detach <scratchpad>/base-33fd556 origin/main
cd <scratchpad>/base-33fd556 && .venv/bin/python tools/eval/run_eval.py --split holdout
cd <wt> && .venv/bin/python tools/eval/run_eval.py --split holdout --compare out/eval/33fd556-holdout/metrics.json
```

Все 16 метрик (медиана по bag) — Δ 0,000 %: speed_rmse 0,062, along_rmse 2,610, drift_pct 0,028,
cross_rmse 1,075, pos3d_rmse 3,219. «D-012: главные метрики не хуже», 26/26 bag, упали: нет.

Порог по данным (скрипт `jumps.py` в scratchpad сессии: по каждому из 122 bag в порядке записи —
наибольшее превышение stamp сообщения тележки или контроллера над самым новым stamp этих
потоков до него): максимум 2,6 с (`30618_defd0170`, `30618_a869780d`, задняя тележка), далее
2,4 и 2,3 с. Порог 10 с — запас ×4; в holdout ни один вход не отброшен гейтом, отсюда Δ 0.
