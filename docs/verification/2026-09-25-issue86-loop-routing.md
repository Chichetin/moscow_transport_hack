# Issue #86 — западный терминал: holdout

Сравнение полного holdout для точных commit `origin/main` и ветки с #86:

- base: `5390aac80cdc39a06afc3ccc5c6a063045099771`
- head: `ca38c71ae3698a9520c62df6abca13769ad67f90`
- 26 holdout bag, окно GNSS 5 с, `--jobs 4`; целевой bag также прогнан отдельно.
- Оценка выполнялась на нативном хосте, без Docker image: `Linux archbook 7.2.6-arch2-1 #1 SMP PREEMPT_DYNAMIC Mon, 14 Sep 2026 22:41:30 UTC x86_64 GNU/Linux`, 16 CPU. Python 3.14.7, NumPy 2.5.3, PyYAML 6.0.3. Данные: `/home/ir6/my/moscow_transport_hack/dataset/data`.
- Для обеих версий созданы чистые detached worktree на указанных полных SHA. Запуск использовал внешний `/home/ir6/my/moscow_transport_hack/.venv/bin/python`; в checkout не было `.venv`-ссылок. В `metrics.json` указаны чистые commit `5390aac` и `ca38c71`.

## Команды

```bash
git worktree add --detach /tmp/moscow-transport-eval86-base-5390aac 5390aac80cdc39a06afc3ccc5c6a063045099771
git worktree add --detach /tmp/moscow-transport-eval86-candidate-ca38c71 ca38c71ae3698a9520c62df6abca13769ad67f90

cd /tmp/moscow-transport-eval86-base-5390aac
/home/ir6/my/moscow_transport_hack/.venv/bin/python tools/eval/run_eval.py --split holdout --jobs 4 --out /tmp/eval86-base-5390aac

cd /tmp/moscow-transport-eval86-candidate-ca38c71
/home/ir6/my/moscow_transport_hack/.venv/bin/python tools/eval/run_eval.py --split holdout --jobs 4 --out /tmp/eval86-candidate-ca38c71 --compare /tmp/eval86-base-5390aac/metrics.json

cd /tmp/moscow-transport-eval86-base-5390aac
/home/ir6/my/moscow_transport_hack/.venv/bin/python tools/eval/run_eval.py --bag 30618_27e994fc --out /tmp/eval86-target-base-5390aac

cd /tmp/moscow-transport-eval86-candidate-ca38c71
/home/ir6/my/moscow_transport_hack/.venv/bin/python tools/eval/run_eval.py --bag 30618_27e994fc --out /tmp/eval86-target-candidate-ca38c71 --compare /tmp/eval86-target-base-5390aac/metrics.json
```

Holdout завершился без падений, оценок NaN/inf и несовпадений `Estimate.t` со stamp входа. Совпали наборы bag и `n_matched`; у целевого bag — 18 629 matches на обеих версиях. JSON: `/tmp/eval86-base-5390aac/metrics.json`, `/tmp/eval86-candidate-ca38c71/metrics.json` и соответствующие `target-*` каталоги.

## Главные метрики D-012

Медиана по bag; худший bag приведён отдельно для каждой версии.

| Метрика | base | head | Δ медианы | Худший bag base | Худший bag head |
|---|---:|---:|---:|---|---|
| `speed_rmse`, м/с | 0.06215 | 0.06215 | 0.000% | `30618_27e994fc` (0.1675) | `30618_27e994fc` (0.1675) |
| `along_rmse`, м | 2.60975 | 2.54940 | −2.312% | `30618_27e994fc` (41.4559) | `30618_0686195f` (8.6789) |
| `drift_pct`, % | 0.0278 | 0.0278 | 0.000% | `30639_0be558e2` (0.6804) | `30639_0be558e2` (1.0225) |

**D-012: проходит.** Ни одна главная медианная метрика не ухудшилась более чем на 2%; along RMSE улучшился на 2.312%, speed RMSE и drift медианы не изменились.

## Целевой bag и per-bag изменения

На `30618_27e994fc` along RMSE снизился с 41.4559 до 5.7931 м (−86.026%). Остальные основные значения этого bag: drift 0.4783 → 0.4353%, cross RMSE 7.3695 → 2.1858 м, pos3d RMSE 10.8025 → 6.3291 м. Явный `--bag`-прогон подтвердил те же метрики, что и строка полного holdout.

Материальная per-bag регрессия остаётся на `30639_0be558e2`: along RMSE улучшился 2.7487 → 2.628 м, но drift вырос 0.6804 → 1.0225%, cross RMSE 4.1003 → 6.1221 м (+49.3%), pos3d RMSE 4.9793 → 6.6934 м (+34.4%), cross max 31.9908 → 56.1867 м и pos3d max 37.7352 → 57.0709 м. Это худший bag по drift на candidate; медиана drift при этом не изменилась. У остальных затронутых bag изменения малы либо улучшают ошибки; например, на `30639_4285f2bc` drift снизился 0.2168 → 0.0308%, cross RMSE — 0.5446 → 0.2325 м.

Предыдущий прогон против `eed83a7` дал те же метрики; этот отчёт фиксирует свежие точные хеши `5390aac` и `ca38c71`.
