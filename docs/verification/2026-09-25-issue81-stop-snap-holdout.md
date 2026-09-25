# #81: неоднозначная привязка к остановке — holdout

Проверка D-012 по тому же holdout из 26 bag до и после изменения правила в `PathTracker.on_stop`.

| Вариант | Along RMSE, медиана | Худший bag | Along RMSE худшего bag |
|---|---:|---|---:|
| `origin/main` (`db8513f`) | 2,60975 м | `30639_4285f2bc` | 1425,3827 м |
| Кандидат: reject при разнице расстояний ≤ 2·`stop_std_m` (`stop_std_m = 2 м`) | 2,60975 м | `30639_4285f2bc` | 1425,3827 м |
| Изменение | 0 % | без изменения | 0 % |

Базовая копия `origin/main` собрана без изменения Git worktrees: `git archive origin/main` в `/tmp/issue81-origin-main`; данные holdout — `TRAM_DATA_DIR=/home/ir6/my/moscow_transport_hack/dataset/data`. На обеих копиях выполнено:

```bash
.venv/bin/python tools/eval/run_eval.py --split holdout --jobs 4 --out <каталог>
```

Ветка кандидата до коммита имела dirty marker (`db8513f-dirty`); численные результаты полные и сохранены в `/tmp/eval-81-origin/metrics.json` и `/tmp/eval-81-margin2/metrics.json`. На 26 bag — без падений, `NaN/inf` и несовпадений stamp. Изменённое правило не ухудшает ни медиану, ни худший bag по along RMSE.

Синтетическое обоснование: при местах `s=1500, 1517.8 м` и оценке `s=1510 м` расстояния 10 и 7,8 м отличаются на 2,2 м, что меньше `2·stop_std_m = 4 м`; snap отклоняется. При точной оценке у `s=1500 м` расстояния 0 и 17,8 м различимы, snap сохраняется.
