# Сбой высоты GNSS в окне не переносится на прогон (#169, D-087)

- Было: `origin/main` `82e22fd`. Стало: ветка `worktree-169-dz-glitch` `095a5da` (tracker, params, types, тест).
- Среда: Windows 10, `.venv` (Python 3.10), данные `dataset/data`; эталон eval — `base_link` (`bag.bag_reference`).

## Разбор

Медиана по bag ошибки z оценки относительно эталона и смещение высоты окна `dz`
(`PathTracker._dz`), все 77 bag с GNSS: у 70 ошибка |z| < 0,5 м. Выбросы:

| bag | статус fix | dz окна, м | ошибка z, м | при dz = 0, м |
|---|---|---:|---:|---:|
| holdout `30618_defd0170` (скачок 10,6 м внутри окна) | 0 | −11,69 | −10,86 | +0,83 |
| train `30639_92226df0` | 0/1/2 | −65,31 | −65,35 | −0,04 |
| train `30639_9c362687` | 0/2 | −27,42 | −26,68 | +0,74 |
| train `30639_253671cc` | 0/2 | +11,90 | +10,36 | −1,54 |
| train `30639_3b3d9eb8` (настоящее смещение) | 0 | −7,36 | −1,42 | +5,94 |
| train `30639_dce52be4` (настоящее смещение) | 0 | −5,75 | −0,39 | +5,36 |

Порог 9,5 м — посередине между настоящими смещениями train (до 7,4 м) и сбоями (от 11,9 м).

## Метрики

```bash
.venv/bin/python tools/eval/run_eval.py --split holdout   # и --split train, --split holdout --stress
```

| набор | что изменилось |
|---|---|
| holdout 26 | только `30618_defd0170`: pos3d RMSE 13,34 → 8,13 м, pos3d_max 46,2 → 43,9 м (along_rmse 4,105 → 4,096, along_max 29,17 → 28,48, cross_max 2,628 → 2,631 — через 3D-сопоставление eval); среднее pos3d RMSE 3,643 → 3,443 м; медианы speed / along / cross / drift / pos3d — те же |
| train 51 | `92226df0` pos3d RMSE 65,7 → 5,6 м, `9c362687` 36,1 → 24,2 м, `253671cc` 11,0 → 5,0 м (pos3d_max 16,97 → 17,40 м); остальные 48 bag те же |
| stress | медиана пика ошибки положения ниже в `outlier`, `gap_1`, `gap_10`, `gap_70`, `spike`, `noise`, `jitter`, `rollback` (`outlier` 3,04 → 2,51, `gap_10` 4,09 → 3,37 м), в `freeze`, `gap_both_30`, `scale_up`, `scale_down` — та же; добавка к пику позиции на `defd0170` выросла (`scale_down` 22,4 → 31,5, `gap_both_30` 0,13 → 0,46 м) — сам пик не вырос, ниже стал чистый уровень, от которого она считается; скорость та же |

## Тесты

`test_position.py`: `test_height_offset_beyond_the_limit_is_a_gnss_glitch_not_the_run` — опубликованная
z при смещении за порогом равна высоте карты, в пределах порога смещение сохраняется;
`test_height_offset_glitch_taking_the_window_median_falls_back_to_the_map` — сначала верные высоты,
потом скачок на большую часть окна; `test_load_params_rejects_bad_base_link_keys` — порог 0 отвергается.
Порог жёсткий: когда медиана окна переходит 9,5 м, z внутри окна прыгает на смещение (без порога она
прыгала бы так же в момент сбоя); после окна значение не меняется.
`.venv/bin/python -m pytest -q` — 0 failed.
