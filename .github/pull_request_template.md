Closes #

## Что сделано

-

## Метрики holdout: было → стало

Команда: `.venv/bin/python -m tram_eval --split holdout --compare out/eval/<base>/metrics.json`
База: `origin/main` @ `<hash>`; ветка @ `<hash>`.

| Метрика (медиана по bag) | было | стало | Δ |
|---|---|---|---|
| speed RMSE, м/с | | | |
| along-track RMSE, м | | | |
| drift, % | | | |
| худший bag | | | |

Если метрики не считались — почему (например, «eval ещё не в main»).

## Контракт

- [ ] не затронут
- [ ] затронут: что меняется, потребители (issue), их код приведён в этом PR, комментарии в их issue (D-018)

## Проверки

- [ ] `.venv/bin/python -m pytest`
- [ ] `bash docker/dev.sh python3 -m pytest` (numpy 1.21)
- [ ] `bash docker/dev.sh colcon build`
- [ ] `docs/decisions.md` / `docs/tz-compliance.md` обновлены или не нужно
