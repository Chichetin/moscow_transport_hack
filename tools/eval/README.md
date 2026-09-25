# tools/eval — метрики без ROS

Читает bag библиотекой `rosbags`, подаёт сообщения в `tram_odometry_core.pipeline.Odometry`
в порядке `header.stamp` быстрее реального времени, **обрезает GNSS после первых
`--gnss-window` секунд** (по умолчанию = `gnss.init_window_s` из `params.yaml`, D-005), строит
эталон из GNSS master и считает все метрики критериев 1–2. Формат вывода и определения
метрик — `docs/contracts.md` §4 (контракт). Наборы — `splits.yaml` (контракт, D-011).

Статус: **не реализовано** — пакет плана `eval: каркас tools/eval`. Ниже — интерфейс, который он обязан дать.

```bash
.venv/bin/python -m tram_eval --split quick                 # 3 bag, цикл разработки, < 1 мин
.venv/bin/python -m tram_eval --split holdout               # решение о merge
.venv/bin/python -m tram_eval --split holdout --compare out/eval/<base-commit>/metrics.json
.venv/bin/python -m tram_eval --bag 30639_d927f360 --plot   # один bag + графики
.venv/bin/python -m tram_eval --split holdout --stress all  # стресс-наборы
```

Одна команда → таблица markdown в stdout (медиана и худший bag по каждой метрике, при
`--compare` — «было → стало → Δ») + `out/eval/<commit>/metrics.json`.

Стресс (`--stress`): генератор на holdout-bag — выбросы, пропуски 1–70 с одной тележки,
всплеск одной тележки («проскальзывание»), шум, дрожание и откаты stamp. Метрики те же +
всплеск ошибки во время аномалии и время восстановления после неё.
