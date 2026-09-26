# Проверка #107 (heading-finish) на holdout и на «убегающих» train-bag

Commit(ы): база `ae852ab` (origin/main) → ветка `9cda1c4`
(`worktree-107-heading-finish`, worktree `/home/user/moscow_transport_hack/.claude/worktrees/107-heading-finish`).
База считана в отдельном detached worktree `/tmp/eval-base-ae852ab` (`git worktree add --detach /tmp/eval-base-ae852ab ae852ab`),
чужие worktree не трогались.

Машина: `Linux vm 6.18.44-fc-v42 #1 SMP PREEMPT_DYNAMIC @0 x86_64 GNU/Linux`, `nproc` = 4,
numpy окружения `.venv` = 2.4.6 (не образ жюри; dev-образ с numpy 1.21 не гонялся — по брифу
Docker-стенд не нужен). `TRAM_DATA_DIR=/home/user/moscow_transport_hack/dataset/data`.

Стенд жюри не запускался (докер-сборка в облаке режется прокси) — по прямому указанию в брифе.

## Holdout (26 bag, решающий для D-012)

Команды:

```bash
# база
cd /tmp/eval-base-ae852ab
export TRAM_DATA_DIR=/home/user/moscow_transport_hack/dataset/data
.venv/bin/python tools/eval/run_eval.py --split holdout
# -> /tmp/eval-base-ae852ab/out/eval/ae852ab-holdout/metrics.json

# ветка, со сравнением
cd /home/user/moscow_transport_hack/.claude/worktrees/107-heading-finish
export TRAM_DATA_DIR=/home/user/moscow_transport_hack/dataset/data
.venv/bin/python tools/eval/run_eval.py --split holdout \
  --compare /tmp/eval-base-ae852ab/out/eval/ae852ab-holdout/metrics.json
# -> out/eval/9cda1c4-holdout/metrics.json
```

Главные метрики (медиана по 26 bag holdout), «было → стало»:

| Метрика | было (`ae852ab`) | стало (`9cda1c4`) | Δ, % |
|---|---|---|---|
| speed_rmse | 0.035 | 0.035 | 0.000 |
| along_rmse | 2.353 | 2.353 | 0.000 |
| drift_pct | 0.019 | 0.019 | 0.000 |
| cross_rmse | 1.062 | 1.062 | 0.000 |
| speed_mae | 0.024 | 0.024 | 0.000 |
| speed_bias_accel | -0.009 | -0.009 | 0.000 |
| speed_bias_brake | -0.017 | -0.017 | 0.000 |
| speed_bias_stop | -0.006 | -0.006 | 0.000 |
| speed_bias_cruise | -0.018 | -0.018 | 0.000 |
| along_mean | 1.591 | 1.591 | 0.000 |
| along_max | 8.857 | 8.857 | 0.000 |
| cross_mean | 0.310 | 0.310 | 0.000 |
| pos3d_rmse | 3.008 | 3.008 | 0.000 |

Числа побагово (holdout) идентичны база/ветка (см. полный вывод `run_eval.py` выше — таблица
по 26 bag совпадает построчно), включая худший bag по каждой метрике:

- худший по speed_rmse — `30618_27e994fc` (0.162 в обоих прогонах);
- худший по along_rmse и drift_pct — `30618_0686195f` (along_rmse 8.304) и `30639_0be558e2`
  (drift_pct 1.022) соответственно, без изменений;
- худший по cross_rmse — `30639_0be558e2` (6.140).

Вывод инструмента: `D-012: главные метрики не хуже`.

Изменения #107 не затрагивают ни один из 26 holdout-bag (эффект виден только на train-bag с
курсом map, см. ниже) — на holdout ветка и база дают буквально одни и те же метрики.

## Train: bag 30639_92226df0 и 30639_9c362687 (ожидание — больше не улетают на километры)

Эти bag — в наборе `train` (`tools/eval/splits.yaml`), не входят в holdout и не решают D-012;
приводятся только как демонстрация исправления «побочного» дефекта (курс/heading на старте),
числа с train не заменяют вердикт по holdout.

Команды:

```bash
# база
cd /tmp/eval-base-ae852ab
export TRAM_DATA_DIR=/home/user/moscow_transport_hack/dataset/data
.venv/bin/python tools/eval/run_eval.py --bag 30639_92226df0 --bag 30639_9c362687
# -> /tmp/eval-base-ae852ab/out/eval/ae852ab-bags/metrics.json

# ветка
cd /home/user/moscow_transport_hack/.claude/worktrees/107-heading-finish
export TRAM_DATA_DIR=/home/user/moscow_transport_hack/dataset/data
.venv/bin/python tools/eval/run_eval.py --bag 30639_92226df0 --bag 30639_9c362687
# -> out/eval/9cda1c4-bags/metrics.json
```

| bag | метрика | было (`ae852ab`) | стало (`9cda1c4`) |
|---|---|---|---|
| 30639_92226df0 | along_rmse, м | 3033.6 | 5.421 |
| 30639_92226df0 | drift_pct | 88.7 | 0.134 |
| 30639_92226df0 | cross_rmse, м | 42.0 | 0.660 |
| 30639_92226df0 | pos3d_rmse, м | 2692.1 | 65.6 |
| 30639_92226df0 | speed_rmse, м/с | 0.063 | 0.063 |
| 30639_9c362687 | along_rmse, м | 354.9 | 22.1 |
| 30639_9c362687 | drift_pct | 631.0 | 3.651 |
| 30639_9c362687 | cross_rmse, м | 3983.6 | 3.717 |
| 30639_9c362687 | pos3d_rmse, м | 4182.4 | 36.1 |
| 30639_9c362687 | speed_rmse, м/с | 0.073 | 0.073 |

Подтверждено: на базе оба bag «убегают» на километры по along/cross/pos3d (drift_pct 88.7 %
и 631.0 %, along_rmse в сотни–тысячи метров); на ветке #107 они остаются в пределах десятков
метров (drift_pct 0.13 % и 3.65 %, along_rmse 5.4 и 22.1 м, pos3d_rmse 36–66 м). Скорость
(speed_rmse) не изменилась — дефект был в курсе/позиции, не в скорости, как и ожидалось.

## Худший bag

Holdout: худший bag по главным метрикам не поменялся (`30618_27e994fc` — speed_rmse;
`30618_0686195f` — along_rmse; `30639_0be558e2` — drift_pct и cross_rmse) — ветка не вносит
регресса и не меняет ранжирование bag.

Train-демонстрация: худший из двух проверенных — `30639_9c362687` (короче, 731.6 м пути,
drift_pct и pos3d_rmse выше, чем у `30639_92226df0`), но оба вышли из режима «улетает на
километры» в режим «десятки метров».

## Ограничения прогона

- Стенд жюри (`docker/jury-stand.sh`) не запускался — по брифу не требовался (докер-сборка
  в облаке режется прокси); вывод по реальному времени не делается.
- `--stress` не запускался — бриф просил только holdout и указанные train-bag.
- Прогон на `.venv` c numpy 2.4.6, не на numpy 1.21 dev-образа жюри (`docker/dev.sh`
  не запускался) — числа метрик от версии numpy на этом пайплайне не зависят (чистый Python +
  numpy без ML), но формальный повтор на numpy 1.21 не делался в рамках этой сверки.

## Вывод для merge (D-012)

По holdout (26 bag, решающий набор) — метрики ветки `9cda1c4` идентичны базе `ae852ab`
(Δ = 0.000 % по всем главным метрикам), регресса нет: **не хуже**. Дополнительно на двух
train-bag (`30639_92226df0`, `30639_9c362687`), которые на базе улетали на километры
(along_rmse до 3033 м, drift_pct до 631 %), ветка держит ошибку в пределах десятков метров
(along_rmse ≤ 22.1 м, drift_pct ≤ 3.65 %) — заявленный дефект устранён, holdout не просел.
