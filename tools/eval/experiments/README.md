# Проверка идей конкурентов (#141)

Разовые эксперименты к `docs/verification/2026-09-27-issue141-competitor-ideas.md`. Ядро и
`params.yaml` не меняются: варианты подменяются в памяти (обёртка метода или `dataclasses.replace`).
Настройка — только train; `snap_diag.py` — диагностика привязки к остановкам на готовом `stress.json`.

```bash
.venv/bin/python tools/eval/experiments/nis_count.py                     # сколько колёс отбросил NIS-гейт на train
.venv/bin/python tools/eval/experiments/snap_diag.py out/eval/<run>/stress.json   # привязки к остановкам: clean / scale_up / scale_down
.venv/bin/python tools/eval/experiments/snap_gate_exp.py train           # порог привязки min(cap, max(20 м, k·σ_along))
.venv/bin/python tools/eval/experiments/speed_scale_exp.py train         # скорость × онлайн-масштаб пути
.venv/bin/python tools/eval/experiments/filter_ideas_exp.py train        # R от |cmd| (ktoyart), трение c0·tanh(v/ε)
.venv/bin/python tools/eval/experiments/r_stress.py                      # R от |cmd| в стресс-сценариях, train
```

Онлайн-масштаб пути колеса (#154, D-088): гипотезы H1 (якорь — первая опора масштаба),
H2 (фильтр Калмана вместо EMA), H3 (восстановление захвата) подменой `PathTracker` в памяти; train,
clean и `scale_up`/`scale_down`/`gap_both_30`, ложные привязки — по GNSS-эталону (только в eval;
ветки переводятся в сетку MGRS эталона, #159). `base` — масштаб пути `main` до #154 (плюс цепочка
скорости #153); `h123` — отвергнутый кандидат, код в ветке `worktree-154-wheel-scale` @ `20e3e1e`;
`h3` — только H3 поверх EMA с правилом «вторая стоянка ближе `stop_snap_max_m` пути к ожидающему
промаху — та же стоянка» (ревью #160), ядро ветки `worktree-154-wheel-scale-report` совпадает с ним
побайтно; `h3_v0` — H3 без этого правила (замороженный `6853330`); `h3_arc`, `h3_grow`, `h3_grow3` —
строже: та же стоянка ближе `scale_min_arc_m` или рост `|y₁|(q − 1)` в пределах 2σ / 3σ;
`h3_chain_from` — цепочка скорости #153 пропускает и пару от места восстановления (отчёт
`docs/verification/2026-09-27-issue154-wheel-scale.md`, разделы 8–9).

```bash
.venv/bin/python tools/eval/experiments/wheel_scale_exp.py train                 # все варианты, ~8 мин на 12 процессах
.venv/bin/python tools/eval/experiments/wheel_scale_exp.py train base h3_v0 h3 h3_arc h3_grow h3_grow3 h3_chain_from   # правило q ≈ 1, ~7 мин
```

Расширенная оценка (#154, evaluator): `wheel_scale_ext.py` — прогоны, `wheel_scale_stats.py` —
статистика (markdown в stdout). Запуск из корня репозитория; вывод — `out/wheel_scale_ext/<режим>-<набор>/`,
по файлу на bag × вариант (готовые файлы не пересчитываются). Holdout — только `grid` и только
замороженные кандидаты `base`, `h123`, `h3` (D-011). `WSX_BAGS=a,b` — короткая проверка на части bag,
`WSX_JOBS` — процессы (20), `WSX_OUT` — каталог вывода. Готовый файл другого `cfg` под тем же именем
варианта (`h3` до правила q ≈ 1) считается заново.

```bash
.venv/bin/python tools/eval/experiments/wheel_scale_ext.py grid train base h123 h3   # 54 ячейки: масштаб 0,97…1,03 × старт, gap_both 30/60 с
.venv/bin/python tools/eval/experiments/wheel_scale_ext.py std train base h3 h123_k1.5   # clean + scale_up/scale_down/gap_both_30, чувствительность
.venv/bin/python tools/eval/experiments/wheel_scale_ext.py verify <dir> core <bag> ...  # Estimates ядра (или варианта) для побайтной сверки
.venv/bin/python tools/eval/experiments/wheel_scale_ext.py cmp <dir1> <dir2>
.venv/bin/python tools/eval/experiments/wheel_scale_stats.py pairs out/eval/<base>/metrics.json out/eval/<head>/metrics.json   # бутстрэп, знаковый, Wilcoxon, перестановки
.venv/bin/python tools/eval/experiments/wheel_scale_stats.py grid out/wheel_scale_ext/grid-train   # и sensgrid, curve; std out/wheel_scale_ext/std-train
```
