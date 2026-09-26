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
