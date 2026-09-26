#!/usr/bin/env python3
"""Issue #18: coordinate-descent search of the speed filter noises and the slip thresholds on train.

Starts from params.yaml and walks the keys of GRID in order: for each key it evaluates every grid
value with the other keys at the current best, and keeps the value with the lowest score if it
beats the current best by `--margin` and no D-012 main metric is worse than at the start by more
than 2 %. Rounds repeat until a round changes nothing (or `--rounds`). Then `--random N` joint
samples of GRID (random.Random(--seed)) get the same acceptance rule: coordinate descent does not
see moves that help only together. The result depends only on the code, params.yaml, GRID, the
seed and the bags.

The score is the mean over the D-012 main metrics (speed_rmse, along_rmse, drift_pct) of
median(candidate) / median(start); the medians are those of tools/eval (metrics.summarize).
Parameters are replaced in memory (dataclasses.replace), params.yaml is not written.

Only train: holdout decides the merge (D-011, D-012) and is never used for the choice.

    .venv/bin/python tools/eval/tune_estimator.py --split train --jobs 2 --report <md>
    .venv/bin/python tools/eval/tune_estimator.py --split train --rounds 0 --random 16 --seed 18
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tram_eval import bag  # noqa: E402
from tram_eval.cli import D012_TOL  # noqa: E402
from tram_eval.metrics import MAIN_METRICS, summarize  # noqa: E402

# The tuned keys (issue #18: filter noises and slip thresholds) and their candidate values.
# Values span roughly x0.1..x10 around params.yaml for the noises; slip.noise_accel_mps2 stays
# above the train p99.9 of the clean residual, 2.3 m/s^2 (D-054).
GRID = {
    'filter.r_wheel': (0.005, 0.015, 0.05, 0.15, 0.5),
    'filter.q_accel': (0.05, 0.15, 0.5, 1.5, 5.0),
    'filter.q_bias': (0.0004, 0.02, 0.2, 2.0),
    'filter.initial_bias_var': (0.025, 0.25, 2.5),
    'filter.nis_gate': (4.0, 6.25, 9.0, 16.0, 25.0),
    'slip.front_rear_threshold_mps': (0.25, 0.35, 0.5, 0.7, 1.0),
    'slip.model_residual_threshold_mps2': (0.5, 1.0, 2.0),
    'slip.noise_accel_mps2': (2.5, 3.0, 4.0, 5.0),
    'slip.noise_hold_s': (0.5, 1.0, 2.0),
}
ALLOWED_SPLITS = ('train',)    # holdout, quick (its subset) and any other split decide or check, never tune
REPORT_METRICS = ('speed_rmse', 'speed_mae', 'speed_bias_accel', 'speed_bias_brake',
                  'speed_bias_stop', 'speed_bias_cruise', 'along_rmse', 'drift_pct', 'cross_rmse')


def override(params, values: dict):
    """Params with `section.key` fields replaced; only the keys of GRID may change."""
    from tram_odometry_core.types import _validate_filter
    for key in values:
        if key not in GRID:
            raise KeyError(f'{key} is not a tuned key (tune_estimator.GRID)')
    sections = {}
    for key, value in values.items():
        section, field = key.split('.')
        sections.setdefault(section, {})[field] = float(value)
    new = dataclasses.replace(params, **{s: dataclasses.replace(getattr(params, s), **f)
                                         for s, f in sections.items()})
    _validate_filter(new.filter)
    return new


def score(cand: dict, start: dict) -> float:
    """Mean of median(candidate) / median(start) over the D-012 main metrics; a lost one is inf."""
    ratios = []
    for k in MAIN_METRICS:
        if cand.get(k) is None:
            return float('inf')
        ratios.append(cand[k] / start[k])
    return sum(ratios) / len(ratios)


def d012_ok(cand: dict, start: dict) -> bool:
    """No main metric worse than at the start by more than D012_TOL (cli.summary_table rule)."""
    return all(start.get(k) is None or (cand.get(k) is not None and cand[k] <= start[k] * (1 + D012_TOL))
               for k in MAIN_METRICS)


def better(cand: dict, best: dict, start: dict, margin: float) -> bool:
    return d012_ok(cand, start) and score(cand, start) < score(best, start) - margin


def make_odometry(values: dict):
    def factory():
        odo = bag.default_odometry()
        return type(odo)(override(odo.params, values), route=odo.route)
    return factory


def run_bag(task) -> list[dict]:
    """Metrics of one bag for each config; the bag is read once."""
    path, window, configs = task
    msgs = bag.read_bag(path)
    out = []
    for values in configs:
        m = bag.evaluate_bag(path, window, make_odometry=make_odometry(values), msgs=msgs)
        m.pop(bag.NOTES, None)
        out.append(m)
    return out


def evaluate(configs: list[dict], paths, window, pool) -> list[dict]:
    """Median summary (metrics.summarize) of each config over the bags."""
    per_bag = list(pool.map(run_bag, [(p, window, configs) for p in paths]))
    results = []
    for i in range(len(configs)):
        bags = {p.name: rows[i] for p, rows in zip(paths, per_bag)}
        s = summarize(bags)['median']
        s['crashed'] = sum(bool(m['crashed']) for m in bags.values())
        results.append(s)
    return results


def sample(start: dict, n: int, seed: int) -> list[dict]:
    """n joint configs, each key drawn from its GRID values; none equal to `start`, no repeats."""
    rng, out, seen = random.Random(seed), [], {key_of(start)}
    space = 1
    for grid in GRID.values():
        space *= len(grid)
    while len(out) < min(n, space - 1):
        c = {k: float(rng.choice(GRID[k])) for k in GRID}
        if key_of(c) not in seen:
            seen.add(key_of(c))
            out.append(c)
    return out


def key_of(values: dict) -> tuple:
    return tuple(sorted(values.items()))


def fmt(x) -> str:
    return '—' if x is None else f'{x:.4f}'


def row(label: str, values: dict, s: dict, start: dict) -> str:
    return (f'| {label} | ' + ' | '.join(fmt(s.get(k)) for k in REPORT_METRICS)
            + f" | {s['crashed']} | {score(s, start):.4f} | {'да' if d012_ok(s, start) else 'нет'} |")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--split', default='train', help='набор splits.yaml; holdout и quick запрещены (D-011)')
    ap.add_argument('--bag', action='append', help='только эти bag train (проверка скрипта)')
    ap.add_argument('--jobs', type=int, default=2)
    ap.add_argument('--rounds', type=int, default=2, help='проходов покоординатного спуска, максимум')
    ap.add_argument('--random', type=int, default=0, help='совместных случайных проб GRID после спуска')
    ap.add_argument('--seed', type=int, default=18, help='seed случайных проб')
    ap.add_argument('--margin', type=float, default=0.005,
                    help='на сколько score должен стать меньше лучшего, чтобы значение сменилось')
    ap.add_argument('--report', type=Path, help='markdown-отчёт; иначе только stdout')
    args = ap.parse_args(argv)
    if args.split not in ALLOWED_SPLITS:
        ap.error(f'{args.split}: подбор только на train, holdout решает merge (D-011, D-012)')
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    names = bag.load_splits()[args.split]
    if args.bag:
        if set(args.bag) - set(names):
            ap.error(f'не из {args.split}: {sorted(set(args.bag) - set(names))}')
        names = args.bag
    paths = [bag.data_dir() / n for n in names]
    window = bag.default_gnss_window()
    base = bag.default_odometry().params
    current = {k: float(getattr(getattr(base, k.split('.')[0]), k.split('.')[1])) for k in GRID}
    cache: dict = {}
    log: list[str] = []
    t0 = time.monotonic()

    with ProcessPoolExecutor(args.jobs) as pool:
        def run(configs):
            todo = [c for c in configs if key_of(c) not in cache]
            for c, s in zip(todo, evaluate(todo, paths, window, pool) if todo else []):
                cache[key_of(c)] = s
                print(f'[{time.monotonic() - t0:6.0f} s] {json.dumps(c)} -> '
                      + ', '.join(f'{k} {fmt(s[k])}' for k in MAIN_METRICS), flush=True)
            return [cache[key_of(c)] for c in configs]

        start = run([current])[0]
        best = start
        for rnd in range(1, args.rounds + 1):
            changed = False
            for key, grid in GRID.items():
                configs = [dict(current, **{key: float(v)}) for v in grid]
                for c, s in zip(configs, run(configs)):
                    tag = f'{key}={c[key]:g}' + (' (текущее)' if c[key] == current[key] else '')
                    log.append(row(f'{rnd} | {tag}', c, s, start))
                    if better(s, best, start, args.margin):
                        best, current, changed = s, c, True
                log.append(f'| {rnd} | → {key}={current[key]:g} |' + ' |' * (len(REPORT_METRICS) + 3))
            if not changed:
                break
        for i, c in enumerate(sample(current, args.random, args.seed), 1):
            s = run([c])[0]
            diff = ', '.join(f'{k}={v:g}' for k, v in c.items() if v != current[k])
            log.append(row(f'random {i} | {diff}', c, s, start))
            if better(s, best, start, args.margin):
                best, current = s, c
                log.append(f'| random {i} | → принято |' + ' |' * (len(REPORT_METRICS) + 3))

    header = ('| проход | значение | ' + ' | '.join(REPORT_METRICS) + ' | упало bag | score | D-012 |\n'
              + '|---|---|' + '---|' * (len(REPORT_METRICS) + 3))
    chosen = {k: v for k, v in current.items() if v != float(getattr(getattr(base, k.split('.')[0]), k.split('.')[1]))}
    lines = [f'split `{args.split}`, {len(paths)} bag, окно GNSS {window} с; поиск '
             f'{len(cache)} конфигураций за {time.monotonic() - t0:.0f} с; margin {args.margin}, '
             f'rounds {args.rounds}; random {args.random}, seed {args.seed}',
             '', 'Выбрано (отличия от params.yaml): '
             + (', '.join(f'`{k}` {getattr(getattr(base, k.split(".")[0]), k.split(".")[1]):g} → {v:g}'
                          for k, v in chosen.items()) or 'нет'),
             '', '| | ' + ' | '.join(REPORT_METRICS) + ' | упало bag | score | D-012 |',
             '|---|' + '---|' * (len(REPORT_METRICS) + 3),
             row('start (params.yaml)', {}, start, start),
             row('выбрано', current, best, start), '', '### Все шаги', '', header, *log]
    text = '\n'.join(lines)
    print('\n' + text)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    sys.exit(main())
