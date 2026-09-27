"""Streaming leave-one-day-out check: tools/eval metrics of every train bag of a day, pipeline
alone vs pipeline + correction fitted without that day (fit_folds.py output).

    .venv/bin/python tools/speed_residual/stream_eval.py --models <dir> --tag d6_800 --clip 0.03 --out <json>
"""
from __future__ import annotations

import argparse
import functools
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'tools' / 'eval'))
sys.path.insert(0, str(HERE))

from tram_eval import bag as bagmod            # noqa: E402
from tram_eval.metrics import summarize        # noqa: E402
from extract import bag_days                   # noqa: E402
from features import CorrectedOdometry         # noqa: E402
from tree_model import ObliviousTrees          # noqa: E402


def make_corrected(model_path: str, clip: float, integrate: bool):
    return CorrectedOdometry(bagmod.default_odometry(), ObliviousTrees(model_path), clip, integrate)


def run(job):
    name, variant, model_path, clip = job
    make = None
    if variant != 'base':
        make = functools.partial(make_corrected, model_path, clip, variant == 'integrate')
    m = bagmod.evaluate_bag(bagmod.data_dir() / name, bagmod.default_gnss_window(), make_odometry=make)
    m.pop(bagmod.NOTES, None)
    return name, variant, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--models', required=True, type=Path)
    ap.add_argument('--tag', required=True)
    ap.add_argument('--clip', type=float, default=0.03)
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--jobs', type=int, default=20)
    a = ap.parse_args()
    train = [str(b) for b in bagmod.load_splits()['train']]
    days = bag_days()
    jobs = []
    for name in train:
        model = str(a.models / f'{a.tag}_{days[name]}.json')
        for variant in ('base', 'output', 'integrate'):
            jobs.append((name, variant, model, a.clip))
    res = {'base': {}, 'output': {}, 'integrate': {}}
    with ProcessPoolExecutor(a.jobs) as pool:
        for name, variant, m in pool.map(run, jobs):
            res[variant][name] = m
    out = {'tag': a.tag, 'clip': a.clip, 'days': {n: days[n] for n in train}, 'bags': res,
           'summary': {}}
    for day in sorted(set(days[n] for n in train)):
        for variant in res:
            bags = {n: m for n, m in res[variant].items() if days[n] == day}
            out['summary'][f'{day}/{variant}'] = summarize(bags)
    a.out.write_text(json.dumps(out, indent=1))
    keys = ('speed_rmse', 'speed_mae', 'along_rmse', 'along_max', 'drift_pct', 'cross_rmse',
            'speed_bias_accel', 'speed_bias_brake', 'speed_bias_stop', 'speed_bias_cruise')
    print('| fold day / variant | ' + ' | '.join(keys) + ' |')
    for k, s in out['summary'].items():
        print(f'| {k} | ' + ' | '.join(f"{s['median'][q]:.4f}" if s['median'][q] is not None else '—'
                                       for q in keys) + ' |')
    for variant in ('output', 'integrate'):
        for q in ('speed_rmse', 'along_rmse', 'drift_pct'):
            d = []
            for n in train:
                b, c = res['base'][n].get(q), res[variant][n].get(q)
                if b and c is not None and res['base'][n]['n_matched'] > 3000:
                    d.append((c - b) / b * 100)
            d = np.array(d)
            print(f'{variant} {q}: per-bag change % median {np.median(d):+.2f}, worst {d.max():+.2f}, '
                  f'better {int((d < 0).sum())}/{len(d)}')


if __name__ == '__main__':
    main()
