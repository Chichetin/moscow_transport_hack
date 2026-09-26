#!/usr/bin/env python3
"""Experiment #59: speed filter (pipeline default) against the min/max wheel selector.

Runs the same pipeline.Odometry on a split three times: with its speed filter, with
estimator.selector.WheelSelector in place of the filter, and with a control that always
takes the mean of the trusted bogies. Prints a markdown table: medians over bags, the
bias per driving mode, and the bags where the bogies disagree (docs/data.md trap 8, as
annotated in splits.yaml).

In practice the drive model is never exactly 0 (Davis resistance), so the selector takes
the lower bogie under traction and the higher one on coast and braking; its mean branch
does not fire. The mean control reports Estimate.accel = 0 (no effect on the metrics).

    .venv/bin/python tools/eval/exp_selector.py --split holdout --jobs 4
"""
from __future__ import annotations

import argparse
import statistics
import sys
import re
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tram_eval import bag  # noqa: E402

VARIANTS = ('filter', 'selector', 'mean')
METRICS = ('speed_rmse', 'speed_mae', 'speed_bias_accel', 'speed_bias_brake',
           'speed_bias_stop', 'speed_bias_cruise', 'drift_pct', 'along_rmse')
# docs/data.md trap 8: splits.yaml annotates bags whose bogies disagree by > 1 km/h
DISAGREE = re.compile(r"'(\S+)'\s*#.*front/rear >1 km/h ([0-9.]+)%")


def make(variant: str):
    def factory():
        odo = bag.default_odometry()
        if variant == 'filter':
            return odo
        from tram_odometry_core.estimator.selector import WheelSelector

        class MeanOnly(WheelSelector):
            def predict(self, t, accel_model):
                super().predict(t, accel_model)
                self._accel = 0.0
        odo._filter = (MeanOnly if variant == 'mean' else WheelSelector)(odo.params)
        return odo
    return factory


def disagreement() -> dict:
    """bag -> % of motion time the bogies disagree, from the splits.yaml annotations."""
    text = (Path(__file__).resolve().parent / 'splits.yaml').read_text()
    return {m.group(1): float(m.group(2)) for m in DISAGREE.finditer(text)}


def run(path: Path) -> dict:
    msgs = bag.read_bag(path)
    w = bag.default_gnss_window()
    out = {'bag': path.name}
    for v in VARIANTS:
        out[v] = bag.evaluate_bag(path, w, make_odometry=make(v), msgs=msgs)
    return out


def fmt(x):
    return '—' if x is None else f'{x:.4f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--split', default='holdout')
    ap.add_argument('--jobs', type=int, default=1)
    args = ap.parse_args()
    names = bag.load_splits()[args.split]
    paths = [bag.data_dir() / n for n in names]
    with ProcessPoolExecutor(args.jobs) as pool:
        rows = list(pool.map(run, paths))

    def med(v, k, subset=rows):
        xs = [r[v].get(k) for r in subset if r[v].get(k) is not None and not r[v].get('crashed')]
        return statistics.median(xs) if xs else None

    print(f'split {args.split}: {len(rows)} bag; crashed: '
          + ', '.join(f'{v} {sum(bool(r[v].get("crashed")) for r in rows)}' for v in VARIANTS))
    print('\n| медиана по bag | ' + ' | '.join(VARIANTS) + ' |\n|---|' + '---|' * len(VARIANTS))
    for k in METRICS:
        print(f'| {k} | ' + ' | '.join(fmt(med(v, k)) for v in VARIANTS) + ' |')
    share = disagreement()
    split = [r for r in rows if r['bag'] in share]
    print(f'\nbag с расхождением тележек (ловушка 8, splits.yaml): {len(split)}')
    print('\n| bag | расхождение, % времени | ' + ' | '.join(f'{v} speed_rmse' for v in VARIANTS)
          + ' | ' + ' | '.join(f'{v} drift_pct' for v in VARIANTS) + ' |')
    print('|---|---|' + '---|' * (2 * len(VARIANTS)))
    for r in sorted(split, key=lambda r: -share[r['bag']]):
        print(f'| {r["bag"]} | {share[r["bag"]]:.1f} | '
              + ' | '.join(fmt(r[v].get('speed_rmse')) for v in VARIANTS) + ' | '
              + ' | '.join(fmt(r[v].get('drift_pct')) for v in VARIANTS) + ' |')
    worst = {v: max(rows, key=lambda r: r[v].get('speed_rmse') or 0) for v in VARIANTS}
    print('\nхудший bag по speed_rmse: ' + '; '.join(
        f'{v} {worst[v]["bag"]} {fmt(worst[v][v].get("speed_rmse"))}' for v in VARIANTS))


if __name__ == '__main__':
    main()
