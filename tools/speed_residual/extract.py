"""Train rows of the speed residual v_GNSS - v_pipeline: one row per matched reference sample.

The pipeline runs exactly as in tools/eval (recording order, GNSS cut after the init window);
features come from `FeatureTap` after each step. Only the `train` split is accepted.

    .venv/bin/python tools/speed_residual/extract.py --out <dir> --jobs 20
"""
from __future__ import annotations

import argparse
import re
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'tools' / 'eval'))
sys.path.insert(0, str(HERE))

from tram_eval import bag as bagmod                      # noqa: E402
from tram_eval.metrics import match_nearest, speed_modes  # noqa: E402
from tram_eval.reference import build_reference          # noqa: E402
from features import FEATURES, FeatureTap                # noqa: E402

MODES = ('accel', 'brake', 'stop', 'cruise')


def bag_days() -> dict[str, str]:
    """Recording day of every bag from the comments of splits.yaml."""
    text = bagmod.SPLITS_YAML.read_text(encoding='utf-8')
    return dict(re.findall(r"'(\d+_[0-9a-f]+)'\s*#\s*(\d{4}-\d{2}-\d{2})", text))


def extract_bag(name: str) -> dict:
    msgs = bagmod.read_bag(bagmod.data_dir() / name)
    window_end = bagmod.gnss_window_end(msgs, bagmod.default_gnss_window())
    odo = bagmod.default_odometry()
    tap = FeatureTap()
    t, rows = [], []
    for topic, msg in msgs:
        if topic in bagmod.GNSS and bagmod.stamp(msg) > window_end:
            continue
        est = odo.step(bagmod.to_raw(topic, msg))
        x = tap.observe(odo, est)
        if est is not None:
            t.append(est.t)
            rows.append(x)
    t = np.asarray(t, float)
    X = np.asarray(rows, float).reshape(-1, len(FEATURES))
    ref = build_reference(*bagmod.reference_inputs(msgs), window_end)
    ri, ei = match_nearest(ref.vel_t, t)
    mode = speed_modes(ref.vel_t, ref.speed)[ri]
    return {'bag': name, 't': ref.vel_t[ri], 'v_ref': ref.speed[ri], 'X': X[ei],
            'mode': np.array([MODES.index(m) for m in mode], np.int8),
            'after_window': ref.vel_t[ri] > window_end}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--bag', action='append', help='train bag(s); default: whole train split')
    a = ap.parse_args()
    train = [str(b) for b in bagmod.load_splits()['train']]
    bags = a.bag or train
    if any(b not in train for b in bags):
        raise SystemExit('only train bags are allowed')
    days = bag_days()
    a.out.mkdir(parents=True, exist_ok=True)
    with ProcessPoolExecutor(a.jobs) as pool:
        for r in pool.map(extract_bag, bags):
            np.savez_compressed(a.out / f"{r['bag']}.npz", day=days[r['bag']], features=np.array(FEATURES),
                                **{k: v for k, v in r.items() if k != 'bag'})
            print(r['bag'], days[r['bag']], len(r['t']), flush=True)


if __name__ == '__main__':
    main()
