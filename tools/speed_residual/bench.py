"""Per-message cost of the correction in-process: Odometry alone vs CorrectedOdometry.

    .venv/bin/python tools/speed_residual/bench.py --model <json> --bag <train bag>
"""
from __future__ import annotations

import argparse
import sys
import time
import tracemalloc
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'tools' / 'eval'))
sys.path.insert(0, str(HERE))

from tram_eval import bag as bagmod     # noqa: E402
from features import CorrectedOdometry  # noqa: E402
from tree_model import ObliviousTrees   # noqa: E402


def timed(odo, msgs, window_end):
    dts = []
    for topic, msg in msgs:
        if topic in bagmod.GNSS and bagmod.stamp(msg) > window_end:
            continue
        t0 = time.perf_counter()
        odo.step(bagmod.to_raw(topic, msg))
        dts.append(time.perf_counter() - t0)
    return np.array(dts) * 1e6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--bag', required=True)
    a = ap.parse_args()
    if a.bag not in [str(b) for b in bagmod.load_splits()['train']]:
        raise SystemExit('train bags only')
    msgs = bagmod.read_bag(bagmod.data_dir() / a.bag)
    window_end = bagmod.gnss_window_end(msgs, bagmod.default_gnss_window())
    tracemalloc.start()
    model = ObliviousTrees(a.model)
    model_bytes = tracemalloc.get_traced_memory()[0]
    tracemalloc.stop()
    for name, make in (('Odometry', bagmod.default_odometry),
                       ('CorrectedOdometry', lambda: CorrectedOdometry(bagmod.default_odometry(), model, 0.03))):
        us = timed(make(), msgs, window_end)
        print(f'{name:18s} n={len(us)} mean {us.mean():6.1f} us  p50 {np.median(us):6.1f}  p99 {np.quantile(us, 0.99):7.1f}  max {us.max():8.1f}')
    print(f'model {Path(a.model).name}: {len(model.trees)} trees, Python objects ~{model_bytes / 1e6:.1f} MB, '
          f'JSON {Path(a.model).stat().st_size / 1e6:.1f} MB')


if __name__ == '__main__':
    main()
