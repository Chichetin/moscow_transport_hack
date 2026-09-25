"""Plot GNSS master tracks of all bags on one map: docs/data/routes.png.

Usage: .venv/bin/python tools/survey/plot_routes.py [--data $TRAM_DATA_DIR]
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader

from survey_bags import GNSS_FIX, REPO, default_data_dir, typestore

LAT0, LON0 = 55.8104, 37.4623  # reference point near the depot (first bag start)


def track(path: Path) -> np.ndarray:
    with AnyReader([path], default_typestore=typestore()) as r:
        conns = [c for c in r.connections if c.topic == GNSS_FIX]
        pts = [(m.latitude, m.longitude) for c, _, raw in r.messages(connections=conns)
               for m in [r.deserialize(raw, c.msgtype)] if m.status.status >= 0]
    if not pts:
        return np.zeros((0, 2))
    p = np.asarray(pts)
    x = np.radians(p[:, 1] - LON0) * 6378137.0 * math.cos(math.radians(LAT0))
    y = np.radians(p[:, 0] - LAT0) * 6378137.0
    return np.column_stack([x, y])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', type=Path, default=default_data_dir())
    ap.add_argument('--out', default=REPO / 'docs' / 'data' / 'routes.png')
    args = ap.parse_args()
    fig, ax = plt.subplots(figsize=(10, 10))
    for bag in sorted(Path(args.data).iterdir()):
        if not (bag / 'metadata.yaml').exists():
            continue
        xy = track(bag)
        if len(xy):
            color = 'tab:blue' if bag.name.startswith('30618') else 'tab:red'
            ax.plot(xy[::10, 0], xy[::10, 1], lw=0.5, color=color, alpha=0.5)
            ax.plot(*xy[0], 'o', ms=2, color=color)
    ax.set_aspect('equal')
    ax.set_xlabel('x, м (восток)')
    ax.set_ylabel('y, м (север)')
    ax.set_title('GNSS master, все прогоны: синие 30618, красные 30639; точки — старт')
    fig.savefig(args.out, dpi=110, bbox_inches='tight')
    print(args.out)


if __name__ == '__main__':
    main()
