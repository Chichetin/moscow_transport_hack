"""Check maps/route.csv against GNSS master of holdout bags: cross-track distance of the fixes.

Usage:
    .venv/bin/python tools/pathgraph/check_route.py [--route <route.csv>] [--plot <png>]

"own" — distance to the main branch of the bag's direction or to a terminal branch (>= 2): the
stricter number, the two main tracks are ~8 m apart; "nearest" — to the closest branch. Fixes: every status >= 0, the same
outlier filter as the map. Prints a markdown table.
"""
from __future__ import annotations

import argparse
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

import build_route as br

STATUSES = (0, 1, 2)  # NavSatFix status >= 0: every fix with a position


def _load(bag: Path):
    f = br.read_master_fixes(bag)
    f = f[np.isin(f[:, 4], STATUSES) & np.all(np.isfinite(f[:, 1:4]), axis=1)]
    e, n, _ = br.lla_to_enu(f[:, 1], f[:, 2], f[:, 3], *br.ORIGIN)
    xy = np.column_stack([e, n])
    keep = br.reject_outliers(xy, br.OUTLIER_WIN, br.OUTLIER_M)
    return bag.name, xy[keep], f[keep, 4]


def cross_track(route: dict, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """|lateral| distance to the bag's own branches and to the nearest branch."""
    dist = np.column_stack([np.abs(br.project(s, poly, xy)[1]) for s, poly in route.values()])
    if abs(xy[0, 0] - xy[-1, 0]) >= br.MIN_SPAN_M:
        main = 0 if xy[0, 0] > xy[-1, 0] else 1
        own = np.min(dist[:, [main] + list(range(2, dist.shape[1]))], axis=1)
    else:
        own = dist.min(axis=1)
    return own, dist.min(axis=1)


def _stats(d: np.ndarray) -> str:
    return (f'{d.mean():.2f} | {np.median(d):.2f} | {np.percentile(d, 95):.2f} | '
            f'{np.percentile(d, 99):.2f} | {d.max():.1f}')


def plot(route: dict, tracks: list, path: Path) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    views = [('весь маршрут', None), ('восточная конечная', (-40, -50, 90)),
             ('западная конечная', (-4560, -1185, 90)), ('перегон, две колеи', (-1000, -575, 25))]
    fig, axs = plt.subplots(2, 2, figsize=(16, 13))
    for ax, (title, box) in zip(axs.flat, views):
        for _, xy, _ in tracks:
            ax.plot(xy[:, 0], xy[:, 1], '.', ms=0.6 if box else 0.3, color='0.6', alpha=0.4)
        for b, (s, poly) in route.items():
            ax.plot(poly[:, 0], poly[:, 1], '-', lw=1.2, color=f'C{(3, 0, 2, 1, 4, 5)[b]}',
                    label=('ветка 0 — на запад', 'ветка 1 — на восток')[b] if b < 2
                    else f'ветка {b} — путь конечной')
        if box:
            ax.set_xlim(box[0] - box[2], box[0] + box[2])
            ax.set_ylim(box[1] - box[2], box[1] + box[2])
        ax.set_aspect('equal')
        ax.set_title(f'{title}: карта (train) и GNSS master holdout (серые точки)')
        ax.set_xlabel('x, м (восток)')
        ax.set_ylabel('y, м (север)')
        ax.grid(True, lw=0.3)
    axs[0, 0].legend(loc='lower right')
    fig.savefig(path, dpi=80, bbox_inches='tight')


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', type=Path, default=br.default_data_dir())
    ap.add_argument('--route', type=Path, default=br.OUT)
    ap.add_argument('--plot', type=Path)
    args = ap.parse_args()
    route = br.read_route(args.route)
    bags = [args.data / n for n in br.split_bags(br.SPLITS, 'holdout')]
    with ProcessPoolExecutor(max(1, (os.cpu_count() or 2) - 2)) as ex:
        tracks = [t for t in ex.map(_load, bags) if len(t[1]) > br.OUTLIER_WIN]
    own_all, near_all, status_all = [], [], []
    print('| bag | фиксов | own mean, м | own p99, м | nearest mean, м |')
    print('|---|---|---|---|---|')
    for name, xy, status in tracks:
        own, near = cross_track(route, xy)
        own_all.append(own)
        near_all.append(near)
        status_all.append(status)
        print(f'| {name} | {len(xy)} | {own.mean():.2f} | {np.percentile(own, 99):.2f} | {near.mean():.2f} |')
    own, near, status = map(np.concatenate, (own_all, near_all, status_all))
    print(f'\nholdout: {len(tracks)} bag, {len(own)} фиксов\n')
    print('| выборка | mean, м | median, м | p95, м | p99, м | max, м |')
    print('|---|---|---|---|---|---|')
    print(f'| own, все статусы | {_stats(own)} |')
    print(f'| nearest, все статусы | {_stats(near)} |')
    for st in STATUSES:
        if (status == st).any():
            print(f'| own, status {st} ({(status == st).mean():.0%}) | {_stats(own[status == st])} |')
    if args.plot:
        plot(route, tracks, args.plot)
        print(f'\n{args.plot}')


if __name__ == '__main__':
    main()
