"""Where trams go past the start of a side branch and how fast (#138, D-074): the evidence for
`position.side_speed_mps`, `side_min_m`, `side_max_m`.

Usage:
    .venv/bin/python tools/pathgraph/side_speeds.py [--split train] [--split holdout]

For every bag with GNSS: a passage is master fixes on the parent branch (within PASS_M) crossing
the side branch start s_f with s growing. The wheel path (front bogie, km/h -> m/s) after the
crossing gives the window [side_min_m, side_max_m]; the maximum wheel speed there is the
feature. Where the tram went: the branch nearest to the fix 100 m of wheel path later (or the
last fix). Offline analysis only; GNSS here is the reference, the tracker never sees it.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'tools' / 'eval'))
sys.path.insert(0, str(REPO / 'src' / 'tram_odometry_core'))
from rosbags.highlevel import AnyReader  # noqa: E402
from tram_eval import bag as bagmod  # noqa: E402
from tram_eval.reference import geodetic_to_enu  # noqa: E402
from tram_odometry_core.position import PathTracker  # noqa: E402
from tram_odometry_core.types import load_params, load_route  # noqa: E402

PASS_M = 4.0          # m, a fix this close to the parent branch is on it
BEFORE_M = 40.0       # m, a passage starts on the parent at most this far before s_f
DEST_M = 100.0        # m of wheel path after s_f where the destination is read
KMH = 1.0 / 3.6
FIX = '/sensing/gnss/master/fix'
FRONT = '/vehicle/front_bogie_velocity'


def read(path: Path):
    fix, wheel = [], []
    with AnyReader([path], default_typestore=bagmod.typestore()) as rd:
        cons = [c for c in rd.connections if c.topic in (FIX, FRONT)]
        for c, _, raw in rd.messages(connections=cons):
            m = rd.deserialize(raw, c.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if c.topic == FIX:
                if m.status.status >= 0 and m.latitude != 0.0:
                    fix.append((t, m.latitude, m.longitude, m.altitude))
            elif np.isfinite(m.velocity):
                wheel.append((t, max(0.0, m.velocity) * KMH))
    return np.array(sorted(fix)), np.array(sorted(wheel))


def nearest(branches, p):
    """(branch, s, distance) of the nearest map point."""
    best = (0, 0.0, np.inf)
    for k, (s, xy) in enumerate(branches):
        d = np.hypot(xy[:, 0] - p[0], xy[:, 1] - p[1])
        i = int(np.argmin(d))
        if d[i] < best[2]:
            best = (k, float(s[i]), float(d[i]))
    return best


def passages(name, split, route, branches, parent, s_f, side, p):
    fix, wheel = read(bagmod.data_dir() / name)
    if len(fix) < 10 or len(wheel) < 10:
        return []
    xy = geodetic_to_enu(fix[:, 1], fix[:, 2], fix[:, 3], route.origin)[:, :2]
    s_par, xy_par = branches[parent]
    on = []
    for q in xy:
        d = np.hypot(xy_par[:, 0] - q[0], xy_par[:, 1] - q[1])
        i = int(np.argmin(d))
        on.append((float(s_par[i]), float(d[i])))
    rows, i = [], 1
    while i < len(fix):
        (s0, d0), (s1, d1) = on[i - 1], on[i]
        if d0 <= PASS_M and d1 <= PASS_M and s_f - BEFORE_M <= s0 < s_f <= s1:
            t0 = fix[i, 0]
            w = wheel[wheel[:, 0] >= t0]
            path = np.concatenate([[0.0], np.cumsum(np.diff(w[:, 0]) * w[1:, 1])])
            window = (path >= p.side_min_m) & (path <= p.side_max_m)
            vmax = float(w[window, 1].max()) if window.any() else float('nan')
            later = w[path >= DEST_M]
            t_dest = later[0, 0] if len(later) else fix[-1, 0]
            j = min(int(np.searchsorted(fix[:, 0], t_dest)), len(fix) - 1)
            dest = nearest(branches, xy[j])[0]
            rows.append((name, split, vmax, dest, j == len(fix) - 1))
            i += 50
            continue
        i += 1
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--split', action='append')
    args = ap.parse_args()
    p = load_params(REPO / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
    route = load_route(REPO / 'src' / 'tram_odometry' / 'maps' / p.position.map_file)
    tracker = PathTracker(p, route)
    sides = [(k, sd) for k, sd in enumerate(tracker._side) if sd is not None]
    branches = [(np.asarray(b.s), np.column_stack([b.x, b.y])) for b in route.branches]
    splits = bagmod.load_splits()
    for parent, (s_f, side) in sides:
        print(f'side branch {side} starts on branch {parent} at s = {s_f:.1f} m; window '
              f'{p.position.side_min_m:.0f}–{p.position.side_max_m:.0f} m, '
              f'threshold {p.position.side_speed_mps:.1f} m/s')
        print('| bag | split | max wheel speed in window, m/s | branch 100 m later | bag ends |')
        print('|---|---|---:|---:|---|')
        for split in args.split or ['train', 'holdout']:
            for name in splits[split]:
                for row in passages(name, split, route, branches, parent, s_f, side, p.position):
                    print(f'| {row[0]} | {row[1]} | {row[2]:.2f} | {row[3]} | {"yes" if row[4] else ""} |')


if __name__ == '__main__':
    main()
