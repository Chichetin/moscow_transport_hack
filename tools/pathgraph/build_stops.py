"""Build the stop places src/tram_odometry/maps/stops.csv from GNSS master of train bags (D-033).

Usage:
    .venv/bin/python tools/pathgraph/build_stops.py [--data $TRAM_DATA_DIR] [--route <route.csv>] [--out <stops.csv>]

A stop is a stretch where the Doppler speed of GNSS master stays under STOP_SPEED_MPS for at
least STOP_MIN_S. Its place is the median of the fixes of the stretch projected onto the
route map: (branch, s). Stops of all train bags are clustered along each branch; a cluster
seen in at least MIN_BAGS bags is a stop place (platform or signal that recurs), the rest are
one-off stops and are dropped. Only train bags (tools/eval/splits.yaml) are used.
"""
from __future__ import annotations

import argparse
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

import build_route as br

OUT = br.REPO / 'src' / 'tram_odometry' / 'maps' / 'stops.csv'
MASTER_VEL = '/sensing/gnss/master/vel'
STOP_SPEED_MPS = 0.3    # m/s, Doppler speed under this is standing (docs/contracts.md §4)
STOP_MIN_S = 5.0        # s, shortest stretch that counts as a stop
MAX_OFF_M = 10.0        # m, a stop farther than this from every branch is off the map
EDGE_M = 2.0            # m, a stop this close to a branch end is a stop beyond it: the projection clamps
GAP_M = 8.0             # m, stops closer than this along a branch are one place
MIN_BAGS = 5            # bags that must show a place for it to count as recurring


def read_master_vel(bag: Path) -> np.ndarray:
    """(stamp s, Doppler speed m/s) of GNSS master, sorted by header.stamp."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    rows = []
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        conns = [c for c in reader.connections if c.topic == MASTER_VEL]
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         float(np.hypot(m.twist.linear.x, m.twist.linear.y))))
    a = np.array(rows, float).reshape(-1, 2)
    return a[np.argsort(a[:, 0], kind='stable')]


def stop_stretches(t: np.ndarray, speed: np.ndarray) -> list[tuple[float, float]]:
    """(start, end) stamps of the stretches with speed < STOP_SPEED_MPS lasting >= STOP_MIN_S."""
    slow = np.isfinite(speed) & (speed < STOP_SPEED_MPS)
    edges = np.flatnonzero(np.diff(np.concatenate([[0], slow.astype(int), [0]])))
    out = []
    for a, b in zip(edges[::2], edges[1::2]):
        if t[b - 1] - t[a] >= STOP_MIN_S:
            out.append((float(t[a]), float(t[b - 1])))
    return out


def edge_free(s: np.ndarray, at: float) -> bool:
    """False for a projection clamped onto the first or last point of a branch."""
    return bool(s[0] + EDGE_M < at < s[-1] - EDGE_M)


def stops_of_bag(bag: Path, route: dict) -> list[tuple[int, float]]:
    """(branch, s) of every stop of one bag on the route map."""
    fixes = br.read_master_fixes(bag)
    if not len(fixes):
        return []
    vel = read_master_vel(bag)
    statuses = (br.MAP_STATUS,) if (fixes[:, 4] == br.MAP_STATUS).any() else (0, 1, 2)
    ft, xy, _ = br.clean_track(fixes, statuses)
    out = []
    for a, b in stop_stretches(vel[:, 0], vel[:, 1]):
        sel = (ft >= a) & (ft <= b)
        if sel.sum() < 3:
            continue
        p = np.median(xy[sel], axis=0, keepdims=True)
        best = None
        for k, (s, poly, _) in route.items():
            ps, e = br.project(s, poly, p)
            if best is None or abs(e[0]) < best[2]:
                best = (k, float(ps[0]), float(abs(e[0])))
        if best[2] <= MAX_OFF_M and edge_free(route[best[0]][0], best[1]):
            out.append((best[0], best[1]))
    return out


def stop_places(events: list[tuple[str, int, float]]) -> list[tuple[int, float, int]]:
    """Cluster (bag, branch, s) events into places (branch, median s, bags), MIN_BAGS or more."""
    places = []
    for branch in sorted({e[1] for e in events}):
        ev = sorted((e[2], e[0]) for e in events if e[1] == branch)
        groups = [[ev[0]]]
        for s, name in ev[1:]:
            if s - groups[-1][-1][0] > GAP_M:
                groups.append([])
            groups[-1].append((s, name))
        for g in groups:
            bags = len({n for _, n in g})
            if bags >= MIN_BAGS:
                places.append((branch, float(np.median([s for s, _ in g])), bags))
    return places


def write_stops(path: Path, places: list[tuple[int, float, int]], header: str) -> None:
    """maps/stops.csv, docs/contracts.md §5."""
    lines = [f'# {header}', 'branch,s_m,n_bags']
    lines += [f'{b},{s:.1f},{n}' for b, s, n in places]
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _bag_events(args):
    bag, route = args
    return [(bag.name, k, s) for k, s in stops_of_bag(bag, route)]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', type=Path, default=br.default_data_dir())
    ap.add_argument('--route', type=Path, default=br.OUT)
    ap.add_argument('--out', type=Path, default=OUT)
    args = ap.parse_args()
    route = br.read_route(args.route)
    bags = [args.data / n for n in br.split_bags(br.SPLITS, 'train')]
    with ProcessPoolExecutor(max(1, (os.cpu_count() or 2) - 2)) as ex:
        events = [e for ev in ex.map(_bag_events, [(b, route) for b in bags]) for e in ev]
    places = stop_places(events)
    header = (f'stop places: (branch, s_m of route.csv), bags = train bags that stop there; '
              f'{len(events)} stops of {len(bags)} train bags (tools/eval/splits.yaml), '
              f'speed < {STOP_SPEED_MPS} m/s for >= {STOP_MIN_S} s, clusters of >= {MIN_BAGS} bags, '
              f'tools/pathgraph/build_stops.py')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_stops(args.out, places, header)
    print(f'{len(places)} places, {args.out}')


if __name__ == '__main__':
    main()
