"""#58: do wheel-only stops line up with the known stop places?

A stop is detected purely from the allowed inputs -- wheel speed under WHEEL_STOP_MPS for at
least WHEEL_STOP_MIN_S, no GNSS involved in the *detection*. GNSS is then used only to find
the stop's true (branch, s) on the route map, so its distance to the nearest known place in
`maps/stops.csv` (built from GNSS train, #56/D-034) can be measured. That distance is the
question PO2 needs answered: can the pipeline snap to a stop place using wheel/controller
signals alone, without GNSS mid-run?

Usage:
    .venv/bin/python notebooks/identification/stops_vs_speed.py [--data DIR] [--split train]
    .venv/bin/python notebooks/identification/stops_vs_speed.py --split holdout   # report only
"""
from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools' / 'pathgraph'))
import build_route as br      # noqa: E402
import build_stops as bstops  # noqa: E402

REPO = br.REPO
FRONT = '/vehicle/front_bogie_velocity'
REAR = '/vehicle/rear_bogie_velocity'
CMD = '/vehicle/driver_position_cmd'
WHEEL_SPEED_SCALE = 1.0 / 3.6   # km/h -> m/s (D-003)
WHEEL_STOP_MPS = 0.1            # m/s, issue #58: wheel speed under this is standing
WHEEL_STOP_MIN_S = 3.0          # s, shortest stretch that counts as a stop
MAX_OFF_M = 10.0                # m, a stop farther than this from every branch is off the map
FEATURE_WINDOW_S = 5.0          # s, window used for the notch-before/after feature
ROUTE_CSV = REPO / 'src' / 'tram_odometry' / 'maps' / 'route.csv'
STOPS_CSV = REPO / 'src' / 'tram_odometry' / 'maps' / 'stops.csv'
OUT_PNG = REPO / 'docs' / 'data' / 'stops_vs_speed_hist.png'
MSG_DIR = REPO / 'src' / 'tram_vehicle_msgs' / 'msg'


def typestore():
    """ROS2_HUMBLE plus tram_vehicle_msgs (VelocitySensor, DriverControllerCommand)."""
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for name in ('VelocitySensor', 'DriverControllerCommand'):
        types.update(get_types_from_msg((MSG_DIR / f'{name}.msg').read_text(encoding='utf-8'),
                                        f'tram_vehicle_msgs/msg/{name}'))
    ts.register(types)
    return ts


@dataclass
class StopEvent:
    bag: str
    t_start: float
    t_end: float
    duration_s: float
    distance_since_prev_m: float
    notch_before: float
    notch_after: float
    branch: int | None       # None: no GNSS fix in the window, true position unknown
    s_m: float | None
    nearest_known_m: float | None


def read_wheel_and_cmd(bag: Path):
    """(t, v_kmh) for front, rear and (t, notch) for the controller, sorted by header.stamp."""
    from rosbags.highlevel import AnyReader
    front, rear, cmd = [], [], []
    with AnyReader([bag], default_typestore=typestore()) as reader:
        conns = [c for c in reader.connections if c.topic in (FRONT, REAR, CMD)]
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if conn.topic == FRONT:
                front.append((t, m.velocity))
            elif conn.topic == REAR:
                rear.append((t, m.velocity))
            else:
                cmd.append((t, m.position))

    def arr(rows):
        a = np.array(rows, float).reshape(-1, 2)
        return a[np.argsort(a[:, 0], kind='stable')]
    return arr(front), arr(rear), arr(cmd)


def asof(series: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Step-hold value of `series` (t, v) at each time in `t`; NaN before the first sample."""
    if len(series) == 0:
        return np.full(len(t), np.nan)
    idx = np.searchsorted(series[:, 0], t, side='right') - 1
    return np.where(idx >= 0, series[np.clip(idx, 0, len(series) - 1), 1], np.nan)


def combined_speed(front: np.ndarray, rear: np.ndarray) -> np.ndarray:
    """(t, v m/s): mean of the last known front/rear reading at every sample time of either.
    NaN while neither bogie has spoken yet -- not a stop (`wheel_stop_stretches` requires
    `isfinite`); a real reading is never turned into a false zero."""
    if len(front) == 0 and len(rear) == 0:
        return np.zeros((0, 2))
    t = np.union1d(front[:, 0] if len(front) else [], rear[:, 0] if len(rear) else [])
    v = np.nanmean(np.column_stack([asof(front, t), asof(rear, t)]), axis=1) * WHEEL_SPEED_SCALE
    return np.column_stack([t, v])


def wheel_stop_stretches(t: np.ndarray, v: np.ndarray) -> list[tuple[float, float]]:
    """(start, end) stamps of stretches with speed < WHEEL_STOP_MPS lasting >= WHEEL_STOP_MIN_S."""
    slow = np.isfinite(v) & (v < WHEEL_STOP_MPS)
    edges = np.flatnonzero(np.diff(np.concatenate([[0], slow.astype(int), [0]])))
    out = []
    for a, b in zip(edges[::2], edges[1::2]):
        if t[b - 1] - t[a] >= WHEEL_STOP_MIN_S:
            out.append((float(t[a]), float(t[b - 1])))
    return out


def cumulative_distance(t: np.ndarray, v: np.ndarray) -> np.ndarray:
    """m, integral of v dt from the first sample (monotone non-decreasing); NaN speed (neither
    bogie has spoken yet) contributes no distance, same as a silent tram at rest."""
    dt = np.diff(t, prepend=t[0] if len(t) else 0.0)
    return np.cumsum(np.maximum(np.nan_to_num(v, nan=0.0), 0.0) * dt)


def true_position(ft: np.ndarray, xy: np.ndarray, route: dict, t_start: float, t_end: float):
    """(branch, s_m) of the median GNSS fix in [t_start, t_end], or (None, None) off the map,
    without a fix in the window, or projecting onto a branch's clamped end (same rule as
    tools/pathgraph/build_stops.py:stops_of_bag)."""
    sel = (ft >= t_start) & (ft <= t_end)
    if sel.sum() < 3:
        return None, None
    p = np.median(xy[sel], axis=0, keepdims=True)
    best = None
    for k, (s, poly, _) in route.items():
        ps, e = br.project(s, poly, p)
        if best is None or abs(e[0]) < best[2]:
            best = (k, float(ps[0]), float(abs(e[0])))
    if best[2] > MAX_OFF_M or not bstops.edge_free(route[best[0]][0], best[1]):
        return None, None
    return best[0], best[1]


def nearest_known_stop_m(branch: int, s: float, stops: list[tuple[int, float]]) -> float:
    on_branch = [ks for kb, ks in stops if kb == branch]
    return min(abs(s - ks) for ks in on_branch) if on_branch else float('inf')


def bag_stop_events(bag: Path, route: dict, stops: list[tuple[int, float]]) -> list[StopEvent]:
    front, rear, cmd = read_wheel_and_cmd(bag)
    t, v = combined_speed(front, rear).T if len(front) or len(rear) else (np.array([]), np.array([]))
    if len(t) == 0:
        return []
    dist = cumulative_distance(t, v)
    fixes = br.read_master_fixes(bag)
    if len(fixes):
        statuses = (br.MAP_STATUS,) if (fixes[:, 4] == br.MAP_STATUS).any() else (0, 1, 2)
        ft, xy, _ = br.clean_track(fixes, statuses)
    else:
        ft, xy = np.zeros(0), np.zeros((0, 2))
    events = []
    prev_end_dist = 0.0
    for a, b in wheel_stop_stretches(t, v):
        before = cmd[(cmd[:, 0] >= a - FEATURE_WINDOW_S) & (cmd[:, 0] < a)]
        after = cmd[(cmd[:, 0] > b) & (cmd[:, 0] <= b + FEATURE_WINDOW_S)]
        branch, s = true_position(ft, xy, route, a, b) if len(ft) else (None, None)
        nearest = nearest_known_stop_m(branch, s, stops) if branch is not None else None
        events.append(StopEvent(
            bag=bag.name, t_start=a, t_end=b, duration_s=b - a,
            distance_since_prev_m=float(np.interp(a, t, dist)) - prev_end_dist,
            notch_before=float(np.median(before[:, 1])) if len(before) else float('nan'),
            notch_after=float(np.median(after[:, 1])) if len(after) else float('nan'),
            branch=branch, s_m=s, nearest_known_m=nearest))
        prev_end_dist = float(np.interp(b, t, dist))
    return events


def _bag_events(args):
    bag, route, stops = args
    return bag_stop_events(bag, route, stops)


def read_stops_csv(path: Path) -> list[tuple[int, float]]:
    rows = [ln for ln in path.read_text(encoding='utf-8').splitlines()
            if ln and not ln.startswith('#') and not ln.startswith('branch')]
    return [(int(b), float(s)) for b, s, _ in (ln.split(',') for ln in rows)]


def summarize(events: list[StopEvent]) -> None:
    located = [e for e in events if e.nearest_known_m is not None]
    on_covered_branch = [e for e in located if np.isfinite(e.nearest_known_m)]
    print(f'{len(events)} wheel stops, {len(located)} with a GNSS fix in the window '
          f'({len(events) - len(located)} off-map or without GNSS), '
          f'{len(located) - len(on_covered_branch)} on a branch with no known stop place')
    if not on_covered_branch:
        return
    d = np.array([e.nearest_known_m for e in on_covered_branch])
    print(f'distance to nearest known place, m (branches with a known place only): '
          f'p50={np.percentile(d, 50):.1f} p90={np.percentile(d, 90):.1f} '
          f'p99={np.percentile(d, 99):.1f}')
    thresholds = (5.0, 10.0, 20.0, 50.0)
    frac = ' '.join(f'<={t:.0f}m: {(d <= t).mean() * 100:.1f}%' for t in thresholds)
    print(f'  {frac}')
    near = [e for e in on_covered_branch if e.nearest_known_m <= 20.0]
    far = [e for e in on_covered_branch if e.nearest_known_m > 20.0]
    for label, group in (('near (<=20 m)', near), ('far (>20 m)', far)):
        if not group:
            continue
        dur = np.array([e.duration_s for e in group])
        gap = np.array([e.distance_since_prev_m for e in group])
        turn = np.array([abs(e.notch_after - e.notch_before) for e in group
                         if np.isfinite(e.notch_after) and np.isfinite(e.notch_before)])
        print(f'  {label}: n={len(group)} duration median={np.median(dur):.1f}s '
              f'distance_since_prev median={np.median(gap):.0f}m '
              f'|notch change| median={np.median(turn) if len(turn) else float("nan"):.1f}')


def plot_hist(events: list[StopEvent], out: Path) -> None:
    """matplotlib is a host/dev-image only dependency (docker/Dockerfile): imported here, not
    at module level, so importing this module (and running its tests) stays possible in the
    jury/CI container that only has numpy and rosbags."""
    d = np.array([e.nearest_known_m for e in events
                 if e.nearest_known_m is not None and np.isfinite(e.nearest_known_m)])
    if len(d) == 0:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.hist(np.clip(d, 0, 100), bins=40)
    ax.axvline(20.0, color='r', linestyle='--', label='stop_snap_max_m = 20 m')
    ax.set_xlabel('distance from a wheel-only stop to the nearest known place, m')
    ax.set_ylabel('count')
    ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', type=Path, default=br.default_data_dir())
    ap.add_argument('--split', default='train', choices=['train', 'holdout'])
    args = ap.parse_args()
    route = br.read_route(ROUTE_CSV)
    stops = read_stops_csv(STOPS_CSV)
    bags = [args.data / n for n in br.split_bags(br.SPLITS, args.split)]
    with ProcessPoolExecutor(max(1, (os.cpu_count() or 2) - 2)) as ex:
        events = [e for ev in ex.map(_bag_events, [(b, route, stops) for b in bags]) for e in ev]
    print(f'split={args.split}, {len(bags)} bags')
    summarize(events)
    if args.split == 'train':
        plot_hist(events, OUT_PNG)
        print(OUT_PNG)


if __name__ == '__main__':
    main()
