"""Survey of the organizers' rosbags without ROS: per-bag facts for docs/data.md.

Usage:
    .venv/bin/python tools/survey/survey_bags.py [--data $TRAM_DATA_DIR] [--out docs/data/survey.csv]

Reads every bag with `rosbags`, prints a summary and writes one CSV row per bag.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

REPO = Path(__file__).resolve().parents[2]


def default_data_dir() -> Path:
    """TRAM_DATA_DIR, else <main checkout>/dataset/data (works from any worktree)."""
    env = os.environ.get('TRAM_DATA_DIR')
    if env:
        return Path(env) if Path(env).is_absolute() else REPO / env
    common = subprocess.run(['git', '-C', str(REPO), 'rev-parse', '--path-format=absolute',
                             '--git-common-dir'], capture_output=True, text=True).stdout.strip()
    return (Path(common).parent if common else REPO) / 'dataset' / 'data'
MSG_DIR = REPO / 'src' / 'tram_vehicle_msgs' / 'msg'
FRONT = '/vehicle/front_bogie_velocity'
REAR = '/vehicle/rear_bogie_velocity'
CMD = '/vehicle/driver_position_cmd'
GNSS_FIX = '/sensing/gnss/master/fix'
GNSS_VEL = '/sensing/gnss/master/vel'
ROVER_FIX = '/sensing/gnss/rover/fix'
MOVING = 1.0  # m/s, threshold for "moving" statistics


def typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for name in ('VelocitySensor', 'DriverControllerCommand'):
        types.update(get_types_from_msg((MSG_DIR / f'{name}.msg').read_text(),
                                        f'tram_vehicle_msgs/msg/{name}'))
    ts.register(types)
    return ts


def stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(path: Path) -> dict:
    ts = typestore()
    data: dict[str, dict[str, list]] = {}
    with AnyReader([path], default_typestore=ts) as reader:
        t0 = reader.start_time * 1e-9
        duration = (reader.end_time - reader.start_time) * 1e-9
        pubs = {c.topic: len(c.ext.offered_qos_profiles)
                for c in reader.connections}
        for conn, t, raw in reader.messages():
            m = reader.deserialize(raw, conn.msgtype)
            d = data.setdefault(conn.topic, {'t': [], 's': [], 'v': []})
            d['t'].append(t * 1e-9)
            d['s'].append(stamp(m))
            if conn.topic in (FRONT, REAR):
                d['v'].append(m.velocity)
            elif conn.topic == CMD:
                d['v'].append(m.position)
            elif conn.topic.endswith('/fix'):
                d['v'].append((m.latitude, m.longitude, m.altitude, m.status.status))
            elif conn.topic.endswith('/vel'):
                d['v'].append((m.twist.linear.x, m.twist.linear.y, m.twist.linear.z))
    arr = {k: {kk: np.asarray(vv) for kk, vv in v.items()} for k, v in data.items()}
    return {'t0': t0, 'duration': duration, 'pubs': pubs, 'arr': arr}


def rate_stats(s: np.ndarray) -> tuple[float, float, int, int]:
    """Median rate by header stamps, max gap, non-monotonic count, duplicate stamps."""
    if len(s) < 3:
        return float('nan'), float('nan'), 0, 0
    ds = np.diff(s)
    pos = ds[ds > 0]
    rate = 1.0 / np.median(pos) if len(pos) else float('nan')
    return rate, float(ds.max()), int((ds < 0).sum()), int((ds == 0).sum())


def local_xy(lat: np.ndarray, lon: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    r = 6378137.0
    x = np.radians(lon - lon[0]) * r * math.cos(math.radians(lat[0]))
    y = np.radians(lat - lat[0]) * r
    return x, y


def interp(ts: np.ndarray, t: np.ndarray, v: np.ndarray) -> np.ndarray:
    return np.interp(ts, t, v, left=np.nan, right=np.nan)


def survey(path: Path) -> dict:
    b = read_bag(path)
    a, t0 = b['arr'], b['t0']
    row: dict = {'bag': path.name, 'vehicle': path.name.split('_')[0],
                 'duration_s': round(b['duration'], 1)}
    for topic, short in ((FRONT, 'front'), (REAR, 'rear'), (CMD, 'cmd'),
                         (GNSS_FIX, 'fix'), (GNSS_VEL, 'gvel'), (ROVER_FIX, 'rfix')):
        d = a.get(topic)
        n = 0 if d is None else len(d['s'])
        row[f'{short}_n'] = n
        row[f'{short}_pubs'] = b['pubs'].get(topic, 0)
        if n < 3:
            continue
        rate, gap, nonmono, dup = rate_stats(d['s'])
        lag = d['t'] - d['s']
        row.update({f'{short}_hz': round(rate, 2), f'{short}_maxgap_s': round(gap, 3),
                    f'{short}_nonmono': nonmono, f'{short}_dupstamp': dup,
                    f'{short}_lag_med_ms': round(float(np.median(lag)) * 1e3, 1),
                    f'{short}_lag_p99_ms': round(float(np.percentile(lag, 99)) * 1e3, 1),
                    f'{short}_first_s': round(float(d['t'][0] - t0), 2),
                    f'{short}_last_s': round(float(d['t'][-1] - t0), 2)})
    for topic, short in ((FRONT, 'front'), (REAR, 'rear')):
        d = a.get(topic)
        if d is None or len(d['v']) < 3:
            continue
        v = d['v'].astype(float)
        row[f'{short}_vmin'] = round(float(v.min()), 2)
        row[f'{short}_vmax'] = round(float(v.max()), 2)
        row[f'{short}_neg_frac'] = round(float((v < -0.05).mean()), 4)
        row[f'{short}_nan'] = int(np.isnan(v).sum())
        dv = np.abs(np.diff(v)) / np.maximum(np.diff(d['s']), 1e-3)
        row[f'{short}_jumps_gt5mps2'] = int((dv > 5.0).sum())
    if CMD in a:
        c = a[CMD]['v'].astype(int)
        row['cmd_min'], row['cmd_max'] = int(c.min()), int(c.max())
        row['cmd_traction_frac'] = round(float((c > 0).mean()), 3)
        row['cmd_brake_frac'] = round(float((c < 0).mean()), 3)
    if FRONT in a and REAR in a and len(a[FRONT]['s']) > 3 and len(a[REAR]['s']) > 3:
        f, r = a[FRONT], a[REAR]
        rv = interp(f['s'], r['s'], r['v'].astype(float))
        fv = f['v'].astype(float)
        mv = np.isfinite(rv) & (np.abs(fv) > MOVING)
        if mv.any():
            diff = fv[mv] - rv[mv]
            row['fr_diff_med'] = round(float(np.median(diff)), 3)
            row['fr_diff_p99abs'] = round(float(np.percentile(np.abs(diff), 99)), 3)
            row['fr_diff_gt1_frac'] = round(float((np.abs(diff) > 1.0).mean()), 4)
    if GNSS_FIX in a and len(a[GNSS_FIX]['v']) > 3:
        fx = a[GNSS_FIX]['v']
        st = fx[:, 3].astype(int)
        row['fix_status_set'] = ' '.join(str(s) for s in sorted(set(st.tolist())))
        ok = st >= 0
        x, y = local_xy(fx[ok, 0], fx[ok, 1]) if ok.sum() > 1 else (np.zeros(1), np.zeros(1))
        row['gnss_path_m'] = round(float(np.hypot(np.diff(x), np.diff(y)).sum()), 1)
        row['gnss_extent_m'] = round(float(np.hypot(x - x[0], y - y[0]).max()), 1)
    if GNSS_VEL in a and FRONT in a and len(a[GNSS_VEL]['v']) > 3 and len(a[FRONT]['s']) > 3:
        g = a[GNSS_VEL]
        gs = np.hypot(g['v'][:, 0], g['v'][:, 1])
        row['gnss_vmax'] = round(float(gs.max()), 2)
        row['gnss_dist_m'] = round(float(np.trapezoid(gs, g['s'])), 1)
        fv = interp(g['s'], a[FRONT]['s'], a[FRONT]['v'].astype(float))
        mv = np.isfinite(fv) & (gs > 3.0)
        if mv.sum() > 20:
            row['wheel_over_gnss_med'] = round(float(np.median(np.abs(fv[mv]) / gs[mv])), 4)
        # best constant lag between front wheel stamp and GNSS vel stamp (s)
        if mv.sum() > 200:
            lags = np.arange(-2.0, 2.01, 0.05)
            errs = [np.nanmean((interp(g['s'] + L, a[FRONT]['s'], np.abs(a[FRONT]['v'].astype(float))) - gs) ** 2)
                    for L in lags]
            row['wheel_gnss_lag_s'] = round(float(lags[int(np.nanargmin(errs))]), 2)
    return row


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', type=Path, default=default_data_dir())
    ap.add_argument('--out', default=REPO / 'docs' / 'data' / 'survey.csv')
    ap.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    args = ap.parse_args()
    bags = sorted(p for p in Path(args.data).iterdir() if (p / 'metadata.yaml').exists())
    with ProcessPoolExecutor(args.jobs) as ex:
        rows = list(ex.map(survey, bags))
    keys: list[str] = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys, lineterminator='\n')
        w.writeheader()
        w.writerows(rows)
    print(f'{len(rows)} bags -> {out}')


if __name__ == '__main__':
    main()
