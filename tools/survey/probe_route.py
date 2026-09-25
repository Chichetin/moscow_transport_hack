"""Route probe behind docs/data.md traps 14-18: the tram in the GNSS window, the antenna
baseline, vehicle identity in messages, wheel scale by day, recurring stop places, altitude.

Usage:
    .venv/bin/python tools/survey/probe_route.py [--data $TRAM_DATA_DIR]

Reads every unique bag of train, holdout and no_gnss_train (tools/eval/splits.yaml) with the
reader of survey_bags.py and prints the numbers quoted in docs/data.md. Scale per bag is taken
from docs/data/survey.csv (wheel_over_gnss_med).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import yaml

import survey_bags as sb

REPO = Path(__file__).resolve().parents[2]
STOP_V = 0.2          # m/s, GNSS speed below this is standing
STOP_MIN_S = 10.0     # s, shorter standstills are not stop places
CLUSTER_R = 20.0      # m, stops closer than this are one place
RECURRING = 5         # bags: a place seen in fewer train bags is not a stop place
WINDOW_S = 5.0        # s, GNSS init window (D-005)
MOVE_V = 1.0 * 3.6    # km/h, "the tram moves" in wheel units
R_EARTH = 6378137.0


def stop_segments(t: np.ndarray, v: np.ndarray, v_max: float = STOP_V,
                  min_s: float = STOP_MIN_S) -> list[tuple[int, int]]:
    """[i0, i1) index ranges where v < v_max for at least min_s seconds."""
    out, i, n = [], 0, len(t)
    while i < n:
        if v[i] < v_max:
            j = i
            while j < n and v[j] < v_max:
                j += 1
            if t[j - 1] - t[i] >= min_s:
                out.append((i, j))
            i = j
        else:
            i += 1
    return out


def cluster(points: np.ndarray, radius: float = CLUSTER_R) -> list[tuple[np.ndarray, list[int]]]:
    """Greedy clusters: grow each one while any point is within radius of its centre."""
    free, out = set(range(len(points))), []
    while free:
        members = [min(free)]
        free.discard(members[0])
        centre, grown = points[members[0]], True
        while grown:
            grown = False
            for j in sorted(free):
                if math.hypot(*(points[j] - centre)) < radius:
                    members.append(j)
                    free.discard(j)
                    grown = True
            centre = points[members].mean(axis=0)
        out.append((centre, members))
    return out


def to_xy(lat, lon, lat0: float, lon0: float) -> np.ndarray:
    lat, lon = np.asarray(lat, float), np.asarray(lon, float)
    return np.c_[np.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0)),
                 np.radians(lat - lat0) * R_EARTH]


def probe(path: Path) -> dict:
    b = sb.read_bag(path)
    a = b['arr']
    res = {'bag': path.name, 'stops': [], 'alt': [], 'cos': None}
    front = a.get(sb.FRONT)
    if front is not None and len(front['s']):
        s0 = front['s'].min()
        res['moving_in_window'] = bool(np.any(front['v'][front['s'] - s0 < WINDOW_S] > MOVE_V))
    fix, vel, rov = a.get(sb.GNSS_FIX), a.get(sb.GNSS_VEL), a.get(sb.ROVER_FIX)
    if fix is None or vel is None or len(fix['s']) < 300 or len(vel['s']) < 100:
        return res
    f = fix['v'][fix['v'][:, 3] >= 0]
    ft = fix['s'][fix['v'][:, 3] >= 0]
    speed = np.interp(ft, vel['s'], np.hypot(vel['v'][:, 0], vel['v'][:, 1]))
    for i0, i1 in stop_segments(ft, speed):
        res['stops'].append((float(np.median(f[i0:i1, 0])), float(np.median(f[i0:i1, 1]))))
    res['alt'] = f[::10, :3].tolist()
    if rov is not None and len(rov['s']) >= 300:
        lat0, lon0 = f[0, 0], f[0, 1]
        m, r = to_xy(f[:, 0], f[:, 1], lat0, lon0), to_xy(rov['v'][:, 0], rov['v'][:, 1], lat0, lon0)
        wm, wr = ft - ft[0] < WINDOW_S, rov['s'] - ft[0] < WINDOW_S
        if wm.sum() >= 5 and wr.sum() >= 5:
            start = np.median(m[wm], axis=0)
            gone = np.hypot(*(m - start).T) > 30.0
            if gone.any():
                base = start - np.median(r[wr], axis=0)            # rover -> master
                move = m[np.argmax(gone)] - start
                res['cos'] = float(base @ move / (np.linalg.norm(base) * np.linalg.norm(move)))
                res['base_m'] = float(np.linalg.norm(base))
    return res


def frame_ids(path: Path) -> dict:
    from rosbags.highlevel import AnyReader
    seen: dict[str, set] = {}
    with AnyReader([path], default_typestore=sb.typestore()) as reader:
        for conn, _, raw in reader.messages():
            ids = seen.setdefault(conn.topic, set())
            if len(ids) < 3:
                ids.add(reader.deserialize(raw, conn.msgtype).header.frame_id)
            if all(len(v) >= 3 for v in seen.values()) and len(seen) >= 7:
                break
    return {k: sorted(v) for k, v in seen.items()}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', type=Path, default=sb.default_data_dir())
    ap.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    args = ap.parse_args()
    splits = yaml.safe_load((REPO / 'tools/eval/splits.yaml').read_text(encoding='utf-8'))
    train, hold = set(splits['train']), set(splits['holdout'])
    names = splits['train'] + splits['holdout'] + splits['no_gnss_train']
    with ProcessPoolExecutor(args.jobs) as ex:
        rows = list(ex.map(probe, [args.data / n for n in names]))

    moving = [r['moving_in_window'] for r in rows if 'moving_in_window' in r]
    print(f'14. едет (> 1 м/с) в первые {WINDOW_S:.0f} с: {sum(moving)} из {len(moving)} bag')

    cos = [(r['bag'][:5], r['cos'], r['base_m']) for r in rows if r['cos'] is not None]
    for veh in ('30618', '30639'):
        c = [x[1] for x in cos if x[0] == veh]
        if c:
            print(f'15. {veh}: {len(c)} bag, cos(rover->master, первые 30 м пути) '
                  f'{min(c):.2f}..{max(c):.2f}, база {np.median([x[2] for x in cos if x[0] == veh]):.1f} м')

    for veh in ('30618', '30639'):
        name = next(n for n in names if n.startswith(veh))
        print(f'16. {veh} {name}: frame_id {frame_ids(args.data / name)}')

    sv = [r for r in csv.DictReader(open(REPO / 'docs/data/survey.csv', encoding='utf-8'))
          if r.get('wheel_over_gnss_med')]
    for veh in ('30618', '30639'):
        tr = [float(r['wheel_over_gnss_med']) for r in sv if r['vehicle'] == veh and r['bag'] in train]
        ho = [float(r['wheel_over_gnss_med']) for r in sv if r['vehicle'] == veh and r['bag'] in hold]
        print(f'17. {veh}: масштаб train {np.mean(tr):.3f} ± {np.std(tr):.3f} (n={len(tr)}), '
              f'holdout {np.mean(ho):.3f} ± {np.std(ho):.3f} (n={len(ho)})')
    ho = [(float(r['wheel_over_gnss_med']), float(r['gnss_dist_m'])) for r in sv
          if r['bag'] in hold and float(r['gnss_dist_m'] or 0) > 500]
    err = np.array([abs(k / 3.6 - 1) * 100 for k, _ in ho])
    drift = np.array([abs(k / 3.6 - 1) * d for k, d in ho])
    print(f'17. константа 3,6 на holdout: ошибка масштаба медиана {np.median(err):.2f} %, '
          f'p90 {np.percentile(err, 90):.2f} %, max {err.max():.2f} %; '
          f'дрейф в конце от масштаба медиана {np.median(drift):.0f} м, max {drift.max():.0f} м')

    tr_stops = [(r['bag'], p) for r in rows if r['bag'] in train for p in r['stops']]
    ho_stops = [(r['bag'], p) for r in rows if r['bag'] in hold for p in r['stops']]
    lat0, lon0 = tr_stops[0][1]
    pts = to_xy([p[0] for _, p in tr_stops], [p[1] for _, p in tr_stops], lat0, lon0)
    places = [(c, m) for c, m in cluster(pts) if len({tr_stops[k][0] for k in m}) >= RECURRING]
    spread = np.concatenate([np.hypot(*(pts[m] - c).T) for c, m in places])
    centres = np.array([c for c, _ in places])
    nn = [np.sort(np.hypot(*(centres - c).T))[1] for c in centres]
    hp = to_xy([p[0] for _, p in ho_stops], [p[1] for _, p in ho_stops], lat0, lon0)
    d = np.array([np.min(np.hypot(*(centres - p).T)) for p in hp])
    per_bag = [sum(1 for b, _ in ho_stops if b == n) for n in {b for b, _ in ho_stops}]
    print(f'18. train: {len(tr_stops)} остановок >= {STOP_MIN_S:.0f} с в '
          f'{len({b for b, _ in tr_stops})} bag, {len(places)} мест (>= {RECURRING} bag); '
          f'до центра места медиана {np.median(spread):.1f} м, p90 {np.percentile(spread, 90):.1f} м; '
          f'соседние места: медиана {np.median(nn):.0f} м, min {np.min(nn):.0f} м')
    print(f'18. holdout: {len(hp)} остановок, ближе 10 м к месту train {np.mean(d < 10) * 100:.0f} %, '
          f'ближе 20 м {np.mean(d < 20) * 100:.0f} %; на bag медиана {np.median(per_bag):.0f}')


if __name__ == '__main__':
    main()
