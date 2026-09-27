"""Diagnostics of the online wheel scale of PathTracker (D-034, D-076) on train (#153).

Per bag: every stop snap and every pair update of `_scale`, the GNSS truth of the same pair
(Doppler arc of master between the two snap times / wheel path between them), the oracle scale
of the whole bag, and per estimate (t, speed, scale, cumulative sums) so that output-only
variants "speed x some scale" can be scored offline with the eval matching. Only train.

    .venv/bin/python tools/eval/experiments/scale_diag.py --out <json> [--jobs 20]
    .venv/bin/python tools/eval/experiments/scale_diag.py --report <json>   # tables of docs/verification
"""
from __future__ import annotations

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tram_eval import bag  # noqa: E402
from tram_eval.metrics import match_nearest  # noqa: E402

ORACLE_MIN_SPEED = 3.0    # m/s: speed samples used for the oracle scale
ORACLE_MAX_ERR = 0.5      # m/s: larger residuals are GNSS glitches, not scale


def doppler_arc(ref, t):
    """Arc of the GNSS Doppler speed from the first vel sample to each time in t."""
    s = np.concatenate([[0.0], np.cumsum(0.5 * (ref.speed[1:] + ref.speed[:-1]) * np.diff(ref.vel_t))])
    return np.interp(t, ref.vel_t, s)


def one(name: str) -> dict:
    msgs = bag.read_bag(bag.data_dir() / name)
    window_end = bag.gnss_window_end(msgs, bag.default_gnss_window())
    odo = bag.default_odometry()
    tr = odo._tracker
    snaps, pairs = [], []
    orig_stop, orig_update = tr.on_stop, tr._update_scale

    def on_stop(distance):
        ok = orig_stop(distance)
        if ok:
            snaps.append({'t': odo._t, 'distance': distance, 'k': tr._last_snap[0], 'place': tr._last_snap[1]})
        return ok

    def update(k, place, distance):
        last, before = tr._last_snap, tr._scale
        orig_update(k, place, distance)
        if last is not None and last[0] == k and distance > last[2]:
            arc = place - last[1]
            if arc >= tr.p.scale_min_arc_m:
                pairs.append({'t': odo._t, 't0': None, 'k': k, 'arc': arc, 'wheel': distance - last[2],
                              'd0': last[2], 'd1': distance, 'ratio': arc / (distance - last[2]),
                              'before': before, 'after': tr._scale})

    tr.on_stop, tr._update_scale = on_stop, update
    t, speed, scale, npairs, dist = [], [], [], [], []
    for topic, msg in msgs:
        if topic in bag.GNSS and bag.stamp(msg) > window_end:
            continue
        est = odo.step(bag.to_raw(topic, msg))
        if est is None:
            continue
        t.append(est.t)
        speed.append(est.speed / getattr(tr, 'speed_scale', 1.0))   # the speed before #153's scale
        scale.append(tr._scale)
        npairs.append(len(pairs))
        dist.append(odo._distance)
    t, speed, dist = np.array(t), np.array(speed), np.array(dist)
    ref = bag.bag_reference(msgs, window_end)
    # snap times -> GNSS arc of the same pair
    snap_t = {round(s['distance'], 6): s['t'] for s in snaps}
    for p in pairs:
        p['t0'] = snap_t.get(round(p['d0'], 6))
        if p['t0'] is not None:
            p['gnss_arc'] = float(doppler_arc(ref, p['t']) - doppler_arc(ref, p['t0']))
            p['ratio_true'] = p['gnss_arc'] / p['wheel']

    ri, ei = match_nearest(ref.vel_t, t)
    v_ref, v = ref.speed[ri], speed[ei]
    sel = (v > ORACLE_MIN_SPEED) & (np.abs(v_ref - v) < ORACLE_MAX_ERR)
    k_speed = float((v_ref[sel] * v[sel]).sum() / (v[sel] ** 2).sum()) if sel.any() else None
    after = t > window_end
    k_arc = None
    if after.sum() > 10 and dist[after][-1] - dist[after][0] > 100:
        ta, da = t[after], dist[after]
        k_arc = float((doppler_arc(ref, ta[-1]) - doppler_arc(ref, ta[0])) / (da[-1] - da[0]))
    return {'bag': name, 'k_speed': k_speed, 'k_arc': k_arc, 'snaps': snaps, 'pairs': pairs,
            'ref_t': ri.tolist(), 'est_i': ei.tolist(), 'v_ref': v_ref.tolist(),
            't': t[ei].tolist(), 'speed': v.tolist(), 'scale': np.array(scale)[ei].tolist(),
            'npairs': np.array(npairs)[ei].tolist(), 'dist': dist[ei].tolist(),
            'duration': float(t.max() - t.min()) if len(t) else 0.0}


LONG_ROWS = 3000          # matched samples: shorter bags have too few stops
PRIORS_M = (0.0, 500.0, 1000.0, 2000.0, 4000.0)


def rmse(e) -> float:
    return float(np.sqrt(np.mean(np.square(e))))


def chain_sums(snaps):
    """(t, map arc, wheel path) after each snap, summed over consecutive snaps on one branch."""
    out, arc, wheel, prev = [], 0.0, 0.0, None
    for s in snaps:
        if (prev is not None and prev['k'] == s['k'] and s['distance'] > prev['distance']
                and s['place'] >= prev['place']):
            arc += s['place'] - prev['place']
            wheel += s['distance'] - prev['distance']
        prev = s
        out.append((s['t'], arc, wheel))
    return out


def chain_scale(t, sums, prior):
    k = np.ones(len(t))
    for ts, arc, wheel in sums:
        if wheel > 0:
            k[t >= ts] = (arc + prior) / (wheel + prior)
    return k


def report(path: Path):
    import pandas as pd
    import re
    res = json.loads(path.read_text())
    days = dict(re.findall(r"'(\d+_[0-9a-f]+)'\s*#\s*(\d{4}-\d{2}-\d{2})", bag.SPLITS_YAML.read_text()))
    pairs, rows = [], []
    for r in res:
        if r['k_speed'] is None or len(r['t']) < LONG_ROWS:
            continue
        t, v, vr, ema = (np.array(r[k]) for k in ('t', 'speed', 'v_ref', 'scale'))
        for p in r['pairs']:
            if 'ratio_true' in p:
                pairs.append({'day': days[r['bag']], 'arc': p['arc'], 'err_pp': 100 * (p['ratio'] - p['ratio_true'])})
        sums = chain_sums(r['snaps'])
        row = {'bag': r['bag'], 'k_true_pct': 100 * (r['k_speed'] - 1), 'ema_final_pct': 100 * (ema[-1] - 1),
               'ema_max_dev_pct': 100 * np.max(np.abs(ema - 1)),
               'chain4000_final_pct': 100 * (chain_scale(t, sums, 4000.0)[-1] - 1),
               'base': rmse(v - vr), 'ema': rmse(v * ema - vr), 'oracle': rmse(v * r['k_speed'] - vr)}
        for prior in PRIORS_M:
            row[f'chain_L{prior:.0f}'] = rmse(v * chain_scale(t, sums, prior) - vr)
        rows.append(row)
    p, d = pd.DataFrame(pairs), pd.DataFrame(rows)
    pd.set_option('display.width', 250)
    print(f"pairs {len(p)}: map ratio - GNSS ratio, pp: median {p.err_pp.median():.3f}, std {p.err_pp.std():.3f}")
    p['arc_bin'] = pd.cut(p.arc, [300, 500, 800, 1200, 1e5])
    print(p.groupby('arc_bin', observed=True).err_pp.agg(['count', 'median', 'std']).round(3))
    print(d[['bag', 'k_true_pct', 'ema_final_pct', 'ema_max_dev_pct', 'chain4000_final_pct']].round(3).to_string(index=False))
    variants = ['ema', 'oracle'] + [f'chain_L{prior:.0f}' for prior in PRIORS_M]
    out = []
    for tram in ('30618', '30639'):
        g = d[d.bag.str.startswith(tram)]
        for c in variants:
            ch = (g[c] - g.base) / g.base * 100
            out.append({'tram': tram, 'variant': c, 'per_bag_median_pct': ch.median(), 'worst_pct': ch.max(),
                        'worse_2pct': int((ch > 2).sum()), 'better': int((ch < 0).sum()), 'n': len(g)})
    print(pd.DataFrame(out).round(3).to_string(index=False))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=Path)
    ap.add_argument('--report', type=Path)
    ap.add_argument('--jobs', type=int, default=20)
    a = ap.parse_args()
    if a.report:
        report(a.report)
        return
    names = [str(b) for b in bag.load_splits()['train']]
    with ProcessPoolExecutor(a.jobs) as pool:
        res = list(pool.map(one, names))
    a.out.write_text(json.dumps(res))
    print(f'{len(res)} train bags -> {a.out}')


if __name__ == '__main__':
    main()
