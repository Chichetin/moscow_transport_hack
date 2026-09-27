"""Experiment #154: online wheel scale of PathTracker. Train only (D-011).

H1  the GNSS anchor is the first reference of the scale (main: only snap-to-snap pairs);
H2  the scale is a Kalman state with a variance instead of an EMA: scalar (ratio of arcs from
    the last reference, correlations ignored) or joint with s (2x2, correlations kept);
H3  a lost lock is recovered from two misses of one sign growing with the path.

Variants replace PathTracker in memory (a subclass that overrides every method using the scale,
so 'base' stays the path scale of main c90577f/63762d2 on any core); the core and params.yaml are
not changed. 'h123' is the candidate of #154 that was NOT taken (D-087: holdout drift_pct +28.5 %);
its core code is branch worktree-154-wheel-scale @ 20e3e1e, where run_eval gives the same numbers
as 'h123' here bag for bag.
Per bag and variant: the clean run (metrics of D-012 + along_max), the stress runs scale_up,
scale_down, gap_both_30 (position recovery against the clean run of the same variant, as
run_eval --stress), and a log of every on_stop with the truth from the GNSS master track (eval
only, never in the core): a snap is false when the tram stood more than FALSE_M from the place.

    .venv/bin/python tools/eval/experiments/wheel_scale_exp.py [split] [variant ...]
"""
from pathlib import Path
import json, math, os, sys, time
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tram_eval import bag, stress
from tram_eval.metrics import bag_metrics, summarize
sys.path.insert(0, str(bag.REPO / 'src' / 'tram_odometry_core'))
from tram_odometry_core.position.tracker import PathTracker   # noqa: E402

SCEN = ('clean', 'scale_up', 'scale_down', 'gap_both_30')
FALSE_M = 10.0          # tram farther than this from the snapped place: a false snap (truth)
JOBS = 12
MAIN_SCALE_ALPHA = 0.3  # position.scale_alpha of main c90577f: the EMA weight that #154 replaces

# mode: base = main c90577f (EMA over two snaps on one branch); ema = the same EMA with H1
# (the anchor is the first reference; a reference is kept until an arc of scale_min_arc_m);
# kf1 = scalar Kalman on the ratio of arcs from the last reference (H1+H2); kf2 = joint Kalman
# of (s, scale) with their covariance (H1+H2). cross: arcs through a branch join are used too;
# meas_std: 1 sigma of a place as a scale reference (default stop_std_m); prior: 1 sigma of the
# scale (default along_drift_frac); min_arc (kf1: 0 unless set); recover: H3, rec_scale = what a
# relock does to the scale ('kf': Kalman from the reference with the prior variance again,
# 'implied': jump to the scale the miss implies, 'pos': s only, the reference restarts)
H12 = dict(mode='kf1', min_arc=300.0, keep_ref=True)
VARIANTS = {
    'base': dict(mode='base'),
    'h1_ema': dict(mode='ema', keep_ref=True),
    'h1_ema_cross': dict(mode='ema', keep_ref=True, cross=True),
    'h12_kf2': dict(mode='kf2'),
    'h12_noarc': dict(mode='kf1'),
    'h12': H12,
    'h12_s5': dict(H12, meas_std=5.0),
    'h12_p15': dict(H12, prior=0.015),
    'h123': dict(H12, recover=True, rec_scale='kf'),
    'h123_implied': dict(H12, recover=True, rec_scale='implied'),
    'h123_pos': dict(H12, recover=True, rec_scale='pos'),
    'h123_k3': dict(H12, recover=True, rec_scale='kf', rec_k=3.0),
    'h123_p15': dict(H12, prior=0.015, recover=True, rec_scale='kf'),
    'h123_chain': dict(H12, recover=True, rec_scale='kf', chain_relock=True),
}


class Tracker(PathTracker):
    """PathTracker with the scale variants of #154; mode 'base' is main bit for bit."""

    def __init__(self, params, route, cfg, clock):
        super().__init__(params, route)
        self.cfg, self.mode, self.clock = cfg, cfg['mode'], clock
        prior = cfg.get('prior', self.p.along_drift_frac)
        self._prior_var = prior ** 2
        self._P = (0.0, 0.0, self._prior_var)    # kf2: var s, cov(s, c), var c at the anchor
        self._Pc = self._prior_var                # kf1: var of the scale
        self._ref = None                          # (arc of the ref from the anchor, its path, var, branch)
        self._pending = None                      # H3: (path, innovation) of the last miss
        self._meas_var = cfg.get('meas_std', self.p.stop_std_m) ** 2   # a place as a scale reference
        self.log = []

    # --- state around the anchor -------------------------------------------------------------
    def _anchor_master(self):
        out = super()._anchor_master()
        self._P = (self.p.anchor_std_m ** 2, 0.0, self._prior_var)
        self._Pc = self._prior_var
        self._ref = ((0.0, self._anchor[2], self.p.anchor_std_m ** 2, self._anchor[0])
                     if self.mode != 'base' else None)
        self._pending = None
        return out

    def _prop(self, dd):
        pss, psc, pcc = self._P
        return pss + 2.0 * dd * psc + dd * dd * pcc, psc + dd * pcc, pcc

    def _var_along(self, distance):
        if self.mode != 'kf2':
            return super()._var_along(distance)
        return self._prop(distance - self._anchor[2])[0]

    def _take_side(self, distance, speed):
        if self._undo is not None:
            if self._overrun(distance) > self.p.side_overrun_m:
                self._anchor, self._var0, self._last_snap, self._P, self._ref = self._undo
                self._undo, self._pending = None, None
            return
        if not (math.isfinite(speed) and speed > self.p.side_speed_mps):
            return
        k, s = self._state(distance)
        side = self._side[k]
        if side is None or not self.p.side_min_m <= s - side[0] <= self.p.side_max_m:
            return
        self._undo = (self._anchor, self._var0, self._last_snap, self._P, self._ref)
        var, P = self._var_along(distance), self._prop(distance - self._anchor[2])
        j = side[1]
        self._anchor = (j, float(self._s[j][0]) + s - side[0], distance)
        self._var0, self._P = var, P
        self._last_snap = self._ref = self._pending = None

    # --- stops -------------------------------------------------------------------------------
    def _log(self, kind, distance, k, s, place, y):
        self.log.append(dict(t=self.clock(), d=distance, k=k, s=s, place=place, y=y, kind=kind,
                             scale=self._scale))

    def on_stop(self, distance):
        if self._anchor is None or not math.isfinite(distance):
            return False
        k, s, over = self._walk(distance)
        places = self._stops[k]
        if not len(places):
            self._log('noplace', distance, k, s, None, None)
            return False
        dist = np.abs(places - s)
        i = int(np.argmin(dist))
        place = float(places[i])
        y = place - s
        if dist[i] > self.p.stop_snap_max_m:
            if self.cfg.get('recover') and self._recover(k, s, place, y, distance, places, dist, i):
                return True
            self._log('miss', distance, k, s, place, y)
            return False
        if len(places) > 1 and float(np.partition(dist, 1)[1]) - dist[i] <= 2.0 * self.p.stop_std_m:
            self._log('amb', distance, k, s, place, y)
            return False
        self._snap(k, s, place, distance, over)
        self._log('snap', distance, k, s, place, y)
        return True

    def _snap(self, k, s, place, distance, over, r=None, relock=False):
        R = self.p.stop_std_m ** 2 if r is None else r
        y = place - s
        dd = distance - self._anchor[2]
        if self.mode == 'kf2':
            pss, psc, pcc = self._prop(dd)
            S = pss + R
            ks, kc = pss / S, psc / S
            if over <= 0.0:
                self._scale = self._clamp(self._scale + kc * y)
            self._P = (pss - pss * pss / S, psc - pss * psc / S, pcc - psc * psc / S)
            self._anchor = (k, s + ks * y, distance)
            self._var0 = self._P[0]
        else:
            var = self._var_along(distance)
            gain = var / (var + R)
            walked = self._scale * dd + gain * y          # arc from the old anchor to the new one
            if self._ref is not None and self._ref[3] != k and not self.cfg.get('cross'):
                self._ref = None                          # a join between: its arc is not the map's
            if self.mode == 'base':
                self._update_scale_main(k, place, distance)
                updated = True
            else:
                updated = over <= 0.0 and self._update_ratio(y, distance)
            s_new = s + gain * y
            self._anchor = (k, s_new, distance)
            self._var0 = (1.0 - gain) * var
            if updated or not self.cfg.get('keep_ref') or self._ref is None or over > 0.0:
                self._ref = (place - s_new, distance, self._meas_var, k)
            else:                                         # short arc: the reference stays put
                off, d_ref, v, kr = self._ref
                self._ref = (off - walked, d_ref, v, kr)
        # the speed chain of #153 (main since 63762d2) takes the pair of consecutive snaps;
        # the pair that ends at a relock stays out of it (as in the core) unless chain_relock
        if hasattr(self, '_accumulate_chain') and (not relock or self.cfg.get('chain_relock')):
            self._accumulate_chain(k, place, distance)
        self._undo = None
        self._last_snap = (k, place, distance)
        self._pending = None

    def _update_scale_main(self, k, place, distance):
        """PathTracker._update_scale of main c90577f (EMA over two snaps on one branch)."""
        if self._last_snap is None or self._last_snap[0] != k:
            return
        _, place0, d0 = self._last_snap
        if place - place0 < self.p.scale_min_arc_m or distance <= d0:
            return
        ratio = self._clamp((place - place0) / (distance - d0))
        self._scale += MAIN_SCALE_ALPHA * (ratio - self._scale)

    def _clamp(self, c):
        return min(max(c, 1.0 - self.p.scale_max_dev), 1.0 + self.p.scale_max_dev)

    def _update_ratio(self, y, distance):
        """ema / kf1: ratio of the map arc from the reference to the place over the wheel path;
        True when the scale was updated."""
        if self._ref is None:
            return False
        off, d_ref, var_ref, _ = self._ref
        wheel = distance - d_ref
        if wheel <= 0.0:
            return False
        arc = self._scale * (distance - self._anchor[2]) + y - off
        if arc < self.cfg.get('min_arc', self.p.scale_min_arc_m if self.mode == 'ema' else 0.0):
            return False
        ratio = arc / wheel
        if self.mode == 'ema':
            self._scale += MAIN_SCALE_ALPHA * (self._clamp(ratio) - self._scale)
            return True
        r = (var_ref + self._meas_var) / wheel ** 2
        g = self._Pc / (self._Pc + r)
        self._scale = self._clamp(self._scale + g * (ratio - self._scale))
        self._Pc *= 1.0 - g
        return True

    # --- H3 ----------------------------------------------------------------------------------
    def _recover(self, k, s, place, y, distance, places, dist, i):
        """Two misses past the gate, of one sign and growing with the path from the anchor as a
        scale error does, relock onto the place; a signal stop gives an unrelated miss."""
        dd = distance - self._anchor[2]
        if dd <= 0.0 or abs(self._scale + y / dd - 1.0) > self.p.scale_max_dev:
            self._pending = None
            return False
        if len(places) > 1 and float(np.partition(dist, 1)[1]) - dist[i] <= 2.0 * self.p.stop_std_m:
            return False
        pend, self._pending = self._pending, (distance, y)
        if pend is None or pend[1] * y <= 0.0 or distance <= pend[0]:
            return False
        d1, y1 = pend
        dd1 = d1 - self._anchor[2]
        q = dd / dd1
        R = self.p.stop_std_m ** 2
        var0 = self._var_along(self._anchor[2])
        resid = y - y1 * q
        if abs(resid) > self.cfg.get('rec_k', 2.0) * math.sqrt(R * (1.0 + q * q) + var0 * (1.0 - q) ** 2):
            return False
        # the lock was lost: the scale is not known after all. rec_scale: 'implied' jumps to
        # the scale the miss implies; 'kf' lets the ratio from the reference update it with the
        # prior variance; 'pos' only relocks s and restarts the reference at the place
        how = self.cfg.get('rec_scale', 'kf')
        if how == 'implied':
            self._scale = self._clamp(self._scale + y / dd)
        if self.mode == 'kf2':
            pss, psc, pcc = self._P
            self._P = (pss, psc, max(pcc, self._prior_var))
        elif self.mode == 'kf1':
            self._Pc = max(self._Pc, self._prior_var)
        if how == 'pos':
            self._ref = None
        k2, s2, over = self._walk(distance)
        self._snap(k, s2, place, distance, over, relock=True)
        self._log('recover', distance, k, s, place, y)
        return True


def make(cfg):
    odo = bag.default_odometry()
    if odo._tracker is not None:
        odo._tracker = Tracker(odo.params, odo.route, cfg, lambda: odo._t)
    return odo


def truth(tracker, ref_m):
    """True s of master on the branch of each logged stop (window of +-300 m around s)."""
    out = []
    for e in tracker.log:
        rec = dict(e)
        if len(ref_m.pos_t) and ref_m.pos_t[0] <= e['t'] <= ref_m.pos_t[-1]:
            xy = np.array([np.interp(e['t'], ref_m.pos_t, ref_m.pos[:, j]) for j in (0, 1)])
            k, s = e['k'], e['s']
            xyz, ss = tracker._xyz[k], tracker._s[k]
            sel = np.flatnonzero(np.abs(ss - s) <= 300.0)
            if len(sel):
                d2 = (xyz[sel, 0] - xy[0]) ** 2 + (xyz[sel, 1] - xy[1]) ** 2
                j = sel[int(np.argmin(d2))]
                if d2.min() < 15.0 ** 2:
                    rec['s_true'] = float(ss[j])
        out.append(rec)
    return out


KEYS = ('speed_rmse', 'along_rmse', 'drift_pct', 'along_max', 'along_mean', 'pos3d_rmse')


def one(args):
    name, variants = args
    path = bag.data_dir() / name
    msgs0 = bag.read_bag(path)
    w = bag.default_gnss_window()
    end = bag.gnss_window_end(msgs0, w)
    ref = bag.bag_reference(msgs0, end)
    ref_m = bag.bag_reference(msgs0, end, 'master')
    scen = {'clean': (msgs0, None, None)}
    for sc in SCEN[1:]:
        ev = stress.perturb(msgs0, sc, w)
        if ev is not None:
            scen[sc] = ev
    out = {}
    for vname in variants:
        cfg = VARIANTS[vname]
        clean = None
        for sc, (msgs, start, stop) in scen.items():
            odo = make(cfg)
            est, crash, _ = bag.run_pipeline(msgs, odo, end)
            est, _ = bag.finite_only(est)
            m = bag_metrics(ref, est)
            r = {k: (round(m[k], 4) if isinstance(m.get(k), float) and math.isfinite(m[k]) else None)
                 for k in KEYS}
            r['crashed'] = crash is not None
            r['scale_end'] = odo._tracker._scale if odo._tracker is not None else None
            r['stops'] = truth(odo._tracker, ref_m) if odo._tracker is not None else []
            if sc == 'clean':
                clean = est
            else:
                pk, cpk, exc, rec, *_ = stress._errors(ref.pos_t, ref.pos, clean, est, 'pos',
                                                      start, stop, 2.0)
                r.update(peak_pos=pk, excess_pos=exc, rec_pos=rec)
            out[f'{vname}|{sc}'] = r
    return name, out


def stop_stats(recs):
    snaps = [e for e in recs if e['kind'] in ('snap', 'recover')]
    known = [e for e in snaps if 's_true' in e]
    false = [e for e in known if abs(e['place'] - e['s_true']) > FALSE_M]
    rec = [e for e in recs if e['kind'] == 'recover']
    rec_false = [e for e in rec if 's_true' in e and abs(e['place'] - e['s_true']) > FALSE_M]
    return len(recs), len(snaps), len(false), len(rec), len(rec_false)


def report(res, variants, split):
    f = lambda v: '-' if v is None else f'{v:.4g}'
    print(f'split={split}, {len(res)} bag')
    print('| variant | scenario | speed_rmse med | along_rmse med | drift_pct med | along_max med / max |'
          ' worst along_rmse 30618 | not recovered | stops / snaps / false | recover / false |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    for v in variants:
        for sc in SCEN:
            bags = {n: r[f'{v}|{sc}'] for n, r in res.items() if f'{v}|{sc}' in r}
            med = {k: (float(np.median([b[k] for b in bags.values() if b[k] is not None]))
                       if any(b[k] is not None for b in bags.values()) else None) for k in KEYS}
            amax = max((b['along_max'] for b in bags.values() if b['along_max'] is not None), default=None)
            w618 = max(((b['along_rmse'], n) for n, b in bags.items()
                        if n.startswith('30618') and b['along_rmse'] is not None), default=(None, ''))
            nrec = '' if sc == 'clean' else f"{sum(1 for b in bags.values() if b.get('rec_pos') is None)}/{len(bags)}"
            st = [0] * 5
            for b in bags.values():
                st = [a + c for a, c in zip(st, stop_stats(b['stops']))]
            print(f"| {v} | {sc} | {f(med['speed_rmse'])} | {f(med['along_rmse'])} | {f(med['drift_pct'])} |"
                  f" {f(med['along_max'])} / {f(amax)} | {f(w618[0])} `{w618[1][6:]}` | {nrec} |"
                  f" {st[0]} / {st[1]} / {st[2]} | {st[3]} / {st[4]} |")


if __name__ == '__main__':
    split = sys.argv[1] if len(sys.argv) > 1 else 'train'
    if split not in ('train',):
        raise SystemExit('tuning is train only (D-011)')
    variants = sys.argv[2:] or list(VARIANTS)
    names = bag.load_splits()[split]
    t0 = time.time()
    with ProcessPoolExecutor(JOBS) as pool:
        res = dict(pool.map(one, [(n, variants) for n in names]))
    out = Path(os.environ.get('WSE_OUT', bag.out_dir() / 'wheel_scale_exp'))
    out.mkdir(parents=True, exist_ok=True)
    tag = '_'.join(variants) if len(variants) < 5 else f'{len(variants)}variants'
    (out / f'{split}_{tag}.json').write_text(json.dumps(res))
    report(res, variants, split)
    print(f'{time.time() - t0:.0f} s -> {out}/{split}_{tag}.json')
