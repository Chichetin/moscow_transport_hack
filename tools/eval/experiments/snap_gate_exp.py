"""Experiment: stop-snap gate grows with along-track sigma: min(cap, max(20, k*sigma)). Train only."""
from pathlib import Path
import dataclasses, math, sys, types
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tram_eval import bag, stress
from tram_eval.metrics import summarize

CONFIGS = {'base': None, 'k2_cap50': (2.0, 50.0), 'k3_cap60': (3.0, 60.0), 'k3_cap100': (3.0, 100.0)}
SCEN = ('clean', 'scale_up', 'scale_down')

def factory(cfg):
    def make():
        odo = bag.default_odometry()
        if cfg is None or odo._tracker is None:
            return odo
        k, cap = cfg
        tr = odo._tracker
        orig = type(tr).on_stop
        base_p = tr.p
        def on_stop(self, distance):
            if self._anchor is None or not math.isfinite(distance):
                return False
            gate = min(cap, max(base_p.stop_snap_max_m, k * math.sqrt(self._var_along(distance))))
            self.p = dataclasses.replace(base_p, stop_snap_max_m=gate)
            try:
                return orig(self, distance)
            finally:
                self.p = base_p
        tr.on_stop = types.MethodType(on_stop, tr)
        return odo
    return make

def one(name):
    path = bag.data_dir() / name
    msgs0 = bag.read_bag(path)
    w = bag.default_gnss_window()
    out = {}
    for sc in SCEN:
        if sc == 'clean':
            msgs = msgs0
        else:
            ev = stress.perturb(msgs0, sc, w)
            if ev is None:
                continue
            msgs = ev[0]
        for c, cfg in CONFIGS.items():
            m = bag.evaluate_bag(path, w, make_odometry=factory(cfg), msgs=msgs)
            m.pop(bag.NOTES, None)
            out[(sc, c)] = m
    return name, out

if __name__ == '__main__':
    split = sys.argv[1] if len(sys.argv) > 1 else 'train'
    names = bag.load_splits()[split]
    with ProcessPoolExecutor(6) as pool:
        res = dict(pool.map(one, names))
    print(f'split={split}, {len(res)} bag')
    print('| scenario | config | speed_rmse med | along_rmse med / max | drift_pct med / max |')
    print('|---|---|---|---|---|')
    for sc in SCEN:
        for c in CONFIGS:
            bags = {n: r[(sc, c)] for n, r in res.items() if (sc, c) in r}
            s = summarize(bags)
            md = s['median']; mx = {k: bags[b][k] for k, b in s['worst_bag'].items()}
            f = lambda d, k: f"{d.get(k):.4g}" if isinstance(d.get(k), (int, float)) else '-'
            print(f"| {sc} | {c} | {f(md,'speed_rmse')} | {f(md,'along_rmse')} / {f(mx,'along_rmse')} | {f(md,'drift_pct')} / {f(mx,'drift_pct')} |")
