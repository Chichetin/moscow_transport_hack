"""Experiments: wheel R grows with |notch| (ktoyart), smooth dry friction c0*tanh(v/eps). Train only."""
import dataclasses, math, sys, types
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tram_eval import bag, stress
from tram_eval.metrics import summarize

CONFIGS = {'base': None, 'R_k3': ('R', 3.0), 'R_k10': ('R', 10.0), 'tanh_0.3': ('tanh', 0.3), 'tanh_1.0': ('tanh', 1.0)}
SCEN = ('clean',)

bag.default_odometry()
from tram_odometry_core import pipeline as _pl  # noqa: E402
_ORIG_MODEL = _pl.model_accel


def factory(cfg):
    def make():
        odo = bag.default_odometry()
        _pl.model_accel = _ORIG_MODEL
        if cfg is None:
            return odo
        kind, val = cfg
        if kind == 'R':
            f = odo._filter
            base_p = f._p
            orig = f.update
            nmax = odo.params.drive.notch_max
            def update(sample, trust):
                n = odo._notch_at(sample.t) / nmax
                f._p = dataclasses.replace(base_p, r_wheel=base_p.r_wheel * (1.0 + val * n * n))
                try:
                    return orig(sample, trust)
                finally:
                    f._p = base_p
            f.update = update
        else:
            c0 = odo.params.resistance.c0
            def model(notch, speed, params):
                return _ORIG_MODEL(notch, speed, params) + c0 * (1.0 - math.tanh(speed / val))
            _pl.model_accel = model
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
    print('| scenario | config | speed_rmse med | along_rmse med / max | drift_pct med / max | bias accel / brake / stop med |')
    print('|---|---|---|---|---|---|')
    for sc in SCEN:
        for c in CONFIGS:
            bags = {n: r[(sc, c)] for n, r in res.items() if (sc, c) in r}
            s = summarize(bags)
            md = s['median']; mx = {k: bags[b][k] for k, b in s['worst_bag'].items()}
            f = lambda d, k: f"{d.get(k):.4g}" if isinstance(d.get(k), (int, float)) else '-'
            print(f"| {sc} | {c} | {f(md,'speed_rmse')} | {f(md,'along_rmse')} / {f(mx,'along_rmse')} | {f(md,'drift_pct')} / {f(mx,'drift_pct')} | {f(md,'speed_bias_accel')} / {f(md,'speed_bias_brake')} / {f(md,'speed_bias_stop')} |")
