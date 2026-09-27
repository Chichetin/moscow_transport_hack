"""Experiment: publish speed multiplied by the online path scale of PathTracker (D-034).

After #153/#154 this script does not reproduce D-076 (a): Estimate.speed already carries the
speed scale of the stop chain (D-082) and the path scale is a Kalman state (D-083); the
numbers of D-076 (a) hold on ef47f9a only."""
from pathlib import Path
import dataclasses, math, sys
from concurrent.futures import ProcessPoolExecutor
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tram_eval import bag, stress
from tram_eval.metrics import summarize

CONFIGS = {'base': None, 'speed_x_scale': True}
SCEN = ('clean', 'scale_up', 'scale_down')

def factory(cfg):
    def make():
        odo = bag.default_odometry()
        if cfg is None or odo._tracker is None:
            return odo
        tr = odo._tracker
        orig = odo.step
        def step(raw):
            est = orig(raw)
            if est is None:
                return est
            return dataclasses.replace(est, speed=est.speed * tr._scale)
        odo.step = step
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
