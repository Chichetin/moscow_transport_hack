"""Why does position not recover under wheel scale x1.015? Log PathTracker.on_stop outcomes."""
from pathlib import Path
import json, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tram_eval import bag, stress

def run(name, scenario):
    msgs = bag.read_bag(bag.data_dir() / name)
    if scenario != 'clean':
        ev = stress.perturb(msgs, scenario, bag.default_gnss_window())
        if ev is None:
            return None
        msgs = ev[0]
    end = bag.gnss_window_end(msgs, bag.default_gnss_window())
    odo = bag.default_odometry()
    tr = odo._tracker if hasattr(odo, '_tracker') else None
    log = []
    orig = type(tr).on_stop
    def on_stop(self, distance):
        k, s = self._state(distance)
        places = self._stops[k]
        d = float(np.min(np.abs(places - s))) if len(places) else None
        ok = orig(self, distance)
        log.append((round(d, 1) if d is not None else None, ok))
        return ok
    type(tr).on_stop = on_stop
    for topic, msg in msgs:
        if topic in bag.GNSS and bag.stamp(msg) > end:
            continue
        odo.step(bag.to_raw(topic, msg))
    type(tr).on_stop = orig
    return dict(stops=len(log), snapped=sum(o for _, o in log),
                far=sum(1 for d, o in log if not o and d is not None and d > 20),
                dists=[d for d, _ in log], scale=round(tr._scale, 4))

def one(name):
    return name, {sc: run(name, sc) for sc in ('clean', 'scale_up', 'scale_down')}

if __name__ == '__main__':
    st = json.load(open(sys.argv[1]))
    bags = st.get('bags', st)
    names = sorted(bags)
    with ProcessPoolExecutor(4) as pool:
        for name, r in pool.map(one, names):
            rec = bags[name]['scale_up'].get('pos_recovery_s', 'skip')
            print(name, 'rec_up=', rec, 'rec_dn=', bags[name]['scale_down'].get('pos_recovery_s', 'skip'), {k: (v['stops'], v['snapped'], v['far'], v['scale']) for k, v in r.items() if v})
            if rec is None:
                print('   up dists', r['scale_up']['dists'][:25])
