"""How often does the NIS gate reject a wheel on train? (idea 1 from competitor review)"""
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tram_eval import bag

def one(name):
    path = bag.data_dir() / name
    msgs = bag.read_bag(path)
    end = bag.gnss_window_end(msgs, bag.default_gnss_window())
    odo = bag.default_odometry()
    n = rej = 0
    rej_moving = 0
    for topic, msg in msgs:
        if topic in bag.GNSS and bag.stamp(msg) > end:
            continue
        est = odo.step(bag.to_raw(topic, msg))
        d = getattr(est, 'filter_diagnostics', None) if est is not None else None
        if d is None:
            continue
        n += 1
        if not d.accepted:
            rej += 1
            rej_moving += est.speed > 0.5
    return name, n, rej, rej_moving

if __name__ == '__main__':
    names = bag.load_splits()['train']
    with ProcessPoolExecutor(12) as pool:
        rows = list(pool.map(one, names))
    tot = sum(r[1] for r in rows); rj = sum(r[2] for r in rows); rm = sum(r[3] for r in rows)
    print(f'train {len(rows)} bag: wheel updates {tot}, rejected by NIS {rj} ({100*rj/tot:.3f} %), while moving >0.5 m/s {rm}')
    for r in sorted(rows, key=lambda r: -r[2])[:5]:
        print(r)
