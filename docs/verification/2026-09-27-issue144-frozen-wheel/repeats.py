"""#144: runs of exactly repeated bogie readings above 1 km/h on train (by header.stamp).

Run from the repository root: .venv/bin/python docs/verification/2026-09-27-issue144-frozen-wheel/repeats.py
"""
import sys, json
from concurrent.futures import ProcessPoolExecutor
sys.path[:0] = ['tools/eval', 'src/tram_odometry_core']
from tram_eval import bag

def one(name):
    msgs = bag.read_bag(bag.data_dir() / name)
    out = []
    for topic in (bag.FRONT, bag.REAR):
        seq = sorted((bag.stamp(m), m.velocity) for tp, m in msgs if tp == topic)
        run_v, run_t0, run_n = None, None, 0
        prev_t = None
        vals = set()
        for t, v in seq:
            vals.add(round(v, 6))
            if v == run_v and prev_t is not None and t > prev_t:
                run_n += 1
            else:
                if run_v is not None and run_v > 1.0 and run_n >= 3:
                    out.append((topic.split('/')[-1].split('_')[0], run_v, run_n, prev_t - run_t0))
                run_v, run_t0, run_n = v, t, 0
            prev_t = t
        dts = [b[0]-a[0] for a, b in zip(seq, seq[1:]) if b[0] > a[0]]
        dts.sort()
    return name, out, len(vals), dts[len(dts)//2] if dts else None

if __name__ == '__main__':
    names = bag.load_splits()['train']
    allruns = []
    with ProcessPoolExecutor(6) as ex:
        for name, runs, nvals, dt in ex.map(one, names):
            allruns += [(name,) + r for r in runs]
            print(name, 'distinct', nvals, 'median dt', round(dt, 4) if dt else None, 'runs>=3 at >1kmh', len(runs), flush=True)
    allruns.sort(key=lambda r: -r[4])
    print('TOP runs by duration (bag, bogie, km/h, repeats, seconds):')
    for r in allruns[:25]: print(r)
    print('total runs', len(allruns))
