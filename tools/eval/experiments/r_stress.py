"""Adaptive R (ktoyart) in the stress scenarios, train only: peak excess vs base."""
import sys
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tram_eval import bag, stress
from filter_ideas_exp import factory

CONFIGS = {'base': None, 'R_k10': ('R', 10.0)}
SC = ('outlier', 'spike', 'noise', 'gap_10', 'gap_both_30')

def one(name):
    msgs = bag.read_bag(bag.data_dir() / name)
    out = {}
    for c, cfg in CONFIGS.items():
        out[c] = stress.evaluate_stress_bag(None, bag.default_gnss_window(), make_odometry=factory(cfg), msgs=msgs)
    return out

if __name__ == '__main__':
    stress.SCENARIOS = SC
    names = bag.load_splits()['train']
    with ProcessPoolExecutor(6) as pool:
        res = list(pool.map(one, names))
    print('| scenario | config | speed excess med / max | pos excess med / max |')
    print('|---|---|---|---|')
    for sc in SC:
        for c in CONFIGS:
            rows = [r[c][sc] for r in res if sc in r[c] and not r[c][sc].get('skipped')]
            se = [x['peak_speed_excess_mps'] for x in rows if x['peak_speed_excess_mps'] is not None]
            pe = [x['peak_pos3d_excess_m'] for x in rows if x['peak_pos3d_excess_m'] is not None]
            print(f'| {sc} | {c} | {np.median(se):.3f} / {max(se):.3f} | {np.median(pe):.3f} / {max(pe):.3f} |')
