"""LODO grid on the label without the per-bag wheel scale (the table «метка без масштаба bag»).

    <env>/bin/python tools/speed_residual/cv_detrended.py --rows <dir> [--gpu]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from cv import LABEL_CLIP, load, report
from fit_folds import detrend

TRANSIENT = ['a', 'am', 'ab', 'dv05', 'dv1', 'dv2', 'sf03', 'sr03', 'sf1', 'sr1', 'sm03', 'sm1', 'n0', 'n03',
             'n1', 'n2', 't_notch', 'lag', 'dfr', 'ft', 'rt', 'slip', 'nis', 't_stop', 't_move', 'var',
             'age_f', 'age_r', 'age_cmd']
GRID = [('all', 3, 100, 0.03), ('all', 4, 300, 0.03), ('all', 4, 300, 0.05), ('all', 6, 300, 0.03),
        ('all', 4, 800, 0.03), ('transient', 4, 300, 0.03), ('all', 6, 800, 0.03)]


def linear(cols, clip=0.03):
    def predict(tr, va):
        X = np.where(tr[cols].values == -999.0, 0.0, tr[cols].values)
        X = np.c_[np.ones(len(X)), X]
        w = np.linalg.solve(X.T @ X + 1e-3 * len(X) * np.eye(X.shape[1]), X.T @ tr.res_dt.values)
        Xv = np.where(va[cols].values == -999.0, 0.0, va[cols].values)
        return np.clip(np.c_[np.ones(len(Xv)), Xv] @ w, -clip, clip)
    return predict


def catboost(feats, depth, iters, clip, gpu):
    from catboost import CatBoostRegressor

    def predict(tr, va):
        m = CatBoostRegressor(depth=depth, iterations=iters, learning_rate=0.05, task_type='GPU' if gpu else 'CPU',
                              verbose=0, border_count=64, l2_leaf_reg=10, random_seed=0, allow_writing_files=False)
        m.fit(tr[feats].values, tr.res_dt.values)
        return np.clip(m.predict(va[feats].values), -clip, clip)
    return predict


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rows', required=True, type=Path)
    ap.add_argument('--gpu', action='store_true')
    a = ap.parse_args()
    df, feats = load(a.rows)
    df['res_dt'] = detrend(df)
    days = sorted(df.day.unique())
    models = {'lin_small': linear(['a', 'am', 'dv05', 'sm03']), 'lin_all': linear(feats)}
    for fs, depth, iters, clip in GRID:
        models[f'cb_{fs}_d{depth}_{iters}_clip{clip}'] = catboost(feats if fs == 'all' else TRANSIENT,
                                                                  depth, iters, clip, a.gpu)
    rows = []
    for name, predict in models.items():
        per = []
        for day in days:
            tr = df[(df.day != day) & (np.abs(df.res) < LABEL_CLIP)]
            va = df[df.day == day]
            per.append(report(va, predict(tr, va)))
        gains = [p['bag_gain_med_pct'] for p in per]
        rows.append({'model': name, **{f'gain_{d}': g for d, g in zip(days, gains)},
                     'gain_mean': float(np.mean(gains)), 'gain_min': min(gains),
                     'worse2': sum(p['bags_worse_2pct'] for p in per), 'better': sum(p['bags_better'] for p in per)})
        print(rows[-1], flush=True)
    pd.set_option('display.width', 250)
    print(pd.DataFrame(rows).round(3).to_string(index=False))


if __name__ == '__main__':
    main()
