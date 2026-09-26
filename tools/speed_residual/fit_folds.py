"""Fit the chosen residual model on each leave-one-day-out fold of train and export JSON.

Label: v_GNSS - v_pipeline with the per-bag wheel scale removed (a least-squares k*v per bag):
the scale of a bag is not observable from the allowed inputs, the shape of the residual may be.
Rows with |residual| >= LABEL_CLIP (skids, GNSS glitches) are left out of training.

    <env>/bin/python tools/speed_residual/fit_folds.py --rows <dir> --out <dir> --depth 6 --iters 800 [--all]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from cv import LABEL_CLIP, load


def detrend(df):
    out = df.res.values.copy()
    for _, idx in df.groupby('bag').indices.items():
        r, v = df.res.values[idx], df.v.values[idx]
        s = np.abs(r) < LABEL_CLIP
        k = (r[s] * v[s]).sum() / max((v[s] ** 2).sum(), 1e-9)
        out[idx] = r - k * v
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rows', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--depth', type=int, default=6)
    ap.add_argument('--iters', type=int, default=800)
    ap.add_argument('--all', action='store_true', help='also fit on the whole train split')
    ap.add_argument('--gpu', action='store_true')
    a = ap.parse_args()
    from catboost import CatBoostRegressor
    df, feats = load(a.rows)
    df['res_dt'] = detrend(df)
    a.out.mkdir(parents=True, exist_ok=True)
    folds = [(d, df.day != d) for d in sorted(df.day.unique())]
    if a.all:
        folds.append(('all', np.ones(len(df), bool)))
    for name, sel in folds:
        tr = df[np.asarray(sel) & (np.abs(df.res.values) < LABEL_CLIP)]
        m = CatBoostRegressor(depth=a.depth, iterations=a.iters, learning_rate=0.05, loss_function='RMSE',
                              task_type='GPU' if a.gpu else 'CPU', verbose=0, random_seed=0,
                              border_count=64, l2_leaf_reg=10)
        m.fit(tr[feats].values, tr.res_dt.values)
        path = a.out / f'd{a.depth}_{a.iters}_{name}.json'
        m.save_model(str(path), format='json')
        va = df[df.day == name] if name != 'all' else df.iloc[:2000]
        np.save(a.out / f'd{a.depth}_{a.iters}_{name}_check.npy',
                np.c_[va[feats].values[:2000], m.predict(va[feats].values[:2000])])
        print(name, len(tr), path, flush=True)


if __name__ == '__main__':
    main()
