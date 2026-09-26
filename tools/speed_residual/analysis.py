"""Tables of the report: per-bag oracles, errors by regime with the fold models, tail episodes.

    <env>/bin/python tools/speed_residual/analysis.py --rows <dir> --models <dir> --tag d6_800 --clip 0.03
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from cv import LABEL_CLIP, LONG_ROWS, load

DIVERGE_MPS = 0.3      # |front - rear| above this: the bogies disagree
EPISODE_GAP_S = 2.0    # tail rows closer than this belong to one episode


def rmse(e) -> float:
    return float(np.sqrt(np.mean(np.square(e))))


def oracles(df):
    """Per-bag least squares on its own GNSS: an upper bound, not a model."""
    rows = []
    for bag, g in df.groupby('bag'):
        if len(g) < LONG_ROWS:
            continue
        r, v, a = g.res.values, g.v.values, g.a.values
        k = (r * v).sum() / (v * v).sum()
        A = np.c_[v, a]
        coef = np.linalg.lstsq(A, r, rcond=None)[0]
        rows.append({'bag': bag, 'day': g.day.iloc[0], 'rmse': rmse(r), 'scale_oracle': rmse(r - k * v),
                     'scale_lag_oracle': rmse(r - A @ coef), 'scale_pct': 100 * k,
                     'white': float(np.sqrt(np.var(np.diff(r)) / 2))})
    t = pd.DataFrame(rows)
    print(t.groupby('day')[['rmse', 'scale_oracle', 'scale_lag_oracle', 'scale_pct', 'white']].median().round(4))


def regimes(df, feats, models: Path, tag: str, clip: float):
    from catboost import CatBoostRegressor
    df['p'] = 0.0
    for day in sorted(df.day.unique()):
        m = CatBoostRegressor()
        m.load_model(str(models / f'{tag}_{day}.json'), format='json')
        sel = df.day == day
        df.loc[sel, 'p'] = np.clip(m.predict(df.loc[sel, feats].values), -clip, clip)
    df['e0'] = df.v - df.v_ref
    df['e1'] = np.maximum(0.0, df.v + df.p) - df.v_ref
    wf, wr = df.wf.where(df.wf != -999.0), df.wr.where(df.wr != -999.0)
    diverge = (wf - wr).abs() > DIVERGE_MPS
    out = []
    for day, g in df.groupby('day'):
        for name in ('accel', 'brake', 'cruise', 'stop', 'diverge'):
            h = g[diverge[g.index]] if name == 'diverge' else g[g['mode'] == name]
            if h.empty:
                continue
            c = h.e0.abs() < LABEL_CLIP
            out.append({'day': day, 'regime': name, 'share': len(h) / len(g), 'clean_rmse0': rmse(h.e0[c]),
                        'clean_rmse1': rmse(h.e1[c]), 'bias0': h.e0[c].mean(), 'bias1': h.e1[c].mean()})
    print(pd.DataFrame(out).round(4).to_string(index=False))


def tails(df, n_bags: int = 3):
    df['e0'] = df.v - df.v_ref
    per = df.groupby('bag').apply(lambda g: rmse(g.e0) if len(g) >= LONG_ROWS else 0.0)
    for bag in per.sort_values(ascending=False).index[:n_bags]:
        g = df[df.bag == bag]
        t = g.t.values - g.t.values[0]
        idx = np.nonzero(np.abs(g.e0.values) > LABEL_CLIP)[0]
        print(bag)
        if not len(idx):
            continue
        cuts = np.nonzero(np.diff(t[idx]) > EPISODE_GAP_S)[0]
        for s, e in zip(np.r_[0, cuts + 1], np.r_[cuts, len(idx) - 1]):
            h = g.iloc[idx[s]:idx[e] + 1]
            print(f'  t={t[idx[s]]:7.1f}-{t[idx[e]]:7.1f}s v_ref={h.v_ref.mean():5.2f} v={h.v.mean():5.2f} '
                  f'front={h.wf.mean():5.2f} rear={h.wr.mean():5.2f} notch={h.n0.mean():5.1f} '
                  f'trust={h.ft.mean():.2f}/{h.rt.mean():.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rows', required=True, type=Path)
    ap.add_argument('--models', required=True, type=Path)
    ap.add_argument('--tag', default='d6_800')
    ap.add_argument('--clip', type=float, default=0.03)
    a = ap.parse_args()
    df, feats = load(a.rows)
    df['t'] = np.concatenate([np.load(f)['t'] for f in sorted(a.rows.glob('*.npz'))])
    oracles(df)
    regimes(df, feats, a.models, a.tag, a.clip)
    tails(df)


if __name__ == '__main__':
    main()
