"""Leave-one-day-out CV of speed-residual models on train rows (extract.py output).

Row level only: the validation rows keep their pipeline speed `v` and get `v + clip(model)`.
Folds are whole recording days of the train split; no row of a validation day is in training.
Runs in a separate env with catboost (not a dependency of the repo):

    <env>/bin/python tools/speed_residual/cv.py --rows <dir> --out <dir> [--gpu]
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np
import pandas as pd

MODES = np.array(['accel', 'brake', 'stop', 'cruise'])
LABEL_CLIP = 0.5     # m/s: |v_ref - v| above this is a skid/GNSS glitch, not a learnable bias
LONG_ROWS = 3000     # bags with fewer matched rows are too short for a per-bag RMSE


def load(rows: Path) -> tuple[pd.DataFrame, list[str]]:
    parts = []
    for f in sorted(glob.glob(str(rows / '*.npz'))):
        d = np.load(f)
        feats = [str(x) for x in d['features']]
        assert not {'bag', 'day', 'v_ref', 'mode', 'res'} & set(feats), 'feature name collides with a label column'
        df = pd.DataFrame(d['X'], columns=feats)
        df['bag'], df['day'] = Path(f).stem, str(d['day'])
        df['v_ref'], df['mode'] = d['v_ref'], MODES[d['mode']]
        parts.append(df)
    df = pd.concat(parts, ignore_index=True)
    df['res'] = df.v_ref - df.v
    return df, feats


def rmse(e) -> float:
    return float(np.sqrt(np.mean(np.square(e))))


def report(val: pd.DataFrame, pred: np.ndarray) -> dict:
    """Metrics of a corrected validation day: per-bag RMSE median, modes, tails."""
    v_new = np.maximum(0.0, val.v.values + pred)
    e0, e1 = val.v.values - val.v_ref.values, v_new - val.v_ref.values
    out = {'n': len(val)}
    per0, per1 = [], []
    for _, idx in val.groupby('bag').indices.items():
        if len(idx) >= LONG_ROWS:
            per0.append(rmse(e0[idx]))
            per1.append(rmse(e1[idx]))
    per0, per1 = np.array(per0), np.array(per1)
    out['bag_rmse_med0'], out['bag_rmse_med1'] = float(np.median(per0)), float(np.median(per1))
    out['bag_rmse_max0'], out['bag_rmse_max1'] = float(per0.max()), float(per1.max())
    out['bags_better'] = int((per1 < per0).sum())
    out['bags_worse_2pct'] = int((per1 > per0 * 1.02).sum())
    out['bag_gain_med_pct'] = float(np.median((per0 - per1) / per0 * 100))
    out['pooled0'], out['pooled1'] = rmse(e0), rmse(e1)
    small = np.abs(e0) < LABEL_CLIP
    out['clean0'], out['clean1'] = rmse(e0[small]), rmse(e1[small])
    out['p99_0'], out['p99_1'] = float(np.quantile(np.abs(e0), 0.99)), float(np.quantile(np.abs(e1), 0.99))
    for m in MODES:
        s = (val['mode'].values == m) & small
        out[f'{m}_bias0'], out[f'{m}_bias1'] = float(e0[s].mean()), float(e1[s].mean())
        out[f'{m}_rmse0'], out[f'{m}_rmse1'] = rmse(e0[s]), rmse(e1[s])
    return out


def fit_linear(tr, feats, cols):
    """Ridge on a few columns, robust target (clipped rows dropped)."""
    X = tr[cols].values.copy()
    X[X == -999.0] = 0.0
    X = np.c_[np.ones(len(X)), X]
    y = tr.res.values
    lam = 1e-3 * len(X)
    w = np.linalg.solve(X.T @ X + lam * np.diag([0.0] + [1.0] * len(cols)), X.T @ y)
    return lambda d: np.c_[np.ones(len(d)), np.where(d[cols].values == -999.0, 0.0, d[cols].values)] @ w, w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rows', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--gpu', action='store_true')
    a = ap.parse_args()
    from catboost import CatBoostRegressor
    df, feats = load(a.rows)
    a.out.mkdir(parents=True, exist_ok=True)
    days = sorted(df.day.unique())
    results, preds = [], {}
    for day in days:
        tr_all, val = df[df.day != day], df[df.day == day]
        tr = tr_all[np.abs(tr_all.res) < LABEL_CLIP]
        cands = {'zero': lambda d: np.zeros(len(d))}
        const = float(tr.res.mean())
        cands['const'] = lambda d, c=const: np.full(len(d), c)
        lin_a, w_a = fit_linear(tr, feats, ['a', 'am', 'v'])
        cands['lin_a_am_v'] = lin_a
        lin_all, _ = fit_linear(tr, feats, [f for f in feats if f not in ('acc',)])
        cands['lin_all'] = lin_all
        for depth, iters in ((4, 200), (6, 600)):
            m = CatBoostRegressor(depth=depth, iterations=iters, learning_rate=0.05, loss_function='RMSE',
                                  task_type='GPU' if a.gpu else 'CPU', verbose=0, random_seed=0,
                                  border_count=64, l2_leaf_reg=10, allow_writing_files=False)
            m.fit(tr[feats].values, tr.res.values)
            name = f'cb_d{depth}_{iters}'
            m.save_model(str(a.out / f'{name}_{day}.json'), format='json')
            p = m.predict(val[feats].values)
            cands[name] = lambda d, p=p: p
            for c in (0.03, 0.1):
                cands[f'{name}_clip{c}'] = lambda d, p=p, c=c: np.clip(p, -c, c)
        for name, f in cands.items():
            p = f(val)
            preds[(day, name)] = p
            results.append({'day': day, 'model': name, **report(val, p)})
        print(day, 'done', flush=True)
    res = pd.DataFrame(results)
    res.to_csv(a.out / 'cv.csv', index=False)
    pd.set_option('display.width', 250)
    cols = ['day', 'model', 'bag_rmse_med0', 'bag_rmse_med1', 'bag_gain_med_pct', 'bags_better',
            'bags_worse_2pct', 'bag_rmse_max0', 'bag_rmse_max1', 'clean0', 'clean1', 'p99_0', 'p99_1']
    print(res[cols].round(4).to_string(index=False))
    print(res[['day', 'model'] + [f'{m}_bias{k}' for m in MODES for k in (0, 1)]].round(4).to_string(index=False))
    print(json.dumps({'const': const, 'lin_a_am_v_w': list(map(float, w_a))}))


if __name__ == '__main__':
    main()
