"""Experiment #60: drive model as a table a(notch, v) (D-029/D-033) against the parametric
F0/P form (constant force up to the knee speed, then constant power; constant brake).

Both are fitted on train only (the same pooled samples, delay and resistance as identify.py) and
compared on holdout by integrating the model from the measured speed over windows of 5, 20 and
60 s -- the length of sensor gaps the model has to bridge (docs/data.md, trap 7).

    .venv/bin/python notebooks/identification/compare_forms.py            # ~1 min on 24 cores
    .venv/bin/python notebooks/identification/compare_forms.py --json out/ident/forms.json
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drive_ident as di          # noqa: E402
import identify as idn            # noqa: E402

WINDOWS_S = (5.0, 20.0, 60.0)
STEP_S = 5.0
RARE_MIN = 2000          # train samples of a signed notch below this: a rare cell of the table
F_GRID = np.round(np.arange(0.0, 2.0001, 0.01), 3)       # m/s^2, candidate F (traction plateau)
P_GRID = np.round(np.arange(0.5, 15.0001, 0.1), 2)       # W/kg = m^2/s^3, candidate P/m


def fit_force_power(v, y):
    """Least squares of y = min(F, P / v) over a grid of (F, P); v > 0. Returns (F, P, rmse)."""
    v = np.maximum(np.asarray(v, float), 1e-3)
    y = np.asarray(y, float)
    best = (0.0, P_GRID[-1], np.inf)
    inv_v = 1.0 / v
    for P in P_GRID:
        cap = P * inv_v                                      # power branch
        for F in F_GRID:
            r = np.minimum(F, cap) - y
            sse = float(r @ r)
            if sse < best[2]:
                best = (float(F), float(P), sse)
    return best[0], best[1], float(np.sqrt(best[2] / max(len(y), 1)))


def fit_forms(v, y, n, notch_max=idn.NOTCH_MAX, min_count=30):
    """Per |notch|: traction (F, P) by fit_force_power, brake B = mean of -y. Notches without
    data take the nearest lower notch that has data (monotone like the tables)."""
    F = np.zeros(notch_max + 1)
    P = np.full(notch_max + 1, P_GRID[0])
    B = np.zeros(notch_max + 1)
    for u in range(1, notch_max + 1):
        s = n == u
        if s.sum() >= min_count:
            F[u], P[u], _ = fit_force_power(v[s], y[s])
        else:
            F[u], P[u] = F[u - 1], P[u - 1]
        s = n == -u
        B[u] = float(np.mean(-y[s])) if s.sum() >= min_count else B[u - 1]
    # monotone in notch: a higher notch never pulls or brakes less
    F, P, B = np.maximum.accumulate(F), np.maximum.accumulate(P), np.maximum.accumulate(np.maximum(B, 0))
    return {'F': F.round(4).tolist(), 'P': P.round(3).tolist(), 'B': B.round(4).tolist()}


def form_accel(notch, v, q, c, adhesion):
    """a = sign(n) A(|n|, v) - resistance with A = min(F, P/v) for traction and B for brake."""
    n = np.asarray(notch, int)
    v = np.maximum(np.asarray(v, float), 0.0)
    u = np.clip(np.abs(n), 0, len(q['F']) - 1)
    F, P, B = (np.asarray(q[k])[u] for k in ('F', 'P', 'B'))
    with np.errstate(divide='ignore'):
        trac = np.minimum(F, np.where(v > 0, P / np.maximum(v, 1e-9), np.inf))
    drive = np.where(n > 0, trac, np.where(n < 0, -B, 0.0))
    drive = np.clip(drive, -adhesion, adhesion)
    return drive - di.resistance(v, c)


def simulate(s, accel, delay_s, window_s, step_s=STEP_S):
    """Per-window speed RMSE of a model integrated from the measured speed (no grade), windows
    without gaps and slip, moving at least once; `accel(notch, v)` vectorised."""
    n_w = int(round(window_s / di.DT))
    stride = int(round(step_s / di.DT))
    notch = di.notch_at(s.cmd_t, s.cmd_n, s.t - delay_s)
    good = s.valid & (np.abs(s.dfr) < di.BOGIE_AGREE_MPS)
    starts = np.array([i0 for i0 in range(0, len(s.t) - n_w, stride)
                       if good[i0:i0 + n_w].all() and s.v[i0:i0 + n_w].max() >= di.MOVING_MPS], int)
    if len(starts) == 0:
        return np.zeros(0)
    idx = starts[:, None] + np.arange(n_w)[None, :]
    sim = np.empty(idx.shape)
    sim[:, 0] = s.v[starts]
    for k in range(1, n_w):
        j = idx[:, k - 1]
        sim[:, k] = np.maximum(sim[:, k - 1] + accel(notch[j], sim[:, k - 1]) * di.DT, 0.0)
    return np.sqrt(np.mean((sim - s.v[idx]) ** 2, axis=1))


def with_true_speed(b, fallback):
    vb, ab = idn.true_speed(b, fallback)
    return di.BagSamples(b.name, b.t, vb, ab, b.dfr, b.grade, b.cmd_t, b.cmd_n,
                         b.ratio_front, b.ratio_rear, b.valid)


def run(jobs=8):
    train, hold = idn.load_split('train', jobs), idn.load_split('holdout', jobs)
    res = idn.identify(train)                                  # tables, delay, resistance: train only
    p, delay = res['params'], res['delay_s']
    fallback = float(np.mean([res['wheel'][s]['ratio_median'] for s in ('front', 'rear')]))
    v, a, _, n, _ = idn.pooled(train, fallback, delay, need_grade=False)
    y = a + di.resistance(v, p['c'])                            # = sign(n) A(|n|, v), as the tables
    q = fit_forms(v, y, n)
    table = lambda nn, vv: di.drive_accel(nn, vv, p)                       # noqa: E731
    form = lambda nn, vv: form_accel(nn, vv, q, p['c'], p['adhesion_accel_mps2'])  # noqa: E731
    models = {
        'table': table,
        'form': form,
        # hybrids: which half of the form (traction or brake) costs the accuracy
        'form_traction+table_brake': lambda nn, vv: np.where(np.asarray(nn) > 0, form(nn, vv), table(nn, vv)),
        'table_traction+form_brake': lambda nn, vv: np.where(np.asarray(nn) < 0, form(nn, vv), table(nn, vv)),
        'zero': lambda nn, vv: np.zeros(np.broadcast(nn, vv).shape),
    }
    counts = {int(u): int((n == u).sum()) for u in range(-idn.NOTCH_MAX, idn.NOTCH_MAX + 1) if u}
    rare_notches = [u for u, c in counts.items() if c < RARE_MIN]
    # point residuals on train (fit quality) and on holdout (generalisation)
    fit = {}
    for split, bags in (('train', train), ('holdout', hold)):
        vv, aa, _, nn, _ = idn.pooled(bags, fallback, delay, need_grade=False)
        fit[split] = {m: float(np.std(aa - f(nn, vv))) for m, f in models.items()}
        for mode, sel in (('traction', nn > 0), ('brake', nn < 0)):
            fit[f'{split}_{mode}'] = {m: float(np.std((aa - f(nn, vv))[sel])) for m, f in models.items()}
        rare = np.isin(nn, rare_notches)
        fit[split + '_rare_notches'] = ({m: float(np.std((aa - f(nn, vv))[rare])) for m, f in models.items()}
                                        if rare.any() else None)
    width = len(p['speed_grid_mps'])
    out = {'delay_s': round(delay, 3), 'rare_notches': rare_notches, 'train_counts': counts,
           'n_params': {'table': 2 * (len(p['traction_accel_table']) - width), 'form': 3 * idn.NOTCH_MAX},
           'form_params': q, 'point_resid_std': fit, 'windows': {}}
    hb = [with_true_speed(b, fallback) for b in hold]
    for w in WINDOWS_S:
        errs = {m: np.concatenate([simulate(b, f, delay, w) for b in hb]) for m, f in models.items()}
        out['windows'][str(int(w))] = {
            'n': int(len(errs['table'])),
            'hybrid_median': {m: float(np.median(errs[m])) for m in ('form_traction+table_brake', 'table_traction+form_brake')},
            **{m: {'median': float(np.median(e)), 'mean': float(np.mean(e)), 'p90': float(np.percentile(e, 90)),
                   'max': float(np.max(e))} for m, e in errs.items() if len(e)},
            'form_better_than_table_share': float(np.mean(errs['form'] < errs['table'])) if len(errs['table']) else None,
        }
    return out


def md(out):
    rows = ['| окно, с | окон | таблица: медиана / p90 | форма F0/P: медиана / p90 | a = 0: медиана / p90 | форма лучше таблицы, доля окон |',
            '|---|---|---|---|---|---|']
    for w, r in out['windows'].items():
        if not r['n']:
            continue
        f = lambda m: f"{r[m]['median']:.3f} / {r[m]['p90']:.3f}"
        rows.append(f"| {w} | {r['n']} | {f('table')} | {f('form')} | {f('zero')} | {r['form_better_than_table_share']:.2f} |")
    fit = out['point_resid_std']
    rows += ['', '| std невязки ускорения, м/с² | таблица | форма F0/P | a = 0 |', '|---|---|---|---|']
    for k in ('train', 'holdout', 'train_traction', 'holdout_traction', 'train_brake', 'holdout_brake',
              'train_rare_notches', 'holdout_rare_notches'):
        if fit.get(k):
            rows.append(f"| {k} | {fit[k]['table']:.3f} | {fit[k]['form']:.3f} | {fit[k]['zero']:.3f} |")
    rows += ['', '| окно, с | тяга формы + торможение таблицы | тяга таблицы + торможение формы |', '|---|---|---|']
    for w, r in out['windows'].items():
        if r['n']:
            h = r['hybrid_median']
            rows.append(f"| {w} | {h['form_traction+table_brake']:.3f} | {h['table_traction+form_brake']:.3f} |")
    rows.append(f"\nПараметров (без нулевой строки позиции 0): таблица {out['n_params']['table']}, форма "
                f"{out['n_params']['form']}; задержка {out['delay_s']} с; редкие позиции (< {RARE_MIN} точек train): "
                f"{out['rare_notches']}.")
    return '\n'.join(rows)


def ensure_utf8_stdout() -> None:
    """Console codepage must not crash md()'s 'м/с²' (#128, same class of bug as
    #124/#126)."""
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')


def main(argv=None):
    ensure_utf8_stdout()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--json', type=Path)
    args = ap.parse_args(argv)
    out = run(args.jobs)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding='utf-8')
    print(md(out))


if __name__ == '__main__':
    main()
