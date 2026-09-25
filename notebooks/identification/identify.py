"""Drive model identification on train bags (issue #9, D-030).

    .venv/bin/python notebooks/identification/identify.py                  # report + figures
    .venv/bin/python notebooks/identification/identify.py --write-params   # + params.yaml
    .venv/bin/python notebooks/identification/identify.py --check-split holdout   # read-only check

Only train bags with GNSS are used for fitting (tools/eval/splits.yaml, D-011): GNSS gives the
wheel scale and the grade that is removed from the targets. Output: markdown report to stdout,
out/ident/<commit>/ident.json, figures in notebooks/identification/figures/.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import drive_ident as di                               # noqa: E402

PARAMS_YAML = di.REPO / 'src' / 'tram_odometry' / 'config' / 'params.yaml'
FIG_DIR = Path(__file__).resolve().parent / 'figures'
SPEED_GRID = (0.0, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0)   # m/s
NOTCH_MAX = 15
DELAYS = np.round(np.arange(0.0, 1.01, 0.05), 2)     # s, candidates for the drive delay
WHEEL_SCALE_MIN_CHANGE = 1e-3   # smaller wheel-scale corrections are not written to params.yaml
DELAY_TIE = 1e-4          # scores within this of the best are the same delay (grid resolution)
COAST_HOLD_S = 1.0        # notch 0 for +-this around the sample: steady coasting
ADHESION_PCT = 99.9        # reported tail of |a_drive| in the data (noise + slip), for comparison
POWER_PCT = 95.0           # P/m = this percentile of a_drive * v at the top notches above POWER_MIN_V
POWER_MIN_V = 6.0          # m/s
POWER_TOP_NOTCHES = 3      # notch_max - 2 .. notch_max


def load(name):
    return di.extract(di.evbag.read_bag(di.evbag.data_dir() / name), name)


def load_split(split, jobs):
    names = [n for n in di.evbag.load_splits()[split]]
    with ProcessPoolExecutor(max(1, min(jobs, len(names)))) as ex:
        return list(ex.map(load, names))


def wheel_scales(bags):
    """Per-bogie km/h-per-m/s ratio (median over bags) and the wheel_scale that corrects it."""
    out = {}
    for side in ('front', 'rear'):
        r = np.array([getattr(b, f'ratio_{side}') for b in bags], float)
        r = r[np.isfinite(r)]
        out[side] = {'ratio_median': float(np.median(r)), 'ratio_p5': float(np.percentile(r, 5)),
                     'ratio_p95': float(np.percentile(r, 95)), 'bags': int(len(r)),
                     'wheel_scale': float(3.6 / np.median(r))}
    return out


def true_speed(b, fallback_ratio):
    """Speed and accel of a bag in m/s with the bag's own wheel scale (offline, from GNSS)."""
    r = [x for x in (b.ratio_front, b.ratio_rear) if np.isfinite(x)]
    k = 3.6 / (float(np.mean(r)) if r else fallback_ratio)
    return b.v * k, b.a * k


def pooled(bags, fallback_ratio, delay):
    v, a, g, n, coast = [], [], [], [], []
    for b in bags:
        ok = di.usable(b)
        if not ok.any():
            continue
        vb, ab = true_speed(b, fallback_ratio)
        nb = di.notch_at(b.cmd_t, b.cmd_n, b.t - delay)
        hold = int(round(COAST_HOLD_S / di.DT))
        zero = (nb == 0).astype(int)
        c = np.convolve(zero, np.ones(2 * hold + 1, int), mode='same') == 2 * hold + 1
        v.append(vb[ok]); a.append(ab[ok]); g.append(b.grade[ok]); n.append(nb[ok]); coast.append(c[ok])
    return (np.concatenate(v), np.concatenate(a), np.concatenate(g), np.concatenate(n),
            np.concatenate(coast))


def explained(v, y, n):
    """Share of variance of y explained by the mean in (notch, 1 m/s speed bin) cells."""
    key = (n + 64) * 64 + np.clip(v.astype(int), 0, 63)
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    mean = np.bincount(inv, weights=y) / cnt
    return float(1 - np.var(y - mean[inv]) / np.var(y))


def fit_delay(bags, fallback_ratio):
    scores = []
    for d in DELAYS:
        v, a, g, n, _ = pooled(bags, fallback_ratio, d)
        scores.append(explained(v, a + di.G * g, n))
    # the 10 Hz grid resolves the delay only to DT: equal scores form a plateau, take its middle
    scores = np.asarray(scores)
    top = DELAYS[scores >= scores.max() - DELAY_TIE]
    return float(np.round(top.mean() / 0.05) * 0.05), [round(float(s), 4) for s in scores]


def fit_tables(v, y, n, grid):
    trac = np.full((NOTCH_MAX + 1, len(grid)), np.nan)
    brake = np.full((NOTCH_MAX + 1, len(grid)), np.nan)
    counts = {}
    for u in range(1, NOTCH_MAX + 1):
        for sign, table in ((1, trac), (-1, brake)):
            s = n == sign * u
            if s.sum():
                table[u], c = di.fit_curve(v[s], sign * y[s], grid)
                counts[sign * u] = int(s.sum())
    return di.fill_monotone(trac), di.fill_monotone(brake), counts


def identify(bags):
    grid = np.asarray(SPEED_GRID)
    scales = wheel_scales(bags)
    fallback = float(np.mean([scales['front']['ratio_median'], scales['rear']['ratio_median']]))
    delay, delay_scores = fit_delay(bags, fallback)
    v, a, g, n, coast = pooled(bags, fallback, delay)
    y_ng = a + di.G * g                                  # accel with the grade removed
    c = di.fit_resistance(v[coast & (v > 1.0)], y_ng[coast & (v > 1.0)])
    y = y_ng + di.resistance(v, c)                       # = sign(n) A(|n|, v)
    trac, brake, counts = fit_tables(v, y, n, grid)
    top = n >= NOTCH_MAX - POWER_TOP_NOTCHES + 1
    power = float(np.percentile((y * v)[top & (v > POWER_MIN_V)], POWER_PCT))
    # Adhesion: the largest drive accel the tables ask for, so the limit does not cut them; the
    # tail of |a_drive| in the data is noise and slip (reported, not used).
    adhesion = float(max(trac.max(), brake.max()))
    tail = float(np.percentile(np.abs(y[n != 0]), ADHESION_PCT))
    p = {'notch_max': NOTCH_MAX, 'speed_grid_mps': list(SPEED_GRID),
         'traction_accel_table': [round(float(x), 3) for x in trac.ravel()],
         'brake_accel_table': [round(float(x), 3) for x in brake.ravel()],
         'adhesion_accel_mps2': round(adhesion, 2), 'traction_power_w_per_kg': round(power, 2),
         'c': tuple(round(x, 5) for x in c)}
    resid = y_ng - di.drive_accel(n, v, p)
    noise = {'front_minus_rear_std_mps': float(np.std(np.concatenate([b.dfr[di.usable(b, False)] for b in bags]))),
             'accel_residual_std_mps2': float(np.std(resid)),
             'accel_residual_by_mode': {m: float(np.std(resid[s])) for m, s in
                                        (('traction', n > 0), ('coast', n == 0), ('brake', n < 0))}}
    return {'params': p, 'wheel': scales, 'delay_s': delay, 'delay_scores': delay_scores,
            'n_samples': int(len(v)), 'n_coast': int((coast & (v > 1.0)).sum()), 'notch_counts': counts,
            'explained_notch_speed': explained(v, y_ng, n), 'noise': noise,
            'abs_drive_accel_p999': tail}


def validate(bags, p, delay, fallback_ratio):
    rows = {}
    for grade in (True, False):
        me, ze = [], []
        for b in bags:
            vb, ab = true_speed(b, fallback_ratio)
            bb = di.BagSamples(b.name, b.t, vb, ab, b.dfr, b.grade, b.cmd_t, b.cmd_n,
                               b.ratio_front, b.ratio_rear, b.valid)
            m, z = di.simulate_windows(bb, p, delay, use_grade=grade)
            me.append(m); ze.append(z)
        me, ze = np.concatenate(me), np.concatenate(ze)
        rows['with_grade' if grade else 'no_grade'] = {
            'windows': int(len(me)), 'model_rmse_median': float(np.median(me)), 'model_rmse_mean': float(np.mean(me)),
            'model_rmse_p90': float(np.percentile(me, 90)), 'zero_rmse_median': float(np.median(ze)),
            'zero_rmse_mean': float(np.mean(ze)), 'model_better_share': float(np.mean(me < ze))}
    return rows


def write_params(p, scales, path=PARAMS_YAML):
    """Replace the values of the identified keys in params.yaml, keeping every comment."""
    text = path.read_text(encoding='utf-8')
    def num(x):
        return repr(float(x)) if not isinstance(x, int) else str(x)
    values = {
        # a correction below WHEEL_SCALE_MIN_CHANGE is under the spread between bags: keep 1.0
        **{f'wheel_scale_{side}': num(round(scales[side]['wheel_scale'], 4))
           for side in ('front', 'rear') if abs(scales[side]['wheel_scale'] - 1.0) >= WHEEL_SCALE_MIN_CHANGE},
        'speed_grid_mps': '[' + ', '.join(num(x) for x in p['speed_grid_mps']) + ']',
        'traction_accel_table': '[' + ', '.join(num(x) for x in p['traction_accel_table']) + ']',
        'brake_accel_table': '[' + ', '.join(num(x) for x in p['brake_accel_table']) + ']',
        'adhesion_accel_mps2': num(p['adhesion_accel_mps2']),
        'traction_power_w_per_kg': num(p['traction_power_w_per_kg']),
        'c0': num(p['c'][0]), 'c1': num(p['c'][1]), 'c2': num(p['c'][2]),
    }
    for key, val in values.items():
        pat = re.compile(rf'^(\s+{key}:\s*)(\[[^\]]*\]|[^\s#]+)', re.M)
        text, k = pat.subn(lambda m: m.group(1) + val, text, count=1)
        if k != 1:
            raise SystemExit(f'params.yaml: key {key} not found')
    path.write_text(text, encoding='utf-8')


def plots(res, bags, fallback_ratio):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    p, grid = res['params'], np.asarray(SPEED_GRID)
    v, a, g, n, coast = pooled(bags, fallback_ratio, res['delay_s'])
    y = a + di.G * g + di.resistance(v, p['c'])
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    vv = np.linspace(0, 16, 161)
    cmap = plt.get_cmap('viridis')
    for ax, sign, title in ((axes[0], 1, 'тяга'), (axes[1], -1, 'торможение (модуль)')):
        for u in range(1, NOTCH_MAX + 1, 2):
            col = cmap(u / NOTCH_MAX)
            model = np.abs(di.drive_accel(sign * u, vv, dict(p, c=(0.0, 0.0, 0.0))))
            ax.plot(vv, model, color=col, lw=2, label=f'{sign * u:+d}')
            s = n == sign * u
            if s.sum() > 50:
                bins = np.arange(0, 16.5, 1.0)
                k = np.digitize(v[s], bins)
                mid = [(bins[i - 1] + 0.5, np.median(sign * y[s][k == i])) for i in range(1, len(bins))
                       if (k == i).sum() >= 20]
                if mid:
                    ax.plot(*zip(*mid), 'o', color=col, ms=4)
        ax.set_title(f'a(notch, v): {title}; линии — таблица, точки — медиана train')
        ax.set_xlabel('скорость, м/с')
        ax.grid(alpha=0.3)
        ax.legend(title='notch', ncol=2, fontsize=8)
    axes[0].set_ylabel('ускорение привода, м/с²')
    fig.tight_layout()
    fig.savefig(FIG_DIR / 'drive_accel_table.png', dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4.5))
    s = coast & (v > 1.0)
    ax.plot(v[s][::5], -(a + di.G * g)[s][::5], '.', ms=1.5, alpha=0.3, label='выбег, train (каждая 5-я точка)')
    ax.plot(vv, di.resistance(vv, p['c']), 'r', lw=2,
            label='c0 + c1 v + c2 v² = %.4f + %.5f v + %.6f v²' % p['c'])
    ax.set_ylim(-0.4, 0.4)
    ax.set_xlabel('скорость, м/с')
    ax.set_ylabel('замедление без уклона, м/с²')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(FIG_DIR / 'resistance.png', dpi=110)
    plt.close(fig)


def md_report(res, val, check):
    p, w = res['params'], res['wheel']
    lines = [f"Выборка: {res['n_samples']} точек 10 Гц на движении без проскальзывания, выбег {res['n_coast']}",
             '', '| Что | Значение |', '|---|---|',
             f"| масштаб колеса, км/ч на м/с GNSS (медиана по bag, p5–p95) | перед {w['front']['ratio_median']:.4f} "
             f"({w['front']['ratio_p5']:.4f}–{w['front']['ratio_p95']:.4f}), зад {w['rear']['ratio_median']:.4f} "
             f"({w['rear']['ratio_p5']:.4f}–{w['rear']['ratio_p95']:.4f}) |",
             f"| `wheel_scale_front / rear` = 3,6 / отношение | {w['front']['wheel_scale']:.4f} / {w['rear']['wheel_scale']:.4f} |",
             f"| задержка отклика привода | {res['delay_s']:.2f} с |",
             f"| сопротивление c0, c1, c2 | {p['c'][0]:.4f} м/с², {p['c'][1]:.5f} 1/с, {p['c'][2]:.6f} 1/м |",
             f"| предел сцепления `adhesion_accel_mps2` = максимум таблиц | {p['adhesion_accel_mps2']} м/с² "
             f"(хвост p99,9 \\|a\\| в данных {res['abs_drive_accel_p999']:.2f} — шум и юз) |",
             f"| удельная мощность `traction_power_w_per_kg` | {p['traction_power_w_per_kg']} Вт/кг |",
             f"| доля дисперсии ускорения (без уклона), объяснённая ячейкой notch × v | {res['explained_notch_speed']:.3f} |",
             f"| std невязки ускорения: всё / тяга / выбег / торможение | {res['noise']['accel_residual_std_mps2']:.3f} / "
             + ' / '.join(f"{res['noise']['accel_residual_by_mode'][m]:.3f}" for m in ('traction', 'coast', 'brake')) + ' м/с² |',
             f"| std (перед − зад) без проскальзывания | {res['noise']['front_minus_rear_std_mps']:.3f} м/с |",
             '', 'Приёмка: окна 10 с, шаг 5 с, интегрирование модели от измеренной скорости; RMSE скорости в окне, м/с',
             '', '| Набор | уклон | окон | модель, медиана | модель, среднее | модель, p90 | a = 0, медиана | a = 0, среднее | модель лучше, доля окон |',
             '|---|---|---|---|---|---|---|---|---|']
    for split, rows in (('train', val),) + ((('holdout (проверка)', check),) if check else ()):
        for k, r in rows.items():
            lines.append(f"| {split} | {'из GNSS' if k == 'with_grade' else 'нет'} | {r['windows']} | {r['model_rmse_median']:.3f} | "
                         f"{r['model_rmse_mean']:.3f} | {r['model_rmse_p90']:.3f} | {r['zero_rmse_median']:.3f} | "
                         f"{r['zero_rmse_mean']:.3f} | {r['model_better_share']:.2f} |")
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--split', default='train')
    ap.add_argument('--check-split', default=None, help='набор только для проверки (не для подгонки)')
    ap.add_argument('--jobs', type=int, default=max(1, min(8, (os.cpu_count() or 2) - 1)))
    ap.add_argument('--write-params', action='store_true')
    ap.add_argument('--no-plots', action='store_true')
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    bags = [b for b in load_split(args.split, args.jobs) if len(b.t)]
    res = identify(bags)
    fallback = float(np.mean([res['wheel']['front']['ratio_median'], res['wheel']['rear']['ratio_median']]))
    val = validate(bags, res['params'], res['delay_s'], fallback)
    check = None
    if args.check_split:
        cb = [b for b in load_split(args.check_split, args.jobs) if len(b.t)]
        check = validate(cb, res['params'], res['delay_s'], fallback)
    commit = subprocess.run(['git', '-C', str(di.REPO), 'rev-parse', '--short', 'HEAD'],
                            capture_output=True, text=True).stdout.strip() or 'nogit'
    out = di.evbag.out_dir() / 'ident' / commit
    out.mkdir(parents=True, exist_ok=True)
    (out / 'ident.json').write_text(json.dumps({'split': args.split, 'bags': len(bags), **res,
                                                'validation': val, 'check': check},
                                               ensure_ascii=False, indent=1, default=list), encoding='utf-8')
    if not args.no_plots:
        plots(res, bags, fallback)
    if args.write_params:
        write_params(res['params'], res['wheel'])
    print(md_report(res, val, check))
    print(f"\n{len(bags)} bag ({args.split}); -> {out / 'ident.json'}"
          + ('; params.yaml обновлён' if args.write_params else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
