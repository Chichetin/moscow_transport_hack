"""Experiment #154, extension: statistics of base against h123 / h3 (markdown to stdout).

  pairs <base metrics.json> <head metrics.json>   paired per-bag statistics of run_eval outputs:
        bootstrap by bag of the relative change of the median (the statistic of D-012) and of
        the median of paired differences, sign test, Wilcoxon, the null of swapping base and
        head within a bag (sign flips of the paired difference), per day, bags around the
        median of drift_pct, short bags
  days  <wheel_scale_exp json>                     clean medians per day of every variant
  grid  <dir of wheel_scale_ext grid-*> [versus]    stress grid per cell and variant against base (or versus)
  std   <dir of wheel_scale_ext std-train>          sensitivity around h123
  curve <dir of wheel_scale_ext grid-*>             drift at fractions of the run, end error in m
  sensgrid <dir of wheel_scale_ext grid-*>         one row per variant over the grid, against base and h123

    .venv/bin/python tools/eval/experiments/wheel_scale_stats.py pairs out/eval/63762d2-holdout-stress/metrics.json out/eval/20e3e1e-holdout-stress/metrics.json
"""
from pathlib import Path
import json, math, os, re, sys
import numpy as np

EVAL = Path(os.environ.get('WSX_EVAL') or Path(__file__).resolve().parents[1])
SPLITS = EVAL / 'splits.yaml'
B_BOOT = 20000
N_PERM_MC = 400000
SEED = 154
D012 = 0.02
METRICS = ('speed_rmse', 'along_rmse', 'drift_pct', 'along_max', 'pos3d_max')


def bag_days():
    days = {}
    for line in SPLITS.read_text(encoding='utf-8').splitlines():
        m = re.match(r"\s*-\s*'([0-9_a-f]+)'\s*#\s*(\d{4}-\d\d-\d\d)", line)
        if m:
            days[m.group(1)] = m.group(2)
    return days


def f(v, n=4):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return '—'
    return f'{v:.{n}g}'


def pct(v, n=1):
    return '—' if v is None or not math.isfinite(v) else f'{100 * v:+.{n}f} %'


def rel(b, h):
    mb = np.median(b)
    return (np.median(h) - mb) / abs(mb) if mb != 0 else math.nan


def boot(b, h, B=B_BOOT, seed=SEED):
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(b), (B, len(b)))
    mb, mh = np.median(b[idx], axis=1), np.median(h[idx], axis=1)
    with np.errstate(divide='ignore', invalid='ignore'):
        r = (mh - mb) / np.abs(mb)
    return r, np.median((h - b)[idx], axis=1)


def swap_null(b, h, n_mc=N_PERM_MC, seed=SEED):
    """Relative change of the median when base and head are swapped at random within a bag:
    exact over all swaps of the bags that differ when there are at most 18 of them."""
    nz = np.flatnonzero(b != h)
    k = len(nz)
    rng = np.random.default_rng(seed)
    out = []
    if k <= 18:
        codes = np.arange(2 ** k, dtype=np.int64)
        masks = [((codes[i:i + 65536, None] >> np.arange(k)) & 1).astype(bool) for i in range(0, 2 ** k, 65536)]
        exact = True
    else:
        masks = [rng.random((min(50000, n_mc - i), k)) < 0.5 for i in range(0, n_mc, 50000)]
        exact = False
    for mk in masks:
        S = np.zeros((len(mk), len(b)), bool)
        S[:, nz] = mk
        B2, H2 = np.where(S, h, b), np.where(S, b, h)
        mb, mh = np.median(B2, axis=1), np.median(H2, axis=1)
        with np.errstate(divide='ignore', invalid='ignore'):
            out.append((mh - mb) / np.abs(mb))
    return np.concatenate(out), exact, k


def paired(b, h):
    """b, h: arrays of the same bags (no None). Dict of all the statistics."""
    from scipy import stats
    d = h - b
    worse, better = int((d > 0).sum()), int((d < 0).sum())
    r_obs = rel(b, h)
    rb, dm = boot(b, h)
    null, exact, k = swap_null(b, h)
    out = dict(n=len(b), med_b=float(np.median(b)), med_h=float(np.median(h)), rel=float(r_obs),
               mean_b=float(np.mean(b)), mean_h=float(np.mean(h)),
               rel_ci=(float(np.nanpercentile(rb, 2.5)), float(np.nanpercentile(rb, 97.5))),
               p_boot_gt2=float(np.mean(rb > D012)),
               dmed=float(np.median(d)), dmed_ci=(float(np.percentile(dm, 2.5)), float(np.percentile(dm, 97.5))),
               dmean=float(np.mean(d)), worse=worse, better=better, ties=int(len(d) - worse - better),
               p_sign=float(stats.binomtest(worse, worse + better, 0.5).pvalue) if worse + better else 1.0,
               null_exact=exact, null_k=k,
               p_null_gt2=float(np.mean(null > D012)), p_null_ge_obs=float(np.mean(null >= r_obs - 1e-12)),
               p_null_abs=float(np.mean(np.abs(null) >= abs(r_obs) - 1e-12)),
               null_q=(float(np.percentile(null, 2.5)), float(np.percentile(null, 97.5))))
    try:
        out['p_wilcoxon'] = float(stats.wilcoxon(d, zero_method='wilcox').pvalue) if worse + better else 1.0
        out['p_wilcoxon_pratt'] = float(stats.wilcoxon(d, zero_method='pratt').pvalue) if worse + better else 1.0
    except ValueError:
        out['p_wilcoxon'] = out['p_wilcoxon_pratt'] = math.nan
    return out


def pair_arrays(base, head, key, names=None):
    names = names or sorted(set(base) & set(head))
    ok = [n for n in names if base[n].get(key) is not None and head[n].get(key) is not None]
    return ok, np.array([base[n][key] for n in ok], float), np.array([head[n][key] for n in ok], float)


def row(name, s, nd=4):
    return (f"| {name} | {s['n']} | {f(s['med_b'], nd)} → {f(s['med_h'], nd)} | **{pct(s['rel'])}** "
            f"[{pct(s['rel_ci'][0])}; {pct(s['rel_ci'][1])}] | {s['p_boot_gt2']:.2f} | "
            f"{f(s['dmed'], 3)} [{f(s['dmed_ci'][0], 3)}; {f(s['dmed_ci'][1], 3)}] | {f(s['dmean'], 3)} | "
            f"{s['better']}/{s['worse']}/{s['ties']} | {s['p_sign']:.3f} | {s['p_wilcoxon']:.3f} | "
            f"{s['p_null_gt2']:.3f} | {s['p_null_ge_obs']:.3f} | {s['p_null_abs']:.3f} | "
            f"[{pct(s['null_q'][0])}; {pct(s['null_q'][1])}] |")


HEAD = ('| метрика / срез | n | медиана base → head | Δ медианы, % [95 % ДИ бутстрэп] | P_boot(Δ>+2 %) | '
        'медиана парных Δ [95 % ДИ] | среднее парных Δ | лучше/хуже/равно | p знак | p Wilcoxon | '
        'P_0(Δ>+2 %) | P_0(Δ≥набл.) | P_0(|Δ|≥|набл.|) | 95 % нуля |\n'
        '|---|---:|---|---|---:|---|---:|---|---:|---:|---:|---:|---:|---|')


def cmd_pairs(pb, ph):
    B, H = json.loads(Path(pb).read_text()), json.loads(Path(ph).read_text())
    base, head = B['bags'], H['bags']
    days = bag_days()
    print(f"base `{B['commit']}` ({B['split']}, {B['created']}) → head `{H['commit']}` ({H['split']}, {H['created']}); "
          f"bootstrap {B_BOOT}, seed {SEED}\n")
    print(HEAD)
    for key in METRICS:
        names, b, h = pair_arrays(base, head, key)
        print(row(key, paired(b, h)))
    ds = sorted({days[n] for n in base})
    for key in ('drift_pct', 'along_rmse', 'speed_rmse'):
        for d in ds:
            names, b, h = pair_arrays(base, head, key, [n for n in base if days[n] == d])
            if len(names) >= 3:
                print(row(f'{key} · {d}', paired(b, h)))
    # bags around the median of drift_pct
    names, b, h = pair_arrays(base, head, 'drift_pct')
    print(f"\nbag с drift_pct: {len(names)}; без drift_pct (путь < 50 м, metrics.MIN_DRIFT_PATH_M, в медиану "
          f"не входят): " + ', '.join(f"`{n}` {base[n]['distance_m']:.1f} м" for n in sorted(base) if n not in names))
    print('\n| bag | день | путь, м | drift base, % | drift head, % | ≈ ошибка в конце base → head, м | ранг base / head |\n|---|---|---:|---:|---:|---|---|')
    rb, rh = np.argsort(np.argsort(b)), np.argsort(np.argsort(h))
    lo, hi = len(b) // 2 - 4, len(b) // 2 + 4
    for i in sorted(range(len(b)), key=lambda i: b[i]):
        if lo <= rb[i] < hi or lo <= rh[i] < hi:
            dist = base[names[i]]['distance_m']
            print(f"| `{names[i]}` | {days[names[i]]} | {dist:.0f} | {b[i]:.4f} | {h[i]:.4f} | "
                  f"{b[i] * dist / 100:.2f} → {h[i] * dist / 100:.2f} | {rb[i] + 1} / {rh[i] + 1} |")
    # jackknife: drop one bag, relative change of the median
    jk = [(names[i], rel(np.delete(b, i), np.delete(h, i))) for i in range(len(b))]
    over = [n for n, r in jk if r > D012]
    print(f"\nбез одного bag (jackknife) Δ медианы drift_pct: от {pct(min(r for _, r in jk))} до {pct(max(r for _, r in jk))}; "
          f"> +2 % в {len(over)} из {len(jk)}")


def cmd_days(path):
    res = json.loads(Path(path).read_text())
    days = bag_days()
    variants = list(dict.fromkeys(k.split('|')[0] for r in res.values() for k in r))
    ds = sorted({days[n] for n in res})
    print('| вариант | ' + ' | '.join(f'{d} (n) speed / along / drift' for d in ds) + ' |')
    print('|---|' + '---|' * len(ds))
    for v in variants:
        cells = []
        for d in ds:
            bags = [res[n][f'{v}|clean'] for n in res if days[n] == d]
            med = [np.median([x[k] for x in bags if x[k] is not None]) for k in ('speed_rmse', 'along_rmse', 'drift_pct')]
            nd = sum(1 for x in bags if x['drift_pct'] is not None)
            cells.append(f'({len(bags)}/{nd}) {med[0]:.4f} / {med[1]:.3f} / {med[2]:.4f}')
        print(f'| {v} | ' + ' | '.join(cells) + ' |')
    print('\nh123 и другие против base по дням (Δ медианы; лучше/хуже/равно по bag; парная медиана Δ):\n')
    print('| вариант | метрика | ' + ' | '.join(ds) + ' | все |\n|---|---|' + '---|' * (len(ds) + 1))
    for v in variants:
        if v == 'base':
            continue
        for key in ('along_rmse', 'drift_pct', 'speed_rmse'):
            cells = []
            for d in ds + ['all']:
                names = [n for n in res if d == 'all' or days[n] == d]
                bb = {n: res[n]['base|clean'] for n in names}
                hh = {n: res[n][f'{v}|clean'] for n in names}
                _, b, h = pair_arrays(bb, hh, key)
                dd = h - b
                cells.append(f'{pct(rel(b, h))}; {int((dd < 0).sum())}/{int((dd > 0).sum())}/{int((dd == 0).sum())}; {np.median(dd):+.4f}')
            print(f'| {v} | {key} | ' + ' | '.join(cells) + ' |')


# ------------------------------------------------------------------------------------------ grid
def load_dir(d):
    out = {}
    for p in sorted(Path(d).glob('*__*.json')):
        j = json.loads(p.read_text())
        out.setdefault(j['variant'], {})[j['bag']] = j['res']
    return out


def stop_counts(ev, t0=-math.inf, false_m=10.0):
    sn = [e for e in ev if e[1] in ('snap', 'recover') and e[0] >= t0]
    false = [e for e in sn if e[4] is not None and abs(e[2] - e[4]) > false_m]
    rec = [e for e in sn if e[1] == 'recover']
    rec_false = [e for e in rec if e[4] is not None and abs(e[2] - e[4]) > false_m]
    return len(sn), len(false), len(rec), len(rec_false)


def cell(res, sc):
    """Aggregates of one variant and scenario over bags."""
    rows = {n: r[sc] for n, r in res.items() if sc in r and not r[sc].get('skipped')}
    if not rows:
        return None
    exc = np.array([r['excess_pos'] if r['excess_pos'] is not None else np.nan for r in rows.values()])
    pk = np.array([r['peak_pos'] if r['peak_pos'] is not None else np.nan for r in rows.values()])
    rec = [r['rec_pos'] for r in rows.values() if r['rec_pos'] is not None]
    st = np.array([stop_counts(r.get('stops', []), r['start']) for r in rows.values()]).sum(axis=0)
    med = lambda k: float(np.median([r[k] for r in rows.values() if r[k] is not None]))
    return dict(n=len(rows), nrec=sum(1 for r in rows.values() if r['rec_pos'] is None),
                exc_med=float(np.nanmedian(exc)), exc_p90=float(np.nanpercentile(exc, 90)), exc_max=float(np.nanmax(exc)),
                pk_max=float(np.nanmax(pk)), pk_med=float(np.nanmedian(pk)),
                rec_med=float(np.median(rec)) if rec else math.nan,
                along=med('along_rmse'), drift=med('drift_pct'),
                snaps=int(st[0]), false=int(st[1]), relock=int(st[2]), relock_false=int(st[3]),
                exc_by_bag={n: r['excess_pos'] for n, r in rows.items()},
                worst=max(rows, key=lambda n: rows[n]['excess_pos'] or 0))


def family(sc):
    kind, at = sc.split('@')
    if kind.startswith('gap'):
        return f'{kind}', at
    return ('масштаб < 1' if float(kind[2:]) < 1 else 'масштаб > 1'), at


def cmd_grid(d, versus='base'):
    res = load_dir(d)
    variants = [v for v in ('base', 'h1_ema', 'h12', 'h3', 'h3_v0', 'h13', 'h123', 'h123_k1.5', 'h123_k2.5', 'h123_k3',
                            'h123_p0.5', 'h123_p1.5', 'h123_p2', 'h123_s1.5', 'h123_s3', 'h123_s5',
                            'h123_a200', 'h123_a500') if v in res]
    only = os.environ.get('WSX_VARIANTS')
    if only:
        variants = [v for v in variants if v in only.split(',')]
    scen = [k for k in next(iter(next(iter(res.values())).values())) if k != 'clean']
    nb = len(res[variants[0]])
    print(f'{d}: варианты {" / ".join(variants)}; bag {nb}. В ячейке — значения вариантов через «/». '
          f'Добавка = пик (ошибка pos3d грязного − чистого того же варианта) во время события; не восст. — добавка '
          f'не ушла ≤ 2 м на 2 с до конца bag (для масштаба до конца — в хвосте 3 с), как run_eval --stress. '
          f'Ложная привязка — место дальше 10 м от истинного s (трек GNSS master, только eval), считаются с начала события.\n')
    summ = {(sc, v): cell(res[v], sc) for sc in scen for v in variants}
    print('| сценарий | bag | не восст. | добавка мед., м | p90, м | max, м | along_rmse мед., м | drift_pct мед. | '
          'ложные / привязки | восст. захвата (ложные) |')
    print('|---|---:|---|---|---|---|---|---|---|---|')
    J = lambda sc, fn: ' / '.join(fn(summ[(sc, v)]) for v in variants)
    for sc in scen:
        print(f"| {sc} | {summ[(sc, variants[0])]['n']} | {J(sc, lambda c: str(c['nrec']))} | {J(sc, lambda c: f'{c[chr(101)+'xc_med']:.1f}')} | "
              f"{J(sc, lambda c: f'{c['exc_p90']:.0f}')} | {J(sc, lambda c: f'{c['exc_max']:.0f}')} | {J(sc, lambda c: f'{c['along']:.2f}')} | "
              f"{J(sc, lambda c: f'{c['drift']:.3f}')} | {J(sc, lambda c: f'{c['false']}/{c['snaps']}')} | "
              f"{J(sc, lambda c: f'{c['relock']} ({c['relock_false']})')} |")
    for v in variants:
        if v == versus:
            continue
        print(f'\n### {v} против {versus}\n')
        fams = {}
        for sc in scen:
            a, b = summ[(sc, versus)], summ[(sc, v)]
            fam = family(sc)
            for key in (fam, (fam[0], 'все'), ('все', 'все')):
                x = fams.setdefault(key, dict(cells=0, nrec=[0, 0], med=[0, 0], p90=[0, 0], mx=[0, 0], nrec_a=0, nrec_b=0,
                                             bags=0, w5=0, b5=0, w20=0, b20=0, w20_false=0))
                x['cells'] += 1
                x['nrec_a'] += a['nrec']
                x['nrec_b'] += b['nrec']
                for k, k2 in (('nrec', 'nrec'), ('med', 'exc_med'), ('p90', 'exc_p90'), ('mx', 'exc_max')):
                    x[k][0] += b[k2] < a[k2]
                    x[k][1] += b[k2] > a[k2]
                for n, ea in a['exc_by_bag'].items():
                    eb = b['exc_by_bag'].get(n)
                    if ea is None or eb is None:
                        continue
                    x['bags'] += 1
                    x['w5'] += eb - ea > 5
                    x['b5'] += eb - ea < -5
                    x['w20'] += eb - ea > 20
                    x['b20'] += eb - ea < -20
        print('| семейство · начало | ячеек | не восст., сумма | не восст. лучше/хуже, ячеек | медиана добавки лучше/хуже | p90 лучше/хуже | max лучше/хуже | '
              'bag-ячейки хуже > 5 м / лучше > 5 м | хуже > 20 м / лучше > 20 м |')
        print('|---|---:|---|---|---|---|---|---|---|')
        for key in sorted(fams, key=lambda k: (k[0] == 'все', k[0], k[1] == 'все', k[1])):
            x = fams[key]
            print(f"| {key[0]} · {key[1]} | {x['cells']} | {x['nrec_a']} → {x['nrec_b']} | {x['nrec'][0]}/{x['nrec'][1]} | {x['med'][0]}/{x['med'][1]} | "
                  f"{x['p90'][0]}/{x['p90'][1]} | {x['mx'][0]}/{x['mx'][1]} | {x['w5']}/{x['b5']} из {x['bags']} | {x['w20']}/{x['b20']} |")
        # the tail: bag-cells where v loses more than 20 m, and whether v snapped falsely after the start
        losses = []
        for sc in scen:
            for n, rb in res[v].items():
                ra = res[versus].get(n, {}).get(sc)
                rv = rb.get(sc)
                if not ra or not rv or ra.get('skipped') or rv.get('skipped') or ra['excess_pos'] is None or rv['excess_pos'] is None:
                    continue
                dl = rv['excess_pos'] - ra['excess_pos']
                if dl > 20:
                    fv = [e for e in rv.get('stops', []) if e[1] in ('snap', 'recover') and e[0] >= rv['start'] and e[4] is not None and abs(e[2] - e[4]) > 10]
                    fa = [e for e in ra.get('stops', []) if e[1] in ('snap', 'recover') and e[0] >= ra['start'] and e[4] is not None and abs(e[2] - e[4]) > 10]
                    losses.append((dl, sc, n, ra['excess_pos'], rv['excess_pos'], len(fv), len(fa),
                                   fv[0][1] if fv else '', rv.get('scale_end')))
        losses.sort(reverse=True)
        withf = sum(1 for x in losses if x[5] > 0)
        print(f'\nхвост: bag-ячеек, где {v} хуже {versus} больше чем на 20 м: {len(losses)}; из них с ложной привязкой у {v} после начала события: {withf}; '
              f'bag: ' + ', '.join(f'`{b}` {c}' for b, c in sorted(__import__("collections").Counter(x[2] for x in losses).items(), key=lambda t: -t[1])[:8]))
        if losses:
            print('\n| потеря, м | сценарий | bag | добавка base → вариант, м | ложных привязок вариант / base | масштаб в конце |\n|---:|---|---|---|---|---:|')
            for x in losses[:12]:
                print(f'| {x[0]:+.1f} | {x[1]} | `{x[2]}` | {x[3]:.1f} → {x[4]:.1f} | {x[5]} / {x[6]} | {x[8]:.4f} |')
        # gap recovery time on bags that both recover
        rows = []
        for sc in scen:
            if not sc.startswith('gap'):
                continue
            both, only_a, only_b = [], 0, 0
            for n, rb in res[v].items():
                ra, rv = res[versus][n].get(sc), rb.get(sc)
                if not ra or ra.get('skipped'):
                    continue
                if ra['rec_pos'] is not None and rv['rec_pos'] is not None:
                    both.append(rv['rec_pos'] - ra['rec_pos'])
                only_a += ra['rec_pos'] is not None and rv['rec_pos'] is None
                only_b += ra['rec_pos'] is None and rv['rec_pos'] is not None
            rows.append(f"{sc}: оба {len(both)}, Δ времени восст. мед. {np.median(both):+.1f} с (лучше/хуже {sum(x < 0 for x in both)}/{sum(x > 0 for x in both)}), "
                        f"восст. только {versus} {only_a}, только {v} {only_b}")
        if rows:
            print('\nпропуск обеих тележек, время восстановления: ' + '; '.join(rows))


def cmd_std(d):
    res = load_dir(d)
    order = [v for v in ('base', 'h1_ema', 'h12', 'h3', 'h3_v0', 'h13', 'h123', 'h123_k1.5', 'h123_k2.5', 'h123_k3',
                         'h123_p0.5', 'h123_p1.5', 'h123_p2', 'h123_s1.5', 'h123_s3', 'h123_s5',
                         'h123_a200', 'h123_a500') if v in res]
    days = bag_days()
    base = {n: r['clean'] for n, r in res['base'].items()}
    h123 = {n: r['clean'] for n, r in res['h123'].items()}
    print('| вариант | speed_rmse мед. | along_rmse мед. (Δ к base) | drift_pct мед. (Δ к base; к h123) | drift: лучше/хуже/равно к base | '
          'drift Δ медианы к base по дням | along_max max | не восст. up / down / gap | добавка пика мед. up / down / gap, м | '
          'пик pos3d max up / down / gap, м | ложные привязки clean / стресс | восст. захвата (ложные) clean / up / down / gap |')
    print('|---|---:|---|---|---|---|---:|---|---|---|---|---|')
    for v in order:
        cl = {n: r['clean'] for n, r in res[v].items()}
        _, b, h = pair_arrays(base, cl, 'drift_pct')
        _, hb, hh = pair_arrays(h123, cl, 'drift_pct')
        _, ab, ah = pair_arrays(base, cl, 'along_rmse')
        dd = h - b
        per_day = []
        for day in sorted({days[n] for n in cl}):
            _, b2, h2 = pair_arrays({n: x for n, x in base.items() if days[n] == day},
                                    {n: x for n, x in cl.items() if days[n] == day}, 'drift_pct')
            per_day.append(pct(rel(b2, h2)))
        cs = {sc: cell(res[v], sc) for sc in STD_SC}
        stc = lambda sc: np.array([stop_counts(r[sc].get('stops', [])) for r in res[v].values()
                                   if sc in r and not r[sc].get('skipped')]).sum(axis=0)
        s0 = stc('clean')
        s_all = [stc(sc) for sc in STD_SC]
        speed = np.median([x['speed_rmse'] for x in cl.values() if x['speed_rmse'] is not None])
        amax = max(x['along_max'] for x in cl.values() if x['along_max'] is not None)
        print(f"| {v} | {speed:.4f} | {np.median(ah):.3f} ({pct(rel(ab, ah))}) | {np.median(h):.4f} ({pct(rel(b, h))}; {pct(rel(hb, hh))}) | "
              f"{int((dd < 0).sum())}/{int((dd > 0).sum())}/{int((dd == 0).sum())} | {' · '.join(per_day)} | {amax:.1f} | "
              + ' / '.join(str(cs[sc]['nrec']) for sc in STD_SC) + ' | '
              + ' / '.join(f"{cs[sc]['exc_med']:.2f}" for sc in STD_SC) + ' | '
              + ' / '.join(f"{cs[sc]['pk_max']:.1f}" for sc in STD_SC) + ' | '
              + f"{s0[1]} / {sum(s[1] for s in s_all)} | "
              + ' / '.join(f'{s[2]} ({s[3]})' for s in [s0] + s_all) + ' |')


STD_SC = ('scale_up', 'scale_down', 'gap_both_30')


def cmd_curve(d):
    res = load_dir(d)
    days = bag_days()
    base, head = res['base'], res['h123']
    names = sorted(n for n in base if n in head and base[n]['clean'].get('curve') and head[n]['clean'].get('curve'))
    fr = [round(x, 2) for x in np.arange(0.30, 1.0001, 0.05)]
    print(f'{d}: {len(names)} bag. drift в доле f прогона = гориз. ошибка в этот момент / путь эталона до него · 100 '
          '(bag с путём < 50 м к этому моменту — вне медианы)\n')
    print(HEAD)
    for i, x in enumerate(fr):
        ok = [n for n in names if base[n]['clean']['curve'][i][1] >= 50 and head[n]['clean']['curve'][i][1] >= 50]
        b = np.array([base[n]['clean']['curve'][i][0] / base[n]['clean']['curve'][i][1] * 100 for n in ok])
        h = np.array([head[n]['clean']['curve'][i][0] / head[n]['clean']['curve'][i][1] * 100 for n in ok])
        print(row(f'drift @ {x:.2f}', paired(b, h)))
    # the end, unrounded, and the end error in metres
    ok = [n for n in names if base[n]['clean']['drift_pct'] is not None]
    for key, lab in (('drift_pct', 'drift_pct (без округления)'), ('end_err_m', 'ошибка в конце, м')):
        b = np.array([base[n]['clean'][key] for n in ok])
        h = np.array([head[n]['clean'][key] for n in ok])
        print(row(lab, paired(b, h)))
        for day in sorted({days[n] for n in ok}):
            sel = [i for i, n in enumerate(ok) if days[n] == day]
            if len(sel) >= 3:
                print(row(f'{lab} · {day}', paired(b[sel], h[sel])))


def cmd_sensgrid(d):
    """One row per variant over the whole stress grid: against base and against h123."""
    res = load_dir(d)
    order = [v for v in ('base', 'h1_ema', 'h12', 'h3', 'h3_v0', 'h13', 'h123', 'h123_k1.5', 'h123_k2.5', 'h123_k3',
                         'h123_p0.5', 'h123_p1.5', 'h123_p2', 'h123_s1.5', 'h123_s3', 'h123_s5',
                         'h123_a200', 'h123_a500') if v in res]
    scen = [k for k in next(iter(res['base'].values())) if k != 'clean']
    summ = {(sc, v): cell(res[v], sc) for sc in scen for v in order}
    print(f'{d}: {len(scen)} ячеек, bag {len(res["base"])}\n')
    print('| вариант | не восст., сумма bag-ячеек | к base: не восст. лучше/хуже, ячеек | медиана добавки лучше/хуже | p90 лучше/хуже | max лучше/хуже | '
          'bag-ячейки хуже/лучше > 20 м | к h123: не восст. лучше/хуже | медиана лучше/хуже | max лучше/хуже | хуже/лучше > 20 м | '
          'ложные привязки / привязки после начала | восст. захвата (ложные) |')
    print('|---|---:|---|---|---|---|---|---|---|---|---|---|---|')
    for v in order:
        def cmp(ref):
            out = dict(nrec=[0, 0], med=[0, 0], p90=[0, 0], mx=[0, 0], w20=0, b20=0)
            for sc in scen:
                a, b = summ[(sc, ref)], summ[(sc, v)]
                for k, k2 in (('nrec', 'nrec'), ('med', 'exc_med'), ('p90', 'exc_p90'), ('mx', 'exc_max')):
                    out[k][0] += b[k2] < a[k2]
                    out[k][1] += b[k2] > a[k2]
                for n, ea in a['exc_by_bag'].items():
                    eb = b['exc_by_bag'].get(n)
                    if ea is not None and eb is not None:
                        out['w20'] += eb - ea > 20
                        out['b20'] += eb - ea < -20
            return out
        cb, ch = cmp('base'), cmp('h123')
        nrec = sum(summ[(sc, v)]['nrec'] for sc in scen)
        fs = sum(summ[(sc, v)]['false'] for sc in scen)
        sn = sum(summ[(sc, v)]['snaps'] for sc in scen)
        rl = sum(summ[(sc, v)]['relock'] for sc in scen)
        rlf = sum(summ[(sc, v)]['relock_false'] for sc in scen)
        print(f"| {v} | {nrec} | {cb['nrec'][0]}/{cb['nrec'][1]} | {cb['med'][0]}/{cb['med'][1]} | {cb['p90'][0]}/{cb['p90'][1]} | "
              f"{cb['mx'][0]}/{cb['mx'][1]} | {cb['w20']}/{cb['b20']} | {ch['nrec'][0]}/{ch['nrec'][1]} | {ch['med'][0]}/{ch['med'][1]} | "
              f"{ch['mx'][0]}/{ch['mx'][1]} | {ch['w20']}/{ch['b20']} | {fs}/{sn} ({100 * fs / sn:.1f} %) | {rl} ({rlf}) |")


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'pairs':
        cmd_pairs(sys.argv[2], sys.argv[3])
    elif cmd == 'days':
        cmd_days(sys.argv[2])
    elif cmd == 'grid':
        cmd_grid(*sys.argv[2:4])
    elif cmd == 'std':
        cmd_std(sys.argv[2])
    elif cmd == 'sensgrid':
        cmd_sensgrid(sys.argv[2])
    elif cmd == 'curve':
        cmd_curve(sys.argv[2])
    else:
        raise SystemExit(__doc__)
