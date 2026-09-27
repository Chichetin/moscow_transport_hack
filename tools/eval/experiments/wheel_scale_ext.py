"""Experiment #154, extension: how robust the online wheel scale 'h123' (rejected, D-088) and the
relock alone 'h3' (the frozen candidate after it) are against 'base'.

Runs (numbers only; the statistics are in wheel_scale_stats.py):

  grid    stress grid. A wheel scale error of both bogies c in SCALES from a start in STARTS
          ('g' = end of the GNSS window + 1 s, as scale_up/scale_down of stress.py: "another
          tram" for the whole run; or a fraction of the bag) to the 3 s tail; both bogies silent
          30 / 60 s from 0.25 / 0.4 / 0.6 of the bag (gap_both_30 of stress.py is 30 s at 0.4).
          Per bag and variant: the clean run and every scenario, with the position recovery
          against the clean run as run_eval --stress (2 m, 2 s), the metrics of the dirty run
          and the stops with the truth from the GNSS master track (eval only).
  std     clean + scale_up / scale_down / gap_both_30 of stress.py: sensitivity around h123.
  verify  dumps Estimates of the core (mode 'core') or of a variant of wheel_scale_exp, to
          check that 'base' / 'h123' / 'h3' are the commits bit for bit (compare with 'cmp');
          a bag outside train only with 'core' or a frozen candidate.

The variants are those of wheel_scale_exp.py (PathTracker replaced in memory) plus EXTRA.
Holdout takes only 'base', 'h123' and 'h3', the frozen candidates (D-011); 'std' is train only.

    .venv/bin/python tools/eval/experiments/wheel_scale_ext.py grid train base h1_ema h12 h123
    .venv/bin/python tools/eval/experiments/wheel_scale_ext.py grid holdout base h123 h3
    .venv/bin/python tools/eval/experiments/wheel_scale_ext.py std train h123 h123_k1.5 ...
    .venv/bin/python tools/eval/experiments/wheel_scale_ext.py verify <dir> core <bag> ...
    .venv/bin/python tools/eval/experiments/wheel_scale_ext.py cmp <dir1> <dir2>

Run from the repository root. WSX_EVAL = tools/eval to import (default: the parent of this
file), WSX_OUT = output directory (default <out>/wheel_scale_ext), WSX_JOBS = processes
(default 20), WSX_BAGS = comma-separated bags of the split to run (default: all; a quick check).
"""
from pathlib import Path
import copy, json, math, os, re, sys, time
from concurrent.futures import ProcessPoolExecutor
import numpy as np

EVAL = Path(os.environ.get('WSX_EVAL') or Path(__file__).resolve().parents[1])
sys.path.insert(0, str(EVAL))
sys.path.insert(0, str(EVAL / 'experiments'))
from tram_eval import bag, stress                              # noqa: E402
from tram_eval.bag import FRONT, REAR, stamp, gnss_window_end  # noqa: E402
from tram_eval.metrics import bag_metrics, match_nearest       # noqa: E402

SCALES = (0.995, 0.99, 0.985, 0.98, 0.975, 0.97, 1.005, 1.01, 1.015, 1.02, 1.025, 1.03)
STARTS = ('g', 0.25, 0.4, 0.6)
GAPS = (30.0, 60.0)
GAP_STARTS = (0.25, 0.4, 0.6)
TAIL_S = 3.0
STD = ('scale_up', 'scale_down', 'gap_both_30')
HOLDOUT_OK = ('base', 'h123', 'h3')   # frozen candidates only: holdout is not for tuning
KEYS = ('speed_rmse', 'along_rmse', 'drift_pct', 'along_max', 'along_mean', 'pos3d_rmse',
        'pos3d_max', 'cross_rmse')
CURVE = tuple(round(f, 2) for f in np.arange(0.30, 1.0001, 0.05))   # fractions of the run


def _variants():
    import wheel_scale_exp as w
    h12 = w.H12
    h123 = w.VARIANTS['h123']
    extra = {
        # H3 without H2 on the EMA with the anchor (H1); on main's EMA it is 'h3' of wheel_scale_exp
        'h13': dict(mode='ema', keep_ref=True, recover=True, rec_scale='kf'),
        # one parameter at a time around h123: relock_sigma, scale prior, sigma of a place as
        # a scale reference, minimal arc from the reference
        'h123_k1.5': dict(h123, rec_k=1.5), 'h123_k2.5': dict(h123, rec_k=2.5),
        'h123_k3': dict(h123, rec_k=3.0),
        'h123_p0.5': dict(h123, prior=0.005), 'h123_p1.5': dict(h123, prior=0.015),
        'h123_p2': dict(h123, prior=0.02),
        'h123_s1.5': dict(h123, meas_std=1.5), 'h123_s3': dict(h123, meas_std=3.0),
        'h123_s5': dict(h123, meas_std=5.0),
        'h123_a200': dict(h123, min_arc=200.0), 'h123_a500': dict(h123, min_arc=500.0),
    }
    assert h12['min_arc'] == 300.0 and 'rec_k' not in h123 and 'prior' not in h123
    return {**w.VARIANTS, **extra}


def grid_names():
    return ([f'sc{c:g}@{s}' for c in SCALES for s in STARTS]
            + [f'gap{g:g}@{s:g}' for g in GAPS for s in GAP_STARTS])


def perturb_ext(msgs, name, gnss_window_s):
    """(messages, start, end) of a grid scenario, or None; the logic of stress.perturb for
    scale_* and gap_both_*, with the start and the size as parameters."""
    kind, at = name.split('@')
    wheel_t = [stamp(m) for topic, m in msgs if topic in (FRONT, REAR)]
    if not wheel_t:
        return None
    first, last = min(wheel_t), max(wheel_t)
    begin = max(first, gnss_window_end(msgs, gnss_window_s)) + 1.0
    if kind.startswith('sc'):
        factor = float(kind[2:])
        start = begin if at == 'g' else max(begin, first + (last - first) * float(at))
        end = last - TAIL_S
        if end <= start:
            return None
    else:
        factor, duration = None, float(kind[3:])
        if last - begin < duration + TAIL_S:
            return None
        start = max(begin, min(first + (last - first) * float(at), last - duration - TAIL_S))
        end = start + duration
    out, changed = [], 0
    for topic, msg in msgs:
        if topic in (FRONT, REAR) and start <= stamp(msg) < end:
            changed += 1 if factor is None else 0
            if factor is None:
                continue
            new = copy.copy(msg)          # only velocity changes: a shallow copy is enough
            new.velocity = msg.velocity * factor
            changed += new.velocity != msg.velocity
            out.append((topic, new))
            continue
        out.append((topic, msg))
    return (out, start, end) if changed else None


def make(cfg):
    if cfg is None:
        return bag.default_odometry()
    import wheel_scale_exp as w
    return w.make(cfg)


def stops(tracker, ref_m):
    import wheel_scale_exp as w
    ev = []
    for e in w.truth(tracker, ref_m):
        if e['kind'] in ('snap', 'recover', 'miss'):
            ev.append([round(e['t'], 2), e['kind'], round(e['place'], 2), round(e['y'], 2),
                       round(e['s_true'], 2) if 's_true' in e else None, round(e['scale'], 5)])
    return ev


def stop_counts(ev, t0=-math.inf):
    """snaps (incl. relocks), known, false, relocks, false relocks at t >= t0."""
    import wheel_scale_exp as w
    sn = [e for e in ev if e[1] in ('snap', 'recover') and e[0] >= t0]
    known = [e for e in sn if e[4] is not None]
    false = [e for e in known if abs(e[2] - e[4]) > w.FALSE_M]
    rec = [e for e in sn if e[1] == 'recover']
    rec_false = [e for e in rec if e[4] is not None and abs(e[2] - e[4]) > w.FALSE_M]
    return len(sn), len(known), len(false), len(rec), len(rec_false)


def record(ref, est, crash, odo, ref_m, curve=False):
    m = bag_metrics(ref, est)
    r = {k: (float(m[k]) if isinstance(m.get(k), float) and math.isfinite(m[k]) else None) for k in KEYS}
    r['crashed'] = crash is not None
    ri, ei = match_nearest(ref.pos_t, est.t)
    if len(ri):
        eh = np.hypot(*(est.pos[ei, :2] - ref.pos[ri, :2]).T)
        path = ref.pos_s[ri] - ref.pos_s[ri[0]]
        r['end_err_m'], r['path_m'] = float(eh[-1]), float(path[-1])
        if curve:
            tt = ref.pos_t[ri]
            idx = np.searchsorted(tt, tt[0] + np.array(CURVE) * (tt[-1] - tt[0]), side='right') - 1
            r['curve'] = [[round(float(eh[i]), 4), round(float(path[i]), 2)] for i in np.maximum(idx, 0)]
    tr = odo._tracker
    if tr is not None and hasattr(tr, 'log'):
        r['scale_end'] = tr._scale
        r['stops'] = stops(tr, ref_m)
    return r


def scenarios(mode, msgs, w):
    names = grid_names() if mode == 'grid' else STD
    for sc in names:
        ev = perturb_ext(msgs, sc, w) if mode == 'grid' else stress.perturb(msgs, sc, w)
        yield sc, ev


def task(args):
    mode, name, vname, cfg, path = args
    t0 = time.time()
    msgs = bag.read_bag(bag.data_dir() / name)
    w = bag.default_gnss_window()
    end = gnss_window_end(msgs, w)
    ref = bag.bag_reference(msgs, end)
    ref_m = bag.bag_reference(msgs, end, 'master')
    odo = make(cfg)
    clean, ccrash, _ = bag.run_pipeline(msgs, odo, end)
    clean, _ = bag.finite_only(clean)
    out = {'clean': record(ref, clean, ccrash, odo, ref_m, curve=True)}
    for sc, ev in scenarios(mode, msgs, w):
        if ev is None:
            out[sc] = {'skipped': True}
            continue
        dmsgs, start, stop = ev
        odo = make(cfg)
        est, crash, _ = bag.run_pipeline(dmsgs, odo, end)
        est, _ = bag.finite_only(est)
        r = record(ref, est, crash, odo, ref_m)
        pk, cpk, exc, rec, *_ = stress._errors(ref.pos_t, ref.pos, clean, est, 'pos', start, stop, 2.0)
        spk, scpk, sexc, srec, *_ = stress._errors(ref.vel_t, ref.speed, clean, est, 'speed', start, stop, 0.2)
        ok = ccrash is None and crash is None
        r.update(skipped=False, start=round(start, 3), end=round(stop, 3), peak_pos=pk, clean_peak_pos=cpk,
                 excess_pos=exc, rec_pos=rec if ok else None, peak_speed=spk, excess_speed=sexc,
                 rec_speed=srec if ok else None)
        out[sc] = r
        del dmsgs, ev
    Path(path).write_text(json.dumps({'bag': name, 'variant': vname, 'mode': mode, 'cfg': cfg,
                                      'seconds': round(time.time() - t0, 1), 'res': out}))
    return name, vname, time.time() - t0


def out_root():
    return Path(os.environ.get('WSX_OUT') or bag.out_dir() / 'wheel_scale_ext')


def run(mode, split, variants):
    allv = _variants()
    if split != 'train' and (mode != 'grid' or not set(variants) <= set(HOLDOUT_OK)):
        raise SystemExit(f'{split}: only grid with {HOLDOUT_OK} (D-011)')
    unknown = [v for v in variants if v not in allv]
    if unknown:
        raise SystemExit(f'unknown variants {unknown}; known: {sorted(allv)}')
    names = bag.load_splits()[split]
    only = os.environ.get('WSX_BAGS')
    if only:
        names = [n for n in names if n in only.split(',')]
    size = {n: sum(f.stat().st_size for f in (bag.data_dir() / n).glob('*')) for n in names}
    d = out_root() / f'{mode}-{split}'
    d.mkdir(parents=True, exist_ok=True)
    jobs = []
    for n in sorted(names, key=lambda n: -size[n]):
        for v in variants:
            p = d / f'{n}__{v}.json'
            if not p.exists():
                jobs.append((mode, n, v, allv[v], str(p)))
    print(f'{mode} {split}: {len(jobs)} tasks -> {d}', flush=True)
    t0 = time.time()
    with ProcessPoolExecutor(int(os.environ.get('WSX_JOBS', 20))) as pool:
        for i, (n, v, s) in enumerate(pool.map(task, jobs), 1):
            print(f'{i}/{len(jobs)} {n} {v} {s:.0f} s ({time.time() - t0:.0f} s)', flush=True)
    print(f'done {time.time() - t0:.0f} s', flush=True)


def verify(dump, vname, bags):
    """Estimates of clean + STD on each bag: mode 'core' = default_odometry of this tree.
    A bag outside train only with 'core' or a frozen candidate (D-011)."""
    train = set(bag.load_splits()['train'])
    if vname not in ('core',) + HOLDOUT_OK and not set(bags) <= train:
        raise SystemExit(f'{sorted(set(bags) - train)}: not train, only core or {HOLDOUT_OK} (D-011)')
    cfg = None if vname == 'core' else _variants()[vname]
    d = Path(dump)
    d.mkdir(parents=True, exist_ok=True)
    w = bag.default_gnss_window()
    for name in bags:
        msgs = bag.read_bag(bag.data_dir() / name)
        end = gnss_window_end(msgs, w)
        runs = [('clean', msgs)] + [(sc, (stress.perturb(msgs, sc, w) or (None,))[0]) for sc in STD]
        for sc, m in runs:
            if m is None:
                continue
            est, crash, _ = bag.run_pipeline(m, make(cfg), end)
            np.savez(d / f'{name}__{sc}.npz', t=est.t, speed=est.speed, pos=est.pos, crash=crash is not None)
        # the grid's copy of perturb gives the same messages as stress.perturb for STD
        for sc, g in (('scale_up', 'sc1.015@g'), ('scale_down', 'sc0.985@g'), ('gap_both_30', 'gap30@0.4')):
            a, b = stress.perturb(msgs, sc, w), perturb_ext(msgs, g, w)
            same = (a is None) == (b is None) and (a is None or (
                a[1:] == b[1:] and len(a[0]) == len(b[0]) and all(
                    ta == tb and stamp(ma) == stamp(mb) and getattr(ma, 'velocity', None) == getattr(mb, 'velocity', None)
                    for (ta, ma), (tb, mb) in zip(a[0], b[0]))))
            print(f'{name} perturb {sc} == {g}: {same}', flush=True)


def cmp(d1, d2):
    bad = 0
    files = sorted(Path(d1).glob('*.npz'))
    for f in files:
        a, b = np.load(f), np.load(Path(d2) / f.name)
        same = all(np.array_equal(a[k], b[k], equal_nan=True) for k in ('t', 'speed', 'pos', 'crash'))
        dmax = float(np.max(np.abs(a['pos'] - b['pos']))) if a['pos'].shape == b['pos'].shape else math.inf
        bad += not same
        print(f"{f.stem}: {'same bit for bit' if same else 'DIFFERENT'} n={len(a['t'])}/{len(b['t'])} max|dpos|={dmax:.3g}")
    print(f'{len(files) - bad}/{len(files)} identical')


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd in ('grid', 'std'):
        run(cmd, sys.argv[2], sys.argv[3:])
    elif cmd == 'verify':
        verify(sys.argv[2], sys.argv[3], sys.argv[4:])
    elif cmd == 'cmp':
        cmp(sys.argv[2], sys.argv[3])
    else:
        raise SystemExit(__doc__)
