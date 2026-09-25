"""Identification of the drive model (issue #9): pure functions over per-bag samples.

Model identified here (D-029 schema, D-030):
    a = sign(n) * A(|n|, v) - (c0 + c1 v + c2 v^2) - g * grade,   n = notch(t - delay)
    A = traction table for n > 0 (capped by adhesion and P/(m v)), brake table for n < 0.
The grade term is removed from the training targets with GNSS altitude (offline only, train
bags); online the model sees it through the map height or as a filter disturbance.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'tools' / 'eval'))

from tram_eval import bag as evbag                     # noqa: E402
from tram_eval.reference import build_reference        # noqa: E402

G = 9.80665
KMH = 1.0 / 3.6            # the unit conversion of the data (D-003)
DT = 0.1                   # s, resampling step of the samples
DERIV_HALF_S = 0.2         # s, accel = centred difference over +-this
GRADE_HALF_M = 30.0        # m, grade = dz/ds over +-this along the reference arc
MOVING_MPS = 0.3           # below: standing, accel not driven by the notch
BOGIE_AGREE_MPS = 0.3      # |front - rear| above: slip or sensor fault, sample excluded
MAX_ABS_ACCEL = 2.5        # m/s^2, above: wheel glitch, sample excluded
GAP_S = 0.5                # s, wheel samples farther than this from both neighbours: gap
SCALE_MIN_GNSS_MPS = 3.0   # wheel/GNSS ratio only on motion (docs/data.md, trap 1)
SMOOTH_REL = 0.01          # curvature penalty of a speed curve relative to the data weight per node


@dataclass
class BagSamples:
    name: str
    t: np.ndarray            # (N,) s, 10 Hz grid in header.stamp time
    v: np.ndarray            # (N,) m/s, mean of the bogies after unit conversion (scale 1)
    a: np.ndarray            # (N,) m/s^2, centred difference of v
    dfr: np.ndarray          # (N,) m/s, front - rear
    grade: np.ndarray        # (N,) dz/ds, NaN without GNSS
    cmd_t: np.ndarray        # (M,) s, controller stamps
    cmd_n: np.ndarray        # (M,) int, controller notch
    ratio_front: float       # median wheel km/h / GNSS m/s on motion (NaN without GNSS)
    ratio_rear: float
    valid: np.ndarray        # (N,) bool, grid point has both bogies within GAP_S


def _series(msgs, topic, field):
    t = np.array([evbag.stamp(m) for tp, m in msgs if tp == topic], float)
    v = np.array([field(m) for tp, m in msgs if tp == topic], float)
    order = np.argsort(t, kind='stable')
    t, v = t[order], v[order]
    keep = np.concatenate([[True], np.diff(t) > 0]) if len(t) else np.zeros(0, bool)
    keep &= np.isfinite(v)
    return t[keep], v[keep]


def _near(grid, t):
    """True where a sample of t lies within GAP_S of the grid point."""
    if len(t) == 0:
        return np.zeros(len(grid), bool)
    j = np.clip(np.searchsorted(t, grid), 1, max(len(t) - 1, 1))
    return np.minimum(np.abs(t[j - 1] - grid), np.abs(t[np.minimum(j, len(t) - 1)] - grid)) <= GAP_S


def notch_at(cmd_t, cmd_n, tq):
    """Zero-order hold of the notch at times tq (0 before the first command)."""
    i = np.searchsorted(cmd_t, tq, side='right') - 1
    return np.where(i >= 0, cmd_n[np.clip(i, 0, None)], 0).astype(int)


def grade_along(pos_t, pos_s, z, tq, half_m=GRADE_HALF_M):
    """dz/ds at times tq from the reference track: z(s) sampled at 1 m, slope over +-half_m."""
    if len(pos_s) < 2 or pos_s[-1] - pos_s[0] < 2 * half_m:
        return np.full(len(tq), np.nan)
    grid = np.arange(pos_s[0], pos_s[-1], 1.0)
    zg = np.interp(grid, pos_s, z)
    s = np.interp(tq, pos_t, pos_s, left=np.nan, right=np.nan)
    g = (np.interp(s + half_m, grid, zg) - np.interp(s - half_m, grid, zg)) / (2 * half_m)
    g[~np.isfinite(s)] = np.nan
    return g


def extract(msgs, name='bag') -> BagSamples:
    """Samples of one bag (messages as tram_eval.bag.read_bag returns them)."""
    tf, vf = _series(msgs, evbag.FRONT, lambda m: m.velocity)
    tr, vr = _series(msgs, evbag.REAR, lambda m: m.velocity)
    tc, nc = _series(msgs, evbag.CMD, lambda m: m.position)
    empty = np.zeros(0)
    if len(tf) < 3 or len(tr) < 3:
        return BagSamples(name, empty, empty, empty, empty, empty, tc, nc.astype(int),
                          float('nan'), float('nan'), np.zeros(0, bool))
    t = np.arange(max(tf[0], tr[0]), min(tf[-1], tr[-1]), DT)
    f, r = np.interp(t, tf, vf) * KMH, np.interp(t, tr, vr) * KMH
    v = (f + r) / 2
    k = int(round(DERIV_HALF_S / DT))
    a = np.full(len(t), np.nan)
    if len(t) > 2 * k:
        a[k:-k] = (v[2 * k:] - v[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
    valid = _near(t, tf) & _near(t, tr)

    ref = build_reference(*evbag.reference_inputs(msgs), float('inf'))
    grade = grade_along(ref.pos_t, ref.pos_s, ref.pos[:, 2], t) if len(ref.pos) else np.full(len(t), np.nan)
    ratios = []
    for ts, vs in ((tf, vf), (tr, vr)):
        w = np.interp(ref.vel_t, ts, vs, left=np.nan, right=np.nan) if len(ref.vel_t) else empty
        ok = np.isfinite(w) & (ref.speed > SCALE_MIN_GNSS_MPS) if len(w) else np.zeros(0, bool)
        ratios.append(float(np.median(w[ok] / ref.speed[ok])) if ok.sum() > 50 else float('nan'))
    return BagSamples(name, t, v, a, f - r, grade, tc, nc.astype(int), ratios[0], ratios[1], valid)


def usable(s: BagSamples, need_grade=True) -> np.ndarray:
    """Samples fit for identification: moving, bogies agree, no gap, sane accel, grade known."""
    ok = s.valid & np.isfinite(s.a) & (s.v > MOVING_MPS) & (np.abs(s.dfr) < BOGIE_AGREE_MPS) \
        & (np.abs(s.a) < MAX_ABS_ACCEL)
    if need_grade:
        ok &= np.isfinite(s.grade)
    return ok


# --- fitting -------------------------------------------------------------------------------

def tent_basis(v, grid):
    """(N, K) hat functions of linear interpolation on grid, flat outside."""
    v = np.clip(np.asarray(v, float), grid[0], grid[-1])
    j = np.clip(np.searchsorted(grid, v, side='right') - 1, 0, len(grid) - 2)
    w = (v - grid[j]) / (grid[j + 1] - grid[j])
    B = np.zeros((len(v), len(grid)))
    B[np.arange(len(v)), j] = 1 - w
    B[np.arange(len(v)), j + 1] = w
    return B


def interp_table(table, grid, notch_abs, v):
    """Linear interpolation in v of row |notch| of a row-major (notch_max+1) x len(grid) table."""
    t = np.asarray(table, float).reshape(-1, len(grid))
    return np.interp(np.clip(v, grid[0], grid[-1]), grid, t[notch_abs])


def fit_curve(v, y, grid, smooth=1.0, min_count=30):
    """Piecewise-linear y(v) on grid by least squares with a second-difference penalty.

    Returns (values, counts); nodes with fewer than min_count samples in their hat support
    are NaN (filled later from neighbouring notches).
    """
    B = tent_basis(v, grid)
    K = len(grid)
    D = np.diff(np.eye(K), 2, axis=0) if K > 2 else np.zeros((0, K))
    lhs = B.T @ B + smooth * SMOOTH_REL * len(v) / max(K, 1) * (D.T @ D) + 1e-9 * np.eye(K)
    vals = np.linalg.solve(lhs, B.T @ y) if len(v) else np.full(K, np.nan)
    counts = (B > 0.25).sum(axis=0)
    vals = np.where(counts >= min_count, vals, np.nan)
    return vals, counts


def fill_monotone(table):
    """Fill NaN cells: along speed inside a row (edge value beyond the data), then rows without
    data along the notch axis; then force the modulus to be non-negative and non-decreasing in
    |notch| for every speed (more notch, more force)."""
    t = np.array(table, float)
    cols = np.arange(t.shape[1])
    for u in range(1, t.shape[0]):
        ok = np.isfinite(t[u])
        if ok.any() and not ok.all():
            t[u, ~ok] = np.interp(cols[~ok], cols[ok], t[u, ok])
    rows = np.arange(t.shape[0])
    for k in range(t.shape[1]):
        col = t[:, k]
        ok = np.isfinite(col)
        ok[0] = True
        col[0] = 0.0
        col[~ok] = np.interp(rows[~ok], rows[ok], col[ok])
        t[:, k] = np.maximum.accumulate(np.maximum(col, 0.0))
    return t


def fit_resistance(v, a_plus_grade):
    """c0 + c1 v + c2 v^2 = -(a + g*grade) on coasting samples; least squares with c >= 0
    (Davis form: resistance never helps motion). Best non-negative fit over term subsets."""
    v = np.asarray(v, float)
    y = -np.asarray(a_plus_grade, float)
    X = np.column_stack([np.ones_like(v), v, v * v])
    best = (np.inf, (0.0, 0.0, 0.0))
    for cols in ((0, 1, 2), (0, 1), (0, 2), (1, 2), (0,), (1,), (2,)):
        c_sub, *_ = np.linalg.lstsq(X[:, cols], y, rcond=None)
        if (c_sub < 0).any():
            continue
        c = np.zeros(3)
        c[list(cols)] = c_sub
        err = float(np.sum((X @ c - y) ** 2))
        if err < best[0]:
            best = (err, tuple(float(x) for x in c))
    return best[1]


def resistance(v, c):
    return c[0] + c[1] * v + c[2] * v * v


def drive_accel(notch, v, p):
    """Model acceleration of the D-029 schema without grade: sign(n) A(|n|, v) - resistance.

    p: dict with notch_max, speed_grid_mps, traction/brake tables, adhesion_accel_mps2,
    traction_power_w_per_kg, c (c0, c1, c2). Vectorised over notch and v.
    """
    grid = np.asarray(p['speed_grid_mps'], float)
    n = np.clip(np.asarray(notch, int), -p['notch_max'], p['notch_max'])
    v = np.maximum(np.asarray(v, float), 0.0)
    tr = np.asarray(p['traction_accel_table'], float).reshape(-1, len(grid))
    br = np.asarray(p['brake_accel_table'], float).reshape(-1, len(grid))
    vc = np.clip(v, grid[0], grid[-1])
    out = np.zeros(np.broadcast(n, v).shape)
    n_b, v_b, vc_b = np.broadcast_arrays(n, v, vc)
    for u in np.unique(n_b):
        s = n_b == u
        if u > 0:
            a = np.interp(vc_b[s], grid, tr[u])
            with np.errstate(divide='ignore'):
                cap = np.where(v_b[s] > 0, p['traction_power_w_per_kg'] / np.maximum(v_b[s], 1e-9), np.inf)
            out[s] = np.minimum(np.minimum(a, cap), p['adhesion_accel_mps2'])
        elif u < 0:
            out[s] = -np.minimum(np.interp(vc_b[s], grid, br[-u]), p['adhesion_accel_mps2'])
    return out - resistance(v, p['c'])


def simulate_windows(s: BagSamples, p, delay_s, window_s=10.0, step_s=5.0, use_grade=True):
    """Integrate the model from the measured speed over windows; returns per-window RMSE of the
    model and of 'a = 0' (constant speed). Windows with a gap, slip or no grade are skipped."""
    n_w = int(round(window_s / DT))
    stride = int(round(step_s / DT))
    notch = notch_at(s.cmd_t, s.cmd_n, s.t - delay_s)
    good = s.valid & (np.abs(s.dfr) < BOGIE_AGREE_MPS)
    if use_grade:
        good &= np.isfinite(s.grade)
    grade = np.nan_to_num(s.grade) if use_grade else np.zeros(len(s.t))
    starts = np.array([i0 for i0 in range(0, len(s.t) - n_w, stride)
                       if good[i0:i0 + n_w].all() and s.v[i0:i0 + n_w].max() >= MOVING_MPS], int)
    if len(starts) == 0:
        return np.zeros(0), np.zeros(0)
    idx = starts[:, None] + np.arange(n_w)[None, :]          # (W, n_w) sample indices
    sim = np.empty(idx.shape)
    sim[:, 0] = s.v[starts]
    for k in range(1, n_w):                                   # all windows advance together
        j = idx[:, k - 1]
        acc = drive_accel(notch[j], sim[:, k - 1], p) - G * grade[j]
        sim[:, k] = np.maximum(sim[:, k - 1] + acc * DT, 0.0)
    meas = s.v[idx]
    model_err = np.sqrt(np.mean((sim - meas) ** 2, axis=1))
    zero_err = np.sqrt(np.mean((meas - meas[:, :1]) ** 2, axis=1))
    return model_err, zero_err
