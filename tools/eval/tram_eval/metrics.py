"""Metrics of docs/contracts.md §4 for one bag, and the summary over bags.

Matching is reference-driven: every GNSS sample takes the estimate with the nearest
header.stamp within MATCH_TOL_S (the judge's tolerance), so the score does not depend on
the publication rate. Units: m, m/s, %.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .reference import Reference

MATCH_TOL_S = 0.05        # judge: nearest stamp within ~0.05 s (README of the organizers)
MODE_SMOOTH_S = 0.5       # reference speed is averaged over +-0.5 s before differentiating
ACCEL_MPS2 = 0.2          # a > +0.2 -> accel, a < -0.2 -> brake
STOP_MPS = 0.3            # v < 0.3 -> stop (takes priority over accel/brake)
MIN_DRIFT_PATH_M = 50.0   # shorter runs have no drift_pct (docs/data.md, trap 13)
PROJ_WINDOW_M = 30.0      # projection searches +-(this + step) around the previous arc
PROJ_BACK_SLACK_M = 0.5   # going back along the arc by more than this costs its metres
PROJ_MAX_SPEED_MPS = 20.0  # the window also covers this speed times the time between matched samples
PROJ_EXTEND_M = 2000.0    # the track is extended along its end directions: an estimate ahead of
                          # the end (or behind the start) keeps its along error
PROJ_TANGENT_M = 5.0      # end direction = chord over the last (first) this many metres

MODES = ('accel', 'brake', 'stop', 'cruise')
MAIN_METRICS = ('speed_rmse', 'along_rmse', 'drift_pct')     # D-012
METRIC_KEYS = (
    'speed_rmse', 'speed_mae',
    'speed_bias_accel', 'speed_bias_brake', 'speed_bias_stop', 'speed_bias_cruise',
    'drift_pct', 'along_mean', 'along_max', 'along_rmse',
    'cross_mean', 'cross_max', 'cross_rmse', 'pos3d_rmse', 'pos3d_max', 'slip_flag_frac',
)
SIGNED_METRICS = ('speed_bias_accel', 'speed_bias_brake', 'speed_bias_stop', 'speed_bias_cruise')


@dataclass
class Estimates:
    t: np.ndarray           # (N,) s, Estimate.t (= stamp of the input that produced it)
    speed: np.ndarray       # (N,) m/s
    pos: np.ndarray         # (N, 3) m, map when absolute; otherwise local odom
    slip: np.ndarray        # (N,) bool, slip_front or slip_rear
    position_absolute: np.ndarray | None = None  # (N,) bool; None for legacy synthetic data (all absolute)

    def absolute_mask(self) -> np.ndarray:
        return np.ones(len(self.t), dtype=bool) if self.position_absolute is None else self.position_absolute


def match_nearest(ref_t: np.ndarray, est_t: np.ndarray, tol: float = MATCH_TOL_S):
    """Index pairs (ref, est): for each reference stamp the nearest estimate within tol."""
    if len(ref_t) == 0 or len(est_t) == 0:
        return np.zeros(0, int), np.zeros(0, int)
    order = np.argsort(est_t, kind='stable')
    st = est_t[order]
    j = np.clip(np.searchsorted(st, ref_t), 1, len(st) - 1) if len(st) > 1 else np.zeros(len(ref_t), int)
    left = np.maximum(j - 1, 0)
    pick = np.where(np.abs(st[left] - ref_t) <= np.abs(st[j] - ref_t), left, j)
    ok = np.abs(st[pick] - ref_t) <= tol
    return np.nonzero(ok)[0], order[pick[ok]]


def window_mean(t: np.ndarray, v: np.ndarray, half: float) -> np.ndarray:
    lo = np.searchsorted(t, t - half, side='left')
    hi = np.searchsorted(t, t + half, side='right')
    c = np.concatenate([[0.0], np.cumsum(v)])
    return (c[hi] - c[lo]) / (hi - lo)


def speed_modes(t: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Mode of every reference speed sample: accel / brake / stop / cruise."""
    mode = np.full(len(t), 'cruise', dtype=object)
    if len(t) < 2:
        mode[v < STOP_MPS] = 'stop'
        return mode
    vs = window_mean(t, v, MODE_SMOOTH_S)
    a = np.gradient(vs, t)
    mode[a > ACCEL_MPS2] = 'accel'
    mode[a < -ACCEL_MPS2] = 'brake'
    mode[v < STOP_MPS] = 'stop'
    return mode


def extend_track(poly: np.ndarray, poly_s: np.ndarray):
    """Polyline with straight extensions of PROJ_EXTEND_M at both ends; arc starts below 0."""
    if len(poly) < 2 or poly_s[-1] <= 0:
        return poly, poly_s
    head = poly[min(int(np.searchsorted(poly_s, PROJ_TANGENT_M)), len(poly) - 1)] - poly[0]
    tail = poly[-1] - poly[max(int(np.searchsorted(poly_s, poly_s[-1] - PROJ_TANGENT_M)) - 1, 0)]
    if np.hypot(*head) < 1e-6 or np.hypot(*tail) < 1e-6:
        return poly, poly_s
    head, tail = head / np.hypot(*head), tail / np.hypot(*tail)
    poly = np.vstack([poly[0] - PROJ_EXTEND_M * head, poly, poly[-1] + PROJ_EXTEND_M * tail])
    return poly, np.concatenate([[poly_s[0] - PROJ_EXTEND_M], poly_s, [poly_s[-1] + PROJ_EXTEND_M]])


def project_track(poly: np.ndarray, poly_s: np.ndarray, pts: np.ndarray, t: np.ndarray, s0: float):
    """Arc and distance of each point projected onto the polyline, point by point in time.

    A global nearest-point search jumps to the parallel track of the other direction and
    across the end loops. Here the search is a window around the previous arc, and a
    candidate behind it pays the metres it goes back: a tram never reverses (docs/data.md,
    trap 11), so following the other leg backwards costs more than the true leg. Ahead of
    the previous arc the nearest point wins: a penalty there locks the projection behind
    after a GNSS gap. step = the larger of the estimate's move and PROJ_MAX_SPEED_MPS * dt.
    """
    s_out, d_out = np.zeros(len(pts)), np.zeros(len(pts))
    if len(poly) == 1:
        d_out[:] = np.hypot(*(pts - poly[0]).T)
        return s_out, d_out
    seg = np.diff(poly, axis=0)
    seg_len2 = np.maximum((seg ** 2).sum(axis=1), 1e-12)
    seg_ds = np.diff(poly_s)          # arc of a segment: may differ from its length (Doppler arc)
    s_prev = s0
    for k in range(len(pts)):
        step = max(float(np.hypot(*(pts[k] - pts[k - 1]))), PROJ_MAX_SPEED_MPS * (t[k] - t[k - 1])) if k else 0.0
        w = PROJ_WINDOW_M + step
        i0 = max(int(np.searchsorted(poly_s, s_prev - w, side='left')) - 1, 0)
        i1 = max(min(int(np.searchsorted(poly_s, s_prev + w, side='right')), len(seg)), i0 + 1)
        a, d = poly[i0:i1], seg[i0:i1]
        u = np.clip(((pts[k] - a) * d).sum(axis=1) / seg_len2[i0:i1], 0.0, 1.0)
        dist = np.hypot(*(a + u[:, None] * d - pts[k]).T)
        s = poly_s[i0:i1] + u * seg_ds[i0:i1]
        cost = dist + np.maximum(s_prev - s - PROJ_BACK_SLACK_M, 0.0)
        j = int(np.argmin(cost))
        s_prev = s_out[k] = s[j]
        d_out[k] = dist[j]
    return s_out, d_out


def _rmse(e):
    return float(np.sqrt(np.mean(e ** 2)))


def bag_metrics(ref: Reference, est: Estimates) -> dict:
    """All per-bag metric fields of docs/contracts.md §4 (None where undefined)."""
    m: dict = {k: None for k in METRIC_KEYS}
    m['distance_m'] = float(ref.pos_s[-1] - ref.pos_s[0]) if len(ref.pos_s) else 0.0
    m['slip_flag_frac'] = float(np.mean(est.slip)) if len(est.slip) else None

    ri, ei = match_nearest(ref.vel_t, est.t)
    m['n_matched'] = int(len(ri))
    if len(ri):
        err = est.speed[ei] - ref.speed[ri]
        m['speed_rmse'], m['speed_mae'] = _rmse(err), float(np.mean(np.abs(err)))
        mode = speed_modes(ref.vel_t, ref.speed)[ri]
        for name in MODES:
            sel = mode == name
            if sel.any():
                m[f'speed_bias_{name}'] = float(np.mean(err[sel]))

    absolute = np.flatnonzero(est.absolute_mask())
    ri, local_ei = match_nearest(ref.pos_t, est.t[absolute])
    ei = absolute[local_ei]
    if len(ri):
        ref_xyz, est_xyz = ref.pos[ri], est.pos[ei]
        s_est, cross = project_track(*extend_track(ref.poly, ref.poly_s), est_xyz[:, :2], ref.pos_t[ri],
                                     ref.pos_s[ri[0]])
        along = np.abs(s_est - ref.pos_s[ri])
        m.update(along_mean=float(along.mean()), along_max=float(along.max()), along_rmse=_rmse(along),
                 cross_mean=float(cross.mean()), cross_max=float(cross.max()), cross_rmse=_rmse(cross))
        d3 = np.linalg.norm(est_xyz - ref_xyz, axis=1)
        m['pos3d_rmse'], m['pos3d_max'] = _rmse(d3), float(d3.max())
        path = float(ref.pos_s[ri[-1]] - ref.pos_s[ri[0]])
        if path >= MIN_DRIFT_PATH_M:
            m['drift_pct'] = float(np.hypot(*(est_xyz[-1, :2] - ref_xyz[-1, :2])) / path * 100.0)
    return m


def summarize(bags: dict[str, dict]) -> dict:
    """Median over bags and the worst bag of every metric (largest; |bias| for biases)."""
    median, worst = {}, {}
    for k in METRIC_KEYS:
        vals = {b: m[k] for b, m in bags.items() if m.get(k) is not None}
        if not vals:
            median[k] = None
            continue
        median[k] = float(np.median(list(vals.values())))
        key = (lambda b: abs(vals[b])) if k in SIGNED_METRICS else (lambda b: vals[b])
        worst[k] = max(vals, key=key)
    return {'median': median, 'worst_bag': worst}
