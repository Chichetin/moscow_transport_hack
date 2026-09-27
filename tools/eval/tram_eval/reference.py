"""GNSS master reference for one bag: ENU track with outliers removed, arc, speed.

The reference uses the whole GNSS record of the bag (it is the ground truth, not a model
input). The model sees GNSS only inside the init window — that cut is done in `bag.py`.

`point='base_link'` (the default of the eval, D-077) moves the reference to base_link by the
organizers' tf: the master antenna is 9.873 m behind base_link (the front bogie pivot) and the
rover 2.563 m ahead of it, both 3.0 m above it (rail level). With a rover fix of the same moment
base_link is on the line master -> rover; without one it is where master will be 9.873 m of arc
later (the tram is rigid and runs forward on its track). `point='master'` keeps the antenna.

`grid=(zone, e0, n0)` (what `bag.py` passes, from `frames.grid_*` of params.yaml) turns the
finished track into the flat MGRS grid of /result/position (D-083) with the core's own
conversion (`tram_odometry_core.position.geo`); without it the track stays in ENU of `origin`.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

CORE = Path(__file__).resolve().parents[3] / 'src' / 'tram_odometry_core'

# WGS84
A_WGS84 = 6378137.0
F_WGS84 = 1.0 / 298.257223563
E2_WGS84 = F_WGS84 * (2.0 - F_WGS84)

MIN_STATUS = 0            # NavSatFix: -1 NO_FIX is dropped; 0 FIX, 1 SBAS, 2 GBAS are usable
BEST_STATUS = 2           # if a bag has GBAS fixes, only they are the reference: switching between
                          # status 0 and 2 jumps the track by 1-9 m (30618_27e994fc: 1792 switches)
OUTLIER_WINDOW = 21       # samples (~2 s at 10 Hz) for the rolling median of the track
OUTLIER_DIST_M = 15.0     # farther than this from the rolling median -> outlier (docs/data.md, trap 10)
OUTLIER_HALF_S = 1.0      # s, half-width of the time window of the second median (#174)
MAX_SPEED_MPS = 30.0      # GNSS vel above this is a glitch (tram max ~16 m/s)
ARC_STEP_M = 0.5          # arc is accumulated over vertices >= this apart: GNSS jitter at stops adds no path
BASE_AHEAD_M = 9.873      # organizers' tf (27.09): master at x = -9.873 m in base_link, rover at +2.563
ANTENNA_HEIGHT_M = 3.0    # both antennas at z = 3.0 m in base_link; base_link is at rail level
ANTENNA_BASE_M = 12.436   # master -> rover in the tf (the data: median 12.44 m)
ROVER_BASE_TOL_M = 0.5    # a pair whose base is farther than this from ANTENNA_BASE_M is not the tf (outlier)
ROVER_MATCH_S = 0.06      # rover fix paired with the master fix of the nearest stamp within this
HEADING_HOLD_M = 1.0      # a fix without a pair takes the heading of the nearest pair this close in arc
END_CHORD_M = 5.0         # past the end the track goes on along its chord over the last this many metres
REF_POINTS = ('base_link', 'master')


def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    n = A_WGS84 / np.sqrt(1.0 - E2_WGS84 * np.sin(lat) ** 2)
    x = (n + alt_m) * np.cos(lat) * np.cos(lon)
    y = (n + alt_m) * np.cos(lat) * np.sin(lon)
    z = (n * (1.0 - E2_WGS84) + alt_m) * np.sin(lat)
    return np.stack([x, y, z], axis=-1)


def geodetic_to_enu(lat_deg, lon_deg, alt_m, origin):
    """ENU metres (east, north, up) of WGS84 points relative to origin (lat, lon, alt)."""
    lat0, lon0 = np.radians(origin[0]), np.radians(origin[1])
    d = geodetic_to_ecef(np.asarray(lat_deg, float), np.asarray(lon_deg, float),
                         np.asarray(alt_m, float)) - geodetic_to_ecef(*origin)
    sl, cl, so, co = np.sin(lat0), np.cos(lat0), np.sin(lon0), np.cos(lon0)
    e = -so * d[..., 0] + co * d[..., 1]
    n = -sl * co * d[..., 0] - sl * so * d[..., 1] + cl * d[..., 2]
    u = cl * co * d[..., 0] + cl * so * d[..., 1] + sl * d[..., 2]
    return np.stack([e, n, u], axis=-1)


def core_geo():
    """tram_odometry_core.position.geo: the same conversion as the published estimate."""
    if str(CORE) not in sys.path:
        sys.path.insert(0, str(CORE))
    from tram_odometry_core.position import geo
    return geo


def enu_to_grid(pos: np.ndarray, origin, grid) -> np.ndarray:
    """(N, 3) ENU of `origin` (lat, lon, alt) -> (N, 3) MGRS grid x, y and ellipsoidal height."""
    geo = core_geo()
    rot, ecef0 = geo.enu_rotation(*origin[:2]), geo.ecef(*origin)
    return np.array([geo.enu_to_grid(rot, ecef0, grid, *p) for p in np.asarray(pos, float)],
                    float).reshape(-1, 3)


def rolling_median(v: np.ndarray, window: int) -> np.ndarray:
    half = window // 2
    padded = np.pad(v, half, mode='reflect')   # 'edge' would let an outlier at the end outvote its neighbours
    return np.median(np.lib.stride_tricks.sliding_window_view(padded, window), axis=-1)


def outlier_mask(xy: np.ndarray, t: np.ndarray | None = None) -> np.ndarray:
    """True for points far from the rolling median of the track (single-point jumps of km).

    With stamps `t` (sorted), also far from the median of the points within +-OUTLIER_HALF_S:
    next to a gap of the kept status the window of neighbouring indices reaches points a
    minute away and lets a short jump through (#174, 30639_4285f2bc)."""
    if len(xy) < 3:
        return np.zeros(len(xy), bool)
    w = min(OUTLIER_WINDOW, len(xy) if len(xy) % 2 else len(xy) - 1)
    med = np.stack([rolling_median(xy[:, 0], w), rolling_median(xy[:, 1], w)], axis=-1)
    far = np.hypot(*(xy - med).T) > OUTLIER_DIST_M
    if t is None:
        return far
    lo = np.searchsorted(t, t - OUTLIER_HALF_S, side='left')
    hi = np.searchsorted(t, t + OUTLIER_HALF_S, side='right')
    for i in range(len(t)):
        if hi[i] - lo[i] >= 3:
            m = np.median(xy[lo[i]:hi[i]], axis=0)
            far[i] |= bool(np.hypot(*(xy[i] - m)) > OUTLIER_DIST_M)
    return far


def sort_unique(t: np.ndarray) -> np.ndarray:
    """Indices that sort t and keep the first of equal stamps."""
    order = np.argsort(t, kind='stable')
    keep = np.ones(len(order), bool)
    keep[1:] = np.diff(t[order]) > 0
    return order[keep]


def arc_polyline(xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Decimated polyline (vertices >= ARC_STEP_M apart), its arc, and arc of every point."""
    keep = [0]
    for i in range(1, len(xy)):
        if np.hypot(*(xy[i] - xy[keep[-1]])) >= ARC_STEP_M:
            keep.append(i)
    if keep[-1] != len(xy) - 1 and len(xy) > 1:
        keep.append(len(xy) - 1)      # the track ends where the run ends
    poly = xy[keep]
    seg = np.hypot(*np.diff(poly, axis=0).T)
    poly_s = np.concatenate([[0.0], np.cumsum(seg)])
    last = np.searchsorted(np.asarray(keep), np.arange(len(xy)), side='right') - 1
    point_s = poly_s[last] + np.hypot(*(xy - poly[last]).T)
    return poly, poly_s, point_s


def clean_fixes(fix_t: np.ndarray, fix_llas: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Fixes sorted by stamp (first of equal stamps) without the km jumps of the track."""
    idx = sort_unique(fix_t)
    fix_t, fix_llas = fix_t[idx], fix_llas[idx]
    if len(fix_t):
        provisional = tuple(np.median(fix_llas[:, :3], axis=0))
        enu = geodetic_to_enu(fix_llas[:, 0], fix_llas[:, 1], fix_llas[:, 2], provisional)
        good = ~outlier_mask(enu[:, :2], fix_t)
        fix_t, fix_llas = fix_t[good], fix_llas[good]
    return fix_t, fix_llas


def rover_pairs(fix_t, pos, rover_t, rover_llas, origin, min_status: int = MIN_STATUS):
    """For every master fix: the vector master -> rover (ENU, m) of the rover fix with the
    nearest stamp, and whether it is a pair of the tf (stamps within ROVER_MATCH_S, base within
    ROVER_BASE_TOL_M of ANTENNA_BASE_M). Rover fixes below `min_status` are not paired: the
    same status rule as the master track (a plain fix next to GBAS ones jumps by metres)."""
    d = np.zeros((len(fix_t), 3))
    good = np.zeros(len(fix_t), bool)
    rover_t, rover_llas = np.asarray(rover_t, float), np.asarray(rover_llas, float).reshape(-1, 4)
    ok = np.isfinite(rover_t) & np.isfinite(rover_llas).all(axis=1) & (rover_llas[:, 3] >= min_status)
    rover_t, rover_llas = rover_t[ok], rover_llas[ok]
    if not len(rover_t) or not len(fix_t):
        return d, good
    idx = sort_unique(rover_t)
    rover_t, rover_llas = rover_t[idx], rover_llas[idx]
    rover = geodetic_to_enu(rover_llas[:, 0], rover_llas[:, 1], rover_llas[:, 2], origin)
    j = np.clip(np.searchsorted(rover_t, fix_t), 0, len(rover_t) - 1)
    prev = np.maximum(j - 1, 0)
    j = np.where(np.abs(rover_t[prev] - fix_t) < np.abs(rover_t[j] - fix_t), prev, j)
    d = rover[j] - pos
    base = np.hypot(*d[:, :2].T)
    good = (np.abs(rover_t[j] - fix_t) <= ROVER_MATCH_S) & (np.abs(base - ANTENNA_BASE_M) <= ROVER_BASE_TOL_M)
    return d, good


def track_ahead(pos: np.ndarray, pos_s: np.ndarray, ahead: float, heading=None) -> np.ndarray:
    """Points `ahead` metres of arc further along the track `pos` (N, 3) with arc `pos_s`: where
    base_link is while master is at `pos` (D-077). Past the end the track goes on along its
    chord over the last END_CHORD_M; a track shorter than that goes along `heading` (unit ENU
    xy master -> rover) or, without one, stays where it ends."""
    if not len(pos):
        return pos.copy()
    keep = np.concatenate([[True], pos_s[1:] > np.maximum.accumulate(pos_s)[:-1]])
    s, p = pos_s[keep], pos[keep]
    target = pos_s + ahead
    out = np.column_stack([np.interp(target, s, p[:, c]) for c in range(3)])
    past = target > s[-1]
    if past.any():
        j = max(int(np.searchsorted(s, s[-1] - END_CHORD_M, side='right')) - 1, 0)
        chord = p[-1, :2] - p[j, :2]
        n = float(np.hypot(*chord))
        if s[-1] - s[0] >= END_CHORD_M and n > 0:
            u = chord / n
        elif heading is not None:
            u = np.asarray(heading, float)
        else:
            u = np.zeros(2)
        out[past, :2] = p[-1, :2] + (target[past] - s[-1])[:, None] * u
    return out


@dataclass
class Reference:
    origin: tuple[float, float, float] | None   # lat, lon, alt of the ENU the track is built in
    pos_t: np.ndarray       # (N,) s, header.stamp of master fix, sorted
    pos: np.ndarray         # (N, 3) m, MGRS grid with `grid` (D-083), else ENU of `origin`
    pos_s: np.ndarray       # (N,) m, arc of each point along the reference track, non-decreasing
    poly: np.ndarray        # (M, 2) m, track for projection (the fixes; decimated without vel)
    poly_s: np.ndarray      # (M,) m, arc of the vertices, non-decreasing
    vel_t: np.ndarray       # (K,) s, header.stamp of master vel, sorted
    speed: np.ndarray       # (K,) m/s, hypot(ve, vn)


def build_reference(fix_t, fix_llas, vel_t, vel_en, window_end: float, point: str = 'master',
                    rover_t=(), rover_llas=(), grid=None) -> Reference:
    """fix_llas: (N, 4) lat, lon, alt, status; vel_en: (K, 2) ENU east/north m/s.

    point: 'master' — the antenna; 'base_link' — the front bogie pivot at rail level by the
    organizers' tf (module docstring, D-077); rover_llas: (R, 4) like fix_llas, for base_link.
    The frame origin is the master fix either way.

    Origin = the rule of frame `map` of the tracker (docs/contracts.md §1, D-030): the first
    status-2 fix with stamp <= window_end, else the first valid fix of the window whatever its
    status, even when the track is status 2 only and they come later (#70); a run without a
    fix in the window starts at the first fix of the track.
    """
    fix_t, fix_llas = np.asarray(fix_t, float), np.asarray(fix_llas, float).reshape(-1, 4)
    ok = np.isfinite(fix_llas).all(axis=1) & (fix_llas[:, 3] >= MIN_STATUS) \
        & (np.abs(fix_llas[:, 0]) + np.abs(fix_llas[:, 1]) > 0)
    any_t, any_llas = clean_fixes(fix_t[ok], fix_llas[ok])
    if (fix_llas[ok, 3] >= BEST_STATUS).any():
        ok &= fix_llas[:, 3] >= BEST_STATUS
    fix_t, fix_llas = clean_fixes(fix_t[ok], fix_llas[ok])

    origin = None
    pos = np.zeros((0, 3))
    if len(fix_t):
        if (fix_t <= window_end).any():
            o = fix_llas[np.argmax(fix_t <= window_end)]
        elif (any_t <= window_end).any():
            o = any_llas[np.argmax(any_t <= window_end)]
        else:
            o = fix_llas[0]
        origin = tuple(float(v) for v in o[:3])
        pos = geodetic_to_enu(fix_llas[:, 0], fix_llas[:, 1], fix_llas[:, 2], origin)
    vel_t, vel_en = np.asarray(vel_t, float), np.asarray(vel_en, float).reshape(-1, 2)
    speed = np.hypot(vel_en[:, 0], vel_en[:, 1])
    ok = np.isfinite(vel_t) & np.isfinite(speed) & (speed < MAX_SPEED_MPS)
    vel_t, speed = vel_t[ok], speed[ok]
    idx = sort_unique(vel_t)
    vel_t, speed = vel_t[idx], speed[idx]

    if len(pos) and len(vel_t) >= 2:
        # Arc = integral of the Doppler speed: exactly 0 while standing, while the length of
        # the fix track grows by the 0.5-1 m wander of the fixes at a crawl (30618_27e994fc).
        vel_s = np.concatenate([[0.0], np.cumsum((speed[1:] + speed[:-1]) / 2 * np.diff(vel_t))])
        pos_s = np.interp(fix_t, vel_t, vel_s)
        poly, poly_s = pos[:, :2], pos_s
    elif len(pos):
        poly, poly_s, pos_s = arc_polyline(pos[:, :2])
    else:
        poly, poly_s, pos_s = np.zeros((0, 2)), np.zeros(0), np.zeros(0)
    if point not in REF_POINTS:
        raise ValueError(f'point must be one of {REF_POINTS}, not {point!r}')
    if point == 'base_link' and len(pos):
        # a rover pair of the tf gives base_link of the rigid body exactly (on the line master ->
        # rover, BASE_AHEAD_M from master); without one base_link is taken 9.873 m of arc ahead
        # on the master track, which cuts the curve at the end loops by up to 4.6 m (holdout)
        gbas = bool(len(fix_llas)) and bool((fix_llas[:, 3] >= BEST_STATUS).all())
        d, good = rover_pairs(fix_t, pos, rover_t, rover_llas, origin,
                              BEST_STATUS if gbas else MIN_STATUS)
        heading = None
        if good.any():
            u = np.median(d[good, :2] / np.hypot(*d[good, :2].T)[:, None], axis=0)
            heading = u / np.hypot(*u) if np.hypot(*u) > 0 else None
        ahead = track_ahead(pos, pos_s, BASE_AHEAD_M, heading)
        if good.any():
            # a fix without a pair: the direction of the nearest pair while the tram has moved
            # less than HEADING_HOLD_M since (a missing rover fix at the end of a run)
            idx = np.flatnonzero(good)
            j = np.clip(np.searchsorted(idx, np.arange(len(pos))), 0, len(idx) - 1)
            prev = idx[np.maximum(j - 1, 0)]
            near = np.where(np.abs(pos_s[prev] - pos_s) < np.abs(pos_s[idx[j]] - pos_s), prev, idx[j])
            use = np.abs(pos_s[near] - pos_s) <= HEADING_HOLD_M
            u = d[near] / np.hypot(*d[near, :2].T)[:, None]
            ahead[use] = pos[use] + BASE_AHEAD_M * u[use]
        pos = ahead
        pos[:, 2] -= ANTENNA_HEIGHT_M
        if len(vel_t) >= 2:
            poly = pos[:, :2]
        else:
            poly, poly_s, pos_s = arc_polyline(pos[:, :2])
    if grid is not None and len(pos):
        # the output frame (D-083); the arc stays the Doppler one or is taken in the grid
        # (scale 1 - 3e-4: along compares arcs on the same polyline either way)
        pos = enu_to_grid(pos, origin, grid)
        if len(vel_t) >= 2:
            poly = pos[:, :2]
        else:
            poly, poly_s, pos_s = arc_polyline(pos[:, :2])
    return Reference(origin, fix_t, pos, pos_s, poly, poly_s, vel_t, speed)
