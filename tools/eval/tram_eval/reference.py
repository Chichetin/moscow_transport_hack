"""GNSS master reference for one bag: ENU track with outliers removed, arc, speed.

The reference uses the whole GNSS record of the bag (it is the ground truth, not a model
input). The model sees GNSS only inside the init window — that cut is done in `bag.py`.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# WGS84
A_WGS84 = 6378137.0
F_WGS84 = 1.0 / 298.257223563
E2_WGS84 = F_WGS84 * (2.0 - F_WGS84)

MIN_STATUS = 0            # NavSatFix: -1 NO_FIX is dropped; 0 FIX, 1 SBAS, 2 GBAS are usable
BEST_STATUS = 2           # if a bag has GBAS fixes, only they are the reference: switching between
                          # status 0 and 2 jumps the track by 1-9 m (30618_27e994fc: 1792 switches)
OUTLIER_WINDOW = 21       # samples (~2 s at 10 Hz) for the rolling median of the track
OUTLIER_DIST_M = 15.0     # farther than this from the rolling median -> outlier (docs/data.md, trap 10)
MAX_SPEED_MPS = 30.0      # GNSS vel above this is a glitch (tram max ~16 m/s)
ARC_STEP_M = 0.5          # arc is accumulated over vertices >= this apart: GNSS jitter at stops adds no path


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


def rolling_median(v: np.ndarray, window: int) -> np.ndarray:
    half = window // 2
    padded = np.pad(v, half, mode='reflect')   # 'edge' would let an outlier at the end outvote its neighbours
    return np.median(np.lib.stride_tricks.sliding_window_view(padded, window), axis=-1)


def outlier_mask(xy: np.ndarray) -> np.ndarray:
    """True for points far from the rolling median of the track (single-point jumps of km)."""
    if len(xy) < 3:
        return np.zeros(len(xy), bool)
    w = min(OUTLIER_WINDOW, len(xy) if len(xy) % 2 else len(xy) - 1)
    med = np.stack([rolling_median(xy[:, 0], w), rolling_median(xy[:, 1], w)], axis=-1)
    return np.hypot(*(xy - med).T) > OUTLIER_DIST_M


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


@dataclass
class Reference:
    origin: tuple[float, float, float] | None   # lat, lon, alt of the frame `map` origin
    pos_t: np.ndarray       # (N,) s, header.stamp of master fix, sorted
    pos: np.ndarray         # (N, 3) m, ENU
    pos_s: np.ndarray       # (N,) m, arc of each point along the reference track, non-decreasing
    poly: np.ndarray        # (M, 2) m, track for projection (the fixes; decimated without vel)
    poly_s: np.ndarray      # (M,) m, arc of the vertices, non-decreasing
    vel_t: np.ndarray       # (K,) s, header.stamp of master vel, sorted
    speed: np.ndarray       # (K,) m/s, hypot(ve, vn)


def build_reference(fix_t, fix_llas, vel_t, vel_en, window_end: float) -> Reference:
    """fix_llas: (N, 4) lat, lon, alt, status; vel_en: (K, 2) ENU east/north m/s.

    Origin = first filtered fix with stamp <= window_end (else the first filtered fix): the
    same rule as frame `map` in docs/contracts.md §1.
    """
    fix_t, fix_llas = np.asarray(fix_t, float), np.asarray(fix_llas, float).reshape(-1, 4)
    ok = np.isfinite(fix_llas).all(axis=1) & (fix_llas[:, 3] >= MIN_STATUS) \
        & (np.abs(fix_llas[:, 0]) + np.abs(fix_llas[:, 1]) > 0)
    if (fix_llas[ok, 3] >= BEST_STATUS).any():
        ok &= fix_llas[:, 3] >= BEST_STATUS
    fix_t, fix_llas = fix_t[ok], fix_llas[ok]
    idx = sort_unique(fix_t)
    fix_t, fix_llas = fix_t[idx], fix_llas[idx]

    origin = None
    pos = np.zeros((0, 3))
    if len(fix_t):
        provisional = tuple(np.median(fix_llas[:, :3], axis=0))
        enu = geodetic_to_enu(fix_llas[:, 0], fix_llas[:, 1], fix_llas[:, 2], provisional)
        good = ~outlier_mask(enu[:, :2])
        fix_t, fix_llas = fix_t[good], fix_llas[good]
    if len(fix_t):
        in_window = np.nonzero(fix_t <= window_end)[0]
        o = in_window[0] if len(in_window) else 0
        origin = tuple(float(v) for v in fix_llas[o, :3])
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
    return Reference(origin, fix_t, pos, pos_s, poly, poly_s, vel_t, speed)
