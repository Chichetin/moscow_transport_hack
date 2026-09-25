"""Position along the route map (D-007, D-024).

Alignment in the GNSS init window: the frame `map` of the run is ENU at the first GBAS fix
(else the first valid one), the same rule as the reference of tools/eval. The route, stored in
ENU of its own fixed origin, is converted into that frame once (map ENU -> ECEF -> run ENU, so
the tangent-plane rotation between the two origins is kept). Every fix of the window anchors
the tram to the nearest branch: (branch, s, distance). After that the tram only moves forward
along its branch (trams here are single-ended): s = s_anchor + distance - distance_anchor,
continuing onto the next branch at the end. x, y, z and yaw come from the branch at s, z is
shifted by the run's median height offset from the map in the window.
"""
import math
from typing import Optional, Tuple

import numpy as np

from ..types import Params, Route

WGS84_A = 6378137.0          # m
WGS84_E2 = 6.69437999014e-3
STATUS_FIX = 0               # sensor_msgs/NavSatStatus: a position is present
STATUS_GBAS_FIX = 2          # preferred by the reference of tools/eval when the run has it
JOIN_MAX_TURN_RAD = 2.0 * math.pi / 3.0   # a branch never continues onto a track going back


def _ecef(lat: float, lon: float, alt: float) -> np.ndarray:
    la, lo = math.radians(lat), math.radians(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(la) ** 2)
    return np.array([(n + alt) * math.cos(la) * math.cos(lo),
                     (n + alt) * math.cos(la) * math.sin(lo),
                     (n * (1.0 - WGS84_E2) + alt) * math.sin(la)])


def _enu_rotation(lat: float, lon: float) -> np.ndarray:
    """Rows: east, north, up unit vectors in ECEF."""
    la, lo = math.radians(lat), math.radians(lon)
    return np.array([[-math.sin(lo), math.cos(lo), 0.0],
                     [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
                     [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])


class PathTracker:
    def __init__(self, params: Params, route: Route):
        self.p = params.position
        self.route = route
        self._s = [b.s for b in route.branches]
        self._step = [float(b.s[1] - b.s[0]) for b in route.branches]
        self._map = [np.column_stack([b.x, b.y, b.z]) for b in route.branches]
        self._next = [self._join(k) for k in range(len(self._map))]
        self._xyz = self._map                 # branches in the frame of the run
        lat0, lon0, alt0 = route.origin       # the map's own ENU: the outlier gate lives here
        self._map_rot, self._map_ecef0 = _enu_rotation(lat0, lon0), _ecef(lat0, lon0, alt0)
        self._origin_status: Optional[int] = None
        self._rot = self._ecef0 = None
        self._anchor: Optional[Tuple[int, float, float]] = None   # branch, s, distance
        self._dz = []                         # run height - map height, one per window fix
        self._dz_median = 0.0

    @property
    def ready(self) -> bool:
        return self._anchor is not None

    def _join(self, k: int):
        """(branch, s) where branch k continues after its end, or None for a dead end."""
        end, tangent = self._map[k][-1, :2], self._map[k][-1, :2] - self._map[k][-2, :2]
        best = None
        for j, xyz in enumerate(self._map):
            if j == k:
                continue
            d = np.hypot(xyz[:, 0] - end[0], xyz[:, 1] - end[1])
            i = int(np.argmin(d))
            if d[i] > self.p.join_m:
                continue
            i = min(i, len(xyz) - 2)
            other = xyz[i + 1, :2] - xyz[i, :2]
            cos = float(np.dot(tangent, other) / (np.linalg.norm(tangent) * np.linalg.norm(other)))
            if cos < math.cos(JOIN_MAX_TURN_RAD):
                continue
            if best is None or d[i] < best[2]:
                best = (j, float(self._s[j][i]), float(d[i]))
        return None if best is None else best[:2]

    def _set_origin(self, lat: float, lon: float, alt: float, status: int) -> None:
        self._origin_status = status
        self._rot, self._ecef0 = _enu_rotation(lat, lon), _ecef(lat, lon, alt)
        lat0, lon0, alt0 = self.route.origin
        a = self._rot @ _enu_rotation(lat0, lon0).T
        c = self._rot @ (_ecef(lat0, lon0, alt0) - self._ecef0)
        self._xyz = [xyz @ a.T + c for xyz in self._map]
        self._anchor, self._dz = None, []

    def _locate(self, xy: np.ndarray) -> Tuple[int, float]:
        """Nearest branch and arc length of a point in the frame of the run."""
        k, s, _ = self._nearest(self._xyz, xy)
        return k, s

    def _nearest(self, branches, xy: np.ndarray) -> Tuple[int, float, float]:
        """(branch, s, distance) of the nearest point of `branches` (the map's or the run's
        frame: s is the same in both). Window only: brute force over the map is fine there."""
        best = (0, 0.0, math.inf)
        for k, xyz in enumerate(branches):
            i = int(np.argmin((xyz[:, 0] - xy[0]) ** 2 + (xyz[:, 1] - xy[1]) ** 2))
            for j in (max(i - 1, 0), min(i, len(xyz) - 2)):
                a, t = xyz[j, :2], xyz[j + 1, :2] - xyz[j, :2]
                u = min(max(float(np.dot(xy - a, t) / np.dot(t, t)), 0.0), 1.0)
                d = float(np.hypot(*(a + u * t - xy)))
                if d < best[2]:
                    best = (k, float(self._s[k][j] + u * self._step[k]), d)
        return best

    def _at(self, k: int, s: float):
        """x, y, z, yaw of branch k at arc length s (uniform step: O(1))."""
        xyz = self._xyz[k]
        i = min(max(int(s / self._step[k]), 0), len(xyz) - 2)
        u = min(max((s - self._s[k][i]) / self._step[k], 0.0), 1.0)
        p = xyz[i] + u * (xyz[i + 1] - xyz[i])
        yaw = math.atan2(xyz[i + 1, 1] - xyz[i, 1], xyz[i + 1, 0] - xyz[i, 0])
        return float(p[0]), float(p[1]), float(p[2]), yaw

    def on_fix(self, lat: float, lon: float, alt: float, status: int, distance: float) -> None:
        """A GNSS master fix of the init window; `distance` is the path at that moment."""
        if not all(math.isfinite(v) for v in (lat, lon, alt, distance)) or status < STATUS_FIX:
            return
        # a fix farther than fix_gate_m from every branch is an outlier (docs/data.md, trap 10:
        # kilometres off at the start of a run, lat = lon = 0): it must not become the origin
        # of the frame, the anchor or a height sample
        p_map = self._map_rot @ (_ecef(lat, lon, alt) - self._map_ecef0)
        if self._nearest(self._map, p_map[:2])[2] > self.p.fix_gate_m:
            return
        if self._origin_status is None or self._origin_status < STATUS_GBAS_FIX <= status:
            self._set_origin(lat, lon, alt, status)
        elif status < STATUS_GBAS_FIX <= self._origin_status:
            return                            # plain fixes after a GBAS origin: noise
        p = self._rot @ (_ecef(lat, lon, alt) - self._ecef0)
        k, s = self._locate(p[:2])
        self._anchor = (k, s, distance)
        self._dz.append(float(p[2]) - self._at(k, s)[2])
        self._dz_median = float(np.median(self._dz))

    def advance(self, distance: float):
        """(x, y, z, yaw, (var_x, var_y, cov_xy)) at path `distance`, None before alignment."""
        if self._anchor is None:
            return None
        k, s, d0 = self._anchor
        s += distance - d0
        # bounded number of branch changes per call: a run never crosses more branches than the
        # map has; at the limit the position stays at the end of the last branch (a dead end)
        for _ in range(len(self._xyz)):
            end = float(self._s[k][-1])
            if s <= end:
                break
            if self._next[k] is None:
                s = end
                break
            k, s = self._next[k][0], self._next[k][1] + s - end
        x, y, z, yaw = self._at(k, max(s, 0.0))
        var_cross = self.p.cross_std_m ** 2
        var_along = var_cross + (self.p.along_drift_frac * (distance - d0)) ** 2
        c, sn = math.cos(yaw), math.sin(yaw)
        cov = (var_along * c * c + var_cross * sn * sn, var_along * sn * sn + var_cross * c * c,
               (var_along - var_cross) * c * sn)
        return x, y, z + self._dz_median, yaw, cov
