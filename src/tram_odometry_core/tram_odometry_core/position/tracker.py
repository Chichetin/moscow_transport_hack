"""Position along the route map (D-007, D-024).

Alignment in the GNSS init window: the frame `map` of the run is ENU at the first GBAS fix
(else the first valid one), the same rule as the reference of tools/eval. The route, stored in
ENU of its own fixed origin, is converted into that frame once (map ENU -> ECEF -> run ENU, so
the tangent-plane rotation between the two origins is kept). Every fix of the window anchors
the tram to the nearest branch: (branch, s, distance). The rover antenna stands ahead of master
in the direction of travel (docs/data.md, trap 15), so once both are seen the anchor is the
nearest point of a branch running along master -> rover: at a standstill that tells the track of
one direction from the other and a terminal track from the dead end of the main branch. After that the tram only moves forward
along its branch (trams here are single-ended): s = s_anchor + distance - distance_anchor,
continuing onto the next branch at the end. x, y, z and yaw come from the branch at s, z is
shifted by the run's median height offset from the map in the window.

Stop places (D-034): a stop of the tram (the pipeline detects it) close to a stop place of the
map moves s to that place, weighted by the variances, and resets the along-track variance.
Between two snaps on one branch the ratio of map arc to wheel path updates a slow online wheel
scale. Both use the map only, never GNSS after the window.
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
        self._fork = [self._fork_join(k) for k in range(len(self._map))]
        self._xyz = self._map                 # branches in the frame of the run
        lat0, lon0, alt0 = route.origin       # the map's own ENU: the outlier gate lives here
        self._map_rot, self._map_ecef0 = _enu_rotation(lat0, lon0), _ecef(lat0, lon0, alt0)
        self._origin_status: Optional[int] = None
        self._rot = self._ecef0 = None
        self._anchor: Optional[Tuple[int, float, float]] = None   # branch, s, distance
        self._dz = []                         # run height - map height, one per window fix
        self._dz_median = 0.0
        self._stops = [np.array(sorted(s for b, s in route.stops if b == k), float)
                       for k in range(len(self._map))]
        self._var0 = 0.0                      # along-track variance at the anchor, m^2
        self._scale = 1.0                     # online wheel scale: map arc per metre of wheel path
        self._last_snap: Optional[Tuple[int, float, float]] = None   # branch, place s, distance
        self._master = None                   # (ECEF, distance) of the last accepted master fix
        self._rover = None                    # ECEF of the last accepted rover fix

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

    def _fork_join(self, k: int):
        """A forward branch start that bypasses this branch's dead end, if one exists."""
        if self._next[k] is not None:
            return None
        parent = self._map[k]
        best = None
        for j, child in enumerate(self._map):
            if j == k or self._next[j] is None:
                continue
            start = child[0, :2]
            i = int(np.argmin((parent[:, 0] - start[0]) ** 2 +
                              (parent[:, 1] - start[1]) ** 2))
            d = float(np.hypot(*(parent[i, :2] - start)))
            if d > self.p.join_m or i >= len(parent) - 1:
                continue
            tangent = parent[i + 1, :2] - parent[i, :2]
            child_tangent = child[1, :2] - child[0, :2]
            cos = float(np.dot(tangent, child_tangent) /
                        (np.linalg.norm(tangent) * np.linalg.norm(child_tangent)))
            if cos < math.cos(JOIN_MAX_TURN_RAD):
                continue
            candidate = (float(self._s[k][i]), j, d)
            if best is None or d < best[2]:
                best = candidate
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
        """Nearest branch and arc length of a point in the frame of the run, on a branch
        running along the heading when there is one within the fix gate."""
        heading = self._heading()
        if heading is not None:
            k, s, d = self._nearest(self._xyz, xy, heading)
            if d <= self.p.fix_gate_m:
                return k, s
        k, s, _ = self._nearest(self._xyz, xy)
        return k, s

    def _heading(self) -> Optional[np.ndarray]:
        """master -> rover in the frame of the run, None without a usable base."""
        if self._master is None or self._rover is None:
            return None
        h = (self._rot @ (self._rover - self._master[0]))[:2]
        return h if math.hypot(*h) >= self.p.heading_min_base_m else None

    def _nearest(self, branches, xy: np.ndarray, heading=None) -> Tuple[int, float, float]:
        """(branch, s, distance) of the nearest point of `branches` (the map's or the run's
        frame: s is the same in both); with `heading`, only segments running along it.
        Window only: brute force over the map is fine there."""
        best = (0, 0.0, math.inf)
        for k, xyz in enumerate(branches):
            d2 = (xyz[:, 0] - xy[0]) ** 2 + (xyz[:, 1] - xy[1]) ** 2
            if heading is not None:
                seg = xyz[1:, :2] - xyz[:-1, :2]
                along = np.append(seg @ heading, seg[-1] @ heading)   # a vertex: its next segment
                d2 = np.where(along > 0.0, d2, math.inf)
                if not np.isfinite(d2).any():
                    continue
            i = int(np.argmin(d2))
            for j in (max(i - 1, 0), min(i, len(xyz) - 2)):
                a, t = xyz[j, :2], xyz[j + 1, :2] - xyz[j, :2]
                if heading is not None and float(np.dot(t, heading)) <= 0.0:
                    continue
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
        if not self._on_map(lat, lon, alt):
            return
        if self._origin_status is None or self._origin_status < STATUS_GBAS_FIX <= status:
            self._set_origin(lat, lon, alt, status)
        elif status < STATUS_GBAS_FIX <= self._origin_status:
            return                            # plain fixes after a GBAS origin: noise
        self._master = (_ecef(lat, lon, alt), distance)
        k, s, z = self._anchor_master()
        self._dz.append(z - self._at(k, s)[2])
        self._dz_median = float(np.median(self._dz))

    def on_rover(self, lat: float, lon: float, alt: float, status: int) -> None:
        """A GNSS rover fix of the init window: heading only, never the origin or the anchor
        point; an outlier fix is ignored by the gate of `on_fix`. An anchor already set runs
        along the heading or is moved to the master fix on a branch that does; one that runs
        along it is kept (a late rover fix must not undo a stop snap)."""
        if (not all(math.isfinite(v) for v in (lat, lon, alt)) or status < STATUS_FIX
                or not self._on_map(lat, lon, alt)):
            return
        self._rover = _ecef(lat, lon, alt)
        heading = self._heading()
        if self._anchor is None or heading is None:
            return
        yaw = self._at(*self._anchor[:2])[3]
        if heading[0] * math.cos(yaw) + heading[1] * math.sin(yaw) <= 0.0:
            self._anchor_master()

    def _on_map(self, lat: float, lon: float, alt: float) -> bool:
        p_map = self._map_rot @ (_ecef(lat, lon, alt) - self._map_ecef0)
        return self._nearest(self._map, p_map[:2])[2] <= self.p.fix_gate_m

    def _anchor_master(self):
        """Anchor at the last master fix; returns (branch, s, height of the fix in the run)."""
        ecef, distance = self._master
        p = self._rot @ (ecef - self._ecef0)
        k, s = self._locate(p[:2])
        self._anchor = (k, s, distance)
        self._var0 = self.p.anchor_std_m ** 2
        self._last_snap = None
        return k, s, float(p[2])

    def _state(self, distance: float):
        """(branch, s) at path `distance`, following the joins at the ends of branches."""
        k, s, d0 = self._anchor
        s += self._scale * (distance - d0)
        entry_s = self._anchor[1]
        # bounded number of branch changes per call: a run never crosses more branches than the
        # map has; at the limit the position stays at the end of the last branch (a dead end)
        for _ in range(len(self._xyz)):
            fork = self._fork[k]
            if fork is not None and entry_s <= fork[0] < s:
                fork_s, next_k = fork
                s = self._s[next_k][0] + s - fork_s
                k, entry_s = next_k, float(self._s[next_k][0])
                continue
            end = float(self._s[k][-1])
            if s <= end:
                break
            if self._next[k] is None:
                s = end
                break
            k, entry_s = self._next[k]
            s = entry_s + s - end
        return k, max(s, 0.0)

    def _var_along(self, distance: float) -> float:
        return self._var0 + (self.p.along_drift_frac * self._scale * (distance - self._anchor[2])) ** 2

    def on_stop(self, distance: float) -> bool:
        """The tram has stood still for `stop_min_s` at path `distance`. True when s was
        snapped to a stop place of the map."""
        if self._anchor is None or not math.isfinite(distance):
            return False
        k, s = self._state(distance)
        places = self._stops[k]
        if not len(places):
            return False
        distances = np.abs(places - s)
        nearest = int(np.argmin(distances))
        if distances[nearest] > self.p.stop_snap_max_m:
            return False                      # not at a stop place: a signal, keep s
        if len(places) > 1:
            second = float(np.partition(distances, 1)[1])
            if second - distances[nearest] <= 2.0 * self.p.stop_std_m:
                return False                  # map uncertainty cannot distinguish close candidates
        place = float(places[nearest])
        var = self._var_along(distance)
        gain = var / (var + self.p.stop_std_m ** 2)
        self._update_scale(k, place, distance)
        self._anchor = (k, s + gain * (place - s), distance)
        self._var0 = (1.0 - gain) * var
        self._last_snap = (k, place, distance)
        return True

    def _update_scale(self, k: int, place: float, distance: float) -> None:
        """Slow update of the wheel scale from two snaps on one branch far enough apart."""
        if self._last_snap is None or self._last_snap[0] != k:
            return
        _, place0, d0 = self._last_snap
        if place - place0 < self.p.scale_min_arc_m or distance <= d0:
            return
        ratio = min(max((place - place0) / (distance - d0), 1.0 - self.p.scale_max_dev),
                    1.0 + self.p.scale_max_dev)
        self._scale += self.p.scale_alpha * (ratio - self._scale)

    def advance(self, distance: float):
        """(x, y, z, yaw, (var_x, var_y, cov_xy)) at path `distance`, None before alignment."""
        if self._anchor is None:
            return None
        k, s = self._state(distance)
        x, y, z, yaw = self._at(k, s)
        var_cross = self.p.cross_std_m ** 2
        var_along = var_cross + self._var_along(distance)
        c, sn = math.cos(yaw), math.sin(yaw)
        cov = (var_along * c * c + var_cross * sn * sn, var_along * sn * sn + var_cross * c * c,
               (var_along - var_cross) * c * sn)
        return x, y, z + self._dz_median, yaw, cov
