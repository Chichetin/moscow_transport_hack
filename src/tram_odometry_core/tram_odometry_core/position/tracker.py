"""Position along the route map (D-007, D-024).

Alignment in the GNSS init window: the frame `map` of the run is ENU at the first GBAS fix
(else the first valid one), the same rule as the reference of tools/eval. The route, stored in
ENU of its own fixed origin, is converted into that frame once (map ENU -> ECEF -> run ENU, so
the tangent-plane rotation between the two origins is kept). Every fix of the window anchors
the tram to the nearest branch: (branch, s, distance). The rover antenna stands ahead of
master in the direction of travel (docs/data.md, trap 15), so once both are seen the anchor is
the nearest point of a branch running along master -> rover: at a standstill that tells the
track of one direction from the other and a terminal track from the dead end of a main branch. After that the tram only moves forward
along its branch (trams here are single-ended): s = s_anchor + distance - distance_anchor,
continuing onto the next branch at the end. x, y, z and yaw come from the branch at s, z is
shifted by the run's median height offset from the map in the window.

Stop places (D-034): a stop of the tram (the pipeline detects it) close to a stop place of the
map moves s to that place, weighted by the variances, and resets the along-track variance.
Between two snaps on one branch the ratio of map arc to wheel path updates a slow online wheel
scale. A stop past the snap gate is a signal, unless a wheel scale error has carried s off the
places: two such misses in a row on one side, on the line of a scale error from the anchor,
relock s onto the place (#154). All of it uses the map only, never GNSS after the window.
"""
import math
from typing import Optional, Tuple

import numpy as np

from ..types import Params, Route
from .geo import ecef as _ecef
from .geo import enu_rotation as _enu_rotation

STATUS_FIX = 0               # sensor_msgs/NavSatStatus: a position is present
STATUS_GBAS_FIX = 2          # preferred by the reference of tools/eval when the run has it
JOIN_MAX_TURN_RAD = 2.0 * math.pi / 3.0   # a branch never continues onto a track going back


class PathTracker:
    def __init__(self, params: Params, route: Route):
        self.p = params.position
        self.route = route
        self._s = [b.s for b in route.branches]
        self._step = [float(b.s[1] - b.s[0]) for b in route.branches]
        self._map = [np.column_stack([b.x, b.y, b.z]) for b in route.branches]
        self._next = [self._join(k) for k in range(len(self._map))]
        self._fork = [self._fork_join(k) for k in range(len(self._map))]
        self._side = [self._side_branch(k) for k in range(len(self._map))]
        self._undo = None                     # (anchor, var0, last_snap) before a side switch
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
        self._chain_arc = 0.0                 # m, map arc between consecutive snaps on one branch
        self._chain_wheel = 0.0               # m, wheel path between the same snaps
        self._last_snap: Optional[Tuple[int, float, float]] = None   # branch, place s, distance
        self._pending: Optional[Tuple[float, float]] = None   # distance, place - s of the last miss
        self._master = None                   # (ECEF, distance) of the last accepted master fix
        self._rover = None                    # (ECEF, status) of the last accepted rover fix

    @property
    def ready(self) -> bool:
        return self._anchor is not None

    @property
    def frame(self):
        """(ENU rotation rows, ECEF origin) of the frame of `advance`: the run frame once a fix
        is accepted, else the map's own ENU; the pipeline turns it into the output grid."""
        if self._rot is None:
            return self._map_rot, self._map_ecef0
        return self._rot, self._ecef0

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

    def _branch_start_on(self, k: int, j: int):
        """(s on branch k, distance) where branch j starts on k heading along it, or None."""
        parent, child = self._map[k], self._map[j]
        start = child[0, :2]
        i = int(np.argmin((parent[:, 0] - start[0]) ** 2 + (parent[:, 1] - start[1]) ** 2))
        d = float(np.hypot(*(parent[i, :2] - start)))
        if d > self.p.join_m or i >= len(parent) - 1:
            return None
        tangent = parent[i + 1, :2] - parent[i, :2]
        child_tangent = child[1, :2] - child[0, :2]
        cos = float(np.dot(tangent, child_tangent) /
                    (np.linalg.norm(tangent) * np.linalg.norm(child_tangent)))
        return None if cos < math.cos(JOIN_MAX_TURN_RAD) else (float(self._s[k][i]), d)

    def _nearest_start(self, k: int, children):
        """(s on branch k, j) of the child branch starting nearest to branch k, or None."""
        best = None
        for j in children:
            at = self._branch_start_on(k, j)
            if at is not None and (best is None or at[1] < best[2]):
                best = (at[0], j, at[1])
        return None if best is None else best[:2]

    def _fork_join(self, k: int):
        """A forward branch start that bypasses this branch's dead end, if one exists."""
        if self._next[k] is not None:
            return None
        return self._nearest_start(k, [j for j in range(len(self._map))
                                       if j != k and self._next[j] is not None])

    def _side_branch(self, k: int):
        """(s on branch k, j): a dead-end branch j leaving k mid-way that the default path does
        not take (the westbound track into the depot stub at the west terminal, #138)."""
        taken = self._fork[k][1] if self._fork[k] is not None else None
        return self._nearest_start(k, [j for j in range(len(self._map))
                                       if j not in (k, taken) and self._next[j] is None])

    def _take_side(self, distance: float, speed: float) -> None:
        """Move onto the side branch when the tram is past its start and faster than any tram
        heading for the default path there (the loop is slow, the depot track is not). A tram
        that runs past the side branch's dead end by `side_overrun_m` was on the default path
        after all: back to it, as if the switch never happened."""
        if self._undo is not None:
            if self._overrun(distance) > self.p.side_overrun_m:
                self._anchor, self._var0, self._last_snap = self._undo
                self._undo = self._pending = None
            return
        if not (math.isfinite(speed) and speed > self.p.side_speed_mps):
            return
        k, s = self._state(distance)
        side = self._side[k]
        if side is None or not self.p.side_min_m <= s - side[0] <= self.p.side_max_m:
            return
        self._undo = (self._anchor, self._var0, self._last_snap)
        var = self._var_along(distance)
        j = side[1]
        self._anchor = (j, float(self._s[j][0]) + s - side[0], distance)
        self._var0 = var
        self._last_snap = self._pending = None

    def _overrun(self, distance: float) -> float:
        """Path beyond the dead end of the side branch at `distance` (0 before it): the state
        clamps s there, so the overrun comes from the unclamped arc of the anchor."""
        k, s0, d0 = self._anchor
        return max(0.0, s0 + self._scale * (distance - d0) - float(self._s[k][-1]))

    def _set_origin(self, lat: float, lon: float, alt: float, status: int) -> None:
        self._origin_status = status
        self._rot, self._ecef0 = _enu_rotation(lat, lon), _ecef(lat, lon, alt)
        lat0, lon0, alt0 = self.route.origin
        a = self._rot @ _enu_rotation(lat0, lon0).T
        c = self._rot @ (_ecef(lat0, lon0, alt0) - self._ecef0)
        self._xyz = [xyz @ a.T + c for xyz in self._map]
        self._anchor, self._dz = None, []
        self._undo = None                     # a side switch belongs to the anchor it replaced

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
        rover, status = self._rover
        if status < STATUS_GBAS_FIX <= self._origin_status:
            return None                       # a plain fix next to GBAS ones: noise (on_fix)
        h = (self._rot @ (rover - self._master[0]))[:2]
        base = math.hypot(*h)
        ok = self.p.heading_min_base_m <= base <= self.p.heading_max_base_m
        return h if ok else None

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
        dz = float(np.median(self._dz))
        # a run's GNSS height is metres off the map; tens of metres is a glitch of the window's
        # fixes (status 0 jumps), not the run: keep the map height then (#169, D-087)
        self._dz_median = dz if abs(dz) <= self.p.height_offset_max_m else 0.0

    def on_rover(self, lat: float, lon: float, alt: float, status: int) -> None:
        """A GNSS rover fix of the init window: heading only, never the origin or the anchor
        point; an outlier fix is ignored by the gate of `on_fix`. An anchor already set that runs
        against the heading moves to a branch along it, if there is one; otherwise it is kept
        (a late rover fix must not undo a stop snap)."""
        if (not all(math.isfinite(v) for v in (lat, lon, alt)) or status < STATUS_FIX
                or not self._on_map(lat, lon, alt)):
            return
        self._rover = (_ecef(lat, lon, alt), status)
        heading = self._heading()
        if self._anchor is None or heading is None:
            return
        k, s, _ = self._anchor
        yaw = self._at(k, s)[3]
        if heading[0] * math.cos(yaw) + heading[1] * math.sin(yaw) > 0.0:
            return
        if self._locate((self._rot @ (self._master[0] - self._ecef0))[:2])[0] != k:
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
        self._undo = None
        self._var0 = self.p.anchor_std_m ** 2
        self._last_snap = self._pending = None
        return k, s, float(p[2])

    def _state(self, distance: float):
        """(branch, s) at path `distance`, following the joins at the ends of branches."""
        k, s, _ = self._walk(distance)
        return k, s

    def _walk(self, distance: float):
        """(branch, s, metres past a dead end) at path `distance`."""
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
                return k, end, s - end
            k, entry_s = self._next[k]
            s = entry_s + s - end
        return k, max(s, 0.0), 0.0

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
        place = float(places[nearest])
        # map uncertainty cannot distinguish close candidates (D-047)
        ambiguous = len(places) > 1 and (float(np.partition(distances, 1)[1]) - distances[nearest]
                                         <= 2.0 * self.p.stop_std_m)
        relock = bool(distances[nearest] > self.p.stop_snap_max_m)
        if relock:
            if not self._relock(place - s, distance, ambiguous):
                return False                  # not at a stop place: a signal, keep s
        elif ambiguous:
            return False
        var = self._var_along(distance)
        gain = var / (var + self.p.stop_std_m ** 2)
        self._update_scale(k, place, distance)
        # the pair across a lost lock may hold a wheel gap or a false relock; the next pair,
        # from the relock place on, stays in the chain (docs/model.md, #154)
        if not relock:
            self._accumulate_chain(k, place, distance)
        self._anchor = (k, s + gain * (place - s), distance)
        self._undo = self._pending = None
        self._var0 = (1.0 - gain) * var
        self._last_snap = (k, place, distance)
        return True

    def _relock(self, innovation: float, distance: float, ambiguous: bool) -> bool:
        """A stop past the gate is a signal, unless the wheel scale has carried s off the
        places: then the misses (`innovation` = place - s) are on one side and grow with the
        path from the anchor. True for the second such miss in a row when it lies on the line
        of the first within `relock_sigma`; a miss no scale within 1 +- scale_max_dev explains
        drops the first one, a miss at an ambiguous place is skipped (#154). A stop closer than
        stop_snap_max_m of path to the first miss is the same standstill (a creep in a queue,
        which the pipeline sees as two): at q ~ 1 the residual is the creep, well inside the
        line, so that stop changes nothing and the first miss stays."""
        first = self._pending
        if first is not None and distance - first[0] < self.p.stop_snap_max_m:
            return False
        path = distance - self._anchor[2]
        if path <= 0.0 or abs(self._scale + innovation / path - 1.0) > self.p.scale_max_dev:
            self._pending = None
            return False
        if ambiguous:
            return False
        self._pending = (distance, innovation)
        if first is None or first[1] * innovation <= 0.0:
            return False
        # both misses are e = c * path + a (c: scale error, a: anchor error of var0) plus a
        # place error of stop_std_m each; the second minus q times the first cancels c
        q = path / (first[0] - self._anchor[2])
        var = self.p.stop_std_m ** 2 * (1.0 + q * q) + self._var0 * (1.0 - q) ** 2
        return abs(innovation - q * first[1]) <= self.p.relock_sigma * math.sqrt(var)

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

    def _accumulate_chain(self, k: int, place: float, distance: float) -> None:
        """Sums for the speed scale over consecutive snaps on one branch: they telescope, so
        the error of the ratio is that of the chain ends (where exactly the car stood), not of
        every pair; short pairs count too."""
        last = self._last_snap
        if last is None or last[0] != k or distance <= last[2] or place < last[1]:
            return
        arc, wheel = place - last[1], distance - last[2]
        if abs(arc - wheel) > 2.0 * self.p.stop_snap_max_m + self.p.scale_max_dev * wheel:
            return     # not one stretch of track: e.g. a whole loop passed with no snap on the way
        self._chain_arc += arc
        self._chain_wheel += wheel

    @property
    def speed_scale(self) -> float:
        """Wheel scale for the published speed (#153): the chain ratio shrunk to 1 by a prior
        of `speed_scale_prior_m` of path, within 1 +- scale_max_dev. The per-pair `_scale` of the
        path is re-anchored at every stop; the speed has no such reset and needs the quieter
        estimate (one pair scatters by 0.6 % on train, the true scale of 30618 by 0.15 %)."""
        prior = self.p.speed_scale_prior_m
        k = (self._chain_arc + prior) / (self._chain_wheel + prior)
        return min(max(k, 1.0 - self.p.scale_max_dev), 1.0 + self.p.scale_max_dev)

    def advance(self, distance: float, speed: float = 0.0):
        """(x, y, z, yaw, (var_x, var_y, cov_xy)) of base_link at path `distance` and speed
        (m/s), None before alignment. The map, the anchor and the stop places are the track of
        the master antenna; base_link (the front bogie pivot at rail level, organizers' tf) is
        `base_ahead_m` ahead of it along the track and `antenna_height_m` below (D-077)."""
        if self._anchor is None:
            return None
        self._take_side(distance, speed)
        k, s, over = self._walk(distance + self.p.base_ahead_m / self._scale)
        x, y, z, yaw = self._at(k, s)
        # the map ends where master stood in the recordings, so the rail goes on at least
        # base_ahead_m past a dead end: base_link keeps up to that much ahead of it there
        over = min(over, self.p.base_ahead_m)
        x, y = x + over * math.cos(yaw), y + over * math.sin(yaw)
        var_cross = self.p.cross_std_m ** 2
        var_along = var_cross + self._var_along(distance)
        c, sn = math.cos(yaw), math.sin(yaw)
        cov = (var_along * c * c + var_cross * sn * sn, var_along * sn * sn + var_cross * c * c,
               (var_along - var_cross) * c * sn)
        return x, y, z + self._dz_median - self.p.antenna_height_m, yaw, cov
