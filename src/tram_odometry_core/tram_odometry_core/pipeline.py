"""Single entry point used by the ROS node and tools/eval (docs/contracts.md, section 2).

Baseline (D-021): mean of the two bogie speeds, path by integration, straight-line dead
reckoning along the heading taken from GNSS during the init window. The modules that
follow replace the parts of it; the numbers of this version are the first row of the table.
"""
import dataclasses
import math
from collections import deque
from typing import Any, Optional

from .dynamics import model_accel
from .position import PathTracker
from .preprocess import Preprocessor
from .slip import SlipDetector
from .types import CommandSample, Estimate, GnssFix, GnssVel, Params, SlipState, WheelSample

# controller samples kept for the drive response delay: bounded memory (O(1) per message);
# 32 samples at 20 Hz cover 1.6 s, five times the identified delay of 0.3 s (D-033)
CMD_HISTORY = 32


class Odometry:
    """`raw` is a `(topic, message)` pair; message fields are those of the ROS message."""

    def __init__(self, params: Params, route=None):
        self.params = params
        self.route = route
        self._preprocess = Preprocessor(params)
        self._tracker = (PathTracker(params, route)
                         if route is not None and params.position.use_map else None)
        self._t0: Optional[float] = None      # stamp of the first input (from preprocess)
        self._t: Optional[float] = None       # newest stamp seen; the state is at this time
        self._wheel = {'front': None, 'rear': None}   # last WheelSample per bogie, m/s
        self._slip = SlipDetector(params)
        self._slip_state = SlipState(1.0, 1.0, False, False, None)
        self._cmd = deque(maxlen=CMD_HISTORY)  # (stamp, notch), stamps increasing
        self._accel_model = 0.0               # m/s^2, drive model at the state time
        self._v = 0.0                         # current speed, m/s
        self._v_measured = 0.0                # speed at the last accepted wheel sample
        self._t_wheel_rx: Optional[float] = None   # state time when a wheel last arrived
        self._distance = 0.0
        self._x = self._y = 0.0
        self._yaw = 0.0
        self._gnss_used = False
        self._fix_ok = False                  # valid master fix seen in the window
        self._vel_best = 0.0                  # fastest GNSS speed seen in the window
        self._stop_since: Optional[float] = None   # stamp when the current standstill began
        self._stop_snapped = False            # this standstill was already offered to the map

    def step(self, raw: Any) -> Optional[Estimate]:
        """Consume one raw input; None means the input was dropped, nothing to publish."""
        try:
            sample = self._preprocess.accept(raw)
            self._t0 = self._preprocess.t0
            if isinstance(sample, WheelSample):
                self._wheel[sample.bogie] = sample
                return self._advance(sample.t, wheel_arrived=True)
            if isinstance(sample, CommandSample):
                self._cmd.append((sample.t, sample.notch))
                return self._advance(sample.t)
            if isinstance(sample, GnssFix):
                self._on_fix(sample)
            elif isinstance(sample, GnssVel):
                self._on_vel(sample)
            return None
        except (TypeError, ValueError, AttributeError):
            return None

    def _notch_at(self, t: float) -> int:
        """Controller position the drive is acting on at `t`: the newest command stamped at
        or before `t - response_delay_s`; neutral before the first one."""
        deadline = t - self.params.drive.response_delay_s
        notch = 0
        for stamp, n in self._cmd:            # bounded: at most CMD_HISTORY entries
            if stamp <= deadline:
                notch = n
            else:
                break
        return notch

    def _advance(self, t: float, wheel_arrived: bool = False) -> Estimate:
        now = t if self._t is None else max(self._t, t)
        front, rear = self._wheel['front'], self._wheel['rear']
        # the detector predicts from the last measured speed over its own dt (last wheel
        # stamp -> newest wheel stamp): a speed already moved on by the model would count
        # the acceleration twice
        est = self._v_measured if self._t is not None else None
        dt = 0.0 if self._t is None else now - self._t
        drive = self.params.drive
        self._accel_model = (model_accel(self._notch_at(now), self._v, self.params)
                             if drive.use_model else 0.0)
        st = self._slip.update(front, rear, self._accel_model, est)
        if wheel_arrived:
            self._t_wheel_rx = now
        if (drive.use_model and self._t_wheel_rx is not None
                and now - self._t_wheel_rx > self.params.input.stale_timeout_s):
            # no wheel sample has arrived for stale_timeout_s of state time: both bogies
            # silent (the detector sees silence only relative to the other bogie) -> trust
            # neither and predict below. Judged by arrival, not by stamp: wheel stamps may
            # trail the controller by seconds (docs/data.md trap 5) while the wheels talk
            st = dataclasses.replace(st, front_trust=0.0, rear_trust=0.0)
        self._slip_state = st
        used = [(w, s.speed) for w, s in ((st.front_trust, front), (st.rear_trust, rear))
                if s is not None and w > 0.0]
        if used:
            self._v = sum(w * v for w, v in used) / sum(w for w, _ in used)
            self._v_measured = self._v
        elif drive.use_model:
            # no trusted bogie: predict with the drive model; wheels never report reverse
            self._v = max(0.0, self._v + self._accel_model * dt)
        # (without the model: keep the last speed, D-021)
        self._t = now
        ds = self._v * dt
        self._distance += ds
        self._x += ds * math.cos(self._yaw)
        self._y += ds * math.sin(self._yaw)
        self._on_standstill(now)
        return self._estimate(t, now)

    def _on_standstill(self, now: float) -> None:
        """After `stop_min_s` of standing, once per standstill, let the map snap the position
        to a stop place. Not in the GNSS window: the anchor is still being set there."""
        p = self.params.position
        if self._v >= p.stop_speed_mps:
            self._stop_since, self._stop_snapped = None, False
            return
        if self._stop_since is None:
            self._stop_since = now
        if (self._tracker is not None and not self._stop_snapped
                and now - self._stop_since >= p.stop_min_s
                and now - self._t0 > self.params.gnss.init_window_s):
            self._stop_snapped = True
            self._tracker.on_stop(self._distance)

    def _estimate(self, t: float, now: float) -> Estimate:
        var = self.params.filter.r_wheel
        pos_var = var * (now - self._t0) ** 2      # speed noise integrated over the run
        x, y, z, yaw, pos_cov = self._x, self._y, 0.0, self._yaw, (pos_var, pos_var, 0.0)
        on_map = self._tracker.advance(self._distance) if self._tracker is not None else None
        if on_map is not None:
            x, y, z, yaw, pos_cov = on_map
        return Estimate(
            t=t, speed=self._v, speed_var=var, accel=0.0, accel_model=self._accel_model,
            distance=self._distance, x=x, y=y, z=z, yaw=yaw,
            pos_cov=pos_cov,
            slip=self._slip_state, gnss_used=self._gnss_used)

    def _on_fix(self, sample: GnssFix) -> None:
        # the origin of frame `map` is the first valid fix, so the start is (0, 0)
        if sample.status >= 0:
            self._fix_ok = True
            if self._tracker is not None:
                self._tracker.on_fix(sample.lat, sample.lon, sample.alt,
                                     sample.status, self._distance)
                self._gnss_used = self._gnss_used or self._tracker.ready

    def _on_vel(self, sample: GnssVel) -> None:
        if not self._fix_ok:
            return
        speed = math.hypot(sample.ve, sample.vn)
        if speed > self._vel_best:      # heading is only defined while moving
            self._vel_best = speed
            self._yaw = math.atan2(sample.vn, sample.ve)
            self._gnss_used = True
