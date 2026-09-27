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
from .estimator import SpeedFilter
from .position import PathTracker, pose_to_grid
from .position.geo import ecef, enu_rotation
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
        self._filter = SpeedFilter(params)
        self._slip_state = SlipState(1.0, 1.0, False, False, None)
        self._sliding = False                 # the car slides: both bogies out (#156)
        self._cmd = deque(maxlen=CMD_HISTORY)  # (stamp, notch), stamps increasing
        self._accel_model = 0.0               # m/s^2, drive model at the state time
        self._v = 0.0                         # current speed, m/s
        self._v_measured = 0.0                # speed at the last accepted wheel sample
        self._t_wheel_rx: Optional[float] = None   # state time when a wheel last arrived
        self._distance = 0.0
        self._x = self._y = 0.0
        self._yaw = 0.0
        f = params.frames
        self._grid = (f.grid_zone, f.grid_origin_e_m, f.grid_origin_n_m)   # output frame (D-083)
        # ENU frame of the straight line (x, y) without a map: the first valid master fix, where
        # the line restarts from 0; None before it (no absolute position is known then)
        self._line_frame = None
        self._gnss_used = False
        self._fix_ok = False                  # valid master fix seen in the window
        self._vel_best = 0.0                  # fastest GNSS speed seen in the window
        self._stop_since: Optional[float] = None   # stamp when the current standstill began
        self._stop_snapped = False            # this standstill was already offered to the map
        # standstill of the state (#105): when the speed reached 0 (-inf: standing since the
        # start or a clock resync) and the deceleration it came to rest with; None moving
        self._t_zero: Optional[float] = -math.inf
        self._a_zero = 0.0                    # m/s^2, <= 0

    def step(self, raw: Any) -> Optional[Estimate]:
        """Consume one raw input; None means the input was dropped, nothing to publish."""
        try:
            sample = self._preprocess.accept(raw)
            self._t0 = self._preprocess.t0
            if isinstance(sample, WheelSample):
                self._wheel[sample.bogie] = sample
                return self._advance(sample.t, sample=sample, wheel_arrived=True)
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

    def _advance(self, t: float, sample=None, wheel_arrived: bool = False) -> Estimate:
        jump = self.params.input.max_stamp_jump_s
        if self._t is not None and self._t - t > jump:
            # the input clock was resynced back by preprocess (#77, D-043): the state time is
            # in the future of every input now; follow the input instead of freezing there
            self._t = t
            self._filter.rebase_time(t)
            self._t_wheel_rx = t if self._t_wheel_rx is not None else None
            self._stop_since, self._stop_snapped = None, False
            if self._t_zero is not None:
                self._t_zero, self._a_zero = -math.inf, 0.0
        now = t if self._t is None else max(self._t, t)
        front, rear = self._wheel['front'], self._wheel['rear']
        # the detector predicts from the last measured speed over its own dt (last wheel
        # stamp -> newest wheel stamp): a speed already moved on by the model would count
        # the acceleration twice
        est = self._v_measured if self._t is not None else None
        dt = 0.0 if self._t is None else now - self._t
        if dt > jump:
            dt = 0.0     # a clock jump, not travel: no bag holds such a gap (max 2.6 s, D-043)
            self._filter.rebase_time(now)
        drive = self.params.drive
        self._accel_model = (model_accel(self._notch_at(now), self._v, self.params)
                             if drive.use_model else 0.0)
        st = self._slip.update(front, rear, self._accel_model, est, state_time=now)
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
        v_before, t_before = self._v, self._t
        self._filter.predict(now, self._accel_model)
        car = self._slip.car_speed(now)
        if car is not None and not self._sliding:
            # the filter has been following the slipping bogies since the slide began
            self._filter.restart(car)
        self._sliding = car is not None
        if sample is not None:
            trust = st.front_trust if sample.bogie == 'front' else st.rear_trust
            self._filter.update(sample, trust)
        self._v, _, accel = self._filter.state()
        self._track_zero(v_before, t_before, now, accel)
        if sample is not None and self._filter.diagnostics() is not None:
            if self._filter.diagnostics().accepted:
                self._v_measured = self._v
        self._t = now
        ds = self._v * dt
        self._distance += ds
        self._x += ds * math.cos(self._yaw)
        self._y += ds * math.sin(self._yaw)
        self._on_standstill(now)
        return self._estimate(t, now)

    def _track_zero(self, v_before: float, t_before: Optional[float], now: float,
                    accel: float) -> None:
        """Remember when the state speed reached 0 and with what deceleration: the linear
        braking from the previous state time crosses 0 at `t_before + v_before / -accel`,
        never later than `now`. The acceleration at rest (the drive model brakes at v = 0)
        is not motion and must not be integrated back from a standing state."""
        if self._v > 0.0:
            self._t_zero = None
        elif self._t_zero is None:
            self._t_zero, self._a_zero = now, min(accel, 0.0)
            if t_before is not None and v_before > 0.0 and accel < 0.0:
                self._t_zero = min(now, t_before + v_before / -accel)

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
        _, var, accel = self._filter.state()
        # the state is at `now`; an input stamped behind it (a wheel lagging the controller,
        # docs/data.md trap 5) is published with the speed at its own stamp (#105). The
        # speed is moved back along the filter acceleration only while the car moves: at
        # rest it is 0 after the moment the state reached 0 and the braking it stopped with
        # before (under brake the acceleration at v = 0 is negative and would invent |a|*lag)
        if self._t_zero is None:
            speed = max(0.0, self._v - accel * (now - t))
        elif t >= self._t_zero:
            speed = 0.0
        else:
            speed = -self._a_zero * (self._t_zero - t)
        if self._tracker is not None:
            speed *= self._tracker.speed_scale     # wheel scale from the stop chain (#153)
        pos_var = var * (now - self._t0) ** 2      # speed noise integrated over the run
        pose = (self._x, self._y, 0.0, self._yaw, (pos_var, pos_var, 0.0))
        # the output is the flat MGRS grid (D-083): the ENU pose of the map, or of the straight
        # line (D-021) in the map's own ENU before the first fix, goes through geodetic -> UTM
        frame = self._line_frame
        if self._tracker is not None:
            on_map = self._tracker.advance(self._distance, self._v)
            pose = pose if on_map is None else on_map
            frame = self._tracker.frame
        x, y, z, yaw, pos_cov = pose if frame is None else pose_to_grid(*frame, self._grid, *pose)
        return Estimate(
            t=t, speed=speed, speed_var=var, accel=accel, accel_model=self._accel_model,
            distance=self._distance, x=x, y=y, z=z, yaw=yaw,
            pos_cov=pos_cov,
            slip=self._slip_state, gnss_used=self._gnss_used,
            position_absolute=frame is not None,   # no map, no fix: local metres (#162)
            filter_diagnostics=self._filter.diagnostics())

    def _on_fix(self, sample: GnssFix) -> None:
        if sample.antenna == 'rover':
            # heading only (trap 15): never the origin of the frame or the anchor
            if self._tracker is not None:
                self._tracker.on_rover(sample.lat, sample.lon, sample.alt, sample.status)
            return
        # the origin of the straight line is the first valid fix, so the start is (0, 0)
        if sample.status >= 0:
            if (self._tracker is None and self._line_frame is None
                    and all(math.isfinite(v) for v in (sample.lat, sample.lon, sample.alt))
                    and abs(sample.lat) + abs(sample.lon) > 0.0):   # lat = lon = 0: trap 10
                self._line_frame = (enu_rotation(sample.lat, sample.lon),
                                    ecef(sample.lat, sample.lon, sample.alt))
                self._x = self._y = 0.0
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
