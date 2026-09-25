"""Single entry point used by the ROS node and tools/eval (docs/contracts.md, section 2).

Baseline (D-021): mean of the two bogie speeds, path by integration, straight-line dead
reckoning along the heading taken from GNSS during the init window. The modules that
follow replace the parts of it; the numbers of this version are the first row of the table.
"""
import math
from typing import Any, Optional

from .preprocess import Preprocessor
from .slip import SlipDetector
from .types import CommandSample, Estimate, GnssFix, GnssVel, Params, SlipState, WheelSample


class Odometry:
    """`raw` is a `(topic, message)` pair; message fields are those of the ROS message."""

    def __init__(self, params: Params, route=None):
        self.params = params
        self.route = route
        self._preprocess = Preprocessor(params)
        self._t0: Optional[float] = None      # stamp of the first input (from preprocess)
        self._t: Optional[float] = None       # newest stamp seen; the state is at this time
        self._wheel = {'front': None, 'rear': None}   # last WheelSample per bogie, m/s
        self._slip = SlipDetector(params)
        self._slip_state = SlipState(1.0, 1.0, False, False, None)
        self._v = 0.0                         # current speed, m/s
        self._distance = 0.0
        self._x = self._y = 0.0
        self._yaw = 0.0
        self._gnss_used = False
        self._fix_ok = False                  # valid master fix seen in the window
        self._vel_best = 0.0                  # fastest GNSS speed seen in the window

    def step(self, raw: Any) -> Optional[Estimate]:
        """Consume one raw input; None means the input was dropped, nothing to publish."""
        try:
            sample = self._preprocess.accept(raw)
            self._t0 = self._preprocess.t0
            if isinstance(sample, WheelSample):
                self._wheel[sample.bogie] = sample
                return self._advance(sample.t)
            if isinstance(sample, CommandSample):
                return self._advance(sample.t)
            if isinstance(sample, GnssFix):
                self._on_fix(sample)
            elif isinstance(sample, GnssVel):
                self._on_vel(sample)
            return None
        except (TypeError, ValueError, AttributeError):
            return None

    def _advance(self, t: float) -> Estimate:
        now = t if self._t is None else max(self._t, t)
        front, rear = self._wheel['front'], self._wheel['rear']
        est = self._v if self._t is not None else None
        st = self._slip.update(front, rear, 0.0, est)    # no drive model yet: accel_model = 0
        self._slip_state = st
        used = [(w, s.speed) for w, s in ((st.front_trust, front), (st.rear_trust, rear))
                if s is not None and w > 0.0]
        if used:
            self._v = sum(w * v for w, v in used) / sum(w for w, _ in used)
        # no trusted bogie: keep the last speed
        dt = 0.0 if self._t is None else now - self._t
        self._t = now
        ds = self._v * dt
        self._distance += ds
        self._x += ds * math.cos(self._yaw)
        self._y += ds * math.sin(self._yaw)
        return self._estimate(t, now)

    def _estimate(self, t: float, now: float) -> Estimate:
        var = self.params.filter.r_wheel
        pos_var = var * (now - self._t0) ** 2      # speed noise integrated over the run
        return Estimate(
            t=t, speed=self._v, speed_var=var, accel=0.0, accel_model=0.0,
            distance=self._distance, x=self._x, y=self._y, yaw=self._yaw,
            pos_cov=(pos_var, pos_var, 0.0),
            slip=self._slip_state, gnss_used=self._gnss_used)

    def _on_fix(self, sample: GnssFix) -> None:
        # the origin of frame `map` is the first valid fix, so the start is (0, 0)
        if sample.status >= 0:
            self._fix_ok = True

    def _on_vel(self, sample: GnssVel) -> None:
        if not self._fix_ok:
            return
        speed = math.hypot(sample.ve, sample.vn)
        if speed > self._vel_best:      # heading is only defined while moving
            self._vel_best = speed
            self._yaw = math.atan2(sample.vn, sample.ve)
            self._gnss_used = True
