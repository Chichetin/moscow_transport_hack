"""Single entry point used by the ROS node and tools/eval (docs/contracts.md, section 2).

Baseline (D-020): mean of the two bogie speeds, path by integration, straight-line dead
reckoning along the heading taken from GNSS during the init window. The modules that
follow replace the parts of it; the numbers of this version are the first row of the table.
"""
import math
from typing import Any, Optional

from .types import Estimate, Params, SlipState

FRONT_TOPIC = '/vehicle/front_bogie_velocity'
REAR_TOPIC = '/vehicle/rear_bogie_velocity'
CMD_TOPIC = '/vehicle/driver_position_cmd'


def _stamp(msg) -> float:
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


class Odometry:
    """`raw` is a `(topic, message)` pair; message fields are those of the ROS message."""

    def __init__(self, params: Params, route=None):
        self.params = params
        self.route = route
        self._t0: Optional[float] = None      # stamp of the first input
        self._t: Optional[float] = None       # newest stamp seen; the state is at this time
        self._last = {}                       # stream topic -> newest accepted stamp
        self._speed = {FRONT_TOPIC: None, REAR_TOPIC: None}   # last bogie speed, m/s
        self._speed_t = {FRONT_TOPIC: -math.inf, REAR_TOPIC: -math.inf}
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
            topic, msg = raw
            t = _stamp(msg)
            if not math.isfinite(t):
                return None
            if self._t0 is None:
                self._t0 = t
            if topic == FRONT_TOPIC or topic == REAR_TOPIC:
                return self._on_wheel(topic, t, msg.velocity)
            if topic == CMD_TOPIC:
                return self._on_stream(topic, t)
            if topic == self.params.gnss.topic_fix:
                self._on_fix(t, msg)
            elif topic == self.params.gnss.topic_vel:
                self._on_vel(t, msg)
            return None
        except (TypeError, ValueError, AttributeError):
            return None

    def _fresh(self, topic: str, t: float) -> bool:
        """Accept a stream sample only if its stamp is newer than the stream's last one."""
        if t <= self._last.get(topic, -math.inf):
            return False
        self._last[topic] = t
        return True

    def _on_wheel(self, topic: str, t: float, kmh) -> Optional[Estimate]:
        p = self.params
        if not (isinstance(kmh, (int, float)) and math.isfinite(kmh)) or kmh < 0.0:
            return None
        if not self._fresh(topic, t):
            return None
        scale = p.vehicle.wheel_scale_front if topic == FRONT_TOPIC else p.vehicle.wheel_scale_rear
        self._speed[topic] = kmh * p.input.wheel_speed_scale * scale   # the only km/h -> m/s
        self._speed_t[topic] = t
        return self._advance(t)

    def _on_stream(self, topic: str, t: float) -> Optional[Estimate]:
        if not self._fresh(topic, t):
            return None
        return self._advance(t)

    def _advance(self, t: float) -> Estimate:
        now = t if self._t is None else max(self._t, t)
        fresh = [v for k, v in self._speed.items()
                 if v is not None and now - self._speed_t[k] <= self.params.input.stale_timeout_s]
        if fresh:
            self._v = sum(fresh) / len(fresh)
        # both bogies silent: keep the last speed
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
            slip=SlipState(1.0, 1.0, False, False, None), gnss_used=self._gnss_used)

    def _in_window(self, t: float) -> bool:
        return t - self._t0 <= self.params.gnss.init_window_s

    def _on_fix(self, t: float, msg) -> None:
        # the origin of frame `map` is the first valid fix, so the start is (0, 0)
        if self._in_window(t) and msg.status.status >= 0:
            self._fix_ok = True

    def _on_vel(self, t: float, msg) -> None:
        if not (self._fix_ok and self._in_window(t)):
            return
        ve, vn = msg.twist.linear.x, msg.twist.linear.y
        if not (math.isfinite(ve) and math.isfinite(vn)):
            return
        speed = math.hypot(ve, vn)
        if speed > self._vel_best:      # heading is only defined while moving
            self._vel_best = speed
            self._yaw = math.atan2(vn, ve)
            self._gnss_used = True
