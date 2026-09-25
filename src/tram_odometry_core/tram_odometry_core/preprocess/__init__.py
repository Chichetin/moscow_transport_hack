"""Input validation: units (km/h -> m/s), stamps, gaps, outliers, stale sensors.
Owner: area:core-preprocess (docs/contracts.md section 2; docs/data.md traps 5-7, 9).
"""
import math
from typing import Any, Optional

from ..types import CommandSample, GnssFix, GnssVel, Params, Sample, WheelSample

FRONT_TOPIC = '/vehicle/front_bogie_velocity'
REAR_TOPIC = '/vehicle/rear_bogie_velocity'
CMD_TOPIC = '/vehicle/driver_position_cmd'


def _stamp(msg) -> float:
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


def _finite_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


class Preprocessor:
    """Normalizes one raw `(topic, message)` pair into a `Sample`, or drops it (`None`).

    Time is `header.stamp` only, never arrival order (trap 5: buffered messages lag up to
    3.7 s at the start of a run). Each stream (front/rear/cmd) has its own monotonic stamp
    gate (trap 6: up to 6-7 rollbacks on a bogie, up to 20 on the 20 Hz controller — trap 9;
    a repeated or past stamp is dropped, not integrated with `dt < 0`). Wheel speed is
    converted km/h -> m/s here and nowhere else (D-003), then gated by
    `input.max_wheel_accel_mps2`: a jump implying a higher `|dv/dt|` than physically possible
    is a sensor glitch, not real driving, and is dropped rather than fed to the estimator. A
    long silent bogie (trap 7, up to 73 s) needs no special handling here — it simply stops
    producing samples; the gap is large enough that the acceleration implied by whatever
    speed it reports on return is normally small. GNSS fix/vel outside `gnss.init_window_s`
    is dropped (D-005); status filtering and course selection stay in `pipeline` (position is
    not preprocess's job).
    """

    def __init__(self, params: Params):
        self.p = params
        self.t0: Optional[float] = None        # stamp of the first raw input ever seen
        self._last: dict = {}                  # topic -> last accepted stamp (trap 6 gate)
        self._wheel_prev: dict = {}             # bogie -> last accepted WheelSample

    def accept(self, raw: Any) -> Optional[Sample]:
        topic, msg = raw
        t = _stamp(msg)
        if not math.isfinite(t):
            return None
        if self.t0 is None:
            self.t0 = t
        if topic == FRONT_TOPIC or topic == REAR_TOPIC:
            return self._wheel(topic, t, msg)
        if topic == CMD_TOPIC:
            return self._command(topic, t, msg)
        if topic == self.p.gnss.topic_fix:
            return self._fix(t, msg)
        if topic == self.p.gnss.topic_vel:
            return self._vel(t, msg)
        return None

    def _fresh(self, topic: str, t: float) -> bool:
        """Accept a stream sample only if its stamp is newer than the stream's last one."""
        if t <= self._last.get(topic, -math.inf):
            return False
        self._last[topic] = t
        return True

    def _wheel(self, topic: str, t: float, msg) -> Optional[WheelSample]:
        kmh = msg.velocity
        if not _finite_number(kmh) or kmh < 0.0:
            return None
        if not self._fresh(topic, t):
            return None
        bogie = 'front' if topic == FRONT_TOPIC else 'rear'
        scale = (self.p.vehicle.wheel_scale_front if bogie == 'front'
                 else self.p.vehicle.wheel_scale_rear)
        speed = kmh * self.p.input.wheel_speed_scale * scale     # the only km/h -> m/s site
        prev = self._wheel_prev.get(bogie)
        if prev is not None:
            dt = t - prev.t
            if dt > 0 and abs(speed - prev.speed) / dt > self.p.input.max_wheel_accel_mps2:
                return None                    # faster than physically possible: sensor glitch
        sample = WheelSample(t=t, bogie=bogie, speed=speed)
        self._wheel_prev[bogie] = sample
        return sample

    def _command(self, topic: str, t: float, msg) -> Optional[CommandSample]:
        notch = msg.position
        if isinstance(notch, bool) or not isinstance(notch, int):
            return None
        if not self._fresh(topic, t):
            return None
        return CommandSample(t=t, notch=notch)

    def _fix(self, t: float, msg) -> Optional[GnssFix]:
        if t - self.t0 > self.p.gnss.init_window_s:
            return None
        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
        if not (_finite_number(lat) and _finite_number(lon) and _finite_number(alt)):
            return None
        return GnssFix(t=t, antenna='master', lat=lat, lon=lon, alt=alt,
                       status=msg.status.status)

    def _vel(self, t: float, msg) -> Optional[GnssVel]:
        if t - self.t0 > self.p.gnss.init_window_s:
            return None
        ve, vn = msg.twist.linear.x, msg.twist.linear.y
        if not (_finite_number(ve) and _finite_number(vn)):
            return None
        return GnssVel(t=t, ve=ve, vn=vn)
