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
    is a sensor glitch, not real driving, and is dropped rather than fed to the estimator. The
    gate is skipped when the previous or the new sample reads exactly 0: a bogie stuck at 0
    (trap 8) reports the real speed the instant it gets unstuck, and `slip.SlipDetector`
    depends on seeing that exact-0 reading to tell a stuck sensor from real motion — rejecting
    the jump would hide the fault instead of exposing it. A long silent bogie (trap 7, up to
    73 s) needs no special handling here — it simply stops producing samples; the gap is large
    enough that the acceleration implied by whatever speed it reports on return is normally
    small. GNSS fix/vel outside `gnss.init_window_s`
    is dropped (D-005); status filtering and course selection stay in `pipeline` (position is
    not preprocess's job).
    """

    def __init__(self, params: Params):
        self.p = params
        self.t0: Optional[float] = None        # stamp of the first raw input ever seen
        self._last: dict = {}                  # topic -> last accepted stamp (trap 6 gate)
        self._wheel_prev: dict = {}             # bogie -> last accepted WheelSample
        self._clock: Optional[float] = None    # newest accepted stamp over the vehicle streams
        self._jump: Optional[tuple] = None     # (topic, stamp) of an unconfirmed jump ahead
        self._back: dict = {}                  # topic -> stamp far behind its gate, unconfirmed

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
        """Accept a stream sample only if its stamp is newer than the stream's last one and
        not implausibly far from the input clock (#77, D-042).

        Forward: a stamp more than `input.max_stamp_jump_s` ahead of the newest accepted one
        (over the vehicle streams) is a clock glitch unless the very next sample confirms it
        -- another stream near the same stamp, or the same stream moving on from it within the
        limit (a real clock jump, or a stream back after a long silence). Any normal sample
        in between cancels the pending jump.
        Backward: if a jump was accepted anyway (two glitches in a row, or the first sample of
        the bag from the future), the stream's own gate sits in the future and every normal
        sample looks like a stamp from the past. A stamp more than the limit behind the gate,
        followed by a sample of the same stream moving on from it within the limit, resyncs
        the stream there. Ordinary rollbacks (trap 6, up to 3.7 s lag, trap 5) stay dropped.
        """
        jump = self.p.input.max_stamp_jump_s
        last = self._last.get(topic, -math.inf)
        if t <= last:
            back = self._back.get(topic)
            if t < last - jump and back is not None and back < t <= back + jump:
                self._back.pop(topic, None)
                self._last[topic] = t          # resync: the gate was in the future
                self._clock = max(self._last.values())
                self._jump = None
                return True
            if t < last - jump:
                self._back[topic] = t
            return False
        if self._clock is not None and t - self._clock > jump:
            pending = self._jump
            confirmed = (pending is not None and abs(t - pending[1]) <= jump
                         and (pending[0] != topic or t > pending[1]))
            if not confirmed:
                self._jump = (topic, t)
                return False
        self._jump = None                      # accepted: nothing is pending any more
        self._back.pop(topic, None)
        self._last[topic] = t
        self._clock = t if self._clock is None else max(self._clock, t)
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
        # a stuck-at-zero bogie (trap 8) can jump to/from its real speed instantly when it
        # gets unstuck: that is not physical acceleration, so the gate does not apply to it.
        if prev is not None and prev.speed != 0.0 and speed != 0.0:
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
