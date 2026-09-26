"""Input validation: units (km/h -> m/s), stamps, gaps, outliers, stale sensors.
Owner: area:core-preprocess (docs/contracts.md section 2; docs/data.md traps 5-7, 9).
"""
import math
from typing import Any, Optional

from ..types import CommandSample, GnssFix, GnssVel, Params, Sample, WheelSample

FRONT_TOPIC = '/vehicle/front_bogie_velocity'
REAR_TOPIC = '/vehicle/rear_bogie_velocity'
CMD_TOPIC = '/vehicle/driver_position_cmd'
ROVER_FIX_TOPIC = '/sensing/gnss/rover/fix'   # contract §1; heading at a standstill (trap 15)


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
    is a sensor glitch, not real driving, and is dropped rather than fed to the estimator; a
    speed above `input.max_wheel_speed_mps` is dropped always, the first sample too (#73). A
    drop to exactly 0 is let through, and so is a jump from exactly 0 when the other bogie
    (fresh) reads the same speed: a bogie stuck at 0 (trap 8) reports the real speed the
    instant it gets unstuck, and `slip.SlipDetector` depends on seeing both to tell a stuck
    sensor from real motion. A jump from 0 that the other bogie does not confirm is a glitch
    at standstill (#73). A negative reading (slow roll-back or noise around standstill) is
    0 m/s, not dropped (#111, D-064), but not a bogie stuck at 0: the acceleration gate
    applies to it, so in motion it is a glitch. A long silent bogie (trap 7, up to
    73 s) needs no special handling here — it simply stops producing samples; the gap is large
    enough that the acceleration implied by whatever speed it reports on return is normally
    small. GNSS fix/vel of master and the rover fix outside `gnss.init_window_s`
    are dropped (D-005); status filtering and course selection stay in `pipeline` (position is
    not preprocess's job).
    """

    def __init__(self, params: Params):
        self.p = params
        self.t0: Optional[float] = None        # GNSS window start: earliest trusted input stamp
        self._t0_back: Optional[float] = None  # stamp far behind t0, unconfirmed (#80)
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
        self._window_start(t, topic in (FRONT_TOPIC, REAR_TOPIC, CMD_TOPIC))
        if topic == FRONT_TOPIC or topic == REAR_TOPIC:
            return self._wheel(topic, t, msg)
        if topic == CMD_TOPIC:
            return self._command(topic, t, msg)
        if topic == self.p.gnss.topic_fix:
            return self._fix(t, msg, 'master')
        if topic == ROVER_FIX_TOPIC:
            return self._fix(t, msg, 'rover')
        if topic == self.p.gnss.topic_vel:
            return self._vel(t, msg)
        return None

    def _window_start(self, t: float, vehicle: bool) -> None:
        """Keep `t0` at the earliest trusted input stamp (#80, D-055).

        `t0` starts at the first raw input of any topic. An earlier vehicle stamp moves it back:
        within `input.max_stamp_jump_s` at once (buffered inputs, trap 5), farther only when the
        next vehicle input behind `t0` confirms it within the limit -- the first stamp was from
        the future. A single stamp far in the past is a glitch; any input at or after `t0`
        cancels it. GNSS never moves `t0` back: the fixes and velocity of one epoch share a
        stamp and would confirm their own glitch, closing the window before the alignment.
        The window can only close earlier than the real start + window, never later (D-005).
        """
        if self.t0 is None or t >= self.t0:
            self.t0 = t if self.t0 is None else self.t0
            self._t0_back = None
            return
        if not vehicle:
            return
        jump = self.p.input.max_stamp_jump_s
        back = self._t0_back
        confirmed = back is not None and abs(t - back) <= jump
        if confirmed or self.t0 - t <= jump:
            self.t0 = min(t, back) if confirmed else t
            self._t0_back = None
        else:
            self._t0_back = t

    def _fresh(self, topic: str, t: float) -> bool:
        """Accept a stream sample only if its stamp is newer than the stream's last one and
        not implausibly far from the input clock (#77, D-043).

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
        if not _finite_number(kmh):
            return None
        if not self._fresh(topic, t):
            return None
        # A negative reading is a slow roll-back or noise around standstill (#111, D-064):
        # 0 m/s, not a drop -- dropping keeps only the positive half of the noise.
        negative = kmh < 0.0
        bogie = 'front' if topic == FRONT_TOPIC else 'rear'
        scale = (self.p.vehicle.wheel_scale_front if bogie == 'front'
                 else self.p.vehicle.wheel_scale_rear)
        speed = kmh * self.p.input.wheel_speed_scale * scale     # the only km/h -> m/s site
        if negative:
            speed = 0.0
        if speed > self.p.input.max_wheel_speed_mps:
            return None                        # beyond any tram speed: glitch (also the first)
        prev = self._wheel_prev.get(bogie)
        if prev is not None and (speed != 0.0 or negative):
            dt = t - prev.t
            if dt > 0 and abs(speed - prev.speed) / dt > self.p.input.max_wheel_accel_mps2:
                # Faster than physically possible. The one exception is a bogie stuck at
                # exactly 0 (trap 8) that jumps back to the real speed when it gets unstuck
                # -- the slip detector needs that reading (D-027). It is told from a glitch
                # at standstill (#73) by the other bogie: fresh and reading the same speed.
                # A clamped negative is not a stuck bogie: in motion it is a glitch.
                if negative or prev.speed != 0.0 or not self._other_agrees(bogie, t, speed):
                    return None
        sample = WheelSample(t=t, bogie=bogie, speed=speed)
        self._wheel_prev[bogie] = sample
        return sample

    def _other_agrees(self, bogie: str, t: float, speed: float) -> bool:
        other = self._wheel_prev.get('rear' if bogie == 'front' else 'front')
        return (other is not None and abs(t - other.t) <= self.p.input.stale_timeout_s
                and abs(speed - other.speed) <= self.p.slip.front_rear_threshold_mps)

    def _command(self, topic: str, t: float, msg) -> Optional[CommandSample]:
        notch = msg.position
        if isinstance(notch, bool) or not isinstance(notch, int):
            return None
        if not self._fresh(topic, t):
            return None
        return CommandSample(t=t, notch=notch)

    def _in_window(self, t: float) -> bool:
        """GNSS inside `gnss.init_window_s` from `t0`; a stamp far behind `t0` is a glitch."""
        return self.t0 - self.p.input.max_stamp_jump_s <= t <= self.t0 + self.p.gnss.init_window_s

    def _fix(self, t: float, msg, antenna: str) -> Optional[GnssFix]:
        if not self._in_window(t):
            return None
        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
        if not (_finite_number(lat) and _finite_number(lon) and _finite_number(alt)):
            return None
        return GnssFix(t=t, antenna=antenna, lat=lat, lon=lon, alt=alt,
                       status=msg.status.status)

    def _vel(self, t: float, msg) -> Optional[GnssVel]:
        if not self._in_window(t):
            return None
        ve, vn = msg.twist.linear.x, msg.twist.linear.y
        if not (_finite_number(ve) and _finite_number(vn)):
            return None
        return GnssVel(t=t, ve=ve, vn=vn)
