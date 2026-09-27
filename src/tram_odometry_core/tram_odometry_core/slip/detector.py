"""Slip / sensor-failure detector: how much each bogie can be trusted (docs/contracts.md).

Each bogie is compared with the prediction of the speed filter (`est` moved on by the model
acceleration). Two bogies that agree with each other are both trusted, even against a stale
estimate. When they disagree, a bogie reading exactly 0 is blamed if the moving one agrees with
the prediction or the drive pulls (a start from rest), otherwise the one further from the
prediction. Bogies that are
silent for longer than `input.stale_timeout_s` get no trust; if the other bogie is talking, the
silent one is flagged as failed.

Jumps (#82, D-054): the fused speed follows whatever the detector trusted, so a prediction built
from it cannot tell noise of both bogies from a slow slip of one. The bogies' own steps can. A
jump is a step whose acceleration differs from the drive model by more than
`slip.noise_accel_mps2` (the vehicle cannot do that); it stays recent for `slip.noise_hold_s`.
While the bogies disagree:
- recent jumps of both in opposite directions are antiphase noise: the mean of the two is right,
  both get equal trust;
- recent jumps of both in the same direction (and none opposite) are a slide or spin of the whole
  vehicle: neither is trusted, the pipeline predicts with the drive model;
- a jump of one bogie only is left to the prediction rule above.

Frozen bogie (#144): a sensor that repeats exactly the same non-zero reading on at least
`slip.freeze_min_samples` new stamps in a row, while the drive model moves the speed by more than
`slip.freeze_dv_mps` over that run, is failed like a silent one: out of the pair rules above, no
trust, flagged. Honest readings are noisy floats (on train 18 runs of 3+ repeats above 1 km/h, the
longest 1.8 s while coasting); exactly 0 is standstill or the dead zone (trap 8), never frozen.
State: the previous update time, the newest sample of each bogie, the stamp of its last jump
up and down, and its current run of repeats; recovery is immediate when a bogie is consistent
again.
"""
import math
from collections import deque
from typing import Optional

from ..types import Params, SlipState, WheelSample

# samples of one bogie kept for the adhesion window: bounded memory (O(1) per message);
# 16 samples at 10 Hz cover 1.6 s, more than slip.adhesion_window_s + input.stale_timeout_s
ADHESION_HISTORY = 16


class SlipDetector:
    def __init__(self, params: Params):
        self._p = params
        self._t_prev: Optional[float] = None    # stamp of the previous update
        self._last = {'front': None, 'rear': None}      # newest sample of each bogie
        # (bogie, jump up?) -> stamp of the last jump of that bogie in that direction
        self._t_jump = {('front', True): None, ('front', False): None,
                        ('rear', True): None, ('rear', False): None}
        # bogie -> (repeats of its newest reading on new stamps, model speed change over them)
        self._repeat = {'front': (0, 0.0), 'rear': (0, 0.0)}
        self._integral = 0.0                    # m/s, accel_model integrated over the update time
        self._accel = 0.0                       # m/s^2, accel_model of the newest update
        # bogie -> recent (stamp, speed, model integral at that stamp)
        self._hist = {'front': deque(maxlen=ADHESION_HISTORY), 'rear': deque(maxlen=ADHESION_HISTORY)}
        self._slide = {'front': None, 'rear': None}     # stamp the bogie lost adhesion at
        self._released = {'front': -math.inf, 'rear': -math.inf}   # stamp it adhered again
        # bogie -> (stamp, newest window residual m/s^2, sample the window started at)
        self._resid = {'front': None, 'rear': None}
        # (stamp, speed, model integral, spin?) of the car when the bogies lost adhesion
        self._car = None

    def update(self, front: Optional[WheelSample], rear: Optional[WheelSample],
               accel_model: float, est: Optional[float],
               state_time: Optional[float] = None) -> SlipState:
        """`est` is the fused speed before this update. `state_time` is the pipeline time;
        direct callers may omit it when the wheels are on the current timeline."""
        p = self._p
        stamps = [s.t for s in (front, rear) if s is not None]
        if not stamps:
            return SlipState(0.0, 0.0, False, False, None)
        t_now = max(stamps)
        if self._t_prev is not None and self._t_prev - t_now > p.input.max_stamp_jump_s:
            # Both wheel streams are back on a new clock timeline (D-043). The model
            # integral and slide anchors belong to the old one; retaining _t_prev would
            # keep dt at zero until the old future stamp was reached again.
            self._t_prev = None
            self._integral = self._accel = 0.0
            self._car = None
            for name in ('front', 'rear'):
                self._hist[name].clear()
                self._slide[name] = self._resid[name] = None
                self._released[name] = -math.inf
        dt = 0.0 if self._t_prev is None else max(0.0, t_now - self._t_prev)
        self._t_prev = t_now if self._t_prev is None else max(self._t_prev, t_now)
        if math.isfinite(accel_model):
            self._integral += accel_model * dt
            self._accel = accel_model
        lagged = (state_time is not None
                  and state_time - t_now > p.input.stale_timeout_s)
        if lagged:
            # The model acceleration belongs to state_time, not to these buffered wheel
            # stamps. A gradual slide inferred from it would restart the filter from a
            # false car speed, especially when the command changed during the lag.
            self._car = None
            for name in ('front', 'rear'):
                self._hist[name].clear()
                self._slide[name] = self._resid[name] = None
                self._released[name] = -math.inf

        live = {n: s for n, s in (('front', front), ('rear', rear))
                if s is not None and t_now - s.t <= p.input.stale_timeout_s}
        trust = {'front': 0.0, 'rear': 0.0}
        slip = {'front': False, 'rear': False}
        tol = p.slip.front_rear_threshold_mps + p.slip.model_residual_threshold_mps2 * dt
        pred = None if est is None else est + accel_model * dt
        for name, s in (('front', front), ('rear', rear)):
            prev = self._last[name]
            if s is not None and prev is not None and prev.t - s.t > p.input.max_stamp_jump_s:
                # the input clock was resynced back (#77, D-043): forget this bogie's past
                prev = self._last[name] = None
                self._t_jump[(name, True)] = self._t_jump[(name, False)] = None
                self._repeat[name] = (0, 0.0)
                self._hist[name].clear()
                self._slide[name] = self._resid[name] = None
                self._released[name] = -math.inf
            if s is None or (prev is not None and s.t <= prev.t):
                continue                    # not a new sample of this bogie
            if prev is not None:
                resid = (s.speed - prev.speed) / (s.t - prev.t) - accel_model
                if abs(resid) > p.slip.noise_accel_mps2:
                    self._t_jump[(name, resid > 0.0)] = s.t
                n, dv = self._repeat[name]
                self._repeat[name] = ((n + 1, dv + accel_model * (s.t - prev.t))
                                      if s.speed == prev.speed and s.speed != 0.0 else (0, 0.0))
            self._last[name] = s
            if not lagged:
                self._adhesion(name, s, accel_model, t_now)
        if self._slide['front'] is None and self._slide['rear'] is None:
            self._car = None

        frozen = [n for n in live if (self._repeat[n][0] >= p.slip.freeze_min_samples
                  and abs(self._repeat[n][1]) > p.slip.freeze_dv_mps) or self._slide[n] is not None]
        for name in frozen:
            del live[name]
            slip[name] = True

        if len(live) == 1:
            alive = next(iter(live))
            trust[alive] = 1.0
            dead = 'rear' if alive == 'front' else 'front'
            # a bogie that spoke before and is silent now while the other one talks has failed;
            # a bogie never heard yet (run start) and a pause of both bogies are not flagged
            slip[dead] = (front, rear)[dead == 'rear'] is not None
        elif len(live) == 2:
            f, r = live['front'].speed, live['rear'].speed
            jumps = self._jump_relation(t_now)
            if abs(f - r) <= tol:
                trust['front'] = trust['rear'] = 1.0
            elif (f == 0.0) != (r == 0.0) and (
                    pred is None or accel_model > 0.0 or abs(max(f, r) - pred) <= tol):
                # exactly 0 next to a moving bogie that agrees with the prediction, or while the
                # drive pulls away from rest (the estimate lags the dead-zone jump), is the
                # observed failure (stuck / dead zone, docs/data.md trap 8). A moving bogie far
                # from the prediction next to an honest 0 without traction is a spike (#76):
                # left to the prediction below
                bad = 'front' if f == 0.0 else 'rear'
                trust['rear' if bad == 'front' else 'front'] = 1.0
                slip[bad] = True
            elif jumps is not None and jumps[0] and (
                    jumps[1] or (pred is not None and min(f - pred, r - pred) > tol)
                    or (pred is not None and max(f - pred, r - pred) < -tol)):
                # The latest jumps point the same way and belong to one near-simultaneous
                # pair, or both readings lie beyond the car prediction on the same side.
                # Staggered jumps are ambiguous in antiphase noise (#82).
                slip['front'] = slip['rear'] = True
            elif pred is None or jumps is not None:
                trust['front'] = trust['rear'] = 0.5
            else:
                dev = {'front': abs(f - pred), 'rear': abs(r - pred)}
                if dev['front'] == dev['rear']:
                    trust['front'] = trust['rear'] = 0.5
                else:
                    bad = 'front' if dev['front'] > dev['rear'] else 'rear'
                    good = 'rear' if bad == 'front' else 'front'
                    trust[good] = 1.0
                    slip[bad] = dev[bad] > tol
        return SlipState(front_trust=trust['front'], rear_trust=trust['rear'],
                         slip_front=slip['front'], slip_rear=slip['rear'], adhesion_est=None)

    def car_speed(self, t: float) -> Optional[float]:
        """Car speed (m/s) at `t` while both bogies are out on a slide of the whole car (#156):
        the start of the slide windows moved on by the model; None otherwise."""
        if self._car is None or self._slide['front'] is None or self._slide['rear'] is None:
            return None
        return max(0.0, self._car[1] + self._integral - self._car[2]
                   + self._accel * (t - self._t_prev))

    def _adhesion(self, name: str, s: WheelSample, accel_model: float, t_now: float) -> None:
        """Adhesion of the car from the bogies' own acceleration over `slip.adhesion_window_s`
        against the mean drive model over the same time (#156). Under traction both bogies
        speeding up faster than the torque allows spin; under braking both slowing down faster
        than the brake allows skid (judged on a pair of samples within `filter.pair_window_s`:
        99.8 % of 30618 samples), and two wheelsets slip apart: bogies that still agree
        jumped together, which is the car or a late burst of the bus (30618_27e994fc 243 s),
        not a slip. One bogie alone is left to the pair rules, and antiphase noise never moves
        both the same way (D-054). A window starting at exactly 0 (the dead zone of trap 8) or
        before the bogie adhered again (its way back) is no evidence.

        The car speed is the start of the two windows (the faster bogie in a skid, the slower
        in a spin: a slide only lowers or only raises a reading) moved on by the model. The
        rail passes no more force than adhesion, so the car speeds up or slows down no faster
        than the model: in a spin the car is at most that speed, in a skid at least. A bogie
        adheres again once its window is back inside both limits and its speed is back within
        `front_rear_threshold_mps` of the car speed on the slide side and within that plus
        `readhesion_accel_mps2 * t` (the drift of the model over t) on the other."""
        p = self._p.slip
        hist = self._hist[name]
        integral = self._integral
        if math.isfinite(accel_model):
            integral -= accel_model * (t_now - s.t)
        hist.append((s.t, s.speed, integral))
        start = None
        for sample in reversed(hist):          # bounded: at most ADHESION_HISTORY entries
            if s.t - sample[0] >= p.adhesion_window_s:
                start = sample
                break
        if start is None or s.t - start[0] > p.adhesion_window_s + self._p.input.stale_timeout_s:
            return
        span = s.t - start[0]
        model = (integral - start[2]) / span
        resid = (s.speed - start[1]) / span - model
        self._resid[name] = (s.t, resid, start)
        other_name = 'rear' if name == 'front' else 'front'
        other = self._resid[other_name]
        if self._slide[name] is None:
            if (self._slide[other_name] is None and other is not None
                    and abs(s.t - other[0]) <= self._p.filter.pair_window_s
                    and start[1] != 0.0 and other[2][1] != 0.0
                    and start[0] >= self._released[name]
                    and other[2][0] >= self._released[other_name]
                    and abs(s.speed - self._last[other_name].speed) > p.front_rear_threshold_mps):
                spin = (model > p.adhesion_min_accel_mps2
                        and min(resid, other[1]) > p.spin_accel_mps2)
                skid = (model < -p.adhesion_min_accel_mps2
                        and max(resid, other[1]) < -p.skid_accel_mps2)
                if spin or skid:
                    self._slide = {'front': s.t, 'rear': s.t}
                    car = (min if spin else max)((start, other[2]), key=lambda x: x[1])
                    self._car = car + (spin,)
            return
        t_car, v_car, i_car, spin = self._car
        car = max(0.0, v_car + integral - i_car)
        near = p.front_rear_threshold_mps
        far = near + p.readhesion_accel_mps2 * (s.t - t_car)
        low, high = (car - far, car + near) if spin else (car - near, car + far)
        if -p.skid_accel_mps2 <= resid <= p.spin_accel_mps2 and low <= s.speed <= high:
            self._slide[name] = None
            self._released[name] = s.t

    def _jump_relation(self, t_now: float):
        """(same direction, paired stamps) for each bogie's latest recent jump, if both exist."""
        def latest(name):
            up, down = self._t_jump[(name, True)], self._t_jump[(name, False)]
            if up is None and down is None:
                return None
            direction = down is None or (up is not None and up > down)
            t = up if direction else down
            return ((t, direction) if 0.0 <= t_now - t <= self._p.slip.noise_hold_s else None)
        front, rear = latest('front'), latest('rear')
        if front is None or rear is None:
            return None
        return (front[1] == rear[1],
                abs(front[0] - rear[0]) <= self._p.filter.pair_window_s)
