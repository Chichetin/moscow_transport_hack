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
from typing import Optional

from ..types import Params, SlipState, WheelSample


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

    def update(self, front: Optional[WheelSample], rear: Optional[WheelSample],
               accel_model: float, est: Optional[float]) -> SlipState:
        """`est` is the fused speed (m/s) before this update, None until there is one."""
        p = self._p
        stamps = [s.t for s in (front, rear) if s is not None]
        if not stamps:
            return SlipState(0.0, 0.0, False, False, None)
        t_now = max(stamps)
        dt = 0.0 if self._t_prev is None else max(0.0, t_now - self._t_prev)
        self._t_prev = t_now if self._t_prev is None else max(self._t_prev, t_now)

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

        frozen = [n for n in live if self._repeat[n][0] >= p.slip.freeze_min_samples
                  and abs(self._repeat[n][1]) > p.slip.freeze_dv_mps]
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
            elif self._jumps(t_now, same=True) and not self._jumps(t_now, same=False):
                slip['front'] = slip['rear'] = True
            elif pred is None or self._jumps(t_now, same=False):
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

    def _jumps(self, t_now: float, same: bool) -> bool:
        """Both bogies jumped within `slip.noise_hold_s`, in the same or in opposite directions."""
        def recent(key):
            t = self._t_jump[key]
            return t is not None and 0.0 <= t_now - t <= self._p.slip.noise_hold_s
        return any(recent(('front', up)) and recent(('rear', up == same)) for up in (True, False))
