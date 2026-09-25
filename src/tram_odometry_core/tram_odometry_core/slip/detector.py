"""Slip / sensor-failure detector: how much each bogie can be trusted (docs/contracts.md).

Each bogie is compared with the prediction of the speed filter (`est` moved on by the model
acceleration). Two bogies that agree with each other are both trusted, even against a stale
estimate. When they disagree, a bogie reading exactly 0 is blamed, otherwise the one further
from the prediction. Bogies that are
silent for longer than `input.stale_timeout_s` get no trust; if the other bogie is talking, the
silent one is flagged as failed.
Stateless apart from the previous update time; recovery is immediate when a bogie is consistent again.
"""
from typing import Optional

from ..types import Params, SlipState, WheelSample


class SlipDetector:
    def __init__(self, params: Params):
        self._p = params
        self._t_prev: Optional[float] = None    # stamp of the previous update

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
            elif (f == 0.0) != (r == 0.0):
                # exactly 0 next to a moving bogie is the observed failure (stuck / dead zone,
                # docs/data.md trap 8) and the estimate may still be at rest: no need for it
                bad = 'front' if f == 0.0 else 'rear'
                trust['rear' if bad == 'front' else 'front'] = 1.0
                slip[bad] = True
            elif pred is None:
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
