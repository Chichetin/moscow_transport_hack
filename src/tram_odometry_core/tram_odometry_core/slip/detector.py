"""Slip / sensor-failure detector: how much each bogie can be trusted (docs/contracts.md).

Each bogie is compared with the prediction of the speed filter (`est` moved on by the model
acceleration). Two bogies that agree with each other are both trusted, even against a stale
estimate. When they disagree, a bogie reading exactly 0 is blamed if the moving one agrees with
the prediction or the drive pulls (a start from rest), otherwise the one further from the
prediction. Bogies that are
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
        self._pair_now: Optional[tuple] = None   # latest synchronous (stamp, front, rear)
        self._pair_before: Optional[tuple] = None
        self._pair_pred: Optional[float] = None
        self._pair_noise_active = False

    def update(self, front: Optional[WheelSample], rear: Optional[WheelSample],
               accel_model: float, est: Optional[float]) -> SlipState:
        """`est` is the fused speed (m/s) before this update, None until there is one."""
        p = self._p
        stamps = [s.t for s in (front, rear) if s is not None]
        if not stamps:
            return SlipState(0.0, 0.0, False, False, None)
        t_now = max(stamps)
        new_stamp = self._t_prev is None or t_now > self._t_prev
        dt = 0.0 if self._t_prev is None else max(0.0, t_now - self._t_prev)
        self._t_prev = t_now if self._t_prev is None else max(self._t_prev, t_now)

        live = {n: s for n, s in (('front', front), ('rear', rear))
                if s is not None and t_now - s.t <= p.input.stale_timeout_s}
        trust = {'front': 0.0, 'rear': 0.0}
        slip = {'front': False, 'rear': False}
        tol = p.slip.front_rear_threshold_mps + p.slip.model_residual_threshold_mps2 * dt
        pred = None if est is None else est + accel_model * dt
        if new_stamp:
            self._pair_pred = pred

        if len(live) == 1:
            alive = next(iter(live))
            trust[alive] = 1.0
            dead = 'rear' if alive == 'front' else 'front'
            # a bogie that spoke before and is silent now while the other one talks has failed;
            # a bogie never heard yet (run start) and a pause of both bogies are not flagged
            slip[dead] = (front, rear)[dead == 'rear'] is not None
        elif len(live) == 2:
            f, r = live['front'].speed, live['rear'].speed
            if front.t == rear.t and (self._pair_now is None or front.t > self._pair_now[0]):
                self._pair_before, self._pair_now = self._pair_now, (front.t, f, r)
            pair_plausible = (front.t == rear.t and self._pair_before is not None
                              and abs(0.5 * (f + r) - 0.5 * (self._pair_before[1]
                                                            + self._pair_before[2]))
                              <= p.drive.adhesion_accel_mps2 * (front.t - self._pair_before[0]))
            pair_opposite = (pair_plausible
                             and (f - self._pair_before[1])
                             * (r - self._pair_before[2]) < 0.0)
            if front.t == rear.t:
                if not pair_plausible or abs(f - r) <= tol:
                    self._pair_noise_active = False
                elif (pair_opposite
                      and abs(self._pair_before[1] - self._pair_before[2]) <= tol
                      and self._pair_pred is not None
                      and abs(0.5 * (f + r) - self._pair_pred) <= 0.5 * tol):
                    self._pair_noise_active = True
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
            elif pred is None:
                trust['front'] = trust['rear'] = 0.5
            elif self._pair_noise_active and pair_plausible:
                # Opposite changes with a physically plausible pair midpoint are
                # common-mode motion plus cancelling wheel disturbances. This stays
                # usable even after an earlier bad wheel dragged the estimate away.
                trust['front'] = trust['rear'] = 1.0
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
