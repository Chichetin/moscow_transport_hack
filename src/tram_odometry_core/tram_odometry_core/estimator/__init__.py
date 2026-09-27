"""Causal speed and acceleration-bias filter; wheel input is calibrated SI."""
from typing import Optional

import numpy as np

from ..types import FilterDiagnostics, Params, WheelSample

COV_EPS = 1e-8  # (m/s)^2: numerical margin that puts an inflated innovation strictly inside the gate


class SpeedFilter:
    """Kalman state [speed, acceleration bias] driven by the drive model.

    The first trusted wheel initializes speed without a zero-speed prior. Each
    bogie contributes at most once per stamp; a fresh delayed sample is compared
    with the predicted state at its stamp without rolling the state backward.

    Both bogies ride on one car body, so a wheel sample is measured together with
    the other bogie's last accepted reading (if it is live on the wheel timeline):
    the trust-weighted mean of the pair while moving, the smaller one at rest
    without traction (#105, D-061).

    Without a fresh trusted wheel for longer than `input.stale_timeout_s` (both bogies silent,
    frozen or out) the bias decays to 0 with `filter.bias_decay_s` (#176, D-090): it was
    learned for the grade and the table error of the moment before the pause and is not
    carried over the pause. A wheel rejected by the NIS gate still counts as heard.
    """

    def __init__(self, params: Params):
        self._p = params.filter
        self._stale_timeout = params.input.stale_timeout_s
        self._max_wheel_accel = params.input.max_wheel_accel_mps2
        self._rest_speed = max(params.position.stop_speed_mps, self._p.bias_release_speed_mps)
        self._stop_speed = params.position.stop_speed_mps
        self._jump_accel = params.slip.noise_accel_mps2
        self._jump_hold = params.slip.noise_hold_s
        self._restart_var = params.slip.front_rear_threshold_mps ** 2
        if (self._p.q_accel < 0.0 or self._p.r_wheel <= 0.0
                or self._p.q_bias < 0.0 or self._p.initial_bias_var < 0.0
                or self._p.nis_gate <= 0.0):
            raise ValueError('invalid filter covariance or NIS threshold')
        self._t = None
        self._last = {'front': None, 'rear': None}
        self._latest_wheel_t = None
        self._x = np.zeros(2, dtype=float)
        self._cov = np.diag((self._p.r_wheel, self._p.initial_bias_var))
        self._model_accel = 0.0
        self._initialized = False
        self._diagnostic = None
        self._scale_delta = 0.0
        self._recent_wheel = {'front': None, 'rear': None}
        self._zero_wheel = {'front': None, 'rear': None}
        self._confirmed_stop_t = None
        self._pair = {'front': None, 'rear': None}  # last accepted (t, speed m/s, trust)
        self._raw = {'front': None, 'rear': None}   # last fresh (t, speed m/s), any trust
        self._jump_t = {'front': None, 'rear': None}  # stamp of the bogie's last jump
        self._t_heard = None                          # state time of the last fresh trusted wheel
        self._t_rest = None                           # state time the speed reached 0; None moving

    def rebase_time(self, t: float):
        """Keep the estimate but start a fresh input clock after a confirmed jump."""
        self._t = t
        self._last = {'front': None, 'rear': None}
        self._latest_wheel_t = None
        self._recent_wheel = {'front': None, 'rear': None}
        self._zero_wheel = {'front': None, 'rear': None}
        self._confirmed_stop_t = None
        self._pair = {'front': None, 'rear': None}
        self._raw = {'front': None, 'rear': None}
        self._jump_t = {'front': None, 'rear': None}
        self._t_heard = None if self._t_heard is None else t
        self._t_rest = None if self._t_rest is None else t
        self._diagnostic = None

    def predict(self, t: float, accel_model: float):
        self._diagnostic = None
        if not np.isfinite(t) or not np.isfinite(accel_model):
            return
        if self._t is not None and t <= self._t:
            return
        dt = 0.0 if self._t is None else t - self._t
        self._t = t
        self._model_accel = accel_model
        # the part of the step past stale_timeout_s without a trusted wheel: the bias is a
        # Gauss-Markov process there (D-090), a random walk while wheels are heard. It decays
        # before the speed moves, so the step runs on the acceleration state() reports
        quiet = (0.0 if self._t_heard is None
                 else min(dt, t - self._t_heard - self._stale_timeout))
        decay = float(np.exp(-quiet / self._p.bias_decay_s)) if quiet > 0.0 else 1.0
        self._x[1] *= decay
        slope = accel_model + self._x[1]
        speed = self._x[0] + slope * dt
        if speed <= 0.0 and self._t_rest is None:
            # the linear motion stops inside this step; a late wheel is measured up to there
            self._t_rest = t - dt + (self._x[0] / -slope if slope < 0.0 else 0.0)
        self._x[0] = max(0.0, speed)
        if self._x[0] > 0.0:
            self._t_rest = None
        transition = np.array(((1.0, dt * decay), (0.0, decay)))
        process_noise = np.array((
            (self._p.q_accel * dt + self._p.q_bias * dt ** 3 / 3.0,
             self._p.q_bias * dt ** 2 / 2.0),
            (self._p.q_bias * dt ** 2 / 2.0, self._p.q_bias * dt),
        ))
        self._cov = transition @ self._cov @ transition.T + process_noise

    def update(self, sample: WheelSample, trust: float, partner_trust: Optional[float] = None):
        """`partner_trust`: the detector's current trust in the other bogie (#176); None keeps
        the trust the other bogie's last reading was fused with."""
        self._diagnostic = None
        if (sample.bogie not in self._last or not np.isfinite(sample.t)
                or not np.isfinite(sample.speed) or sample.speed < 0.0
                or not np.isfinite(trust) or not 0.0 <= trust <= 1.0):
            return
        # Controller stamps can lead both live wheel streams by several seconds.
        # Freshness is measured on the wheel timeline, not against controller time.
        if (self._latest_wheel_t is not None
                and self._latest_wheel_t - sample.t > self._stale_timeout):
            return
        last = self._last[sample.bogie]
        if last is not None and sample.t <= last:
            return
        if sample.speed == 0.0:
            self._zero_wheel[sample.bogie] = sample.t
        raw = self._raw[sample.bogie]
        if (raw is not None and sample.t > raw[0]
                and abs((sample.speed - raw[1]) / (sample.t - raw[0])
                        - self._model_accel) > self._jump_accel):
            # a step the car cannot make (the detector's jump, D-054); an untrusted sample
            # does not advance the stamp gate above, so the same stamp may come again
            self._jump_t[sample.bogie] = sample.t
        self._raw[sample.bogie] = (sample.t, sample.speed)
        if trust == 0.0:
            # the detector does not trust this bogie now: its last reading gets no weight
            # in the pair mean while moving
            last_pair = self._pair[sample.bogie]
            if last_pair is not None:
                self._pair[sample.bogie] = (last_pair[0], last_pair[1], 0.0)
            return
        self.predict(sample.t, self._model_accel)
        self._last[sample.bogie] = sample.t
        self._t_heard = self._t
        self._latest_wheel_t = (sample.t if self._latest_wheel_t is None
                                else max(self._latest_wheel_t, sample.t))
        age = self._t - sample.t
        before_rest = self._t_rest is not None and sample.t < self._t_rest
        if self._t_rest is not None:
            # the state stands since t_rest: its speed was linear in time only up to there, a
            # wheel from before is compared with that line, a later one with rest (#176)
            age = min(age, max(0.0, self._t_rest - sample.t))
        scale = 1.0 - self._scale_delta if sample.bogie == 'front' else 1.0 + self._scale_delta
        own = sample.speed / scale
        measurement = (self._pair_speed(sample, own, trust, partner_trust)
                       + self._model_accel * age)
        measurement_var = self._p.r_wheel / (trust * scale ** 2) + self._p.q_accel * age
        plausible_departure = (
            self._confirmed_stop_t is not None and sample.speed > 0.0
            and sample.speed <= (self._max_wheel_accel * max(0.0, sample.t - self._confirmed_stop_t)
                                  + self._p.departure_slack_mps))
        if plausible_departure:
            # Velocity at the next departure is a new motion segment. The
            # braking posterior should not slow the first plausible wheel.
            self._cov[0, 0] = max(self._cov[0, 0], self._p.departure_var_factor * measurement_var)
        if not self._initialized:
            self._x[0] = max(0.0, measurement)
            self._t_rest = None if self._x[0] > 0.0 else self._t
            self._cov = np.diag((measurement_var, self._p.initial_bias_var))
            self._initialized = True
            self._diagnostic = FilterDiagnostics(sample.t, sample.bogie, 0.0, True)
            self._pair[sample.bogie] = (sample.t, own, trust)
            self._observe_scale(sample, trust)
            return

        observation = np.array((1.0, -age))
        innovation = measurement - observation @ self._x
        projected = self._cov @ observation
        innovation_var = float(observation @ projected + measurement_var)
        other = 'rear' if sample.bogie == 'front' else 'front'
        other_zero = self._zero_wheel[other]
        paired_zero = (sample.speed == 0.0 and other_zero is not None
                       and abs(sample.t - other_zero) <= self._p.pair_window_s)
        if paired_zero and innovation ** 2 > self._p.nis_gate * innovation_var:
            # Two independent zero readings expose a real stop that a smooth
            # motion prior cannot explain. Inflate speed uncertainty so the
            # ordinary NIS gate can accept the corroborated measurement.
            self._cov[0, 0] += (innovation ** 2 / self._p.nis_gate
                                - innovation_var + COV_EPS)
            projected = self._cov @ observation
            innovation_var = float(observation @ projected + measurement_var)
        nis = float(innovation ** 2 / innovation_var)
        accepted = nis <= self._p.nis_gate
        self._diagnostic = FilterDiagnostics(sample.t, sample.bogie, nis, accepted)
        if not accepted:
            return
        self._pair[sample.bogie] = (sample.t, own, trust)
        if paired_zero:
            self._confirmed_stop_t = sample.t
        elif sample.speed > 0.0:
            self._confirmed_stop_t = None
        gain = projected / innovation_var
        self._x += gain * innovation
        self._x[0] = max(0.0, self._x[0])
        if before_rest:
            self._x[0] = 0.0    # it corrects the braking line, not the rest after it
        if self._x[0] > 0.0:
            self._t_rest = None
        elif self._t_rest is None:
            self._t_rest = self._t
        residual = np.eye(2) - np.outer(gain, observation)
        self._cov = (residual @ self._cov @ residual.T
                     + np.outer(gain, gain) * measurement_var)
        self._cov = (self._cov + self._cov.T) / 2.0
        if (sample.speed < self._rest_speed and self._x[0] < self._rest_speed
                and self._model_accel <= 0.0 and self._x[1] < 0.0):
            # At a confirmed stop, braking cannot keep accumulating negative
            # velocity. Reset the unobservable bias before the next departure.
            self._x[1] = 0.0
            self._cov[0, 1] = self._cov[1, 0] = 0.0
            self._cov[1, 1] = max(self._cov[1, 1], self._p.initial_bias_var)
        self._observe_scale(sample, trust)

    def _pair_speed(self, sample: WheelSample, own: float, trust: float,
                    partner_trust: Optional[float]) -> float:
        """Car speed at `sample.t` from this bogie and the other bogie's last reading."""
        other = self._pair['rear' if sample.bogie == 'front' else 'front']
        if other is not None and partner_trust is not None:
            # the trust the partner was fused with compared it with an older reading of this
            # bogie; the detector's current trust has seen the pair as it is now (#176)
            other = (other[0], other[1], partner_trust)
        if (not self._initialized or other is None
                or abs(sample.t - other[0]) > self._stale_timeout):
            return own       # the other bogie is silent: single-bogie mode
        jump = self._jump_t[sample.bogie]
        if (self._x[0] < self._stop_speed and self._model_accel < 0.0
                and jump is not None and sample.t - jump <= self._jump_hold):
            # The car stands, the drive model decelerates and this bogie has just jumped:
            # no more than the slower bogie shows is motion. At rest the negative half of
            # noise arrives only as 0 (preprocess clamps kmh < 0, D-064), so a filter fed the
            # jumping bogie alone rides its positive half and drives off. A smooth rise of
            # one bogie is a start (a bogie stuck at 0 is trap 8, the notch can lag or read
            # brake at a start), and both bogies moving move the car whatever the controller
            # says.
            return min(own, other[1])
        if trust == 1.0 and other[2] == 1.0:
            return own       # agreeing bogies: independent readings, fused one by one
        # The detector saw the bogies disagree (antiphase noise gets 0.5/0.5, D-054): the car
        # speed is their trust-weighted mean, an untrusted partner (trust 0) gets no weight.
        # Fused one by one, the first of a pair wins and the NIS gate then rejects its
        # opposite partner. The partner is brought to this stamp with the filter's
        # acceleration (model + bias).
        partner = max(0.0, other[1] + (self._model_accel + self._x[1]) * (sample.t - other[0]))
        return (trust * own + other[2] * partner) / (trust + other[2])

    def _observe_scale(self, sample: WheelSample, trust: float):
        self._recent_wheel[sample.bogie] = (sample.t, sample.speed, trust)
        front, rear = self._recent_wheel['front'], self._recent_wheel['rear']
        p = self._p
        if (front is None or rear is None or abs(front[0] - rear[0]) > p.pair_window_s
                or min(front[2], rear[2]) < p.scale_min_trust
                or min(front[1], rear[1]) < p.scale_min_speed_mps
                or abs(front[1] - rear[1]) > p.scale_max_diff_mps):
            return
        # A wheel pair identifies only the relative scale. Keep their common
        # scale fixed at the train calibration applied by Preprocessor.
        target = np.clip((rear[1] - front[1]) / (rear[1] + front[1]),
                         -p.scale_max_rel, p.scale_max_rel)
        self._scale_delta += p.scale_gain * (target - self._scale_delta)

    def restart(self, speed: float):
        """The car slid (#156): the state followed slipping bogies and its bias learned from
        them. Take the car speed from the slide detector, known to the pair tolerance
        `slip.front_rear_threshold_mps`, and forget the bias."""
        if not self._initialized or not np.isfinite(speed):
            return
        self._x = np.array((max(0.0, speed), 0.0))
        self._t_rest = None if self._x[0] > 0.0 else self._t
        self._cov = np.diag((max(self._cov[0, 0], self._restart_var), self._p.initial_bias_var))

    def state(self):
        return (float(self._x[0]), float(self._cov[0, 0]),
                float(self._model_accel + self._x[1]))

    def diagnostics(self):
        return self._diagnostic
