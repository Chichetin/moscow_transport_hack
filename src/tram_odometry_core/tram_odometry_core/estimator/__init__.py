"""Causal speed and acceleration-bias filter; wheel input is calibrated SI."""
import numpy as np

from ..types import FilterDiagnostics, Params, WheelSample

BIAS_RELEASE_MPS = 0.5  # near-stop region: do not extrapolate braking through v=0


class SpeedFilter:
    """Kalman state [speed, acceleration bias] driven by the drive model.

    The first trusted wheel initializes speed without a zero-speed prior. Each
    bogie contributes at most once per stamp; a fresh delayed sample is compared
    with the predicted state at its stamp without rolling the state backward.
    """

    def __init__(self, params: Params):
        self._p = params.filter
        self._stale_timeout = params.input.stale_timeout_s
        self._max_wheel_accel = params.input.max_wheel_accel_mps2
        self._rest_speed = max(params.position.stop_speed_mps, BIAS_RELEASE_MPS)
        if (self._p.q_accel < 0.0 or self._p.r_wheel <= 0.0
                or self._p.q_bias < 0.0 or self._p.initial_bias_var < 0.0
                or self._p.nis_gate <= 0.0):
            raise ValueError('invalid filter covariance or NIS threshold')
        self._t = None
        self._last = {'front': None, 'rear': None}
        self._x = np.zeros(2, dtype=float)
        self._cov = np.diag((self._p.r_wheel, self._p.initial_bias_var))
        self._model_accel = 0.0
        self._initialized = False
        self._diagnostic = None
        self._scale_delta = 0.0
        self._recent_wheel = {'front': None, 'rear': None}
        self._zero_wheel = {'front': None, 'rear': None}
        self._confirmed_stop_t = None

    def predict(self, t: float, accel_model: float):
        self._diagnostic = None
        if not np.isfinite(t) or not np.isfinite(accel_model):
            return
        if self._t is not None and t <= self._t:
            return
        dt = 0.0 if self._t is None else t - self._t
        self._t = t
        self._model_accel = accel_model
        self._x[0] = max(0.0, self._x[0] + (accel_model + self._x[1]) * dt)
        transition = np.array(((1.0, dt), (0.0, 1.0)))
        process_noise = np.array((
            (self._p.q_accel * dt + self._p.q_bias * dt ** 3 / 3.0,
             self._p.q_bias * dt ** 2 / 2.0),
            (self._p.q_bias * dt ** 2 / 2.0, self._p.q_bias * dt),
        ))
        self._cov = transition @ self._cov @ transition.T + process_noise

    def update(self, sample: WheelSample, trust: float):
        self._diagnostic = None
        if (sample.bogie not in self._last or not np.isfinite(sample.t)
                or not np.isfinite(sample.speed) or sample.speed < 0.0
                or not np.isfinite(trust) or not 0.0 <= trust <= 1.0):
            return
        if self._t is not None and self._t - sample.t > self._stale_timeout:
            return
        last = self._last[sample.bogie]
        if last is not None and sample.t <= last:
            return
        if sample.speed == 0.0:
            self._zero_wheel[sample.bogie] = sample.t
        if trust == 0.0:
            return
        self.predict(sample.t, self._model_accel)
        self._last[sample.bogie] = sample.t
        age = self._t - sample.t
        scale = 1.0 - self._scale_delta if sample.bogie == 'front' else 1.0 + self._scale_delta
        measurement = sample.speed / scale + self._model_accel * age
        measurement_var = self._p.r_wheel / (trust * scale ** 2) + self._p.q_accel * age
        plausible_departure = (
            self._confirmed_stop_t is not None and sample.speed > 0.0
            and sample.speed <= self._max_wheel_accel * max(0.0, sample.t - self._confirmed_stop_t) + 0.05)
        if plausible_departure:
            # Velocity at the next departure is a new motion segment. The
            # braking posterior should not slow the first plausible wheel.
            self._cov[0, 0] = max(self._cov[0, 0], 9.0 * measurement_var)
        if not self._initialized:
            self._x[0] = max(0.0, measurement)
            self._cov = np.diag((measurement_var, self._p.initial_bias_var))
            self._initialized = True
            self._diagnostic = FilterDiagnostics(sample.t, sample.bogie, 0.0, True)
            self._observe_scale(sample, trust)
            return

        observation = np.array((1.0, -age))
        innovation = measurement - observation @ self._x
        projected = self._cov @ observation
        innovation_var = float(observation @ projected + measurement_var)
        other = 'rear' if sample.bogie == 'front' else 'front'
        other_zero = self._zero_wheel[other]
        paired_zero = (sample.speed == 0.0 and other_zero is not None
                       and abs(sample.t - other_zero) <= 0.02)
        if paired_zero and innovation ** 2 > self._p.nis_gate * innovation_var:
            # Two independent zero readings expose a real stop that a smooth
            # motion prior cannot explain. Inflate speed uncertainty so the
            # ordinary NIS gate can accept the corroborated measurement.
            self._cov[0, 0] += (innovation ** 2 / self._p.nis_gate
                                - innovation_var + 1e-8)
            projected = self._cov @ observation
            innovation_var = float(observation @ projected + measurement_var)
        nis = float(innovation ** 2 / innovation_var)
        accepted = nis <= self._p.nis_gate
        self._diagnostic = FilterDiagnostics(sample.t, sample.bogie, nis, accepted)
        if not accepted:
            return
        if paired_zero:
            self._confirmed_stop_t = sample.t
        elif sample.speed > 0.0:
            self._confirmed_stop_t = None
        gain = projected / innovation_var
        self._x += gain * innovation
        self._x[0] = max(0.0, self._x[0])
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

    def _observe_scale(self, sample: WheelSample, trust: float):
        self._recent_wheel[sample.bogie] = (sample.t, sample.speed, trust)
        front, rear = self._recent_wheel['front'], self._recent_wheel['rear']
        if (front is None or rear is None or abs(front[0] - rear[0]) > 0.02
                or min(front[2], rear[2]) < 0.8 or min(front[1], rear[1]) < 2.0
                or abs(front[1] - rear[1]) > 0.5):
            return
        # A wheel pair identifies only the relative scale. Keep their common
        # scale fixed at the train calibration applied by Preprocessor.
        target = np.clip((rear[1] - front[1]) / (rear[1] + front[1]), -0.03, 0.03)
        self._scale_delta += 0.02 * (target - self._scale_delta)

    def state(self):
        return (float(self._x[0]), float(self._cov[0, 0]),
                float(self._model_accel + self._x[1]))

    def diagnostics(self):
        return self._diagnostic
