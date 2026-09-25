"""Causal speed and acceleration-bias filter; wheel input is calibrated SI."""
import numpy as np

from ..types import FilterDiagnostics, Params, WheelSample


class SpeedFilter:
    """Kalman state [speed, acceleration bias] driven by the drive model.

    The first trusted wheel initializes speed without a zero-speed prior. Each
    bogie contributes at most once per stamp; a fresh delayed sample is compared
    with the predicted state at its stamp without rolling the state backward.
    """

    def __init__(self, params: Params):
        self._p = params.filter
        self._stale_timeout = params.input.stale_timeout_s
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
                or not np.isfinite(trust) or not 0.0 < trust <= 1.0):
            return
        if self._t is not None and self._t - sample.t > self._stale_timeout:
            return
        last = self._last[sample.bogie]
        if last is not None and sample.t <= last:
            return
        self.predict(sample.t, self._model_accel)
        self._last[sample.bogie] = sample.t
        age = self._t - sample.t
        measurement = sample.speed + self._model_accel * age
        measurement_var = self._p.r_wheel / trust + self._p.q_accel * age
        if not self._initialized:
            self._x[0] = max(0.0, measurement)
            self._cov = np.diag((measurement_var, self._p.initial_bias_var))
            self._initialized = True
            self._diagnostic = FilterDiagnostics(sample.t, sample.bogie, 0.0, True)
            return

        observation = np.array((1.0, -age))
        innovation = measurement - observation @ self._x
        projected = self._cov @ observation
        innovation_var = float(observation @ projected + measurement_var)
        nis = float(innovation ** 2 / innovation_var)
        accepted = nis <= self._p.nis_gate
        self._diagnostic = FilterDiagnostics(sample.t, sample.bogie, nis, accepted)
        if not accepted:
            return
        gain = projected / innovation_var
        self._x += gain * innovation
        self._x[0] = max(0.0, self._x[0])
        residual = np.eye(2) - np.outer(gain, observation)
        self._cov = (residual @ self._cov @ residual.T
                     + np.outer(gain, gain) * measurement_var)
        self._cov = (self._cov + self._cov.T) / 2.0
        if (sample.speed < 0.1 and self._x[0] < 0.1
                and self._model_accel <= 0.0 and self._x[1] < 0.0):
            # At a confirmed stop, braking cannot keep accumulating negative
            # velocity. Reset the unobservable bias before the next departure.
            self._x[1] = 0.0
            self._cov[0, 1] = self._cov[1, 0] = 0.0
            self._cov[1, 1] = max(self._cov[1, 1], self._p.initial_bias_var)

    def state(self):
        return (float(self._x[0]), float(self._cov[0, 0]),
                float(self._model_accel + self._x[1]))

    def diagnostics(self):
        return self._diagnostic
