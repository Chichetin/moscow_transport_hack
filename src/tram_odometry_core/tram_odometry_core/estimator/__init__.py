"""Online speed filter. Wheel samples are already calibrated and expressed in SI."""
import numpy as np

from ..types import Params, WheelSample


class SpeedFilter:
    """Scalar Kalman filter driven by external model acceleration.

    Measurements from the past are ignored; each bogie may update once per stamp.
    No GNSS or ROS input is accepted here. The first trusted wheel initializes speed.
    """

    def __init__(self, params: Params):
        self._p = params.filter
        self._stale_timeout = params.input.stale_timeout_s
        if self._p.q_accel < 0.0 or self._p.r_wheel <= 0.0:
            raise ValueError('filter noise must be nonnegative, wheel variance positive')
        self._t = None
        self._last = {'front': None, 'rear': None}
        self._speed = 0.0
        self._variance = self._p.r_wheel
        self._accel = 0.0
        self._initialized = False

    def predict(self, t: float, accel_model: float):
        if not np.isfinite(t) or not np.isfinite(accel_model):
            return
        if self._t is not None and t <= self._t:
            return
        dt = 0.0 if self._t is None else t - self._t
        self._t = t
        self._accel = accel_model
        self._speed = max(0.0, self._speed + accel_model * dt)
        self._variance += self._p.q_accel * dt

    def update(self, sample: WheelSample, trust: float):
        if (sample.bogie not in self._last or not np.isfinite(sample.t)
                or not np.isfinite(sample.speed) or sample.speed < 0.0
                or not np.isfinite(trust) or not 0.0 < trust <= 1.0):
            return
        if self._t is not None and self._t - sample.t > self._stale_timeout:
            return
        last = self._last[sample.bogie]
        if last is not None and sample.t <= last:
            return
        self.predict(sample.t, self._accel)
        self._last[sample.bogie] = sample.t
        age = self._t - sample.t
        measurement = sample.speed + self._accel * age
        variance = self._p.r_wheel / trust + self._p.q_accel * age
        if not self._initialized:
            self._speed = max(0.0, measurement)
            self._variance = variance
            self._initialized = True
            return
        gain = self._variance / (self._variance + variance)
        self._speed = max(0.0, self._speed + gain * (measurement - self._speed))
        self._variance = ((1.0 - gain) ** 2 * self._variance
                          + gain ** 2 * variance)

    def state(self):
        return self._speed, self._variance, self._accel
