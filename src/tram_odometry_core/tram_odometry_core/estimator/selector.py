"""Min/max wheel selector (#59): the "dumb baseline" the speed filter has to beat.

Under traction (drive model accelerates) a spinning bogie reads too high, so the lower
trusted bogie is taken; under braking a sliding bogie reads too low, so the higher one; at
cruise the mean. A bogie with zero trust or silent for longer than `input.stale_timeout_s`
does not take part. With no live bogie the speed is extrapolated with the drive model.

Same interface as `SpeedFilter` (predict / update / state / diagnostics / rebase_time), so
an experiment can put it in place of the filter. Not wired into `pipeline` (D-059).
"""
import math

from ..types import FilterDiagnostics, Params, WheelSample


class WheelSelector:
    def __init__(self, params: Params):
        self._stale = params.input.stale_timeout_s
        self._var = params.filter.r_wheel
        self._t = None
        self._accel = 0.0
        self._v = 0.0
        self._last = {'front': None, 'rear': None}     # bogie -> (stamp, speed) while trusted
        self._diagnostic = None

    def rebase_time(self, t: float):
        """Keep the speed, start a fresh input clock after a confirmed jump (D-043)."""
        self._t = t
        self._last = {'front': None, 'rear': None}
        self._diagnostic = None

    def predict(self, t: float, accel_model: float):
        self._diagnostic = None
        if not math.isfinite(t) or not math.isfinite(accel_model):
            return
        if self._t is not None and t <= self._t:
            return
        dt = 0.0 if self._t is None else t - self._t
        self._t = t
        self._accel = accel_model
        if not self._live(t):
            self._v = max(0.0, self._v + accel_model * dt)

    def update(self, sample: WheelSample, trust: float):
        self._diagnostic = None
        if (sample.bogie not in self._last or not math.isfinite(sample.t)
                or not math.isfinite(sample.speed) or sample.speed < 0.0
                or not math.isfinite(trust)):
            return
        last = self._last[sample.bogie]
        if last is not None and sample.t <= last[0]:
            return
        if trust <= 0.0:
            self._last[sample.bogie] = None
            return
        self._last[sample.bogie] = (sample.t, sample.speed)
        speeds = self._live(sample.t)
        if self._accel > 0.0:
            self._v = min(speeds)
        elif self._accel < 0.0:
            self._v = max(speeds)
        else:
            self._v = sum(speeds) / len(speeds)
        self._diagnostic = FilterDiagnostics(sample.t, sample.bogie, 0.0, True)

    def _live(self, t: float):
        return [s for ts, s in filter(None, self._last.values()) if t - ts <= self._stale]

    def state(self):
        return self._v, self._var, self._accel

    def diagnostics(self):
        return self._diagnostic
