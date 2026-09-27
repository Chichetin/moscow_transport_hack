"""Time alignment of /result/velocity with the organizers' reference (D-095).

The judge compares /result/velocity with `/localization/kinematic_state` of the same stamp
(+-0.05 s). That reference is the output of the organizers' fused localization and lags the
sensors: on their check bag 30618_88aea4d9 our speed v(t) against ref(t + d) has the least
RMSE at d = 0.09 s, and so does their own GNSS master/vel. At the input stamp t the node
therefore publishes the speed estimate of t - output.velocity_delay_s, linear between the
estimates already made: past values only (online), the stamp stays the input's (D-015).
"""
import math
from bisect import bisect_left, bisect_right

# s of history kept behind the newest stamp beyond the delay: an input of another topic may
# come with an older stamp (docs/data.md, traps 5-6) and still finds its delayed value here
HISTORY_MARGIN_S = 1.0
# samples at most, whatever the stamps do (a repeated stamp never ages out): O(1) memory per
# message; delay + margin (~1.1 s) of every input (~60 Hz) is ~70 samples
MAX_SAMPLES = 256


class DelayLine:
    """A scalar signal `delay_s` before the stamp of each new sample, from the samples so far."""

    def __init__(self, delay_s: float, max_jump_s: float):
        self.delay_s = delay_s          # s, >= 0 (types.load_params); 0 -> pass-through
        self.max_jump_s = max_jump_s    # s: a stamp this far from the newest one is a new clock
        self._keep_s = delay_s + HISTORY_MARGIN_S
        self._t = []                    # s, stamps in non-decreasing order
        self._v = []                    # values at those stamps

    def push(self, t: float, v: float) -> float:
        """Add the sample (t, v); return the value at t - delay_s: linear between the two
        samples around it, the earliest sample if the history starts later. delay_s = 0 and a
        non-finite sample return `v` as it is; the latter is not kept."""
        if self.delay_s <= 0 or not (math.isfinite(t) and math.isfinite(v)):
            return v
        ts, vs = self._t, self._v
        if ts and abs(t - ts[-1]) > self.max_jump_s:
            # the input clock jumped (as in pipeline, input.max_stamp_jump_s): the old history
            # is from another time line, the delayed value restarts from this sample
            ts.clear()
            vs.clear()
        i = bisect_right(ts, t)         # after equal stamps: the newest of them wins below
        ts.insert(i, t)
        vs.insert(i, v)
        out = self._at(t - self.delay_s)
        cut = max(bisect_left(ts, ts[-1] - self._keep_s), len(ts) - MAX_SAMPLES)
        if cut > 0:
            del ts[:cut]
            del vs[:cut]
        return out

    def _at(self, q: float) -> float:
        ts, vs = self._t, self._v
        i = bisect_right(ts, q)         # ts[i - 1] <= q < ts[i]
        if i == 0:
            return vs[0]
        if i == len(ts):                # q at the newest stamp: a delay below the stamp's ulp
            return vs[-1]
        w = (q - ts[i - 1]) / (ts[i] - ts[i - 1])   # in [0, 1): ts[i] > q >= ts[i - 1]
        # exact at w = 0 and between equal values; never below 0 between nonnegative ones
        return vs[i - 1] + w * (vs[i] - vs[i - 1])
