"""Time alignment of /result/velocity with the organizers' reference (D-095).

The judge compares /result/velocity with `/localization/kinematic_state` of the same stamp
(+-0.05 s). That reference is the output of the organizers' fused localization and lags the
sensors: on their check bag 30618_88aea4d9 our speed v(t) against ref(t + d) has the least
RMSE at d = 0.09 s, and so does their own GNSS master/vel. At the input stamp t the node
therefore publishes the speed estimate of t - output.velocity_delay_s, linear between the
estimates already made: past values only (online), the stamp stays the input's (D-015).

RunClock tells the node when a new bag is played into it without a restart (#200, D-098).
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


class RunClock:
    """The vehicle clock of the run the node publishes, and of a candidate run (#200, D-098).

    One node may get several bags one after another: each is a run of its own, and the state of
    the previous one (the GNSS window start, the anchor on the map) is kilometres off in the
    next. A vehicle stamp (bogies, controller) farther than `max_jump_s` from the clock of the
    publishing run may start a candidate run; the node makes it the publishing run only once
    it has its own absolute position (a fix in its window: every check bag starts with GNSS),
    and drops it when its window closes without one -- a silence of every vehicle stream
    within one bag (the publishing run carries on, as before this class). A stamp back near
    the publishing clock cancels the candidate: it was a stamp glitch (D-043). GNSS never
    starts or confirms a candidate by its stamp: only the vehicle streams move the clocks.
    """

    def __init__(self, max_jump_s: float):
        self.max_jump_s = max_jump_s    # s, input.max_stamp_jump_s: farther is another clock
        self.clock = None               # s, newest vehicle stamp of the publishing run
        self.candidate = None           # s, newest vehicle stamp of the candidate; None: none

    def vehicle(self, t: float) -> str:
        """Place the vehicle stamp `t`: 'run' -- it belongs to the publishing run (a candidate,
        if any, was a glitch and is gone); 'new' -- start a candidate run at `t`;
        'candidate' -- a candidate is pending, feed `t` to both runs."""
        if self.clock is None or abs(t - self.clock) <= self.max_jump_s:
            self.clock = t if self.clock is None else max(self.clock, t)
            self.candidate = None
            return 'run'
        if self.candidate is None:
            self.candidate = t
            return 'new'
        if abs(t - self.candidate) <= self.max_jump_s:
            self.candidate = max(self.candidate, t)
        return 'candidate'

    def settle(self) -> None:
        """The candidate is resolved: it became the publishing run, or it was dropped and the
        publishing run went on across the jump; either way its clock is the clock now."""
        if self.candidate is not None:
            self.clock, self.candidate = self.candidate, None
