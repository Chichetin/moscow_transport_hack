"""Time alignment of /result/velocity with the organizers' reference (D-095).

The judge compares /result/velocity with `/localization/kinematic_state` of the same stamp
(+-0.05 s). That reference is the output of the organizers' fused localization and lags the
sensors: on their check bag 30618_88aea4d9 our speed v(t) against ref(t + d) has the least
RMSE at d = 0.09 s, and so does their own GNSS master/vel. At the input stamp t the node
therefore publishes the speed estimate of t - output.velocity_delay_s, linear between the
estimates already made: past values only (online), the stamp stays the input's (D-015).

RunGate tells the node when a new bag is played into it without a restart (#200, D-098).
"""
import math
from bisect import bisect_left, bisect_right
from collections import deque

# s of history kept behind the newest stamp beyond the delay: an input of another topic may
# come with an older stamp (docs/data.md, traps 5-6) and still finds its delayed value here
HISTORY_MARGIN_S = 1.0
# samples at most, whatever the stamps do (a repeated stamp never ages out): O(1) memory per
# message; delay + margin (~1.1 s) of every input (~60 Hz) is ~70 samples
MAX_SAMPLES = 256
# vehicle inputs on a new stamp-to-wall offset that make it a new bag, from at least
# RUN_CONFIRM_TOPICS vehicle topics: glitched stamps of one stream are not a new bag (D-043),
# both bogies at ~10 Hz give three from two topics within ~0.15 s of a real new bag
RUN_CONFIRM_SAMPLES = 3
RUN_CONFIRM_TOPICS = 2
# s of wall clock an input is held for a new bag: GNSS comes ~1 s before the first bogie
RUN_HOLD_S = 3.0
# inputs held for a new bag until it is confirmed: GNSS (3 topics, ~10 Hz each) may come ~1 s
# before its first bogie; O(1) memory per message
RUN_PENDING_MAX = 256


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


class RunGate:
    """Which run -- which bag -- an input belongs to, for one node fed bag after bag (#200, D-098).

    `ros2 bag play` replays at the recorded pace, so within one bag `stamp - wall clock` stays
    put up to the input lag (docs/data.md, traps 5-6: bursts late by up to 3.7 s), across a
    silence of every stream too; the next bag has another offset. An input within `max_jump_s`
    of the current offset belongs to the current run. One off it is held (RUN_HOLD_S at most): a
    new bag starts only once RUN_CONFIRM_SAMPLES vehicle inputs of RUN_CONFIRM_TOPICS topics
    share a new offset, so neither GNSS (never after its window, D-005) nor the glitched stamps
    of one stream can open a run; the held inputs of that offset,
    GNSS that came before the bogies included, go to the new run first. Inputs on the offset of
    the run just left are what was still queued from the old bag and are dropped.
    """

    def __init__(self, max_jump_s: float):
        self.max_jump_s = max_jump_s    # s, input.max_stamp_jump_s: farther is another clock
        self.offset = None              # s, stamp - wall of the latest input of the current run
        self.retired = None             # s, the same of the run before it
        self._held = deque(maxlen=RUN_PENDING_MAX)  # (offset s, wall s, vehicle topic, item)

    def place(self, vehicle, stamp_s: float, wall_s: float, item):
        """Place `item` of the vehicle topic `vehicle` (None: GNSS), stamp `stamp_s`, received
        at monotonic `wall_s`: ('run', [item]) --
        step the current run; ('new', items) -- start a new run and step it with `items`, in
        arrival order; ('hold', []) or ('drop', []) -- nothing to step now."""
        offset = stamp_s - wall_s
        if self.offset is None or abs(offset - self.offset) <= self.max_jump_s:
            self.offset = offset
            return 'run', [item]
        if self.retired is not None and abs(offset - self.retired) <= self.max_jump_s:
            return 'drop', []
        while self._held and wall_s - self._held[0][1] > RUN_HOLD_S:
            self._held.popleft()
        self._held.append((offset, wall_s, vehicle, item))
        same = [h for h in self._held if abs(h[0] - offset) <= self.max_jump_s]
        topics = [h[2] for h in same if h[2] is not None]
        if (vehicle is None or len(topics) < RUN_CONFIRM_SAMPLES
                or len(set(topics)) < RUN_CONFIRM_TOPICS):
            return 'hold', []
        self.retired, self.offset = self.offset, offset
        self._held.clear()
        return 'new', [h[3] for h in same]
