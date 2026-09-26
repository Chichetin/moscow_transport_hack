"""Causal features of a learned speed correction on top of `Odometry.step` (research).

A feature of the estimate stamped `te` uses only inputs already processed and stamped at or
before `te`, plus the pipeline's own state after this step (speed, acceleration, drive-model
acceleration, trust flags). GNSS is never a feature: it is only the label in training and the
reference in evaluation. Memory is bounded (fixed deques), work per message is O(HIST).
"""
from __future__ import annotations

import dataclasses
import math
from collections import deque

WHEEL_HIST = 40      # samples per bogie, ~4 s at 10 Hz
CMD_HIST = 80        # controller samples, 4 s at 20 Hz
STATE_HIST = 160     # (state time, state speed) pairs, ~3-4 s at ~40 messages/s
NAN = -999.0         # missing value: the same sentinel in training and inference
NOTCH_CAP_S = 4.0    # time since the last notch change is capped here
PHASE_CAP_S = 30.0   # time since a stop began / motion began is capped here
STOP_MPS = 0.3       # standstill of the state speed (as eval's stop mode)

FEATURES = (
    'v', 'a', 'am', 'ab', 'var', 'lag',
    'wf', 'wr', 'age_f', 'age_r', 'dfr', 'mfr', 'v_m',
    'sf03', 'sr03', 'sf1', 'sr1', 'sm03', 'sm1',
    'dv05', 'dv1', 'dv2',
    'n0', 'n03', 'n1', 'n2', 't_notch', 'age_cmd',
    'ft', 'rt', 'slip', 'nis', 'acc',
    't_stop', 't_move',
)
INDEX = {name: i for i, name in enumerate(FEATURES)}


def _at_or_before(buf, t):
    """Newest (stamp, value) in a stamp-ordered deque with stamp <= t, else None."""
    for item in reversed(buf):
        if item[0] <= t:
            return item
    return None


def _slope(buf, t, span):
    """Slope of a bogie over ~span seconds ending at its newest sample stamped <= t."""
    end = None
    for item in reversed(buf):
        if end is None:
            if item[0] <= t:
                end = item
            continue
        if end[0] - item[0] >= span:
            return (end[1] - item[1]) / (end[0] - item[0])
    return NAN


class FeatureTap:
    """Observe the pipeline after each step and return the feature vector of its estimate."""

    def __init__(self):
        self.wheel = {'front': deque(maxlen=WHEEL_HIST), 'rear': deque(maxlen=WHEEL_HIST)}
        self.cmd = deque(maxlen=CMD_HIST)
        self.state = deque(maxlen=STATE_HIST)
        self._seen_wheel = {'front': None, 'rear': None}
        self._seen_cmd = None
        self._nis, self._acc = NAN, NAN
        self._stop_since = None
        self._move_since = None

    def _ingest(self, odo):
        for bogie in ('front', 'rear'):
            s = odo._wheel[bogie]
            if s is not None and s is not self._seen_wheel[bogie]:
                self._seen_wheel[bogie] = s
                buf = self.wheel[bogie]
                if buf and s.t < buf[-1][0]:
                    buf.clear()          # input clock resync (D-043): old history is void
                buf.append((s.t, s.speed))
        if odo._cmd and odo._cmd[-1] is not self._seen_cmd:
            self._seen_cmd = odo._cmd[-1]
            if self.cmd and self._seen_cmd[0] < self.cmd[-1][0]:
                self.cmd.clear()
            self.cmd.append(self._seen_cmd)
        if odo._t is not None:
            if self.state and odo._t < self.state[-1][0]:
                self.state.clear()
            self.state.append((odo._t, odo._v))
            if odo._v < STOP_MPS:
                self._move_since = None
                if self._stop_since is None:
                    self._stop_since = odo._t
            else:
                self._stop_since = None
                if self._move_since is None:
                    self._move_since = odo._t

    def observe(self, odo, est):
        """Features of `est` (the estimate `odo.step` just returned), or None if est is None."""
        self._ingest(odo)
        if est is None:
            return None
        d = est.filter_diagnostics
        if d is not None:
            self._nis, self._acc = min(d.nis, 100.0), float(d.accepted)
        te = est.t
        now = odo._t if odo._t is not None else te
        x = [NAN] * len(FEATURES)
        x[0], x[1], x[2] = est.speed, est.accel, est.accel_model
        x[3], x[4], x[5] = est.accel - est.accel_model, est.speed_var, now - te
        fr = [_at_or_before(self.wheel[b], te) for b in ('front', 'rear')]
        for k, s in enumerate(fr):
            if s is not None:
                x[6 + k], x[8 + k] = s[1], te - s[0]
        if fr[0] is not None and fr[1] is not None:
            x[10], x[11] = fr[0][1] - fr[1][1], 0.5 * (fr[0][1] + fr[1][1])
        elif fr[0] is not None or fr[1] is not None:
            x[11] = (fr[0] or fr[1])[1]
        if x[11] != NAN:
            x[12] = est.speed - x[11]
        x[13] = _slope(self.wheel['front'], te, 0.3)
        x[14] = _slope(self.wheel['rear'], te, 0.3)
        x[15] = _slope(self.wheel['front'], te, 1.0)
        x[16] = _slope(self.wheel['rear'], te, 1.0)
        for k, (a, b) in enumerate(((13, 14), (15, 16))):
            vals = [x[a], x[b]] if x[a] != NAN and x[b] != NAN else [v for v in (x[a], x[b]) if v != NAN]
            if vals:
                x[17 + k] = sum(vals) / len(vals)
        for k, span in enumerate((0.5, 1.0, 2.0)):
            s = _at_or_before(self.state, te - span)
            if s is not None:
                x[19 + k] = est.speed - s[1]
        c = _at_or_before(self.cmd, te)
        if c is not None:
            x[22], x[27] = c[1], te - c[0]
            changed = None
            for item in reversed(self.cmd):
                if item[0] > te:
                    continue
                if item[1] != c[1]:
                    changed = item[0]
                    break
            x[26] = min(NOTCH_CAP_S, te - changed) if changed is not None else NOTCH_CAP_S
        for k, span in enumerate((0.3, 1.0, 2.0)):
            s = _at_or_before(self.cmd, te - span)
            if s is not None:
                x[23 + k] = s[1]
        st = est.slip
        x[28], x[29], x[30] = st.front_trust, st.rear_trust, float(st.slip_front or st.slip_rear)
        x[31], x[32] = self._nis, self._acc
        x[33] = min(PHASE_CAP_S, now - self._stop_since) if self._stop_since is not None else 0.0
        x[34] = min(PHASE_CAP_S, now - self._move_since) if self._move_since is not None else 0.0
        return [v if math.isfinite(v) else NAN for v in x]


class CorrectedOdometry:
    """`Odometry` with a learned additive speed correction, same `step` interface.

    `model(features) -> m/s` is clipped to +-`clip`; the published speed stays >= 0. With
    `integrate`, the path integral (and so the position) takes the corrected speed too: the
    correction times the state-time step is added to the pipeline's distance, which reaches
    the map at the next step.
    """

    def __init__(self, odometry, model, clip: float = math.inf, integrate: bool = False):
        self.odo = odometry
        self.model = model
        self.clip = clip
        self.integrate = integrate
        self.tap = FeatureTap()
        self._t_prev = None
        self._corr = 0.0

    def step(self, raw):
        t_before = self.odo._t
        est = self.odo.step(raw)
        x = self.tap.observe(self.odo, est)
        if self.integrate and t_before is not None and self.odo._t is not None:
            dt = self.odo._t - t_before
            if 0.0 < dt <= self.odo.params.input.max_stamp_jump_s:
                self.odo._distance += self._corr * dt
        if est is None:
            return None
        corr = max(-self.clip, min(self.clip, float(self.model(x))))
        if est.speed + corr < 0.0:
            corr = -est.speed
        self._corr = corr if self.odo._v > 0.0 else 0.0
        return dataclasses.replace(est, speed=est.speed + corr)
