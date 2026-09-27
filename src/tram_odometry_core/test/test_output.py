"""DelayLine: /result/velocity as the estimate output.velocity_delay_s before its stamp (D-095)."""
import math
import random
from pathlib import Path

import pytest

from tram_odometry_core.output import HISTORY_MARGIN_S, MAX_SAMPLES, DelayLine
from tram_odometry_core.types import load_params

ROOT = Path(__file__).resolve().parents[3]
PARAMS = load_params(ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
DELAY = PARAMS.output.velocity_delay_s
JUMP = PARAMS.input.max_stamp_jump_s
T0 = 1756195560.0     # s, a bag stamp: float seconds of this size keep ~0.24 us
TOL = 1e-4            # m/s: 0.24 us of a 0.05 s bracket is ~5e-6 of the step between samples
DT = 0.05             # s, one bogie at 10 Hz plus the other: ~20 Hz of estimates


def _line(delay=DELAY):
    return DelayLine(delay, JUMP)


def test_params_carry_the_delay_measured_on_the_judge_bag():
    assert DELAY == pytest.approx(0.09)


def test_zero_delay_passes_every_value_through_untouched():
    line = _line(0.0)
    stamps = [T0 + 0.3, T0, T0 + 0.1, T0 - 50.0, math.nan, T0 + 0.2]
    values = [3.0, 0.0, 7.25, 1.0, 2.0, math.inf]
    assert [line.push(t, v) for t, v in zip(stamps, values)] == values
    assert line._t == [] and line._v == []


def test_ramp_is_delayed_by_the_delay():
    line = _line()
    out = [(k * DT, line.push(T0 + k * DT, 2.0 * k * DT)) for k in range(100)]
    for t, v in out:
        if t >= DELAY:
            assert v == pytest.approx(2.0 * (t - DELAY), abs=TOL)


def test_step_moves_later_by_the_delay():
    line = _line()
    jump_at = 1.0
    out = [(k * DT, line.push(T0 + k * DT, 10.0 if k * DT >= jump_at else 0.0))
           for k in range(60)]
    for t, v in out:
        if t < jump_at + DELAY - DT:        # the whole bracket before the step
            assert v == 0.0
        elif t >= jump_at + DELAY:          # the whole bracket after it
            assert v == 10.0
        else:                               # t - delay between the last 0 and the first 10
            assert 0.0 < v < 10.0
            assert v == pytest.approx(10.0 * (t - DELAY - (jump_at - DT)) / DT, abs=TOL)


def test_before_the_history_covers_the_delay_the_earliest_sample_goes_out():
    line = _line()
    assert line.push(T0, 5.0) == 5.0                # the first sample: nothing older
    assert line.push(T0 + 0.04, 6.0) == 5.0         # T0 - 0.05 is before the history
    assert line.push(T0 + 0.10, 7.0) == pytest.approx(5.0 + (0.01 / 0.04) * 1.0, abs=TOL)


def test_late_stamp_is_inserted_in_order_and_reads_the_history_at_its_own_time():
    line = _line()
    for k in (0, 1, 2, 4, 5):                       # k = 3 comes late (another topic)
        line.push(T0 + k * DT, 2.0 * k * DT)
    assert line.push(T0 + 3 * DT, 2.0 * 3 * DT) == pytest.approx(2.0 * (3 * DT - DELAY), abs=TOL)
    assert line._t == sorted(line._t) and len(line._t) == 6
    assert line.push(T0 + 6 * DT, 2.0 * 6 * DT) == pytest.approx(2.0 * (6 * DT - DELAY), abs=TOL)


def test_stamp_older_than_the_whole_history_goes_out_as_it_is():
    """A burst at the start of a run (trap 5): a bogie stamp seconds behind the newest one."""
    line = _line()
    for k in range(100):
        line.push(T0 + k * DT, 8.0)
    assert line.push(T0 + 99 * DT - 3.0, 4.0) == 4.0
    assert line.push(T0 + 100 * DT, 8.0) == 8.0     # the late sample does not stay in the way


def test_backward_clock_jump_restarts_the_history():
    line = _line()
    for k in range(50):
        line.push(T0 + 20.0 + k * DT, 9.0)
    back = T0 + 20.0 + 49 * DT - JUMP - 1.0          # further back than input.max_stamp_jump_s
    assert line.push(back, 3.0) == 3.0
    assert line._t == [back]
    assert line.push(back + DT, 4.0) == 3.0          # the earliest sample of the new history


def test_backward_stamp_within_the_jump_limit_keeps_the_history():
    line = _line()
    for k in range(50):
        line.push(T0 + k * DT, 9.0)
    line.push(T0 + 49 * DT - 0.5 * JUMP, 3.0)
    assert line.push(T0 + 50 * DT, 9.0) == 9.0 and len(line._t) > 2


def test_forward_clock_jump_restarts_the_history():
    line = _line()
    for k in range(50):
        line.push(T0 + k * DT, 9.0)
    ahead = T0 + 49 * DT + JUMP + 1.0
    assert line.push(ahead, 2.0) == 2.0
    assert line._t == [ahead]


def test_memory_is_bounded_by_time_and_by_count():
    line = _line()
    for k in range(20_000):                          # 1000 s at 20 Hz
        line.push(T0 + k * DT, 1.0)
    assert len(line._t) <= (DELAY + HISTORY_MARGIN_S) / DT + 2
    same = _line()
    for _ in range(10_000):                          # a repeated stamp never ages out by time
        same.push(T0, 1.0)
    assert len(same._t) == len(same._v) == MAX_SAMPLES


def test_repeated_stamp_takes_the_newest_value_without_dividing_by_zero():
    line = DelayLine(0.25, JUMP)                     # binary-exact stamps: t - delay hits 1000.0
    line.push(1000.0, 1.0)
    line.push(1000.0, 2.0)
    assert line.push(1000.25, 3.0) == 2.0


def test_delay_below_the_stamp_resolution_returns_the_newest_sample():
    line = _line(1e-9)                               # T0 - 1e-9 == T0 in float seconds
    assert line.push(T0, 1.0) == 1.0
    assert line.push(T0 + DT, 2.0) == 2.0


@pytest.mark.parametrize('t,v', [(math.nan, 1.0), (math.inf, 1.0), (T0, math.nan),
                                 (T0, -math.inf)])
def test_nonfinite_sample_is_returned_and_not_kept(t, v):
    line = _line()
    line.push(T0 - 1.0, 4.0)
    out = line.push(t, v)
    assert out is v or (math.isnan(out) and math.isnan(v))
    assert line._t == [T0 - 1.0] and line._v == [4.0]


def test_output_of_nonnegative_speeds_is_finite_nonnegative_and_within_the_inputs():
    rng = random.Random(95)
    line = _line()
    for k in range(5000):
        t = T0 + k * DT + rng.uniform(-0.2, 0.2)     # out of order between topics
        v = rng.choice([0.0, 1e-300, rng.uniform(0.0, 20.0)])
        out = line.push(t, v)
        assert math.isfinite(out) and 0.0 <= out <= 20.0

