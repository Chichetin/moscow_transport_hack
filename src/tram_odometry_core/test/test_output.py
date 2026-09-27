"""DelayLine: /result/velocity as the estimate output.velocity_delay_s before its stamp (D-095)."""
import math
import random

import pytest

from tram_odometry_core.output import (HISTORY_MARGIN_S, MAX_SAMPLES, RUN_CONFIRM_SAMPLES,
                                       RUN_HOLD_S, RUN_PENDING_MAX, DelayLine, RunGate)

# literals, not params.yaml: the rollback of D-095 (output.velocity_delay_s: 0) keeps these green
DELAY = 0.09          # s, the lag of the judge's reference on its bag (D-095)
JUMP = 10.0           # s, as input.max_stamp_jump_s
T0 = 1756195560.0     # s, a bag stamp: float seconds of this size keep ~0.24 us
TOL = 1e-4            # m/s: 0.24 us of a 0.05 s bracket is ~5e-6 of the step between samples
DT = 0.05             # s, one bogie at 10 Hz plus the other: ~20 Hz of estimates


def _line(delay=DELAY):
    return DelayLine(delay, JUMP)


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



# RunGate (#200, D-098): which bag an input belongs to when one node gets several

WALL0 = 5000.0        # s, monotonic clock of the node at the first input


def _play(gate, inputs):
    """inputs: (vehicle topic or None, stamp, wall, name) -> the names stepped per run."""
    runs = [[]]
    for vehicle, stamp, wall, name in inputs:
        verdict, items = gate.place(vehicle, stamp, wall, name)
        if verdict == 'new':
            runs.append([])
        runs[-1].extend(items)
    return runs


def _bag(t0, wall0, seconds, gnss_first=0.0, gnss=True, hz=10):
    """One bag played at its pace: GNSS fixes from t0, both bogies from t0 + gnss_first."""
    out = []
    for k in range(int(seconds * hz)):
        dt = k / hz
        if gnss:
            out.append((None, t0 + dt, wall0 + dt, f'g{t0 + dt:.2f}'))
        if dt >= gnss_first:
            out.append(('front', t0 + dt + 0.01, wall0 + dt + 0.01, f'f{t0 + dt + 0.01:.2f}'))
            out.append(('rear', t0 + dt + 0.02, wall0 + dt + 0.02, f'r{t0 + dt + 0.02:.2f}'))
    return out


def test_one_bag_is_one_run_across_a_silence_of_every_stream():
    """A 73 s silence (docs/data.md) keeps the offset: no new run, the GNSS window stays shut."""
    inputs = _bag(T0, WALL0, 20) + _bag(T0 + 93, WALL0 + 93, 5)
    runs = _play(RunGate(JUMP), inputs)
    assert len(runs) == 1 and len(runs[0]) == len(inputs)


def test_vehicle_silence_with_gnss_going_on_is_no_new_run():
    """Review of 2ad79a9: bogies quiet 1020..1035 while GNSS goes on -- the same bag."""
    inputs = _bag(T0, WALL0, 20) + [(None, T0 + 20 + k, WALL0 + 20 + k, 'g') for k in range(15)]
    inputs += _bag(T0 + 35, WALL0 + 35, 3)
    assert len(_play(RunGate(JUMP), inputs)) == 1


def test_next_bag_is_a_new_run_with_its_gnss_that_came_before_the_bogies():
    """Review of 2ad79a9: the new bag's fix before its first bogie goes to the new run."""
    first = _bag(T0, WALL0, 5)
    second = _bag(T0 - 40000, WALL0 + 8, 5, gnss_first=1.0)
    runs = _play(RunGate(JUMP), first + second)
    assert len(runs) == 2
    assert runs[0] == [name for *_, name in first]
    assert runs[1] == [name for *_, name in second]


def test_next_bag_without_gnss_is_a_new_run():
    runs = _play(RunGate(JUMP), _bag(T0, WALL0, 5) + _bag(T0 + 3600, WALL0 + 8, 5, gnss=False))
    assert len(runs) == 2 and runs[1][0].startswith('f')


def test_old_bag_inputs_still_queued_after_the_switch_are_dropped():
    gate = RunGate(JUMP)
    runs = _play(gate, _bag(T0, WALL0, 5) + _bag(T0 + 3600, WALL0 + 6, 2)
                 + [('front', T0 + 5.5, WALL0 + 8.1, 'old')] + _bag(T0 + 3602, WALL0 + 8.2, 1))
    assert len(runs) == 2 and 'old' not in runs[1]


def test_glitched_vehicle_stamp_is_not_a_new_run_and_gnss_never_opens_one():
    inputs = _bag(T0, WALL0, 2)
    inputs += [('front', T0 + 500, WALL0 + 2, 'glitch')] * (RUN_CONFIRM_SAMPLES - 1)
    inputs += [(None, T0 + 900 + k, WALL0 + 2 + k / 10, 'gnss') for k in range(50)]
    inputs += _bag(T0 + 2.1, WALL0 + 2.1, 2)
    runs = _play(RunGate(JUMP), inputs)
    assert len(runs) == 1 and 'glitch' not in runs[0] and 'gnss' not in runs[0]


def test_late_bursts_of_the_same_bag_stay_in_its_run():
    """Inputs up to 3.7 s late (docs/data.md, trap 6) are within the jump limit."""
    inputs = _bag(T0, WALL0, 3) + [('front', T0 + 3 - 3.7 + k * 0.01, WALL0 + 3, f'late{k}')
                                   for k in range(10)] + _bag(T0 + 3, WALL0 + 3, 2)
    runs = _play(RunGate(JUMP), inputs)
    assert len(runs) == 1 and len(runs[0]) == len(inputs)


def test_held_inputs_are_bounded():
    gate = RunGate(JUMP)
    gate.place('front', T0, WALL0, 'a')
    for k in range(10 * RUN_PENDING_MAX):
        gate.place(None, T0 + 900 + k * 1e-3, WALL0, 'g')
    assert len(gate._held) == RUN_PENDING_MAX


def test_one_stream_whose_clock_moves_for_good_is_no_new_run():
    """Review of e8cbafb: any number of glitched stamps of one bogie never opens a run, the
    other streams go on in the current one."""
    inputs = _bag(T0, WALL0, 2)
    for k in range(50):
        dt = 2 + k / 10
        inputs += [('front', T0 + dt + 20, WALL0 + dt, 'glitch'),
                   ('rear', T0 + dt, WALL0 + dt, 'rear'), (None, T0 + dt, WALL0 + dt, 'g')]
    runs = _play(RunGate(JUMP), inputs)
    assert len(runs) == 1 and 'glitch' not in runs[0] and runs[0].count('rear') == 50


def test_held_inputs_age_out():
    """Two glitches long ago and one now of another stream do not add up to a new bag."""
    gate = RunGate(JUMP)
    gate.place('front', T0, WALL0, 'a')
    gate.place('front', T0 + 500, WALL0 + 1, 'x')
    gate.place('rear', T0 + 500, WALL0 + 1, 'x')
    assert gate.place('front', T0 + 500 + RUN_HOLD_S + 2, WALL0 + RUN_HOLD_S + 2, 'y')[0] == 'hold'
