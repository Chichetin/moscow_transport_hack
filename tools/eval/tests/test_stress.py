"""Deterministic stress inputs and excess-error recovery against an unchanged GNSS reference."""
from copy import deepcopy

import numpy as np
import pytest

from tram_eval.bag import FRONT, GNSS, REAR, stamp
from tram_eval.metrics import Estimates
from tram_eval.stress import SCENARIOS, _errors, perturb, recovery_seconds
from test_bag import drive


def wheels(msgs, topic):
    return [(stamp(m), m.velocity) for t, m in msgs if t == topic]


@pytest.mark.parametrize('scenario', SCENARIOS)
def test_scenario_changes_only_wheels_and_is_reproducible(scenario):
    source = drive(100.0)
    original = deepcopy(source)
    first = perturb(source, scenario, 5.0)
    second = perturb(source, scenario, 5.0)
    assert source == original
    assert first == second
    assert first is not None
    changed, start, end = first
    assert start < end
    assert [(t, m) for t, m in changed if t in GNSS] == [(t, m) for t, m in source if t in GNSS]
    assert wheels(changed, REAR) == wheels(source, REAR) if scenario != 'noise' else True
    assert wheels(changed, FRONT) != wheels(source, FRONT)


@pytest.mark.parametrize('duration', [1.0, 10.0, 70.0])
def test_gap_removes_front_samples_only_inside_requested_interval(duration):
    source = drive(100.0)
    changed, start, end = perturb(source, f'gap_{int(duration)}', 5.0)
    assert end - start == duration
    before = wheels(source, FRONT)
    after = wheels(changed, FRONT)
    assert len(before) - len(after) == sum(start <= t < end for t, _ in before)
    assert all(not start <= t < end for t, _ in after)
    assert wheels(changed, REAR) == wheels(source, REAR)


def test_outlier_and_spike_have_distinct_amplitudes():
    source = drive(100.0)
    out, a, b = perturb(source, 'outlier', 5.0)
    assert b - a < 1.0
    assert max(v for t, v in wheels(out, FRONT) if a <= t < b) >= 150.0
    spike, a, b = perturb(source, 'spike', 5.0)
    assert b - a == 5.0
    assert sum(v > 36.0 for t, v in wheels(spike, FRONT) if a <= t < b) >= 40


def test_noise_is_bounded_and_zero_mean_over_full_cycles():
    source = drive(100.0)
    changed, a, b = perturb(source, 'noise', 5.0)
    delta = [v - 36.0 for t, v in wheels(changed, FRONT) if a <= t < b]
    assert delta and max(map(abs, delta)) <= 3.01
    assert abs(np.mean(delta)) < 0.2


def test_jitter_and_rollback_modify_stamps_without_reordering_messages():
    source = drive(100.0)
    for scenario in ('jitter', 'rollback'):
        changed, a, b = perturb(source, scenario, 5.0)
        assert [t for t, _ in changed] == [t for t, _ in source]
        assert [m.velocity for t, m in changed if t == FRONT] == [m.velocity for t, m in source if t == FRONT]
        offsets = [t1 - t0 for (t0, _), (t1, _) in zip(wheels(source, FRONT), wheels(changed, FRONT)) if a <= t0 < b]
        assert min(offsets) < 0
        assert max(map(abs, offsets)) < (0.05 if scenario == 'jitter' else 1.0)
        if scenario == 'rollback':
            assert min(offsets) <= -0.25


@pytest.mark.parametrize('length', [10.0, 12.2, 12.7, 13.5, 20.0, 100.0])
def test_event_starts_after_gnss_window_and_leaves_recovery_tail(length):
    from tram_eval.bag import gnss_window_end

    source = drive(length)
    wheel_t = [t for t, _ in wheels(source, FRONT) + wheels(source, REAR)]
    event = perturb(source, 'spike', 5.0)
    if event is None:
        return
    _, start, end = event
    assert start >= gnss_window_end(source, 5.0) + 1.0
    assert max(wheel_t) - end >= 3.0 - 1e-9


def test_recovery_requires_sustained_error_below_threshold():
    times = np.arange(0, 12, 0.1).round(1)
    excess = np.full(len(times), 0.1)
    excess[(times >= 3.0) & (times < 5.0)] = 1.0
    excess[(times >= 7.0) & (times < 8.0)] = 1.0
    assert recovery_seconds(times, excess, event_end=5.0, threshold=0.2, sustain_s=2.0) == pytest.approx(3.0)
    assert recovery_seconds(times[:90], excess[:90], event_end=5.0, threshold=0.2, sustain_s=2.0) is None


def test_stress_evaluation_measures_peak_and_recovery_without_touching_reference():
    from tram_eval.stress import evaluate_stress_bag
    from test_bag import DeadReckoning

    source = drive(100.0)
    original = deepcopy(source)
    result = evaluate_stress_bag('fake', 5.0, make_odometry=DeadReckoning, msgs=source)
    assert source == original
    assert set(result) == set(SCENARIOS)
    assert all(not r['crashed'] for r in result.values())
    spike = result['spike']
    assert spike['peak_speed_excess_mps'] > 5.0
    assert spike['speed_recovery_s'] is not None
    assert spike['event_end_s'] > spike['event_start_s']
    assert result['gap_70']['skipped'] is False


def test_short_bag_skips_long_gap_with_explanation():
    from tram_eval.stress import evaluate_stress_bag
    from test_bag import DeadReckoning

    result = evaluate_stress_bag('fake', 5.0, make_odometry=DeadReckoning, msgs=drive(20.0))
    assert result['gap_70']['skipped'] is True
    assert result['gap_70']['reason']


def test_recovery_rejects_sparse_publications():
    times = np.array([5.0, 5.1, 7.1, 9.1])
    excess = np.zeros(4)
    assert recovery_seconds(times, excess, event_end=5.0, threshold=0.2, sustain_s=2.0) is None


def test_report_has_absolute_gnss_error_and_event_coverage():
    from tram_eval.stress import evaluate_stress_bag
    from test_bag import DeadReckoning

    report = evaluate_stress_bag('fake', 5.0, make_odometry=DeadReckoning, msgs=drive(20.0))['spike']
    assert report['peak_speed_error_mps'] >= report['peak_speed_excess_mps'] > 0
    assert report['clean_peak_speed_error_mps'] >= 0
    assert report['n_speed_during'] > 0
    assert report['peak_pos3d_error_m'] is not None


def test_peak_and_recovery_keep_worst_error_at_duplicate_stamp():
    times = np.arange(1.0, 4.2, 0.1).round(10)
    duplicate = int(np.flatnonzero(times == 2.0)[0]) + 1
    times = np.insert(times, duplicate, 2.0)
    clean = Estimates(times.copy(), np.zeros(len(times)), np.zeros((len(times), 3)),
                      np.zeros(len(times), bool))
    dirty_speed = np.zeros(len(times))
    dirty_speed[duplicate] = 50.0
    dirty = Estimates(times.copy(), dirty_speed, np.zeros((len(times), 3)),
                      np.zeros(len(times), bool))

    peak, _, _, _, _, _ = _errors(
        np.array([1.0, 4.1]), np.zeros(2), clean, dirty,
        'speed', event_start=1.9, event_end=2.1, threshold=0.2)
    _, _, _, recovery, _, _ = _errors(
        np.array([1.0, 4.1]), np.zeros(2), clean, dirty,
        'speed', event_start=1.5, event_end=1.75, threshold=0.2)

    assert peak == 50.0
    assert recovery == pytest.approx(0.35)


@pytest.mark.parametrize('scenario', ['outlier', 'rollback'])
def test_scenario_skips_when_no_front_sample_can_be_changed(scenario):
    source = drive(100.0)
    initial = perturb(source, scenario, 5.0)
    assert initial is not None
    _, start, end = initial
    no_front = [(topic, msg) for topic, msg in source
                if not (topic == FRONT and start <= stamp(msg) < end)]

    assert perturb(no_front, scenario, 5.0) is None


def test_rollback_skips_when_window_has_fewer_than_ten_front_samples():
    source = drive(100.0)
    initial = perturb(source, 'rollback', 5.0)
    assert initial is not None
    _, start, end = initial
    first_front = next((topic, msg) for topic, msg in source
                       if topic == FRONT and start <= stamp(msg) < end)
    sparse = [(topic, msg) for topic, msg in source
              if topic != FRONT or not start <= stamp(msg) < end or (topic, msg) == first_front]

    assert perturb(sparse, 'rollback', 5.0) is None
