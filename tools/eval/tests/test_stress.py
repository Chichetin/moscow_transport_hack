"""Deterministic stress inputs and excess-error recovery against an unchanged GNSS reference."""
from copy import deepcopy

import numpy as np
import pytest

from tram_eval.bag import FRONT, GNSS, REAR, stamp
from tram_eval.stress import SCENARIOS, perturb, recovery_seconds
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
