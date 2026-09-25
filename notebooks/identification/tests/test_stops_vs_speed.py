"""Tests for notebooks/identification/stops_vs_speed.py on synthetic data (no bags needed)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import stops_vs_speed as svs  # noqa: E402


def test_wheel_stop_needs_min_duration():
    t = np.arange(0.0, 30.0, 1.0)
    v = np.full(30, 5.0)
    v[3:6] = 0.0             # 2 s: too short
    v[10:18] = 0.05          # 7 s: a stop
    v[27:] = 0.0             # open at the end, 2 s: too short
    assert svs.wheel_stop_stretches(t, v) == [(10.0, 17.0)]


def test_wheel_stop_ignores_nan_speed():
    t = np.arange(0.0, 20.0, 1.0)
    v = np.full(20, np.nan)
    assert svs.wheel_stop_stretches(t, v) == []


def test_asof_holds_the_last_sample_and_is_nan_before_the_first():
    series = np.array([[1.0, 10.0], [3.0, 20.0]])
    t = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
    assert np.array_equal(svs.asof(series, t), [np.nan, 10.0, 10.0, 20.0, 20.0], equal_nan=True)


def test_asof_empty_series_is_all_nan():
    assert np.all(np.isnan(svs.asof(np.zeros((0, 2)), np.array([0.0, 1.0]))))


def test_combined_speed_converts_kmh_and_averages_bogies():
    front = np.array([[0.0, 36.0]])    # 36 km/h = 10 m/s
    rear = np.array([[0.0, 0.0]])
    out = svs.combined_speed(front, rear)
    assert out[0, 1] == pytest.approx(5.0)


def test_combined_speed_uses_the_live_bogie_when_the_other_has_not_spoken_yet():
    front = np.array([[0.0, 36.0]])
    rear = np.zeros((0, 2))
    out = svs.combined_speed(front, rear)
    assert out[0, 1] == pytest.approx(10.0)


def test_cumulative_distance_is_v_times_t_for_uniform_motion():
    t = np.arange(0.0, 11.0, 1.0)
    v = np.full(11, 2.0)
    dist = svs.cumulative_distance(t, v)
    assert dist[-1] == pytest.approx(20.0)
    assert dist[0] == pytest.approx(0.0)


def test_cumulative_distance_never_goes_backward_on_negative_speed():
    t = np.array([0.0, 1.0, 2.0])
    v = np.array([1.0, -5.0, 1.0])       # negative reading (bad data) must not subtract distance
    dist = svs.cumulative_distance(t, v)
    assert np.all(np.diff(dist) >= 0.0)


def test_nearest_known_stop_distance_only_looks_at_the_same_branch():
    stops = [(0, 100.0), (0, 300.0), (1, 90.0)]
    assert svs.nearest_known_stop_m(0, 110.0, stops) == pytest.approx(10.0)
    assert svs.nearest_known_stop_m(1, 100.0, stops) == pytest.approx(10.0)


def test_nearest_known_stop_distance_is_infinite_without_a_place_on_the_branch():
    assert svs.nearest_known_stop_m(2, 0.0, [(0, 100.0)]) == float('inf')


def test_summarize_excludes_uncovered_branches_from_the_percentiles(capsys):
    events = [
        svs.StopEvent('b0', 0.0, 10.0, 10.0, 100.0, 5.0, 5.0, branch=0, s_m=110.0, nearest_known_m=10.0),
        svs.StopEvent('b0', 20.0, 30.0, 10.0, 50.0, 5.0, 5.0, branch=2, s_m=5.0, nearest_known_m=float('inf')),
    ]
    svs.summarize(events)
    out = capsys.readouterr().out
    assert '1 on a branch with no known stop place' in out
    # an `inf` leaking into the percentiles would make p50 infinite and <=20m less than 100%:
    # this is the case the inf-percentile bug (fixed alongside this test) silently broke.
    assert 'p50=10.0' in out
    assert '<=20m: 100.0%' in out
    assert '<=5m: 0.0%' in out


def _straight_route(length=100.0, step=1.0):
    s = np.arange(0.0, length + step, step)
    poly = np.column_stack([s, np.zeros_like(s)])
    return {0: (s, poly, np.zeros_like(s))}


def test_true_position_of_a_stop_in_the_middle_of_a_branch():
    route = _straight_route()
    ft = np.array([10.0, 11.0, 12.0])
    xy = np.array([[50.0, 0.0], [50.2, 0.0], [49.8, 0.0]])
    branch, s = svs.true_position(ft, xy, route, 10.0, 12.0)
    assert branch == 0 and s == pytest.approx(50.0, abs=0.5)


def test_true_position_near_a_branch_end_is_rejected():
    # clamped onto the branch end (within build_stops.EDGE_M): a real place there cannot be
    # told apart from a stop just off the mapped end -- same rule as build_stops.stops_of_bag
    route = _straight_route()
    ft = np.array([10.0, 11.0, 12.0])
    xy = np.array([[0.5, 0.0], [0.5, 0.0], [0.5, 0.0]])
    assert svs.true_position(ft, xy, route, 10.0, 12.0) == (None, None)


def test_true_position_needs_at_least_three_fixes_in_the_window():
    route = _straight_route()
    ft = np.array([10.0, 11.0])
    xy = np.array([[50.0, 0.0], [50.0, 0.0]])
    assert svs.true_position(ft, xy, route, 10.0, 12.0) == (None, None)


def test_read_stops_csv_skips_header_and_comment(tmp_path):
    p = tmp_path / 'stops.csv'
    p.write_text('# header comment\nbranch,s_m,n_bags\n0,146.4,17\n1,50.0,9\n', encoding='utf-8')
    assert svs.read_stops_csv(p) == [(0, 146.4), (1, 50.0)]
