"""Metrics on synthetic trajectories with a known error: shift, scale, delay, offset."""
import numpy as np
import pytest

from tram_eval.metrics import Estimates, bag_metrics, match_nearest, speed_modes, summarize
from tram_eval.reference import Reference, arc_polyline

L1, R = 300.0, 100.0          # straight east, then a quarter circle to the north


def curve(s):
    """Point at arc s of: straight east L1, left quarter circle of radius R, straight north."""
    s = np.asarray(s, float)
    th = np.clip((s - L1) / R, 0.0, np.pi / 2)
    x = np.where(s < L1, s, L1 + R * np.sin(th))
    y = np.where(s < L1, 0.0, R * (1 - np.cos(th)))
    past = s - L1 - R * np.pi / 2
    y = np.where(past > 0, R + past, y)
    return np.stack([x, y, np.zeros_like(x)], axis=-1)


def normal(s):
    th = np.clip((np.asarray(s, float) - L1) / R, 0.0, np.pi / 2)
    return np.stack([-np.sin(th), np.cos(th), np.zeros_like(th)], axis=-1)   # left of travel


def cruise(t):
    return 10.0 * t, np.full_like(t, 10.0)


def profile(t):
    """Stop 5 s, accelerate 1 m/s^2 for 10 s, cruise 60 s, brake 1 m/s^2 for 10 s, stop 5 s."""
    v = np.interp(t, [0, 5, 15, 75, 85, 90], [0, 0, 10, 10, 0, 0])
    s = np.concatenate([[0.0], np.cumsum((v[1:] + v[:-1]) / 2 * np.diff(t))])
    return s, v


def reference(motion, duration, rate=10.0):
    t = np.arange(0.0, duration + 1e-9, 1.0 / rate)
    s, v = motion(t)
    pos = curve(s)
    poly, poly_s, pos_s = arc_polyline(pos[:, :2])
    return Reference((55.75, 37.6, 150.0), t, pos, pos_s, poly, poly_s, t, v)


def estimates(motion, duration, ds=0.0, dv=0.0, scale=1.0, lateral=0.0, delay=0.0, dt=0.0, rate=40.0):
    t = np.arange(0.0, duration + 1e-9, 1.0 / rate)
    s, v = motion(np.maximum(t - delay, 0.0))
    s = s * scale + ds
    pos = curve(s) + lateral * normal(s)
    return Estimates(t + dt, v + dv, pos, np.zeros(len(t), bool))


def test_perfect_estimate_scores_zero():
    m = bag_metrics(reference(cruise, 100), estimates(cruise, 100))
    for k in ('speed_rmse', 'speed_mae', 'along_rmse', 'along_max', 'cross_rmse', 'pos3d_rmse', 'drift_pct'):
        assert m[k] == pytest.approx(0.0, abs=0.02), k
    assert m['n_matched'] == 1001
    assert m['distance_m'] == pytest.approx(1000.0, rel=1e-3)


def test_speed_offset_is_rmse_mae_and_bias_in_every_mode():
    m = bag_metrics(reference(profile, 90), estimates(profile, 90, dv=0.5))
    assert m['speed_rmse'] == pytest.approx(0.5)
    assert m['speed_mae'] == pytest.approx(0.5)
    for mode in ('accel', 'brake', 'stop', 'cruise'):
        assert m[f'speed_bias_{mode}'] == pytest.approx(0.5), mode


def test_along_shift_on_straight_and_curve():
    m = bag_metrics(reference(cruise, 100), estimates(cruise, 100, ds=-5.0))
    assert m['along_rmse'] == pytest.approx(5.0, abs=0.1)
    assert m['cross_mean'] == pytest.approx(0.0, abs=0.1)


def test_lateral_offset_is_cross_track_not_along():
    m = bag_metrics(reference(cruise, 100), estimates(cruise, 100, lateral=2.0))
    assert m['cross_mean'] == pytest.approx(2.0, abs=0.05)
    assert m['cross_max'] == pytest.approx(2.0, abs=0.05)
    assert m['along_max'] < 0.1
    assert m['pos3d_rmse'] == pytest.approx(2.0, abs=0.05)


def test_wheel_scale_one_percent_is_one_percent_drift():
    straight = lambda t: (10.0 * t, np.full_like(t, 10.0))   # noqa: E731
    ref = reference(straight, 25)                            # 250 m, stays on the straight
    m = bag_metrics(ref, estimates(straight, 25, scale=1.01))
    assert m['drift_pct'] == pytest.approx(1.0, abs=0.01)
    assert m['along_max'] == pytest.approx(2.5, abs=0.01)


def test_delay_shows_as_along_error_and_accel_brake_bias():
    tau = 0.5
    m = bag_metrics(reference(profile, 90), estimates(profile, 90, delay=tau))
    assert m['speed_bias_accel'] == pytest.approx(-1.0 * tau, abs=0.1)
    assert m['speed_bias_brake'] == pytest.approx(+1.0 * tau, abs=0.1)
    assert m['speed_bias_cruise'] == pytest.approx(0.0, abs=0.1)
    assert m['along_max'] == pytest.approx(10.0 * tau, abs=0.1)


def test_matching_tolerance_is_50_ms():
    ref = reference(cruise, 10, rate=5.0)     # 0.2 s apart: only the own stamp can be within 0.05 s
    assert bag_metrics(ref, estimates(cruise, 10, rate=5.0, dt=0.04))['n_matched'] == 51
    late = bag_metrics(ref, estimates(cruise, 10, rate=5.0, dt=0.06))
    assert late['n_matched'] == 0 and late['speed_rmse'] is None and late['along_rmse'] is None


def test_match_picks_nearest_even_with_unsorted_estimates():
    ri, ei = match_nearest(np.array([1.0, 2.0]), np.array([2.01, 0.98, 1.03, 5.0]))
    assert list(ri) == [0, 1] and list(ei) == [1, 0]


def test_short_run_has_no_drift():
    ref = reference(cruise, 4)                    # 40 m < 50 m
    m = bag_metrics(ref, estimates(cruise, 4, scale=1.1))
    assert m['drift_pct'] is None
    assert m['along_max'] > 0


def test_speed_modes_of_profile():
    t = np.arange(0.0, 90.0, 0.1)
    _, v = profile(t)
    mode = speed_modes(t, v)
    at = lambda sec: mode[int(sec * 10)]            # noqa: E731
    assert (at(2), at(10), at(40), at(80), at(88)) == ('stop', 'accel', 'cruise', 'brake', 'stop')
    assert at(5.1) == 'stop'                        # v < 0.3 wins over the start of the pull-away


def test_projection_stays_on_its_leg_of_a_hairpin():
    # Out 200 m east, back west on a parallel track 3 m north. On the way back the estimate
    # is 2 m south of its track, i.e. 1 m from the way out: the global nearest point is on
    # the wrong leg (along error ~ hundreds of m); the continuous projection must stay.
    t = np.arange(0.0, 40.0 + 1e-9, 0.1)
    s = 10.0 * t
    back = s > 200
    xy = np.stack([np.where(back, 400 - s, s), np.where(back, 3.0, 0.0)], -1)
    pos = np.column_stack([xy, np.zeros(len(t))])
    poly, poly_s, pos_s = arc_polyline(xy)
    ref = Reference(None, t, pos, pos_s, poly, poly_s, t, np.full(len(t), 10.0))
    est = pos.copy()
    est[back, 1] -= 2.0
    m = bag_metrics(ref, Estimates(t, np.full(len(t), 10.0), est, np.zeros(len(t), bool)))
    assert m['along_max'] < 3.5          # the 3 m U-turn chord, not the other leg
    assert m['cross_max'] == pytest.approx(2.0, abs=0.01)


def test_summary_median_and_worst_bag():
    bags = {'a': {'speed_rmse': 1.0, 'speed_bias_stop': -0.9, 'drift_pct': None},
            'b': {'speed_rmse': 3.0, 'speed_bias_stop': 0.1, 'drift_pct': 2.0},
            'c': {'speed_rmse': 2.0, 'speed_bias_stop': 0.5, 'drift_pct': 1.0}}
    s = summarize(bags)
    assert s['median']['speed_rmse'] == 2.0
    assert s['worst_bag']['speed_rmse'] == 'b'
    assert s['worst_bag']['speed_bias_stop'] == 'a'          # by |bias|
    assert s['median']['drift_pct'] == 1.5                    # None skipped
    assert s['median']['along_rmse'] is None
