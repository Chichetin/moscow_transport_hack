"""Behavior of the online speed filter, independently of ROS and the pipeline."""
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from tram_odometry_core.estimator import SpeedFilter
from tram_odometry_core.types import WheelSample, load_params

PARAMS = load_params(Path(__file__).resolve().parents[3] / 'src/tram_odometry/config/params.yaml')


def wheel(t, speed, bogie='front'):
    return WheelSample(t, bogie, speed)


def initialized():
    filt = SpeedFilter(PARAMS)
    filt.predict(0.0, 0.0)
    filt.update(wheel(0.0, 10.0), 1.0)
    return filt


def test_first_wheel_initializes_moving_start_without_zero_prior_bias():
    assert initialized().state() == pytest.approx((10.0, PARAMS.filter.r_wheel, 0.0))


def test_predict_integrates_model_and_grows_uncertainty():
    filt = initialized()
    filt.predict(2.0, 1.0)
    speed, variance, accel = filt.state()
    assert speed == pytest.approx(12.0)
    assert variance > PARAMS.filter.r_wheel
    assert accel == pytest.approx(1.0)


def test_zero_trust_rejects_measurement():
    filt = initialized()
    before = filt.state()
    filt.update(wheel(0.0, 100.0, 'rear'), 0.0)
    assert filt.state() == before


def test_lower_trust_reduces_measurement_influence():
    high, low = initialized(), initialized()
    high.update(wheel(0.0, 10.2, 'rear'), 1.0)
    low.update(wheel(0.0, 10.2, 'rear'), 0.1)
    assert 10.0 < low.state()[0] < high.state()[0] < 10.2
    assert low.state()[1] > high.state()[1]


def test_duplicate_same_bogie_is_not_counted_twice():
    filt = initialized()
    before = filt.state()
    filt.update(wheel(0.0, 11.0), 1.0)
    assert filt.state() == before
    filt.update(wheel(0.0, 10.2, 'rear'), 1.0)
    assert filt.state()[0] > before[0]


def test_past_predictions_and_measurements_do_not_rewind_state():
    filt = initialized()
    filt.predict(2.0, 1.0)
    before = filt.state()
    filt.predict(1.0, -2.0)
    filt.update(wheel(1.0, 0.0, 'rear'), 1.0)
    assert filt.state() == before


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'), -1.0])
def test_invalid_wheel_is_ignored(bad):
    filt = initialized()
    before = filt.state()
    filt.update(wheel(0.0, bad, 'rear'), 1.0)
    assert filt.state() == before


def test_braking_never_produces_negative_speed():
    filt = initialized()
    filt.predict(20.0, -1.0)
    assert filt.state()[0] == 0.0


def test_one_bogie_gap_73_seconds_and_return_remain_finite():
    filt = initialized()
    for k in range(1, 731):
        t = k / 10.0
        filt.predict(t, 0.0)
        filt.update(wheel(t, 10.0), 1.0)
        assert np.isfinite(filt.state()).all()
        assert filt.state()[1] >= 0.0
    filt.update(wheel(73.0, 10.0, 'rear'), 1.0)
    assert filt.state()[0] == pytest.approx(10.0)


def test_invalid_predict_does_not_poison_filter():
    filt = initialized()
    before = filt.state()
    for t, a in [(float('nan'), 0.0), (1.0, float('inf'))]:
        filt.predict(t, a)
        assert filt.state() == before


def test_delayed_fresh_wheel_updates_current_state_without_time_rollback():
    filt = initialized()
    filt.predict(0.2, 1.0)
    prior_variance = filt.state()[1]
    filt.update(wheel(0.1, 10.1), 1.0)
    assert filt.state()[0] == pytest.approx(10.2)
    assert filt.state()[1] < prior_variance
    filt.predict(0.3, 1.0)
    assert filt.state()[0] == pytest.approx(10.3)


def test_same_stamp_independent_wheel_update_matches_kalman_equations():
    filt = initialized()
    filt.update(wheel(0.0, 10.2, 'rear'), 1.0)
    assert filt.state()[0] == pytest.approx(10.1)
    assert filt.state()[1] == pytest.approx(PARAMS.filter.r_wheel / 2.0)


@pytest.mark.parametrize('trust', [float('nan'), float('inf'), -0.1, 1.1])
def test_invalid_trust_does_not_change_state(trust):
    filt = initialized()
    before = filt.state()
    filt.update(wheel(0.0, 10.2, 'rear'), trust)
    assert filt.state() == before


def test_bias_tracks_unmodelled_acceleration():
    filt = SpeedFilter(PARAMS)
    for k in range(41):
        t = k / 10.0
        filt.predict(t, 0.0)
        filt.update(wheel(t, 2.0 + 0.5 * t), 1.0)
    speed, variance, accel = filt.state()
    assert speed == pytest.approx(4.0, abs=0.12)
    assert accel > 0.1
    assert variance >= 0.0


def test_nis_rejects_outlier_and_identifies_measurement():
    filt = initialized()
    before = filt.state()
    filt.update(wheel(0.0, 100.0, 'rear'), 1.0)
    assert filt.state() == before
    diag = filt.diagnostics()
    assert (diag.t, diag.bogie, diag.accepted) == (0.0, 'rear', False)
    assert diag.nis > PARAMS.filter.nis_gate


def test_diagnostics_only_for_new_wheel_measurement():
    filt = initialized()
    assert filt.diagnostics() is not None
    filt.predict(0.1, 0.0)
    assert filt.diagnostics() is None
    filt.update(wheel(0.1, 10.0, 'rear'), 1.0)
    assert filt.diagnostics().accepted is True


def test_stop_does_not_preserve_braking_bias_into_next_start():
    filt = SpeedFilter(PARAMS)
    for k in range(101):
        t = k / 10.0
        filt.predict(t, 0.0)
        filt.update(wheel(t, max(0.0, 3.0 - 0.5 * t)), 1.0)
    assert filt.state()[0] < 0.1
    assert filt.state()[2] > -0.1
    for k in range(101, 121):
        t = k / 10.0
        filt.predict(t, 0.0)
        filt.update(wheel(t, 0.5 * (t - 10.0)), 1.0)
    assert filt.state()[0] == pytest.approx(1.0, abs=0.15)
