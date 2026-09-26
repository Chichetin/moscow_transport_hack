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


def test_process_covariance_respects_seconds_and_bias_units():
    filt = initialized()
    filt.predict(2.0, 0.0)
    expected = (PARAMS.filter.r_wheel
                + PARAMS.filter.initial_bias_var * 2.0 ** 2
                + PARAMS.filter.q_accel * 2.0
                + PARAMS.filter.q_bias * 2.0 ** 3 / 3.0)
    assert filt.state()[1] == pytest.approx(expected)


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


def test_live_wheels_behind_controller_remain_fresh_on_wheel_timeline():
    filt = SpeedFilter(PARAMS)
    filt.predict(1.0, 0.0)
    filt.update(wheel(0.0, 10.0), 1.0)
    assert filt.diagnostics().accepted
    assert filt.state()[0] == pytest.approx(10.0)
    filt.predict(1.1, 0.0)
    filt.update(wheel(0.1, 10.0), 1.0)
    assert filt.diagnostics().accepted
    assert filt.state()[0] == pytest.approx(10.0, abs=0.01)


def test_wheel_behind_latest_wheel_is_stale_even_when_controller_ahead():
    filt = SpeedFilter(PARAMS)
    filt.predict(2.0, 0.0)
    filt.update(wheel(1.0, 10.0), 1.0)
    before = filt.state()
    filt.update(wheel(0.0, 20.0, 'rear'), 1.0)
    assert filt.state() == before
    assert filt.diagnostics() is None


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


def test_two_independent_zero_wheels_confirm_a_stop():
    filt = initialized()
    filt.update(wheel(0.1, 0.0, 'front'), 0.0)
    filt.update(wheel(0.1, 0.0, 'rear'), 1.0)
    assert filt.state()[0] < 0.1
    assert filt.diagnostics().accepted
    assert filt.diagnostics().nis <= PARAMS.filter.nis_gate


def test_first_wheel_after_confirmed_stop_starts_new_motion_segment():
    filt = initialized()
    filt.update(wheel(0.1, 0.0, 'front'), 0.0)
    filt.update(wheel(0.1, 0.0, 'rear'), 1.0)
    filt.predict(0.2, 0.0)
    filt.update(wheel(0.2, 0.15, 'front'), 1.0)
    assert filt.state()[0] == pytest.approx(0.15, abs=0.02)


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


def test_braking_bias_released_near_rest():
    filt = SpeedFilter(PARAMS)
    for k in range(53):
        t = k / 10.0
        filt.predict(t, 0.0)
        filt.update(wheel(t, 3.0 - 0.5 * t), 1.0)
    assert PARAMS.position.stop_speed_mps < filt.state()[0] < 0.5
    assert filt.state()[2] > -0.05


def test_relative_wheel_scale_learned_before_one_bogie_gap():
    filt = SpeedFilter(PARAMS)
    for k in range(300):
        t = k * 0.1
        filt.predict(t, 0.0)
        filt.update(wheel(t, 10.1, 'front'), 1.0)
        filt.update(wheel(t, 9.9, 'rear'), 1.0)
    for k in range(300, 1000):
        t = k * 0.1
        filt.predict(t, 0.0)
        filt.update(wheel(t, 10.1, 'front'), 1.0)
    assert filt.state()[0] == pytest.approx(10.0, abs=0.03)


# --- #105: the two bogies are measured as a pair --------------------------------------------

def resting():
    """Both bogies at 0, the drive model decelerating (no traction), for 1 s."""
    filt = SpeedFilter(PARAMS)
    for k in range(11):
        t = k / 10.0
        filt.predict(t, -0.1)
        filt.update(wheel(t, 0.0, 'front'), 1.0)
        filt.update(wheel(t + 0.03, 0.0, 'rear'), 1.0)
    assert filt.state()[0] == 0.0
    return filt


def test_antiphase_pair_that_the_detector_distrusts_is_fused_as_its_mean():
    # the detector gives 0.5/0.5 to disagreeing bogies with opposite jumps (D-054): the
    # second of the pair must not be rejected by NIS after the first pulled the state
    filt = initialized()
    for k in range(1, 6):
        t = k / 10.0
        filt.predict(t, 0.0)
        filt.update(wheel(t, 10.8, 'front'), 0.5)
        filt.update(wheel(t, 9.2, 'rear'), 0.5)
        assert filt.diagnostics().accepted
    assert filt.state()[0] == pytest.approx(10.0, abs=0.1)


def test_partner_of_the_mean_is_brought_to_the_sample_stamp():
    # accelerating at 2 m/s^2 with antiphase noise, the rear 0.05 s ahead of the front: the
    # older reading is moved on by the acceleration, or the mean lags by a * dt / 2
    filt = SpeedFilter(PARAMS)
    for k in range(40):
        t = k / 10.0
        filt.predict(t, 2.0)
        filt.update(wheel(t, 10.0 + 2.0 * t - 0.8, 'rear'), 0.5)
        filt.predict(t + 0.05, 2.0)
        filt.update(wheel(t + 0.05, 10.0 + 2.0 * (t + 0.05) + 0.8, 'front'), 0.5)
    assert filt.state()[0] == pytest.approx(10.0 + 2.0 * 3.95, abs=0.03)


def test_untrusted_partner_is_left_out_of_the_mean():
    filt = initialized()
    filt.update(wheel(0.0, 10.0, 'rear'), 1.0)
    filt.update(wheel(0.1, 0.0, 'rear'), 0.0)          # the detector drops the rear now
    filt.predict(0.1, 0.0)
    filt.update(wheel(0.1, 10.4, 'front'), 0.5)
    assert filt.state()[0] > 10.1                      # not pulled toward the rear's 10.0


def test_a_jumping_bogie_does_not_start_a_car_standing_without_traction():
    filt = resting()
    for k in range(11, 21):                            # front jumps to 0.8 m/s, rear stays 0
        t = k / 10.0
        filt.predict(t, -0.1)
        filt.update(wheel(t, 0.8, 'front'), 1.0)
        filt.update(wheel(t + 0.03, 0.0, 'rear'), 1.0)
    assert filt.state()[0] < 0.05


def test_a_smooth_rise_of_one_bogie_starts_the_car_without_traction_in_the_model():
    # a start with the notch still 0 (response delay) or read as brake, the rear stuck at 0
    # (trap 8, 30639): the front rising at 1 m/s^2 is no jump, the car is not held at 0
    filt = resting()
    for k in range(11, 21):
        t = k / 10.0
        filt.predict(t, -0.1)
        filt.update(wheel(t, 1.0 * (t - 1.0), 'front'), 1.0)
        filt.update(wheel(t + 0.03, 0.0, 'rear'), 1.0)
    assert filt.state()[0] > 0.3


def test_both_bogies_start_the_car_even_without_traction_from_the_model():
    # a controller that says brake while both bogies move (wrong notch) must not lock the car
    filt = resting()
    for k in range(11, 41):
        t = k / 10.0
        v = 0.5 * (t - 1.0)
        filt.predict(t, -0.1)
        filt.update(wheel(t, v, 'front'), 1.0)
        filt.update(wheel(t + 0.03, v, 'rear'), 1.0)
    assert filt.state()[0] == pytest.approx(1.5, abs=0.2)


def test_one_bogie_starts_the_car_with_traction():
    # trap 8: the rear stuck at 0 while the drive pulls; the detector blames it (trust 0)
    filt = resting()
    for k in range(11, 21):
        t = k / 10.0
        filt.predict(t, 0.5)
        filt.update(wheel(t, 0.5 * (t - 1.0), 'front'), 1.0)
        filt.update(wheel(t + 0.03, 0.0, 'rear'), 0.0)
    assert filt.state()[0] == pytest.approx(0.5, abs=0.1)


def test_single_live_bogie_starts_the_car_when_the_other_is_silent():
    # 30639: one bogie silent for up to 73 s; the live one is all there is (single-bogie mode)
    filt = resting()
    for k in range(11, 41):
        t = k / 10.0
        filt.predict(t, -0.1)
        filt.update(wheel(t, 0.5 * (t - 1.0), 'front'), 1.0)
    assert filt.state()[0] == pytest.approx(1.5, abs=0.2)
