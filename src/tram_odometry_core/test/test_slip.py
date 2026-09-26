from pathlib import Path

import pytest

from tram_odometry_core.slip import SlipDetector
from tram_odometry_core.types import WheelSample, load_params

ROOT = Path(__file__).resolve().parents[3]
P = load_params(ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
TOL = P.slip.front_rear_threshold_mps
ACC_TOL = P.slip.model_residual_threshold_mps2


def ws(bogie, t, v):
    return WheelSample(t=t, bogie=bogie, speed=v)


def upd(front, rear, est, accel_model=0.0, det=None):
    return (det or SlipDetector(P)).update(front, rear, accel_model, est)


def test_healthy_pair_full_trust():
    s = upd(ws('front', 1.0, 10.0), ws('rear', 1.0, 10.1), est=10.0)
    assert (s.front_trust, s.rear_trust) == (1.0, 1.0)
    assert not s.slip_front and not s.slip_rear


@pytest.mark.parametrize('stuck', ['front', 'rear'])
def test_bogie_stuck_at_zero_is_blamed(stuck):
    f, r = (0.0, 24.0) if stuck == 'front' else (24.0, 0.0)
    s = upd(ws('front', 1.0, f), ws('rear', 1.0, r), est=24.0)
    assert getattr(s, f'slip_{stuck}') and getattr(s, f'{stuck}_trust') == 0.0
    other = 'rear' if stuck == 'front' else 'front'
    assert not getattr(s, f'slip_{other}') and getattr(s, f'{other}_trust') == 1.0


def test_zero_reading_blamed_even_when_the_estimate_is_still_at_rest():
    # start of motion: the drive pulls, the estimate is 0, the rear still reads exactly 0
    # (dead zone / stuck); on train every such start had accel_model 0.38-0.43 m/s^2
    s = upd(ws('front', 1.0, 0.7), ws('rear', 1.0, 0.0), est=0.0, accel_model=0.4)
    assert s.slip_rear and s.rear_trust == 0.0 and s.front_trust == 1.0
    s = upd(ws('front', 1.0, 0.0), ws('rear', 1.0, 0.7), est=0.0, accel_model=0.4)
    assert s.slip_front and s.front_trust == 0.0 and s.rear_trust == 1.0


@pytest.mark.parametrize('accel_model', [0.0, -0.01, -1.5])
def test_spike_at_rest_without_traction_is_blamed_not_the_zero_bogie(accel_model):
    # stress `spike` (#76): the tram stands, the front jumps by 25 km/h, the rear honestly reads 0
    s = upd(ws('front', 1.0, 25.0 / 3.6), ws('rear', 1.0, 0.0), est=0.0, accel_model=accel_model)
    assert s.slip_front and s.front_trust == 0.0
    assert not s.slip_rear and s.rear_trust == 1.0


def test_spike_while_the_other_bogie_brakes_to_zero_is_blamed():
    det = SlipDetector(P)
    det.update(ws('front', 0.0, 0.11), ws('rear', 0.0, 0.11), -0.9, 0.11)
    s = det.update(ws('front', 0.1, 6.94), ws('rear', 0.1, 0.0), -0.9, 0.11)
    assert s.slip_front and s.front_trust == 0.0 and s.rear_trust == 1.0


def test_zero_reading_blamed_when_rolling_off_without_traction():
    # 30618_e9a34502: the tram rolls off with the controller braking; the rear is still in its
    # dead zone, the front agrees with the prediction but is further from it than 0 is
    s = upd(ws('front', 1.0, 0.57), ws('rear', 1.0, 0.0), est=0.26, accel_model=-0.01)
    assert s.slip_rear and s.rear_trust == 0.0 and s.front_trust == 1.0


def test_wheelspin_faster_bogie_is_blamed():
    s = upd(ws('front', 1.0, 15.0), ws('rear', 1.0, 10.0), est=10.0)
    assert s.slip_front and s.front_trust == 0.0 and s.rear_trust == 1.0


def test_bogie_never_heard_has_no_trust_and_no_flag():
    s = upd(ws('front', 1.0, 10.0), None, est=10.0)
    assert s.rear_trust == 0.0 and not s.slip_rear and s.front_trust == 1.0
    s = upd(None, None, est=None)
    assert (s.front_trust, s.rear_trust) == (0.0, 0.0) and not (s.slip_front or s.slip_rear)


def test_stale_bogie_flagged_as_failed_while_the_other_talks():
    late = 1.0 + P.input.stale_timeout_s + 0.1
    s = upd(ws('front', late, 10.0), ws('rear', 1.0, 10.0), est=10.0)
    assert s.rear_trust == 0.0 and s.slip_rear and s.front_trust == 1.0 and not s.slip_front
    s = upd(ws('front', 1.0, 10.0), ws('rear', late, 10.0), est=10.0)
    assert s.front_trust == 0.0 and s.slip_front and not s.slip_rear


def test_disagreeing_without_estimate_splits_trust():
    s = upd(ws('front', 1.0, 10.0), ws('rear', 1.0, 5.0), est=None)
    assert s.front_trust == 0.5 and s.rear_trust == 0.5


def test_bogies_agree_against_stale_estimate_both_trusted():
    s = upd(ws('front', 1.0, 20.0), ws('rear', 1.0, 20.2), est=10.0)
    assert (s.front_trust, s.rear_trust) == (1.0, 1.0) and not s.slip_front


def test_model_accel_shifts_the_prediction():
    det = SlipDetector(P)
    det.update(ws('front', 0.0, 10.0), ws('rear', 0.0, 10.0), 0.0, 10.0)
    # one second later the model says +2 m/s^2: the true speed is ~12, the rear still reads 10
    s = det.update(ws('front', 1.0, 12.0), ws('rear', 1.0, 10.0), 2.0, 10.0)
    assert s.slip_rear and not s.slip_front


def test_tolerance_grows_with_gap():
    det = SlipDetector(P)
    det.update(ws('front', 0.0, 10.0), ws('rear', 0.0, 10.0), 0.0, 10.0)
    dev = TOL + 0.5 * ACC_TOL          # inside tolerance after 1 s, outside after 0.1 s
    s = det.update(ws('front', 1.0, 10.0 + dev), ws('rear', 1.0, 10.0), 0.0, 10.0)
    assert not s.slip_front and not s.slip_rear
    det = SlipDetector(P)
    det.update(ws('front', 0.0, 10.0), ws('rear', 0.0, 10.0), 0.0, 10.0)
    s = det.update(ws('front', 0.1, 10.0 + dev), ws('rear', 0.1, 10.0), 0.0, 10.0)
    assert s.slip_front


def test_recovery_is_immediate():
    det = SlipDetector(P)
    bad = det.update(ws('front', 1.0, 24.0), ws('rear', 1.0, 0.0), 0.0, 24.0)
    assert bad.slip_rear
    good = det.update(ws('front', 1.1, 24.0), ws('rear', 1.1, 23.9), 0.0, 24.0)
    assert good.rear_trust == 1.0 and not good.slip_rear


def test_adhesion_unknown_without_dynamics():
    assert upd(ws('front', 1.0, 10.0), ws('rear', 1.0, 10.0), est=10.0).adhesion_est is None


# Jumps of both bogies (#82, D-054): a step faster than slip.noise_accel_mps2 against the model
JUMP = P.slip.noise_accel_mps2
HOLD = P.slip.noise_hold_s


def _pair(det, t, f, r, accel_model=0.0, est=10.0):
    return det.update(ws('front', t, f), ws('rear', t, r), accel_model, est)


def _jump_step(accel, accel_model=0.0):
    """Speed step over 0.1 s whose acceleration is `accel` above the model."""
    return (accel + accel_model) * 0.1


def test_antiphase_jumps_of_both_bogies_are_noise_and_split_trust():
    det = SlipDetector(P)
    _pair(det, 0.0, 10.0, 10.0)
    d = _jump_step(2 * JUMP)
    s = _pair(det, 0.1, 10.0 + d, 10.0 - d)
    assert (s.front_trust, s.rear_trust) == (0.5, 0.5)
    assert not s.slip_front and not s.slip_rear
    # half a period later the noise is at its peak: no step any more, still noise
    s = _pair(det, 0.3, 10.0 + d, 10.0 - d)
    assert (s.front_trust, s.rear_trust) == (0.5, 0.5)


def test_inphase_jumps_of_both_bogies_are_a_slide_and_trust_neither():
    # 30618_2050d396, 444-445 s: both bogies slide on braking, truth stays above both
    det = SlipDetector(P)
    _pair(det, 0.0, 5.8, 5.8, accel_model=-1.5)
    s = _pair(det, 0.1, 5.8 + _jump_step(-4 * JUMP, -1.5), 5.8 + _jump_step(-1.3 * JUMP, -1.5),
              accel_model=-1.5, est=5.8)
    assert (s.front_trust, s.rear_trust) == (0.0, 0.0)
    assert s.slip_front and s.slip_rear


def test_jump_of_one_bogie_is_left_to_the_prediction():
    det = SlipDetector(P)
    _pair(det, 0.0, 10.0, 10.0)
    s = _pair(det, 0.1, 10.0 + _jump_step(3 * JUMP), 10.0)
    assert s.slip_front and s.front_trust == 0.0 and s.rear_trust == 1.0


def test_steps_below_the_jump_limit_are_not_noise():
    det = SlipDetector(P)
    _pair(det, 0.0, 10.0, 10.0)
    d = _jump_step(0.9 * JUMP)
    _pair(det, 0.1, 10.0 + d, 10.0 - d)                      # agree: inside the tolerance
    s = _pair(det, 0.2, 10.0 + 2 * d, 10.0 - 2 * d, est=10.0 + d)
    assert (s.front_trust, s.rear_trust) == (1.0, 0.0)


def test_jumps_expire_after_the_hold():
    det = SlipDetector(P)
    _pair(det, 0.0, 10.0, 10.0)
    d = _jump_step(2 * JUMP)
    # both keep their values; while the jumps are recent the disagreement stays noise
    k = 1
    while 0.1 * k <= 0.1 + HOLD:
        s = _pair(det, 0.1 * k, 10.0 + d, 10.0 - d, est=10.0 - d)
        assert (s.front_trust, s.rear_trust) == (0.5, 0.5)
        k += 1
    # past the hold it is an ordinary disagreement: the bogie nearer the prediction wins
    s = _pair(det, 0.1 * k, 10.0 + d, 10.0 - d, est=10.0 - d)
    assert s.front_trust == 0.0 and s.rear_trust == 1.0


def test_a_bogie_following_hard_braking_is_not_jumping():
    # the model brakes hard: the front follows it (its own step is steep, but expected), the
    # rear slides away; only the rear jumped, so the prediction picks the front
    det = SlipDetector(P)
    a = -(JUMP + 0.5)                  # emergency braking, steeper than the jump limit
    _pair(det, 0.0, 10.0, 10.0, accel_model=a)
    s = _pair(det, 0.1, 10.0 + _jump_step(-0.4, a), 10.0 + _jump_step(-3 * JUMP, a),
              accel_model=a, est=10.0)
    assert s.front_trust == 1.0 and s.rear_trust == 0.0 and s.slip_rear


def test_repeated_or_past_stamp_of_a_bogie_records_no_jump():
    det = SlipDetector(P)
    _pair(det, 0.0, 10.0, 10.0)
    _pair(det, 0.1, 10.0, 10.0)
    far = 10.0 + _jump_step(3 * JUMP)
    for t in (0.1, 0.05):              # same stamp again, then a small rollback (trap 6)
        det.update(ws('front', t, far), ws('rear', 0.1, 10.0), 0.0, 10.0)
    assert all(v is None for v in det._t_jump.values())


def test_nan_model_acceleration_records_no_jump():
    det = SlipDetector(P)
    _pair(det, 0.0, 10.0, 10.0)
    d = _jump_step(2 * JUMP)
    _pair(det, 0.1, 10.0 + d, 10.0 - d, accel_model=float('nan'))
    assert all(v is None for v in det._t_jump.values())


def test_jumps_are_seen_again_after_the_clock_is_resynced_back():
    # two front samples a day ahead stay as its newest sample; after the resync (D-043) the
    # antiphase noise must be recognised again
    det = SlipDetector(P)
    day = 86400.0
    _pair(det, 0.0, 10.0, 10.0)
    det.update(ws('front', day, 10.0), ws('rear', 0.0, 10.0), 0.0, 10.0)
    det.update(ws('front', day + 0.1, 10.0), ws('rear', 0.0, 10.0), 0.0, 10.0)
    _pair(det, 1.0, 10.0, 10.0)
    d = _jump_step(2 * JUMP)
    # the prediction leans to the front: without the noise rule the front alone would win
    s = _pair(det, 1.1, 10.0 + d, 10.0 - d, est=10.0 + 0.5 * d)
    assert (s.front_trust, s.rear_trust) == (0.5, 0.5)


# Frozen bogie (#144): a bogie that repeats exactly the same non-zero reading on new stamps
# while the drive model changes the speed by more than slip.freeze_dv_mps is not trusted
FREEZE_N = P.slip.freeze_min_samples
FREEZE_DV = P.slip.freeze_dv_mps
DT = 0.1


def run_pair(det, n, front, rear, accel_model, t0=0.0):
    """Feed `n` updates at DT; `front`/`rear` map the step index to a speed. Last state."""
    s = None
    for k in range(n):
        t = t0 + k * DT
        est = (front(k) + rear(k)) / 2.0
        s = det.update(ws('front', t, front(k)), ws('rear', t, rear(k)), accel_model, est)
    return s


def steps_to_freeze(accel):
    """Updates after which a bogie frozen from step 0 is past both freeze thresholds."""
    return max(FREEZE_N, int(FREEZE_DV / (abs(accel) * DT)) + 2) + 1


def test_both_bogies_frozen_under_traction_are_not_trusted():
    det = SlipDetector(P)
    s = run_pair(det, steps_to_freeze(1.0), lambda k: 5.0, lambda k: 5.0, accel_model=1.0)
    assert (s.front_trust, s.rear_trust) == (0.0, 0.0)
    assert s.slip_front and s.slip_rear


def test_frozen_bogie_under_braking_is_not_trusted():
    det = SlipDetector(P)
    s = run_pair(det, steps_to_freeze(-1.0), lambda k: 8.0, lambda k: 8.0, accel_model=-1.0)
    assert (s.front_trust, s.rear_trust) == (0.0, 0.0)


@pytest.mark.parametrize('frozen', ['front', 'rear'])
def test_one_frozen_bogie_is_blamed_the_moving_one_trusted(frozen):
    # the frozen one stays within the pair tolerance of the moving one for a while
    n = steps_to_freeze(0.5)
    moving = lambda k: 5.0 + 0.05 * k       # noqa: E731
    still = lambda k: 5.0                   # noqa: E731
    f, r = (still, moving) if frozen == 'front' else (moving, still)
    s = run_pair(SlipDetector(P), n, f, r, accel_model=0.5)
    other = 'rear' if frozen == 'front' else 'front'
    assert getattr(s, f'slip_{frozen}') and getattr(s, f'{frozen}_trust') == 0.0
    assert not getattr(s, f'slip_{other}') and getattr(s, f'{other}_trust') == 1.0


def test_repeated_reading_while_coasting_is_trusted():
    # train 30639_3b3d9eb8: the rear repeats 4.8082 km/h 18 times over 1.8 s at notch 0
    s = run_pair(SlipDetector(P), 19, lambda k: 1.34 + 0.004 * k, lambda k: 1.3356,
                 accel_model=-0.05)
    assert (s.front_trust, s.rear_trust) == (1.0, 1.0)
    assert not s.slip_front and not s.slip_rear


def test_standstill_zero_under_brake_is_not_frozen():
    s = run_pair(SlipDetector(P), 100, lambda k: 0.0, lambda k: 0.0, accel_model=-1.5)
    assert (s.front_trust, s.rear_trust) == (1.0, 1.0)
    assert not s.slip_front and not s.slip_rear


def test_short_repeat_under_traction_is_trusted():
    # fewer than freeze_min_samples repeats: quantization, not a frozen sensor
    s = run_pair(SlipDetector(P), FREEZE_N, lambda k: 5.0, lambda k: 5.0, accel_model=3.0)
    assert (s.front_trust, s.rear_trust) == (1.0, 1.0)


def test_frozen_bogie_trusted_again_once_it_moves():
    det = SlipDetector(P)
    n = steps_to_freeze(1.0)
    run_pair(det, n, lambda k: 5.0, lambda k: 5.0, accel_model=1.0)
    s = run_pair(det, 1, lambda k: 5.05, lambda k: 5.05, accel_model=1.0, t0=n * DT)
    assert (s.front_trust, s.rear_trust) == (1.0, 1.0)
    assert not s.slip_front and not s.slip_rear


def test_repeated_stamp_is_not_a_frozen_sample():
    # the same sample offered again (the other bogie spoke) is not a new repeat
    det = SlipDetector(P)
    s = None
    for k in range(steps_to_freeze(1.0) * 3):
        t = k * DT
        s = det.update(ws('front', 0.0, 5.0), ws('rear', t, 5.0 + 0.1 * k), 1.0, 5.0 + 0.1 * k)
    assert not s.slip_rear and s.rear_trust == 1.0
    assert s.front_trust == 0.0        # silent past stale_timeout_s, not frozen


def test_clock_resync_forgets_the_frozen_run():
    det = SlipDetector(P)
    n = steps_to_freeze(1.0)
    run_pair(det, n - 1, lambda k: 5.0, lambda k: 5.0, accel_model=1.0, t0=100.0)
    # the input clock jumps back: the run before the resync does not count
    s = run_pair(det, 2, lambda k: 5.0, lambda k: 5.0, accel_model=1.0, t0=0.0)
    assert (s.front_trust, s.rear_trust) == (1.0, 1.0)
