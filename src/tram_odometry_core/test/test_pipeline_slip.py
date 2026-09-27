"""One bogie spiking next to an honest 0 through Odometry.step (#76, stress `spike`).
Messages are minimal stand-ins for the ROS ones."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from tram_odometry_core.pipeline import Odometry
from tram_odometry_core.types import load_params

ROOT = Path(__file__).resolve().parents[3]
PARAMS = load_params(ROOT / 'src/tram_odometry/config/params.yaml')
FRONT, REAR, CMD = ('/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity',
                    '/vehicle/driver_position_cmd')
T0 = 1_700_000_000.0


def _msg(t, **fields):
    sec = int(t)
    stamp = SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9)))
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp), **fields)


def _stand(front_kmh, rear_kmh):
    """Tram at rest, controller neutral, for 2 s, then 5 s of disturbed bogies; returns the
    estimates."""
    odo, out = Odometry(PARAMS), []
    for k in range(70):
        t = T0 + 0.1 * k
        f, r = (front_kmh(k), rear_kmh(k)) if t >= T0 + 2.0 else (0.0, 0.0)
        for topic, msg in ((CMD, _msg(t, position=0)), (FRONT, _msg(t, velocity=f)),
                           (REAR, _msg(t, velocity=r))):
            e = odo.step((topic, msg))
            if e is not None:
                out.append(e)
    return out


def test_spike_of_one_bogie_at_rest_does_not_move_the_tram():
    out = _stand(lambda k: 25.0, lambda k: 0.0)
    assert max(e.speed for e in out) < PARAMS.slip.front_rear_threshold_mps
    assert out[-1].distance < 1.0
    assert out[-1].slip.slip_front and not out[-1].slip.slip_rear


def test_run_starting_in_motion_with_the_rear_stuck_at_zero_follows_the_front():
    # coasting (notch 0) from the first message, the rear stuck at 0 (trap 8): the zero rule
    # without traction is limited by the prediction (#76), the estimate must not lock at 0
    odo, est = Odometry(PARAMS), None
    for k in range(20):
        t = T0 + 0.1 * k
        odo.step((CMD, _msg(t, position=0)))
        odo.step((FRONT, _msg(t, velocity=36.0)))
        est = odo.step((REAR, _msg(t, velocity=0.0)))
    assert est.speed == pytest.approx(10.0, abs=0.1)
    assert est.slip.slip_rear and not est.slip.slip_front


def test_antiphase_noise_of_both_bogies_while_cruising_stays_near_the_truth():
    # stress `noise` (#82): +-3 km/h at 1 Hz on the front, in antiphase on the rear, 10 s at
    # a steady 50 km/h; the mean of the bogies is the truth, a single bogie is off by 0.83 m/s
    odo, out, v = Odometry(PARAMS), [], 50.0
    for k in range(150):
        t = T0 + 0.1 * k
        n = 3.0 * math.sin(2 * math.pi * k / 10) if k >= 30 else 0.0
        for topic, msg in ((CMD, _msg(t, position=0)), (FRONT, _msg(t + 0.01, velocity=v + n)),
                           (REAR, _msg(t + 0.05, velocity=v - n))):
            e = odo.step((topic, msg))
            if e is not None and t >= T0 + 3.0:
                out.append(e.speed)
    assert max(abs(s - v / 3.6) for s in out) < 0.5


@pytest.mark.parametrize('n_future', [2, 3])
def test_a_clock_glitch_of_one_bogie_does_not_freeze_the_jump_memory(n_future):
    # review of #99: the front sends samples stamped a day ahead (the last one a step up);
    # preprocess accepts them and resyncs back (D-043). Later the rear slides alone at
    # -4 m/s^2: the detector must still see a single-bogie slide, not antiphase noise
    odo, errors, v = Odometry(PARAMS), [], 36.0
    for k in range(200):
        t = T0 + 0.1 * k
        msgs = [(CMD, _msg(t, position=0))]
        if k == 40:
            msgs += [(FRONT, _msg(t + 86400.0 + 0.1 * i,
                                  velocity=v + (1.8 if i == n_future - 1 else 0.0)))
                     for i in range(n_future)]
        else:
            msgs.append((FRONT, _msg(t + 0.01, velocity=v)))
        r = v - 1.44 * min(max(k - 99, 0), 30)
        msgs.append((REAR, _msg(t + 0.05, velocity=max(r, 0.0))))
        for topic, msg in msgs:
            e = odo.step((topic, msg))
            if e is not None and k >= 90:
                errors.append(abs(e.speed - v / 3.6))
    assert max(errors) < PARAMS.slip.front_rear_threshold_mps


def _spin_run(extra_front, extra_rear, notch=8, v0=3.0, seconds=12.0, spin=(4.0, 6.0)):
    """Car on `notch` from v0 by the drive model (the truth); in `spin` both bogies run
    `extra_*` m/s^2 above it, then drop back to it within 0.5 s (30618_33bec73f, #156).
    Returns [(t from T0, estimate, truth)] of the wheel outputs."""
    from tram_odometry_core.dynamics import model_accel
    odo, out, car, lead = Odometry(PARAMS), [], v0, {'front': 0.0, 'rear': 0.0}
    extra = {'front': extra_front, 'rear': extra_rear}
    for k in range(int(seconds * 10)):
        t = T0 + 0.1 * k
        s = 0.1 * k
        for side in lead:
            if spin[0] <= s < spin[1]:
                lead[side] += extra[side] * 0.1
            elif s >= spin[1]:
                lead[side] = max(0.0, lead[side] - 0.2 * extra[side] * 0.1 * 5)
        for topic, msg in ((CMD, _msg(t, position=notch)),
                           (FRONT, _msg(t, velocity=(car + lead['front']) * 3.6)),
                           (REAR, _msg(t, velocity=(car + lead['rear']) * 3.6))):
            e = odo.step((topic, msg))
            if e is not None and topic != CMD:
                out.append((s, e, car))
        car += model_accel(odo._notch_at(t), car, PARAMS) * 0.1
    return out


def test_spin_of_both_bogies_does_not_drag_the_speed_up():
    # both bogies spin 1.2 and 2.8 m/s^2 faster than the car for 2 s, each step below the jump
    # limit: the pair rules followed the slower one to 2.4 m/s off (holdout 33bec73f: 1.5).
    # The slide is seen after a window of evidence; from then on the filter runs on the model
    # from the car speed of the slide windows and trusts the bogies again once they are back
    out = _spin_run(1.2, 2.8)
    err = {round(s, 1): abs(e.speed - car) for s, e, car in out}
    assert max(err.values()) < 0.8
    assert max(v for s, v in err.items() if 5.0 <= s < 7.0) < 0.4
    assert any(e.slip.slip_front and e.slip.slip_rear for s, e, car in out if 4.0 <= s < 6.5)
    assert not out[-1][1].slip.slip_front and not out[-1][1].slip.slip_rear
    assert err[round(out[-1][0], 1)] < 0.1
