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


def test_opposite_bogie_noise_does_not_turn_into_speed_or_distance_error():
    # Both bogies measure the same constant motion; their 3 km/h disturbances cancel.
    # The real estimator sees each wheel in turn, as it does during bag playback.
    odo = Odometry(PARAMS)
    estimates = []
    for k in range(120):
        t = T0 + 0.1 * k
        noise_kmh = 3.0 * math.sin(2.0 * math.pi * k / 10.0) if k >= 10 else 0.0
        for topic, msg in ((CMD, _msg(t, position=0)),
                           (FRONT, _msg(t, velocity=36.0 + noise_kmh)),
                           (REAR, _msg(t, velocity=36.0 - noise_kmh))):
            estimate = odo.step((topic, msg))
            if estimate is not None:
                estimates.append(estimate)
    noisy = [e for e in estimates if e.t >= T0 + 1.0]
    assert max(abs(e.speed - 10.0) for e in noisy) < 0.3
    assert abs(noisy[-1].distance - 119 * 0.1 * 10.0) < 0.5


def test_opposite_bogie_noise_during_acceleration_stays_near_true_speed():
    odo = Odometry(PARAMS)
    errors = []
    for k in range(120):
        t = T0 + 0.1 * k
        truth = 2.0 + 0.05 * k
        noise_kmh = 3.0 * math.sin(2.0 * math.pi * k / 10.0) if k >= 10 else 0.0
        for topic, msg in ((CMD, _msg(t, position=0)),
                           (FRONT, _msg(t, velocity=3.6 * truth + noise_kmh)),
                           (REAR, _msg(t, velocity=3.6 * truth - noise_kmh))):
            estimate = odo.step((topic, msg))
            if estimate is not None and k >= 10:
                errors.append(abs(estimate.speed - truth))
    assert max(errors) < 0.3


def test_missing_second_wheel_does_not_hold_speed_for_stale_timeout():
    odo = Odometry(PARAMS)
    for k in range(2):
        t = T0 + 0.1 * k
        odo.step((CMD, _msg(t, position=0)))
        odo.step((FRONT, _msg(t, velocity=36.0)))
        odo.step((REAR, _msg(t, velocity=36.0)))
    odo.step((FRONT, _msg(T0 + 0.2, velocity=3.6 * 10.3)))
    estimate = odo.step((FRONT, _msg(T0 + 0.3, velocity=3.6 * 10.45)))
    assert estimate.speed > 10.1
