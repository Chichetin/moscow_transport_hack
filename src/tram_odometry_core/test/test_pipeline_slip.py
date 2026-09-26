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


def test_antiphase_noise_of_both_bogies_at_rest_does_not_move_the_tram():
    # stress `noise` at a standstill (#105): +-3 km/h at 1 Hz, rear in antiphase. Half of the
    # readings are negative; dropping them keeps only the positive half (mean 0.53 m/s) and the
    # standing tram drives off. The readings as measured average to the true 0.
    def noise(sign):
        return lambda k: sign * 3.0 * math.sin(2 * math.pi * k / 10)
    out = _stand(noise(1.0), noise(-1.0))
    assert sum(e.speed for e in out) / len(out) < 0.1
    assert out[-1].distance < 0.5


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
