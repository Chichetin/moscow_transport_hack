"""Speed filter inside Odometry.step (#105): antiphase noise of both bogies at rest and while
cruising, as the stress `noise` scenario of tools/eval makes it. Messages are minimal stand-ins
for the ROS ones."""
import math
from pathlib import Path
from types import SimpleNamespace

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


def _noise_run(kmh, notch=0, seconds=30.0):
    """Both bogies at `kmh`, 10 Hz; from 10 s to 20 s the stress `noise` disturbance:
    +3 km/h * sin(2 pi n / 10) on the n-th front sample, the rear in antiphase
    (tools/eval/tram_eval/stress.py). Negative readings are what the sensor would send at rest.
    Returns (time from T0, Estimate) of every output."""
    odo, out, n = Odometry(PARAMS), [], 0
    for k in range(int(seconds * 10)):
        t = T0 + 0.1 * k
        f = r = kmh
        if 10.0 <= 0.1 * k < 20.0:
            n += 1
            f = kmh + 3.0 * math.sin(2 * math.pi * n / 10)
            r = kmh + 3.0 * math.sin(2 * math.pi * n / 10 + math.pi)
        for topic, msg in ((CMD, _msg(t, position=notch)), (FRONT, _msg(t + 0.01, velocity=f)),
                           (REAR, _msg(t + 0.04, velocity=r)), (CMD, _msg(t + 0.05, position=notch))):
            e = odo.step((topic, msg))
            if e is not None:
                out.append((e.t - T0, e))
    return out


def test_antiphase_noise_at_rest_does_not_move_the_tram():
    # at rest only the positive half of each bogie passes preprocess (kmh < 0 is dropped): a
    # filter fed the single readings sees 0.5-0.8 m/s for the whole event and drives 5.3 m.
    # The pair rule holds the car while the other bogie last read ~0; it leaks in the periods
    # where that bogie's zero crossing came out negative (-1e-16 km/h) and was dropped too,
    # so its last reading is 0.49 m/s (#105). 2 m is the stress position-recovery threshold.
    out = _noise_run(0.0)
    during = [e.speed for t, e in out if 10.0 <= t < 20.0]
    assert during
    assert sum(s < PARAMS.position.stop_speed_mps for s in during) > 0.7 * len(during)
    assert out[-1][1].distance < 2.0
    assert all(e.speed < PARAMS.position.stop_speed_mps for t, e in out if t >= 21.0)
    assert out[-1][1].speed == 0.0


def test_antiphase_noise_while_cruising_follows_the_mean_of_the_bogies():
    # the mean of the pair is the truth; a filter that fuses the bogies one by one follows the
    # first of each pair and the NIS gate then rejects its opposite partner
    out = _noise_run(36.0)
    errors = [abs(e.speed - 10.0) for t, e in out if 5.0 <= t]
    assert max(errors) < 0.3
