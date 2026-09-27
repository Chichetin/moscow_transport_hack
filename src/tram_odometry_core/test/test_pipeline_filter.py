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
    # at rest the negative half of each bogie reaches the filter only as 0 m/s (preprocess
    # clamps kmh < 0, D-064), the positive half as a jump: a filter fed the single readings
    # rides the positive halves (without the pair rule: half of the event above
    # stop_speed_mps, peak 0.7 m/s, 3.4 m of travel). The pair rule takes the slower bogie of
    # the pair, here 0 m, 0 m/s (#105). 2 m is the stress position-recovery threshold.
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


def test_the_filter_gets_the_detectors_current_trust_in_the_other_bogie():
    # the detector judges both bogies on every input; the other bogie's trust it had when that
    # bogie was fused is stale (#176): the pipeline passes the current one
    odo, seen, n = Odometry(PARAMS), [], 0
    update = odo._filter.update

    def spy(sample, trust, partner_trust=None):
        st = odo._slip_state
        seen.append((partner_trust, st.rear_trust if sample.bogie == 'front' else st.front_trust))
        return update(sample, trust, partner_trust)
    odo._filter.update = spy
    for k in range(200):
        t = T0 + 0.1 * k
        n += 1
        f = 36.0 + 3.0 * math.sin(2 * math.pi * n / 10)
        r = 36.0 + 3.0 * math.sin(2 * math.pi * n / 10 + math.pi)
        for topic, msg in ((CMD, _msg(t, position=0)), (REAR, _msg(t + 0.01, velocity=r)),
                           (FRONT, _msg(t + 0.01, velocity=f))):
            odo.step((topic, msg))
    assert seen and all(passed == current for passed, current in seen)
    assert any(current == 0.5 for _, current in seen)    # antiphase noise: the case that matters
