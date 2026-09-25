import dataclasses
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from tram_odometry_core.preprocess import CMD_TOPIC, FRONT_TOPIC, REAR_TOPIC, Preprocessor
from tram_odometry_core.types import CommandSample, GnssFix, GnssVel, WheelSample, load_params

ROOT = Path(__file__).resolve().parents[3]
PARAMS = load_params(ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
MASTER_FIX = PARAMS.gnss.topic_fix
MASTER_VEL = PARAMS.gnss.topic_vel
KMH_36 = 36.0  # = 10 m/s
ACCEL_LIMIT = PARAMS.input.max_wheel_accel_mps2


def _hdr(t):
    sec = math.floor(t)
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=round((t - sec) * 1e9)))


def wheel(topic, t, kmh):
    return topic, SimpleNamespace(header=_hdr(t), velocity=kmh)


def cmd(t, notch):
    return CMD_TOPIC, SimpleNamespace(header=_hdr(t), position=notch)


def gnss_fix(t, status=0, lat=55.7, lon=37.6, alt=150.0):
    return MASTER_FIX, SimpleNamespace(header=_hdr(t), latitude=lat, longitude=lon,
                                       altitude=alt, status=SimpleNamespace(status=status))


def gnss_vel(t, ve, vn):
    return MASTER_VEL, SimpleNamespace(
        header=_hdr(t), twist=SimpleNamespace(linear=SimpleNamespace(x=ve, y=vn, z=0.0)))


def test_kmh_to_mps_is_the_only_conversion():
    sample = Preprocessor(PARAMS).accept(wheel(FRONT_TOPIC, 1.0, KMH_36))
    assert isinstance(sample, WheelSample)
    assert sample.speed == pytest.approx(10.0)
    assert sample.bogie == 'front'
    assert sample.t == pytest.approx(1.0)


def test_wheel_scale_applied_per_bogie():
    p = dataclasses.replace(PARAMS, vehicle=dataclasses.replace(
        PARAMS.vehicle, wheel_scale_front=1.1, wheel_scale_rear=0.9))
    pre = Preprocessor(p)
    front = pre.accept(wheel(FRONT_TOPIC, 1.0, KMH_36))
    rear = pre.accept(wheel(REAR_TOPIC, 1.0, KMH_36))
    assert front.speed == pytest.approx(11.0)
    assert rear.speed == pytest.approx(9.0)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'), -5.0, 'text', True])
def test_bad_wheel_value_dropped(bad):
    assert Preprocessor(PARAMS).accept(wheel(FRONT_TOPIC, 1.0, bad)) is None


def test_non_monotonic_wheel_stamp_dropped_repeat_and_rollback():
    pre = Preprocessor(PARAMS)
    assert pre.accept(wheel(FRONT_TOPIC, 5.0, KMH_36)) is not None
    assert pre.accept(wheel(FRONT_TOPIC, 5.0, KMH_36)) is None    # repeated stamp
    assert pre.accept(wheel(FRONT_TOPIC, 4.0, KMH_36)) is None    # rollback
    assert pre.accept(wheel(FRONT_TOPIC, 5.1, KMH_36)) is not None


def test_streams_gate_independently():
    """A rollback on one bogie must not block the other stream or the controller (trap 6)."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 5.0, KMH_36))
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, KMH_36)) is None      # front rollback dropped
    assert pre.accept(wheel(REAR_TOPIC, 1.0, KMH_36)) is not None   # rear stream unaffected
    assert pre.accept(cmd(1.0, 0)) is not None                      # controller unaffected


def test_controller_20hz_rollback_dropped():
    """Trap 9: controller at 20 Hz can roll back up to 20 times in a bag."""
    pre = Preprocessor(PARAMS)
    assert pre.accept(cmd(1.0, 5)) is not None
    assert pre.accept(cmd(1.0, 5)) is None       # repeated stamp, two publishers (trap 9)
    assert pre.accept(cmd(0.9, 5)) is None       # rollback
    assert pre.accept(cmd(1.05, 5)) is not None


@pytest.mark.parametrize('notch', [3.0, '5', None, True])
def test_bad_command_value_dropped(notch):
    assert Preprocessor(PARAMS).accept(cmd(1.0, notch)) is None


def test_wheel_acceleration_outlier_dropped():
    """A jump faster than `input.max_wheel_accel_mps2` is a sensor glitch, not real driving."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.0))                          # 0 m/s
    jump_mps = ACCEL_LIMIT * 5.0                                      # far beyond the limit over 1 s
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, jump_mps * 3.6)) is None


def test_wheel_acceleration_outlier_does_not_poison_state():
    """A rejected sample must not become the new reference for the next comparison."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.0))
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, (ACCEL_LIMIT * 5.0) * 3.6)) is None   # rejected glitch
    within_mps = (ACCEL_LIMIT - 0.1) * 1.0                            # still close to the real 0 m/s
    sample = pre.accept(wheel(FRONT_TOPIC, 1.0 + 1e-6, within_mps * 3.6))
    assert sample is not None


def test_wheel_acceleration_within_limit_accepted():
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.0))
    ok_mps = ACCEL_LIMIT - 0.1                                        # dt=1 s, just under the limit
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, ok_mps * 3.6)) is not None


def test_wheel_deceleration_outlier_dropped_too():
    """The physical limit is on `|dv/dt|`, braking spikes are outliers just like traction ones."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 100.0))
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, 0.0)) is None


def test_first_wheel_sample_never_outlier_rejected():
    """No prior sample for the bogie: any non-negative finite speed is accepted."""
    assert Preprocessor(PARAMS).accept(wheel(FRONT_TOPIC, 1.0, 190.0)) is not None


def test_long_silent_bogie_resume_not_flagged_as_outlier():
    """Trap 7: a bogie silent up to 73 s; the large dt keeps the implied accel small."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(REAR_TOPIC, 0.0, 0.0))
    sample = pre.accept(wheel(REAR_TOPIC, 73.0, KMH_36))            # 10 m/s over 73 s
    assert sample is not None and sample.speed == pytest.approx(10.0)


def test_gnss_fix_within_window_accepted_status_not_filtered():
    """Preprocess only enforces the window (D-005); status filtering is pipeline's job."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))                     # sets t0
    sample = pre.accept(gnss_fix(1.0, status=-1))
    assert isinstance(sample, GnssFix) and sample.status == -1 and sample.antenna == 'master'


def test_gnss_fix_after_window_dropped():
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    t = PARAMS.gnss.init_window_s + 0.1
    assert pre.accept(gnss_fix(t)) is None


def test_gnss_vel_after_window_dropped():
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    t = PARAMS.gnss.init_window_s + 0.1
    assert pre.accept(gnss_vel(t, 1.0, 0.0)) is None


def test_gnss_vel_within_window_accepted():
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    sample = pre.accept(gnss_vel(1.0, 3.0, 4.0))
    assert isinstance(sample, GnssVel)
    assert sample.ve == pytest.approx(3.0) and sample.vn == pytest.approx(4.0)


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_gnss_vel_non_finite_dropped(bad):
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    assert pre.accept(gnss_vel(1.0, bad, 0.0)) is None


def test_unknown_topic_dropped():
    assert Preprocessor(PARAMS).accept(('/unknown', SimpleNamespace(header=_hdr(0.0)))) is None


def test_t0_is_stamp_of_first_input_regardless_of_topic():
    pre = Preprocessor(PARAMS)
    assert pre.t0 is None
    pre.accept(cmd(3.5, 0))
    assert pre.t0 == pytest.approx(3.5)
