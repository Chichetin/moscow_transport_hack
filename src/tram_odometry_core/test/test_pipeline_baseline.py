import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from tram_odometry_core.pipeline import Odometry
from tram_odometry_core.types import load_params

ROOT = Path(__file__).resolve().parents[3]
PARAMS = load_params(ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
FRONT = '/vehicle/front_bogie_velocity'
REAR = '/vehicle/rear_bogie_velocity'
CMD = '/vehicle/driver_position_cmd'
MASTER_FIX = '/sensing/gnss/master/fix'
MASTER_VEL = '/sensing/gnss/master/vel'
KMH_36 = 36.0  # = 10 m/s


def _hdr(t):
    sec = math.floor(t)
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=round((t - sec) * 1e9)))


def wheel(topic, t, kmh):
    return topic, SimpleNamespace(header=_hdr(t), velocity=kmh)


def cmd(t, notch):
    return CMD, SimpleNamespace(header=_hdr(t), position=notch)


def gnss_fix(t, status=0):
    return MASTER_FIX, SimpleNamespace(header=_hdr(t), latitude=55.7, longitude=37.6,
                                       altitude=150.0, status=SimpleNamespace(status=status))


def gnss_vel(t, ve, vn):
    return MASTER_VEL, SimpleNamespace(
        header=_hdr(t), twist=SimpleNamespace(linear=SimpleNamespace(x=ve, y=vn, z=0.0)))


def drive(odo, t0, t1, kmh, front=True, rear=True, dt=0.1):
    est, n = None, 0
    while t0 + n * dt <= t1 + 1e-9:
        t = t0 + n * dt
        for on, topic in ((front, FRONT), (rear, REAR)):
            if on:
                est = odo.step(wheel(topic, t, kmh)) or est
        n += 1
    return est


def init_east(odo, t=0.0):
    odo.step(gnss_fix(t))
    odo.step(gnss_vel(t, 10.0, 0.0))


def test_kmh_to_mps():
    est = Odometry(PARAMS).step(wheel(FRONT, 1.0, KMH_36))
    assert est.speed == pytest.approx(10.0)


def test_uniform_motion_distance_is_v_times_t():
    odo = Odometry(PARAMS)
    init_east(odo)
    est = drive(odo, 1.0, 11.0, KMH_36)
    assert est.distance == pytest.approx(100.0, abs=1.5)
    assert est.x == pytest.approx(est.distance, abs=1e-6)
    assert est.y == pytest.approx(0.0, abs=1e-6)
    assert est.gnss_used


def test_heading_from_gnss_velocity_north():
    odo = Odometry(PARAMS)
    odo.step(gnss_fix(0.0))
    odo.step(gnss_vel(0.0, 0.0, 8.0))
    est = drive(odo, 1.0, 5.0, KMH_36)
    assert est.yaw == pytest.approx(math.pi / 2)
    assert est.x == pytest.approx(0.0, abs=1e-6)
    assert est.y == pytest.approx(est.distance)


def test_silent_bogie_speed_from_the_other_one():
    odo = Odometry(PARAMS)
    drive(odo, 0.0, 2.0, KMH_36)                      # both alive
    est = drive(odo, 3.0, 20.0, 72.0, rear=False)     # rear silent >> stale_timeout_s
    assert est.speed == pytest.approx(20.0)


def test_both_silent_holds_last_speed_and_keeps_moving():
    odo = Odometry(PARAMS)
    init_east(odo)
    drive(odo, 1.0, 3.0, KMH_36)
    d0 = odo.step(wheel(FRONT, 3.05, KMH_36)).distance
    est = odo.step(cmd(10.0, 0))                      # only the controller talks for 7 s
    assert est.speed == pytest.approx(10.0)
    assert est.distance > d0 + 50.0


def test_non_monotonic_stamp_dropped_and_no_rollback():
    odo = Odometry(PARAMS)
    init_east(odo)
    est = drive(odo, 1.0, 3.0, KMH_36)
    assert odo.step(wheel(FRONT, 2.0, 0.0)) is None
    assert odo.step(wheel(FRONT, est.t, KMH_36)) is None   # repeated stamp
    nxt = odo.step(wheel(FRONT, est.t + 0.1, KMH_36))
    assert nxt.distance >= est.distance


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'), -5.0])
def test_bad_wheel_value_dropped_without_crash(bad):
    odo = Odometry(PARAMS)
    assert odo.step(wheel(FRONT, 1.0, bad)) is None
    assert odo.step(wheel(FRONT, 1.1, KMH_36)).speed == pytest.approx(10.0)


def test_garbage_input_dropped():
    odo = Odometry(PARAMS)
    assert odo.step(('/unknown', SimpleNamespace())) is None
    assert odo.step((FRONT, SimpleNamespace())) is None
    assert odo.step(None) is None


def test_output_stamp_equals_input_stamp():
    odo = Odometry(PARAMS)
    for topic, t in ((FRONT, 5.25), (REAR, 5.3), (CMD, 5.31)):
        msg = cmd(t, 3) if topic == CMD else wheel(topic, t, KMH_36)
        assert odo.step(msg).t == pytest.approx(t)
    # an older stamp of another stream is still published at its own stamp
    assert odo.step(wheel(FRONT, 5.35, KMH_36)).t == pytest.approx(5.35)
    assert odo.step(cmd(5.32, 3)).t == pytest.approx(5.32)


def test_gnss_after_window_is_ignored():
    def run(late):
        odo = Odometry(PARAMS)
        init_east(odo)
        drive(odo, 1.0, 5.0, KMH_36)
        if late:
            t = PARAMS.gnss.init_window_s + 1.0
            odo.step(gnss_fix(t))
            odo.step(gnss_vel(t, 0.0, 30.0))
        return drive(odo, 6.5, 12.0, KMH_36)
    a, b = run(False), run(True)
    assert (a.x, a.y, a.yaw, a.distance) == (b.x, b.y, b.yaw, b.distance)


def test_gnss_late_never_initialises():
    odo = Odometry(PARAMS)
    odo.step(wheel(FRONT, 0.0, KMH_36))
    t = PARAMS.gnss.init_window_s + 1.0
    odo.step(gnss_fix(t))
    odo.step(gnss_vel(t, 0.0, 30.0))
    est = odo.step(wheel(FRONT, t + 0.1, KMH_36))
    assert not est.gnss_used and est.yaw == 0.0


def test_estimate_is_finite_and_filled():
    odo = Odometry(PARAMS)
    est = drive(odo, 0.0, 2.0, KMH_36)
    vals = [est.speed, est.speed_var, est.accel, est.accel_model, est.distance,
            est.x, est.y, est.yaw, *est.pos_cov]
    assert all(math.isfinite(v) for v in vals)
    assert est.speed >= 0.0 and est.speed_var > 0.0


def test_gnss_vel_without_valid_fix_is_ignored():
    odo = Odometry(PARAMS)
    odo.step(gnss_vel(0.0, 0.0, 8.0))
    odo.step(gnss_fix(0.1, status=-1))
    odo.step(gnss_vel(0.2, 0.0, 8.0))
    est = odo.step(wheel(FRONT, 0.3, KMH_36))
    assert not est.gnss_used and est.yaw == 0.0
