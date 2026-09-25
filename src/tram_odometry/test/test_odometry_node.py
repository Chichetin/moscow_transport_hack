"""Node tests (need ROS: bash docker/dev.sh python3 -m pytest src/tram_odometry/test)."""
import math
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor  # noqa: E402

from tram_odometry import odometry_node as on  # noqa: E402
from tram_odometry_core.types import Estimate, SlipState, load_params  # noqa: E402

PARAMS_FILE = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
STAMP = Time(sec=1756195560, nanosec=123456789)   # float seconds would lose the nanoseconds


def _estimate(t=1756195560.123456789):
    return Estimate(t=t, speed=7.5, speed_var=0.04, accel=0.3, accel_model=0.25,
                    distance=120.0, x=-35.0, y=12.5, yaw=math.pi / 3,
                    pos_cov=(4.0, 9.0, 1.5),
                    slip=SlipState(1.0, 1.0, False, False, None), gnss_used=False)


def _wheel(v=36.0):
    m = VelocitySensor()
    m.header.stamp = STAMP
    m.velocity = v
    return m


@pytest.fixture
def node():
    rclpy.init()
    n = on.OdometryNode(params_file=str(PARAMS_FILE))
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_raw_is_topic_and_untouched_message_like_eval(node, monkeypatch):
    seen = []
    monkeypatch.setattr(node.odometry, 'step', lambda raw: seen.append(raw))
    wheel, fix, vel = _wheel(36.0), NavSatFix(), TwistStamped()
    node.on_input('/vehicle/front_bogie_velocity', wheel)
    node.on_input('/sensing/gnss/master/fix', fix)
    node.on_input('/sensing/gnss/master/vel', vel)
    assert [t for t, _ in seen] == ['/vehicle/front_bogie_velocity', '/sensing/gnss/master/fix',
                                    '/sensing/gnss/master/vel']
    assert seen[0][1] is wheel and seen[0][1].velocity == 36.0   # km/h: units are the core's job
    assert seen[1][1] is fix and seen[2][1] is vel


def test_position_message_follows_contract():
    params = load_params(PARAMS_FILE)
    m = on.position_msg(_estimate(), STAMP, params)
    assert m.header.stamp == STAMP
    assert m.header.frame_id == 'map' and m.child_frame_id == 'base_link'
    p, q = m.pose.pose.position, m.pose.pose.orientation
    assert (p.x, p.y, p.z) == (-35.0, 12.5, 0.0)
    assert math.atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2) == pytest.approx(math.pi / 3)
    assert (q.x, q.y) == (0.0, 0.0) and q.w ** 2 + q.z ** 2 == pytest.approx(1.0)
    c = m.pose.covariance
    assert (c[0], c[7], c[1], c[6]) == (4.0, 9.0, 1.5, 1.5)
    assert min(c[14], c[21], c[28], c[35]) >= on.UNKNOWN_VAR and -1.0 not in list(c)
    assert m.twist.twist.linear.x == 7.5 and m.twist.covariance[0] == 0.04
    assert m.twist.covariance[7] >= on.UNKNOWN_VAR


def test_velocity_message_follows_contract():
    params = load_params(PARAMS_FILE)
    m = on.velocity_msg(_estimate(), STAMP, params)
    assert m.header.stamp == STAMP and m.header.frame_id == 'base_link'
    assert m.velocity == 7.5


def test_node_publishes_both_results_with_input_stamp(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node.odometry, 'step', lambda raw: _estimate())
    monkeypatch.setattr(node.pub_velocity, 'publish', lambda m: sent.append(('v', m)))
    monkeypatch.setattr(node.pub_position, 'publish', lambda m: sent.append(('p', m)))
    node.on_input('/vehicle/front_bogie_velocity', _wheel())
    assert [k for k, _ in sent] == ['v', 'p']
    assert all(m.header.stamp == STAMP for _, m in sent)


def test_node_publishes_nothing_when_input_dropped(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node.odometry, 'step', lambda raw: None)
    monkeypatch.setattr(node.pub_velocity, 'publish', sent.append)
    monkeypatch.setattr(node.pub_position, 'publish', sent.append)
    node.on_input('/vehicle/rear_bogie_velocity', _wheel(float('nan')))
    assert sent == []


def test_exception_in_pipeline_does_not_kill_node(node, monkeypatch):
    def boom(raw):
        raise ValueError('core bug')
    monkeypatch.setattr(node.odometry, 'step', boom)
    node.on_input('/vehicle/driver_position_cmd', DriverControllerCommand())
    node.on_input('/vehicle/front_bogie_velocity', _wheel())
    assert node.errors == 2


def test_node_subscribes_to_inputs_and_gnss_from_params(node):
    topics = {s.topic_name for s in node.subscriptions}
    assert topics == {'/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity',
                      '/vehicle/driver_position_cmd', '/sensing/gnss/master/fix',
                      '/sensing/gnss/rover/fix', '/sensing/gnss/master/vel'}


def test_subscriptions_are_best_effort_so_any_bag_publisher_connects(node):
    from rclpy.qos import ReliabilityPolicy
    for s in node.subscriptions:
        infos = node.get_subscriptions_info_by_topic(s.topic_name)
        assert infos, s.topic_name
        assert all(i.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for i in infos)
    assert on.INPUT_QOS.depth == on.INPUT_QUEUE      # the graph does not report depth (rmw: 0)


def test_node_without_params_file_uses_the_installed_one():
    rclpy.init()
    try:
        n = on.OdometryNode()
        assert n.params.gnss.topic_fix == '/sensing/gnss/master/fix'
        n.destroy_node()
    finally:
        rclpy.shutdown()


def _capture(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node.pub_velocity, 'publish', lambda m: sent.append(('v', m)))
    monkeypatch.setattr(node.pub_position, 'publish', lambda m: sent.append(('p', m)))
    return sent


def _stamp(sec, nanosec=0):
    return Time(sec=sec, nanosec=nanosec)


def test_real_core_converts_kmh_to_mps_once(node, monkeypatch):
    sent = _capture(node, monkeypatch)
    node.on_input('/vehicle/front_bogie_velocity', _wheel(36.0))
    p = node.params
    expected = 36.0 * p.input.wheel_speed_scale * p.vehicle.wheel_scale_front
    assert expected == pytest.approx(10.0, abs=0.5)      # km/h -> m/s happens in the core
    assert [k for k, _ in sent] == ['v', 'p']
    assert sent[0][1].velocity == pytest.approx(expected)
    assert sent[1][1].twist.twist.linear.x == pytest.approx(expected)
    assert sent[1][1].header.stamp == STAMP


def test_real_core_drops_nan_wheel_without_publishing(node, monkeypatch):
    sent = _capture(node, monkeypatch)
    node.on_input('/vehicle/front_bogie_velocity', _wheel(float('nan')))
    node.on_input('/vehicle/rear_bogie_velocity', _wheel(float('inf')))
    assert sent == [] and node.errors == 0


def test_real_core_ignores_gnss_after_window_through_the_node(node, monkeypatch):
    sent = _capture(node, monkeypatch)
    t0 = STAMP.sec
    w = _wheel(36.0)
    node.on_input('/vehicle/front_bogie_velocity', w)
    late = t0 + int(node.params.gnss.init_window_s) + 100
    fix = NavSatFix()
    fix.header.stamp = _stamp(late)
    fix.status.status = 2
    fix.latitude, fix.longitude, fix.altitude = 55.75, 37.62, 150.0
    vel = TwistStamped()
    vel.header.stamp = _stamp(late)
    vel.twist.linear.x, vel.twist.linear.y = 0.0, 5.0          # heading north if it were used
    node.on_input('/sensing/gnss/master/fix', fix)
    node.on_input('/sensing/gnss/master/vel', vel)
    w2 = _wheel(36.0)
    w2.header.stamp = _stamp(late, 50_000_000)
    node.on_input('/vehicle/front_bogie_velocity', w2)
    q = sent[-1][1].pose.pose.orientation
    assert sent[-1][0] == 'p' and q.z == 0.0 and q.w == 1.0    # yaw stayed 0: GNSS not used
    assert len(sent) == 4                                       # GNSS inputs publish nothing
