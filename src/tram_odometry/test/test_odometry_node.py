"""Node tests (need ROS: bash docker/dev.sh python3 -m pytest src/tram_odometry/test)."""
import math
import random
from dataclasses import replace
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from builtin_interfaces.msg import Time  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor  # noqa: E402

from tram_odometry import odometry_node as on  # noqa: E402
from tram_odometry_core.types import Estimate, SlipState, load_params  # noqa: E402

PARAMS_FILE = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
STAMP = Time(sec=1756195560, nanosec=123456789)   # float seconds would lose the nanoseconds


def _estimate(t=1756195560.123456789):
    return Estimate(t=t, speed=7.5, speed_var=0.04, accel=0.3, accel_model=0.25,
                    distance=120.0, x=-35.0, y=12.5, z=3.25, yaw=math.pi / 3,
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
    fix.header.stamp = STAMP
    vel.header.stamp = STAMP
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
    assert (p.x, p.y, p.z) == (-35.0, 12.5, 3.25)     # z: map height + run offset (D-024)
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
    cmd = DriverControllerCommand()
    cmd.header.stamp = STAMP
    node.on_input('/vehicle/driver_position_cmd', cmd)
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


def test_node_loads_the_installed_route_map_like_eval(node):
    assert node.params.position.use_map
    assert node.odometry.route is not None and len(node.odometry.route.branches) >= 2
    assert node.odometry._tracker is not None


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
    assert p.input.wheel_speed_scale == pytest.approx(1 / 3.6)   # km/h -> m/s, in the core only
    expected = 10.0 * p.vehicle.wheel_scale_front
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
    # after the GNSS window, but within input.max_stamp_jump_s of the last wheel sample: a lone
    # sample further ahead is treated as a clock glitch until confirmed (#77, D-043)
    late = t0 + int(node.params.gnss.init_window_s) + 2
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


def test_diagnostics_reports_slip_and_input_age_with_input_stamp(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node.pub_diagnostics, 'publish', sent.append)
    node.on_input('/vehicle/front_bogie_velocity', _wheel())
    assert len(sent) == 1
    m = sent[0]
    assert m.header.stamp == STAMP
    assert [s.name for s in m.status] == ['tram_odometry: slip', 'tram_odometry: inputs']
    assert m.status[0].level == DiagnosticStatus.OK
    assert m.status[0].message == 'ok'
    slip = {v.key: v.value for v in m.status[0].values}
    inputs = {v.key: v.value for v in m.status[1].values}
    assert slip == {'slip_front': 'false', 'slip_rear': 'false',
                    'adhesion_est': 'unknown', 'wheel_scale_front': '1.0',
                    'wheel_scale_rear': '1.0'}
    assert inputs == {'front_age_s': '0.0', 'rear_age_s': 'unknown',
                      'cmd_age_s': 'unknown', 'gnss_used': 'false'}


def test_diagnostics_rate_and_warning_for_slipping_bogie(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node.pub_diagnostics, 'publish', sent.append)
    est = replace(_estimate(), slip=SlipState(0.0, 1.0, True, False, 0.8))
    monkeypatch.setattr(node.odometry, 'step', lambda raw: est)
    for ns in (0, 50_000_000, 100_000_000, 150_000_000, 200_000_000):
        cmd = DriverControllerCommand()
        cmd.header.stamp = _stamp(STAMP.sec, STAMP.nanosec + ns)
        node.on_input('/vehicle/driver_position_cmd', cmd)
    assert len(sent) == 3
    assert all(m.status[0].level == DiagnosticStatus.WARN for m in sent)
    assert all(m.status[0].message == 'bogie anomaly' for m in sent)
    assert {v.key: v.value for v in sent[-1].status[0].values}['adhesion_est'] == '0.8'
    assert sent[-1].header.stamp == _stamp(STAMP.sec, STAMP.nanosec + 200_000_000)


def test_empty_zero_stamp_and_nonfinite_output_do_not_publish(node, monkeypatch):
    sent = _capture(node, monkeypatch)
    node.on_input('/vehicle/front_bogie_velocity', _wheel(float('nan')))
    node.on_input('/vehicle/rear_bogie_velocity', _wheel(float('inf')))
    node.on_input('/vehicle/front_bogie_velocity', _wheel(float('-inf')))
    empty = _wheel()
    empty.header.stamp = _stamp(0)
    node.on_input('/vehicle/front_bogie_velocity', empty)
    node.on_input('/vehicle/front_bogie_velocity', None)
    assert sent == []
    assert node.errors == 1  # malformed None is caught; invalid numeric input is dropped
    monkeypatch.setattr(node.odometry, 'step', lambda raw: replace(_estimate(), speed=float('inf')))
    node.on_input('/vehicle/front_bogie_velocity', _wheel())
    assert sent == []


def test_controller_continues_prediction_during_wheel_silence(node, monkeypatch):
    """Trap 7: a bogie silent for 73 s while the controller keeps talking (here every 5 s,
    within input.max_stamp_jump_s, #77): every command publishes on the model prediction."""
    sent = _capture(node, monkeypatch)
    diagnostics = []
    monkeypatch.setattr(node.pub_diagnostics, 'publish', diagnostics.append)
    node.on_input('/vehicle/front_bogie_velocity', _wheel(36.0))
    for sec in range(5, 73, 5):
        c = DriverControllerCommand()
        c.header.stamp = _stamp(STAMP.sec + sec, STAMP.nanosec)
        node.on_input('/vehicle/driver_position_cmd', c)
    cmd = DriverControllerCommand()
    cmd.header.stamp = _stamp(STAMP.sec + 73, STAMP.nanosec)
    node.on_input('/vehicle/driver_position_cmd', cmd)
    assert [k for k, _ in sent] == ['v', 'p'] * 16
    assert sent[-2][1].header.stamp == cmd.header.stamp
    assert sent[-2][1].velocity > 0.0
    assert sent[-1][1].pose.pose.position.x > sent[1][1].pose.pose.position.x
    ages = {v.key: v.value for v in diagnostics[-1].status[1].values}
    assert ages['front_age_s'] == '73.0'
    assert ages['cmd_age_s'] == '0.0'
    assert diagnostics[-1].status[1].level == DiagnosticStatus.WARN
    node.on_input('/vehicle/driver_position_cmd', cmd)  # repeated stamp is dropped
    assert len(sent) == 32


def test_diagnostics_marks_missing_bogies_on_controller_start(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node.pub_diagnostics, 'publish', sent.append)
    cmd = DriverControllerCommand()
    cmd.header.stamp = STAMP
    node.on_input('/vehicle/driver_position_cmd', cmd)
    assert len(sent) == 1
    assert sent[0].status[1].level == DiagnosticStatus.WARN
    values = {v.key: v.value for v in sent[0].status[1].values}
    assert values['front_age_s'] == values['rear_age_s'] == 'unknown'


def test_diagnostics_inputs_ok_when_fresh_and_warn_after_stale_timeout(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node.pub_diagnostics, 'publish', sent.append)
    monkeypatch.setattr(node.odometry, 'step', lambda raw: _estimate())
    stale_ns = int(node.params.input.stale_timeout_s * 1e9)
    for step, (topic, msg) in enumerate((('/vehicle/front_bogie_velocity', _wheel()),
                                         ('/vehicle/rear_bogie_velocity', _wheel()),
                                         ('/vehicle/driver_position_cmd', DriverControllerCommand()))):
        msg.header.stamp = _stamp(STAMP.sec, STAMP.nanosec + step * 100_000_000)  # 10 Hz apart
        node.on_input(topic, msg)
    assert len(sent) == 3
    assert sent[-1].status[1].level == DiagnosticStatus.OK
    assert sent[-1].status[1].message == 'ok'
    cmd = DriverControllerCommand()
    ns = STAMP.nanosec + 200_000_000 + stale_ns + 100_000_000
    cmd.header.stamp = _stamp(STAMP.sec + ns // 1_000_000_000, ns % 1_000_000_000)
    node.on_input('/vehicle/driver_position_cmd', cmd)
    assert sent[-1].status[1].level == DiagnosticStatus.WARN
    assert sent[-1].status[1].message == 'stale input'


def _stress_stream(seed=17, seconds=300.0):
    """Wheels 10 Hz, controller 20 Hz with the faults of docs/data.md: NaN/inf, outliers,
    zero, repeated and backward stamps, a 70 s silence of the rear bogie, bad notches."""
    rng = random.Random(seed)
    t0 = STAMP.sec
    stream = []
    for i in range(int(seconds * 20)):
        t = t0 + i * 0.05
        topics = ['/vehicle/driver_position_cmd']
        if i % 2 == 0:
            topics.append('/vehicle/front_bogie_velocity')
            if not 100.0 <= i * 0.05 <= 170.0:
                topics.append('/vehicle/rear_bogie_velocity')
        for topic in topics:
            if topic == '/vehicle/driver_position_cmd':
                msg = DriverControllerCommand()
                msg.position = rng.choice([-15, -3, 0, 5, 15, 127, -128])
            else:
                msg = VelocitySensor()
                msg.velocity = rng.choice([30.0, 31.5, 0.0, -5.0, 1e9, float('nan'),
                                           float('inf'), float('-inf')])
            fault = rng.random()
            ts = t - rng.uniform(0.0, 3.7) if fault < 0.05 else t   # backward and late stamps
            ts = 0.0 if 0.05 <= fault < 0.07 else ts                   # empty header
            sec = int(ts)
            msg.header.stamp = _stamp(sec, int(round((ts - sec) * 1e9)) % 1_000_000_000)
            stream.append((topic, msg))
            if 0.07 <= fault < 0.1:
                stream.append((topic, msg))                            # repeated message
    return stream


def test_stress_stream_through_real_core_never_breaks_outputs(node, monkeypatch):
    sent = _capture(node, monkeypatch)
    diagnostics = []
    monkeypatch.setattr(node.pub_diagnostics, 'publish', diagnostics.append)
    stream = _stress_stream()
    stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for _, m in stream}
    for topic, msg in stream:
        node.on_input(topic, msg)
    assert node.errors == 0
    assert len(sent) > 1000 and diagnostics
    for kind, m in sent:
        assert (m.header.stamp.sec, m.header.stamp.nanosec) in stamps and m.header.stamp.sec > 0
        if kind == 'v':
            assert math.isfinite(m.velocity) and m.velocity >= 0.0
        else:
            p = m.pose.pose.position
            assert all(math.isfinite(v) for v in (p.x, p.y, p.z, *m.pose.covariance))
    gap = [m for k, m in sent if k == 'v' and STAMP.sec + 110 <= m.header.stamp.sec <= STAMP.sec + 160]
    assert len(gap) > 100                          # rear bogie silent: publication goes on
    rear_age = [float(v.value) for d in diagnostics if STAMP.sec + 110 <= d.header.stamp.sec
                <= STAMP.sec + 160 for v in d.status[1].values if v.key == 'rear_age_s']
    assert rear_age and max(rear_age) > node.params.input.stale_timeout_s
    per_second = {}
    for d in diagnostics:
        per_second[d.header.stamp.sec] = per_second.get(d.header.stamp.sec, 0) + 1
    assert max(per_second.values()) <= 10                      # contract §1: 1-10 Hz


def test_diagnostics_stay_monotonic_when_one_bogie_lags_the_controller(node, monkeypatch):
    """Start-of-bag burst (docs/data.md, trap 5): the front bogie arrives 3.7 s behind the
    controller; each stream is monotonic on its own, so the core accepts both."""
    sent = []
    monkeypatch.setattr(node.pub_diagnostics, 'publish', sent.append)
    monkeypatch.setattr(node.odometry, 'step', lambda raw: _estimate())
    lag_ns = 3_700_000_000
    base = STAMP.sec * 1_000_000_000 + STAMP.nanosec + lag_ns
    for i in range(40):                                  # 2 s of stream, 20 Hz each
        for topic, msg, ns in (('/vehicle/driver_position_cmd', DriverControllerCommand(),
                                base + i * 50_000_000),
                               ('/vehicle/front_bogie_velocity', _wheel(),
                                base - lag_ns + i * 50_000_000)):
            msg.header.stamp = _stamp(ns // 1_000_000_000, ns % 1_000_000_000)
            node.on_input(topic, msg)
    stamps = [m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec for m in sent]
    assert stamps == sorted(stamps)
    assert 15 <= len(sent) <= 21                         # 2 s at <= 10 Hz of the leading stream


def test_nonfinite_z_is_not_published(node, monkeypatch):
    sent = _capture(node, monkeypatch)
    monkeypatch.setattr(node.odometry, 'step', lambda raw: replace(_estimate(), z=float('nan')))
    node.on_input('/vehicle/front_bogie_velocity', _wheel())
    assert sent == [] and node.errors == 1


def _route_run(route, window_s, duration=30.0, speed=10.0, late_north_m=30.0):
    """(topic, ROS message) of a tram along branch 0 of the installed map; after the GNSS
    window the GNSS lies: fixes late_north_m north of the track, vel north and faster (#57)."""
    import numpy as np
    b = route.branches[0]
    lat0, lon0, alt0 = route.origin
    out = []
    for k in range(int(duration * 10)):
        ns = 1_000_000_000 * 1000 + 100_000_000 * k
        s = speed * 0.1 * k
        x, y, z = (float(np.interp(s, b.s, c)) for c in (b.x, b.y, b.z))
        dx, dy = (float(np.interp(s + 1.0, b.s, c)) - v for c, v in ((b.x, x), (b.y, y)))
        late = k * 0.1 > window_s
        for off, topic in ((0, '/vehicle/front_bogie_velocity'), (10, '/vehicle/rear_bogie_velocity')):
            w = VelocitySensor()
            w.header.stamp = _stamp(*divmod(ns + off * 1_000_000, 1_000_000_000))
            w.velocity = speed * 3.6
            out.append((topic, w))
        cmd = DriverControllerCommand()
        cmd.header.stamp = _stamp(*divmod(ns + 20_000_000, 1_000_000_000))
        cmd.position = 3
        out.append(('/vehicle/driver_position_cmd', cmd))
        fix = NavSatFix()
        fix.header.stamp = _stamp(*divmod(ns + 30_000_000, 1_000_000_000))
        fix.status.status = 2
        fix.latitude = lat0 + (y + (late_north_m if late else 0.0)) / 111_500.0
        fix.longitude, fix.altitude = lon0 + x / 62_700.0, alt0 + z
        out += [('/sensing/gnss/master/fix', fix), ('/sensing/gnss/rover/fix', fix)]
        vel = TwistStamped()
        vel.header.stamp = _stamp(*divmod(ns + 40_000_000, 1_000_000_000))
        n = math.hypot(dx, dy)
        vel.twist.linear.x, vel.twist.linear.y = (0.0, 1.5 * speed) if late else (speed * dx / n, speed * dy / n)
        out.append(('/sensing/gnss/master/vel', vel))
    return out


def _published(msgs, monkeypatch, window_end_ns=None):
    """Everything a fresh node publishes on /result/* for the messages; GNSS stamped after
    window_end_ns is not delivered when it is given (the cut of tools/eval)."""
    n = on.OdometryNode(params_file=str(PARAMS_FILE))
    try:
        sent = _capture(n, monkeypatch)
        monkeypatch.setattr(n.pub_diagnostics, 'publish', lambda m: sent.append(('d', m)))
        for topic, m in msgs:
            if (window_end_ns is not None and topic.startswith('/sensing/gnss/')
                    and on._stamp_ns(m.header.stamp) > window_end_ns):
                continue
            n.on_input(topic, m)
        return sent, n.errors
    finally:
        n.destroy_node()


def test_gnss_after_window_changes_no_published_bit(node, monkeypatch):
    # #57: the disqualification condition end to end through the node — the same messages with
    # all GNSS and with GNSS cut after gnss.init_window_s give equal /result/* messages
    window_s = node.params.gnss.init_window_s
    msgs = _route_run(node.odometry.route, window_s)
    window_end_ns = on._stamp_ns(msgs[0][1].header.stamp) + int(window_s * 1e9)
    full, errors_full = _published(msgs, monkeypatch)
    cut, errors_cut = _published(msgs, monkeypatch, window_end_ns)
    assert errors_full == errors_cut == 0
    assert len(full) > 600 and {k for k, _ in full} == {'v', 'p', 'd'}
    for i, ((kf, mf), (kc, mc)) in enumerate(zip(full, cut)):
        assert (kf, mf) == (kc, mc), f'message {i} ({kf}) at {mf.header.stamp}: {mf} != {mc}'
    assert len(full) == len(cut)
