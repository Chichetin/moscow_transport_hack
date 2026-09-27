"""ROS 2 wrapper: /vehicle/* and GNSS -> pipeline.Odometry.step -> /result/*.

Thin by design (D-001): the raw input of Odometry.step is the (topic, message) pair as
received, exactly what tools/eval passes (tools/eval/tram_eval/bag.py: to_raw); parsing, units
and stamps are the core's job. One publication per accepted input; header.stamp is that
input's stamp (D-015).
"""
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

from tram_odometry_core.pipeline import Odometry
from tram_odometry_core.types import load_params, load_route

ODOM_FRAME = 'odom'    # REP-105: continuous local frame of dead reckoning (contract §1, #162)
UNKNOWN_VAR = 1e6     # contract §1: covariance of an unestimated component is large, never -1
DIAGNOSTIC_PERIOD_NS = 100_000_000  # 10 Hz maximum, measured in bag stamp time
INPUT_QUEUE = 100      # messages; bag start delivers a burst of up to ~3.7 s (docs/data.md, trap 5)
VEHICLE_INPUTS = [('/vehicle/front_bogie_velocity', VelocitySensor),
                  ('/vehicle/rear_bogie_velocity', VelocitySensor),
                  ('/vehicle/driver_position_cmd', DriverControllerCommand)]
# contract §1 input; tools/eval feeds it too (bag.py: GNSS), so the core sees the same stream
# and its t0 (start of the GNSS window, D-005) is the same in the node and in eval (D-028)
ROVER_FIX_TOPIC = '/sensing/gnss/rover/fix'
# best-effort matches a publisher of either reliability; a reliable subscription would never
# connect to a best-effort `ros2 bag play` (two publishers of the controller in the bag, trap 9)
INPUT_QOS = QoSProfile(depth=INPUT_QUEUE, reliability=ReliabilityPolicy.BEST_EFFORT)


def default_params_file() -> str:
    """params.yaml installed with the package: `ros2 run` without arguments works too."""
    return os.path.join(get_package_share_directory('tram_odometry'), 'config', 'params.yaml')


def route_file(params) -> str:
    """maps/<position.map_file> installed with the package (contract §5); eval reads the same file."""
    return os.path.join(get_package_share_directory('tram_odometry'), 'maps', params.position.map_file)


def to_raw(topic: str, msg):
    """The raw input of Odometry.step: the same (topic, message) pair as tools/eval."""
    return topic, msg


def velocity_msg(est, stamp, params) -> VelocitySensor:
    m = VelocitySensor()
    m.header.stamp = stamp
    m.header.frame_id = params.frames.base
    m.velocity = est.speed
    return m


def position_msg(est, stamp, params) -> OdometryMsg:
    m = OdometryMsg()
    m.header.stamp = stamp
    # `map` is the MGRS grid (D-083); local metres before any geodetic anchor (no map, no fix)
    # go out in `odom`, the REP-105 frame of dead reckoning, not a project parameter (#162)
    m.header.frame_id = params.frames.map if est.position_absolute else ODOM_FRAME
    m.child_frame_id = params.frames.base
    m.pose.pose.position.x = est.x
    m.pose.pose.position.y = est.y
    m.pose.pose.position.z = est.z
    m.pose.pose.orientation.z = math.sin(est.yaw / 2)
    m.pose.pose.orientation.w = math.cos(est.yaw / 2)
    var_x, var_y, cov_xy = est.pos_cov
    pose_cov = [0.0] * 36
    pose_cov[0], pose_cov[1], pose_cov[6], pose_cov[7] = var_x, cov_xy, cov_xy, var_y
    for i in (14, 21, 28, 35):           # z, roll, pitch, yaw: not estimated
        pose_cov[i] = UNKNOWN_VAR
    m.pose.covariance = pose_cov
    m.twist.twist.linear.x = est.speed
    twist_cov = [0.0] * 36
    twist_cov[0] = est.speed_var
    for i in (7, 14, 21, 28, 35):
        twist_cov[i] = UNKNOWN_VAR
    m.twist.covariance = twist_cov
    return m


def _stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _finite_estimate(est) -> bool:
    return all(math.isfinite(v) for v in (est.speed, est.speed_var, est.x, est.y, est.z,
                                         est.yaw, *est.pos_cov))


def _values(items):
    return [KeyValue(key=key, value=str(value).lower() if isinstance(value, bool)
                     else str(value)) for key, value in items]


def diagnostics_msg(est, stamp, params, ages) -> DiagnosticArray:
    """Two statuses from the accepted estimate and ages of accepted vehicle inputs."""
    m = DiagnosticArray()
    m.header.stamp = stamp
    slip = est.slip
    slip_status = DiagnosticStatus()
    slip_status.name = 'tram_odometry: slip'
    slip_status.level = (DiagnosticStatus.WARN if slip.slip_front or slip.slip_rear
                         else DiagnosticStatus.OK)
    slip_status.message = ('bogie anomaly' if slip_status.level != DiagnosticStatus.OK
                           else 'ok')
    slip_status.values = _values((
        ('slip_front', slip.slip_front), ('slip_rear', slip.slip_rear),
        ('adhesion_est', 'unknown' if slip.adhesion_est is None else slip.adhesion_est),
        ('wheel_scale_front', params.vehicle.wheel_scale_front),
        ('wheel_scale_rear', params.vehicle.wheel_scale_rear)))
    input_status = DiagnosticStatus()
    input_status.name = 'tram_odometry: inputs'
    input_status.level = (DiagnosticStatus.WARN if any(
        age == 'unknown' or age > params.input.stale_timeout_s for age in ages)
                          else DiagnosticStatus.OK)
    input_status.message = ('stale input' if input_status.level != DiagnosticStatus.OK
                            else 'ok')
    stale = params.input.stale_timeout_s
    live = sum(age != 'unknown' and age <= stale for age in ages[:2])
    known = [age for age in ages[:2] if age != 'unknown']
    # model_only: no bogie within stale_timeout_s; the core then predicts on the drive model
    # (D-036, drive.use_model) or holds the speed; model_only_s is the freshest bogie age
    mode = {2: 'wheels', 1: 'one_bogie', 0: 'model_only'}[live]
    model_only_s = 0.0 if live else (min(known) if known else 'unknown')
    input_status.values = _values((
        ('front_age_s', ages[0]), ('rear_age_s', ages[1]), ('cmd_age_s', ages[2]),
        ('gnss_used', est.gnss_used), ('mode', mode), ('model_only_s', model_only_s)))
    m.status = [slip_status, input_status]
    return m


class OdometryNode(Node):
    def __init__(self, params_file=None):
        super().__init__('tram_odometry')
        path = self.declare_parameter('params_file', params_file or '').value
        self.params = load_params(path or default_params_file())
        route = load_route(route_file(self.params)) if self.params.position.use_map else None
        self.odometry = Odometry(self.params, route=route)
        self.errors = 0
        self.pub_velocity = self.create_publisher(VelocitySensor, '/result/velocity', 10)
        self.pub_position = self.create_publisher(OdometryMsg, '/result/position', 10)
        self.pub_diagnostics = self.create_publisher(DiagnosticArray, '/result/diagnostics', 10)
        self._last_input_ns = {}
        self._last_diagnostic_ns = None
        inputs = VEHICLE_INPUTS + [(self.params.gnss.topic_fix, NavSatFix),
                                   (ROVER_FIX_TOPIC, NavSatFix),
                                   (self.params.gnss.topic_vel, TwistStamped)]
        for topic, msg_type in inputs:
            self.create_subscription(msg_type, topic,
                                     lambda msg, topic=topic: self.on_input(topic, msg),
                                     INPUT_QOS)

    def on_input(self, topic: str, msg) -> None:
        """One input -> at most one velocity and one position; a core error skips the input."""
        try:
            stamp_ns = _stamp_ns(msg.header.stamp)
            if stamp_ns <= 0:
                return
            est = self.odometry.step(to_raw(topic, msg))
            if est is None:
                return
            if not _finite_estimate(est):
                self.errors += 1
                self.get_logger().error(f'{topic} non-finite estimate not published',
                                        throttle_duration_sec=5.0)
                return
            self.pub_velocity.publish(velocity_msg(est, msg.header.stamp, self.params))
            if est.position_absolute:
                # before the first valid master fix there is no position at all, not even a local
                # one: a judge that ignores frame_id would compare it with the grid (#163, D-086)
                self.pub_position.publish(position_msg(est, msg.header.stamp, self.params))
            if topic in (VEHICLE_INPUTS[0][0], VEHICLE_INPUTS[1][0], VEHICLE_INPUTS[2][0]):
                self._last_input_ns[topic] = stamp_ns
            if (self._last_diagnostic_ns is None
                    or stamp_ns - self._last_diagnostic_ns >= DIAGNOSTIC_PERIOD_NS):
                ages = tuple('unknown' if self._last_input_ns.get(key) is None else
                             max(0, stamp_ns - self._last_input_ns[key]) / 1_000_000_000
                             for key, _ in VEHICLE_INPUTS)
                self.pub_diagnostics.publish(diagnostics_msg(est, msg.header.stamp,
                                                              self.params, ages))
                self._last_diagnostic_ns = stamp_ns
        except Exception as e:  # noqa: BLE001 — the node must outlive any bad input or core bug
            self.errors += 1
            self.get_logger().error(f'{topic} input skipped: {e!r}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
