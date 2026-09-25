"""ROS 2 wrapper: /vehicle/* and GNSS -> pipeline.Odometry.step -> /result/velocity, /result/position.

Thin by design (D-001): the raw input of Odometry.step is the (topic, message) pair as
received, exactly what tools/eval passes (tools/eval/tram_eval/bag.py: to_raw); parsing, units
and stamps are the core's job. One publication per accepted input; header.stamp is that
input's stamp (D-015).
"""
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

from tram_odometry_core.pipeline import Odometry
from tram_odometry_core.types import load_params, load_route

UNKNOWN_VAR = 1e6      # contract §1: covariance of an unestimated component is large, never -1
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
    m.header.frame_id = params.frames.map
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
            est = self.odometry.step(to_raw(topic, msg))
            if est is None:
                return
            self.pub_velocity.publish(velocity_msg(est, msg.header.stamp, self.params))
            self.pub_position.publish(position_msg(est, msg.header.stamp, self.params))
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
