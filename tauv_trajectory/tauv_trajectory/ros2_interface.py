"""
ROS 2 node that wraps core.py

Subscribers:
  /odometry/filtered  (nav_msgs/Odometry): Current position
  /odometry/target    (nav_msgs/Odometry): Target position

Publishes:
  /setpoint                 (nav_msgs/Odometry): Point for the controller to chase
  /setpoint/accel  (geometry_msgs/AccelStamped): Acceleration at that setpoint

Parameters:
  speed_mps        (double,  default 0.5): cruise speed used to time the maneuver
  resolution_sec   (double,  default 0.1): path sample spacing, in seconds
  lookahead_dist   (double,  default 0.5): Pure Pursuit lookahead distance, in meters
  use_target_vel   (bool,   default True): match target velocity, or come to rest
  control_rate_hz  (double, default 20.0): rate of the setpoint publish timer
"""

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Quaternion, Point, AccelStamped
from tauv_msgs.msg import Status

from core import PurePursuitPlanner, VehicleState


def xyz_to_numpy(msg) -> np.ndarray:
    """Works for both geometry_msgs/Point and geometry_msgs/Vector3 (duck-typed x/y/z)"""
    return np.array([msg.x, msg.y, msg.z], dtype=float)


def quat_xyzw_to_numpy(msg: Quaternion) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.z, msg.w], dtype=float)


def numpy_to_point(v: np.ndarray) -> Point:
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def numpy_to_vector3(v: np.ndarray) -> Vector3:
    return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def numpy_to_quat(q: np.ndarray) -> Quaternion:
    return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))


def odom_to_vehicle_state(msg: Odometry) -> VehicleState:
    """
    Builds a VehicleState from an Odometry message
    """
    return VehicleState(
        position=xyz_to_numpy(msg.pose.pose.position),
        orientation=quat_xyzw_to_numpy(msg.pose.pose.orientation),
        linear_velocity=xyz_to_numpy(msg.twist.twist.linear),
        angular_velocity=xyz_to_numpy(msg.twist.twist.angular),
        linear_acceleration=np.zeros(3),
        angular_acceleration=np.zeros(3),
    )


class CubicPursuitNode(Node):

    def __init__(self):
        super().__init__('cubic_pursuit_node')

        self.declare_parameter('speed_mps', 0.5)
        self.declare_parameter('resolution_sec', 0.1)
        self.declare_parameter('lookahead_dist', 0.5)
        self.declare_parameter('use_target_vel', True)
        self.declare_parameter('control_rate_hz', 20.0)

        self._planner = PurePursuitPlanner()
        self._current_state: Optional[VehicleState] = None

        self._current_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._on_current_state, 10)
        self._target_sub = self.create_subscription(
            Odometry, '/odometry/target', self._on_target_state, 10)

        self._setpoint_pub = self.create_publisher(Odometry, '/setpoint', 10)
        self._accel_pub = self.create_publisher(AccelStamped, '/setpoint/accel', 10)
        self._status_pub = self.create_publisher(Status, '/mission/status', 10)

        rate_hz = self.get_parameter('control_rate_hz').value
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_current_state(self, msg: Odometry) -> None:
        self._current_state = odom_to_vehicle_state(msg)

    def _on_target_state(self, msg: Odometry) -> None:
        if self._current_state is None:
            self.get_logger().warn('Got a target_state before any current_state -- ignoring.')
            return

        target_state = odom_to_vehicle_state(msg)
        speed = self.get_parameter('speed_mps').value
        res = self.get_parameter('resolution_sec').value
        use_target_vel = self.get_parameter('use_target_vel').value

        self._planner.generate_path(self._current_state, target_state, speed, res, use_target_vel)
        self.get_logger().info(
            f'Generated new cubic Hermite path, duration={self._planner.duration:.2f}s.')

    def _on_timer(self) -> None:
        if self._current_state is None or not self._planner.has_active_path():
            return

        msg = Status()
        msg.id = "trajectory"

        lookahead_dist = self.get_parameter('lookahead_dist').value
        setpoint = self._planner.get_lookahead_setpoint(self._current_state.position, lookahead_dist)

        if setpoint is None:
            msg.status = 1
            self._status_pub.publish(msg)
            self.get_logger().info('Reached the end of the path.')
            return

        stamp = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.pose.pose.position = numpy_to_point(setpoint.position)
        odom_msg.pose.pose.orientation = numpy_to_quat(setpoint.orientation)
        odom_msg.twist.twist.linear = numpy_to_vector3(setpoint.linear_velocity)
        odom_msg.twist.twist.angular = numpy_to_vector3(setpoint.angular_velocity)
        self._setpoint_pub.publish(odom_msg)

        accel_msg = AccelStamped()
        accel_msg.header.stamp = stamp
        accel_msg.header.frame_id = 'map'
        accel_msg.accel.linear = numpy_to_vector3(setpoint.linear_acceleration)
        accel_msg.accel.angular = numpy_to_vector3(setpoint.angular_acceleration)
        self._accel_pub.publish(accel_msg)

        msg.status = 2
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CubicPursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
