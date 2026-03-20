#!/usr/bin/env python3
"""
twist_to_wrench.py

Converts geometry_msgs/Twist (published by Foxglove Teleop panel) to
geometry_msgs/WrenchStamped (consumed by ThrusterController).

Foxglove Teleop panel config:
  Topic:      cmd_vel
  Message:    geometry_msgs/Twist
  Linear axes → force [N], angular axes → torque [N·m]

Parameters (all in thruster_params.yaml or as ROS params):
  max_force_xy   [N]   – max forward/strafe force  (default 5.0)
  max_force_z    [N]   – max heave force            (default 3.0)
  max_torque_rp  [N·m] – max roll/pitch torque      (default 2.0)
  max_torque_yaw [N·m] – max yaw torque             (default 2.0)

Twist axes are in [-1, 1] from the Foxglove panel;
they are scaled by the max values above.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped


class TwistToWrench(Node):
    def __init__(self):
        super().__init__('twist_to_wrench')

        self.declare_parameter('max_force_xy',   5.0)
        self.declare_parameter('max_force_z',    3.0)
        self.declare_parameter('max_torque_rp',  2.0)
        self.declare_parameter('max_torque_yaw', 2.0)

        self._pub = self.create_publisher(WrenchStamped, 'cmd_wrench', 10)
        self.create_subscription(Twist, 'cmd_vel', self._cb, 10)

        self.get_logger().info(
            'twist_to_wrench ready. '
            'In Foxglove: add a Teleop panel, set topic=cmd_vel, message=geometry_msgs/Twist.'
        )

    def _cb(self, msg: Twist):
        fx = self.get_parameter('max_force_xy').value
        fz = self.get_parameter('max_force_z').value
        trp = self.get_parameter('max_torque_rp').value
        ty = self.get_parameter('max_torque_yaw').value

        out = WrenchStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'os/base_link'

        out.wrench.force.x  = msg.linear.x  * fx   # surge  (forward)
        out.wrench.force.y  = msg.linear.y  * fx   # sway   (strafe)
        out.wrench.force.z  = msg.linear.z  * fz   # heave  (up/down)
        out.wrench.torque.x = msg.angular.x * trp  # roll
        out.wrench.torque.y = msg.angular.y * trp  # pitch
        out.wrench.torque.z = msg.angular.z * ty   # yaw

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToWrench()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
