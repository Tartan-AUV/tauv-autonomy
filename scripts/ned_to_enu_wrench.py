#!/usr/bin/env python3
"""
NED-to-ENU wrench translator.

Converts a WrenchStamped from body-NED/FRD frame to body-ENU/FLU frame.

NED/FRD: x=forward, y=right,  z=down
ENU/FLU: x=forward, y=left,   z=up

Transformation (body-NED → body-ENU):
  Fx  ->  Fx        (forward unchanged)
  Fy  -> -Fy        (right  → left, negate)
  Fz  -> -Fz        (down   → up,   negate)
  tx  ->  tx        (roll   unchanged — same axis, different handedness cancels)
  ty  -> -ty        (pitch  → negate)
  tz  -> -tz        (yaw    → negate)

Topics:
  Subscribes: cmd_wrench      (geometry_msgs/WrenchStamped)  -- NED body frame
  Publishes:  cmd_wrench_enu  (geometry_msgs/WrenchStamped)  -- ENU body frame
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class NedToEnuWrench(Node):
    def __init__(self):
        super().__init__('ned_to_enu_wrench')
        self._pub = self.create_publisher(WrenchStamped, 'cmd_wrench_enu', 10)
        self.create_subscription(WrenchStamped, 'cmd_wrench', self._cb, 10)
        self.get_logger().info('NedToEnuWrench initialized.')

    def _cb(self, msg: WrenchStamped):
        out = WrenchStamped()
        out.header = msg.header
        out.header.frame_id = 'os/base_link'   # same link, now ENU/FLU convention

        out.wrench.force.x  =  msg.wrench.force.x
        out.wrench.force.y  = -msg.wrench.force.y
        out.wrench.force.z  = -msg.wrench.force.z
        out.wrench.torque.x =  msg.wrench.torque.x
        out.wrench.torque.y = -msg.wrench.torque.y
        out.wrench.torque.z = -msg.wrench.torque.z

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = NedToEnuWrench()
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
