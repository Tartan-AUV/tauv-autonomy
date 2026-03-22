from tauv_msgs.msg import ThrusterSetpoint
from tauv_autonomy.force_optimizer_2 import solve_thrusts, config
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

class thruster_forces(Node):
    def __init__(self):
        super().__init__('thruster_forces')
        self.get_logger().info('Thruster forces node initialized')
        self.create_subscription(Wrench, 'cmd_wrench', self.wrench_callback, 10)
        self.thruster_pub = self.create_publisher(ThrusterSetpoint, 'thruster_forces', 10)

    def wrench_callback(self, msg):
        self.get_logger().info(f'Received wrench command: {msg}')
        wrench = np.array([msg.force.x, -1*msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z])
        motor_commands = solve_thrusts(wrench, config)
        self.get_logger().info(f'Calculated motor commands: {motor_commands}')
        thruster_msg = ThrusterSetpoint()
        thruster_msg.thrust = motor_commands.tolist()
        thruster_msg.armed = False  # Set to True to enable thrusters; set to False for testing without arming
        self.thruster_pub.publish(thruster_msg)


def main(args=None):
    rclpy.init(args=args)
    node = thruster_forces()
    
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