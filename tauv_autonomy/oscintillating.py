import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

class Oscillating(Node):
    def __init__(self):
        super().__init__('oscillating')
        self.get_logger().info('Oscillating node initialized')
        
        # Publisher
        self.wrench_pub = self.create_publisher(Wrench, '/cmd_wrench', 10)
        
        # Control loop timer
        self.timer_period = 1  # 1Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.time = 0

    def control_loop(self):
        """Publish an oscillating wrench command"""
        wrench = Wrench()
        wrench.force.x = 0.6 * (2 * ((self.time / 2) % 2) - 1)
        wrench.force.y = 0.6 * (2 * (((self.time + 1) / 2) % 2) - 1)
        wrench.force.z = 0.6 * (2 * ((self.time / 3) % 2) - 1)
        wrench.torque.x = 0.6 * (2 * (((self.time + 1) / 3) % 2) - 1)
        wrench.torque.y = 0.6 * (2 * ((self.time / 4) % 2) - 1)
        wrench.torque.z = 0.6 * (2 * (((self.time + 1) / 4) % 2) - 1)
        
        self.wrench_pub.publish(wrench)
        
        self.time += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = Oscillating()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()