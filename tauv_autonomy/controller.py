"""
/odometry/filtered (/nav_msgs/msg/Odometry): state estimation
/desired_state (nav_msgs/msg/Odometry): desired state, ignore all the covariance matrices
/cmd_wrench (geometry_msgs/msg/Wrench): the outputted wrench command for the thrusters
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench
from tauv_msgs.msg import PID
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Controller node initialized')
        
        # Subscriptions
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(Odometry, '/desired_state', self.desired_state_callback, 10)
        self.create_subscription(PID, '/pid_depth', self.pid_callback, 10)
        
        # Publisher
        self.wrench_pub = self.create_publisher(Wrench, '/cmd_wrench', 10)
        
        # State variables
        self.current_state = None
        self.desired_state = None
        
        # PID Parameters for Depth (World Z-axis)
        self.kp_z = 10    # Proportional gain
        self.ki_z = 0.0   # Integral gain
        self.kd_z = 0.5   # Derivative gain
        self.ff_z = 28    # Feedforward term to counteract buoyancy (adjust based on testing)
        
        # PID State tracking
        self.integral_z_world = 0.0
        
        # Control loop timer
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def odom_callback(self, msg):
        """Update current state from filtered odometry"""
        self.current_state = msg

    def desired_state_callback(self, msg):
        """Update the desired target state"""
        self.desired_state = msg

    def pid_callback(self, msg):
        """Update PID parameters from the /pid_depth topic"""
        self.kp_z = msg.p
        self.ki_z = msg.i
        self.kd_z = msg.d
        self.ff_z = msg.ff

    def control_loop(self):
        """Main control loop triggered by the timer"""

        # Ensure we have the minimum information to compute a control command
        if self.current_state is None or self.desired_state is None:
            return

        # --- 1. EXTRACT DATA & ORIENTATION ---
        
        current_z_world = self.current_state.pose.pose.position.z # Current depth (world frame)
        desired_z_world = self.desired_state.pose.pose.position.z # Desired depth (world frame)
        
        v_body_msg = self.current_state.twist.twist.linear # Raw linear velocity from EKF (body frame)
        v_body = np.array([v_body_msg.x, v_body_msg.y, v_body_msg.z]) # Convert to numpy array for vectorized operations

        # Orientation quaternion (World to Body rotation)
        q = self.current_state.pose.pose.orientation
        
        # The WORLD Z-axis unit vector [0, 0, 1] expressed in the body frame.
        world_z_body = np.array([
            2.0 * (q.x * q.z - q.w * q.y),
            2.0 * (q.y * q.z + q.w * q.x),
            1.0 - 2.0 * (q.x**2 + q.y**2)
        ])

        # --- 2. CALCULATE WORLD Z VELOCITY ---
        
        # Get the z-velociy in the world frame by projecting the body frame velocity onto the world Z-axis vector
        vz_world = np.dot(v_body, world_z_body)

        # --- 3. PID DEPTH MATH (WORLD FRAME) ---
        
        # Calculate current error in depth
        error_z_world = desired_z_world - current_z_world

        # Calculate PID terms for the WORLD Z-axis
        p_term = self.kp_z * error_z_world 
        self.integral_z_world += error_z_world * self.timer_period 
        i_term = self.ki_z * self.integral_z_world
        d_term = self.kd_z * (-vz_world) 
        
        # Compute total command force required in the WORLD Z-axis
        force_z_world = p_term + i_term + d_term - self.ff_z

        # --- 4. MAP FORCES TO BODY FRAME ---
        
        # Vectorized scalar multiplication: distribute the scalar force 
        # across the local axes based on the orientation vector.
        force_body = force_z_world * world_z_body

        # Create and populate the Wrench message
        wrench_msg = Wrench()
        wrench_msg.force.x = float(force_body[0])
        wrench_msg.force.y = float(force_body[1])
        wrench_msg.force.z = float(force_body[2])
        
        # Keep torques at 0
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = 0.0
        
        # Publish the command
        self.wrench_pub.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()