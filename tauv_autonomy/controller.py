"""
/odometry/filtered (/nav_msgs/msg/Odometry): state estimation
/desired_state (nav_msgs/msg/Odometry): desired state, ignore all the covariance matrices
/thruster_forces (tauv_msgs/msg/ThrusterSetpoint): the outputted thruster commands
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tauv_msgs.msg import ThrusterSetpoint
from geometry_msgs.msg import Wrench, Quaternion
from tauv_msgs.msg import PID
import numpy as np
from tauv_autonomy.pid import PIDController
from tauv_autonomy.utils import *
from tauv_autonomy.thruster_saturation import resolve_wrenches


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Controller node initialized')
        
        # Subscriptions
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(Odometry, '/desired_state', self.desired_state_callback, 10)
        self.create_subscription(PID, '/pid_depth', self.pid_callback, 10)
        
        # Publisher
        self.thruster_pub = self.create_publisher(ThrusterSetpoint, 'thruster_forces', 10)
        self.wrench_pub = self.create_publisher(Wrench, '/cmd_wrench', 10)
        
        # State variables
        self.current_state = None
        self.desired_state = None
        
        self.pid_pos = {
            'x':     PIDController(kp=1.0),
            'y':     PIDController(kp=1.0),
            'z':     PIDController(kp=1.0),
            'roll':  PIDController(kp=1.0),
            'pitch': PIDController(kp=1.0),
            'yaw':   PIDController(kp=1.0)
        }

        self.pid_vel = {
            'x':     PIDController(kp=1.0),
            'y':     PIDController(kp=1.0),
            'z':     PIDController(kp=1.0),
            'roll':  PIDController(kp=-9.0, kd=-2.0),
            'pitch': PIDController(kp=-9.0, kd=-2.0),
            'yaw':   PIDController(kp=9.0, kd=2.0)
        }
        
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
        # self.kp_z = msg.p
        # self.ki_z = msg.i
        # self.kd_z = msg.d
        # self.ff_z = msg.ff
        pass

    def control_loop(self):
        """Main control loop triggered by the timer"""
        if self.current_state is None or self.desired_state is None:
            return

        dt = self.timer_period

        # --- 1. POSE DATA IN WORLD FRAME ---
        cur_pos_world = self.current_state.pose.pose.position
        des_pos_world = self.desired_state.pose.pose.position
        
        cur_orient_world = self.current_state.pose.pose.orientation
        des_orient_world = self.desired_state.pose.pose.orientation

        w_d, x_d, y_d, z_d = des_orient_world.w, des_orient_world.x, des_orient_world.y, des_orient_world.z
        w_c, x_c, y_c, z_c = cur_orient_world.w, cur_orient_world.x, cur_orient_world.y, cur_orient_world.z

        R_body_to_world = quat_to_rot_matrix(cur_orient_world)
        R_world_to_body = R_body_to_world.T
        
        # --- 2. VELOCITY DATA IN WORLD FRAME ---
        cur_lin_vel_body = np.array([
            self.current_state.twist.twist.linear.x,
            self.current_state.twist.twist.linear.y,
            self.current_state.twist.twist.linear.z
        ])
        cur_ang_vel_body = np.array([
            self.current_state.twist.twist.angular.x,
            self.current_state.twist.twist.angular.y,
            self.current_state.twist.twist.angular.z
        ])

        cur_lin_vel_world = R_body_to_world @ cur_lin_vel_body
        cur_ang_vel_world = R_body_to_world @ cur_ang_vel_body

        # --- 3. CALCULATE ERRORS ---
        
        # Positional error (World Frame)
        err_x_world = des_pos_world.x - cur_pos_world.x
        err_y_world = des_pos_world.y - cur_pos_world.y
        err_z_world = des_pos_world.z - cur_pos_world.z
        
        # Quaternion multiplication math (q_des * conjugate(q_cur))
        # Note: The conjugate of q_cur is found by negating its x, y, and z components
        err_w = w_d * w_c + x_d * x_c + y_d * y_c + z_d * z_c
        err_x = x_d * w_c - w_d * x_c - y_d * z_c + z_d * y_c
        err_y = y_d * w_c + x_d * z_c - w_d * y_c - z_d * x_c
        err_z = z_d * w_c - x_d * y_c + y_d * x_c - w_d * z_c
        
        # Normalize the error quaternion to prevent floating point drift
        norm = np.sqrt(err_w**2 + err_x**2 + err_y**2 + err_z**2)
        
        # Create a temporary quaternion object to pass into existing quat_to_euler function
        err_quat = Quaternion()
        err_quat.w = err_w / norm
        err_quat.x = err_x / norm
        err_quat.y = err_y / norm
        err_quat.z = err_z / norm
        err_roll_world, err_pitch_world, err_yaw_world = quat_to_euler(err_quat)

        # --- 3A. DEBUGGING ---
        err_x_world = 0
        err_y_world = 0
        err_z_world = 0
        # err_roll_world = 0
        # err_pitch_world = 0
        # err_yaw_world = 0

        # --- 4. OUTER LOOP: POSITION -> VELOCITY ---
        max_lin = 10
        max_ang = 5
        
        cmd_lin_vel_x_world = np.clip(self.pid_pos['x'].compute(err_x_world, 0.0, dt, cur_lin_vel_world[0]), -max_lin, max_lin)
        cmd_lin_vel_y_world = np.clip(self.pid_pos['y'].compute(err_y_world, 0.0, dt, cur_lin_vel_world[1]), -max_lin, max_lin)
        cmd_lin_vel_z_world = np.clip(self.pid_pos['z'].compute(err_z_world, 0.0, dt, cur_lin_vel_world[2]), -max_lin, max_lin)
        
        cmd_ang_vel_roll_world  = np.clip(self.pid_pos['roll'].compute( err_roll_world,  0.0, dt, cur_ang_vel_world[0]), -max_ang, max_ang)
        cmd_ang_vel_pitch_world = np.clip(self.pid_pos['pitch'].compute(err_pitch_world, 0.0, dt, cur_ang_vel_world[1]), -max_ang, max_ang)
        cmd_ang_vel_yaw_world   = np.clip(self.pid_pos['yaw'].compute(  err_yaw_world,   0.0, dt, cur_ang_vel_world[2]), -max_ang, max_ang)

        # --- 5. INNER LOOP: VELOCITY -> FORCE ---
        force_x_world = self.pid_vel['x'].compute(cmd_lin_vel_x_world, cur_lin_vel_world[0], dt)
        force_y_world = self.pid_vel['y'].compute(cmd_lin_vel_y_world, cur_lin_vel_world[1], dt)
        force_z_world = self.pid_vel['z'].compute(cmd_lin_vel_z_world, cur_lin_vel_world[2], dt)
        
        torque_roll_world  = self.pid_vel['roll'].compute( cmd_ang_vel_roll_world,  cur_ang_vel_world[0], dt)
        torque_pitch_world = self.pid_vel['pitch'].compute(cmd_ang_vel_pitch_world, cur_ang_vel_world[1], dt)
        torque_yaw_world   = self.pid_vel['yaw'].compute(  cmd_ang_vel_yaw_world,   cur_ang_vel_world[2], dt)

        # --- 6. ITS WRENCHING TIME ---
        demands_world = {
            'x':     (np.array([force_x_world, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])),
            'y':     (np.array([0.0, force_y_world, 0.0]), np.array([0.0, 0.0, 0.0])),
            'z':     (np.array([0.0, 0.0, force_z_world]), np.array([0.0, 0.0, 0.0])),
            'roll':  (np.array([0.0, 0.0, 0.0]), np.array([torque_roll_world, 0.0, 0.0])),
            'pitch': (np.array([0.0, 0.0, 0.0]), np.array([0.0, torque_pitch_world, 0.0])),
            'yaw':   (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, torque_yaw_world]))
        }

        wrenches = {}
        for axis, (f_world, t_world) in demands_world.items():
            f_body = R_world_to_body @ f_world
            t_body = R_world_to_body @ t_world
            wrenches[axis] = np.concatenate((f_body, t_body))
            
        thruster_forces, wrench = resolve_wrenches(wrenches)
        thruster_forces.armed = True
        self.thruster_pub.publish(thruster_forces)
        self.wrench_pub.publish(wrench)

    
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