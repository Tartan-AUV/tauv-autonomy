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
import csv
from pathlib import Path
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Controller node initialized')

        self.declare_parameter('tune', False)
        self.tune = self.get_parameter('tune').get_parameter_value().bool_value
        if self.tune:
            self.get_logger().info('Tuning is ENABLED.')
        else:
            self.get_logger().info('Tuning is DISABLED.')
        self.pid_file_path = Path("/tauv-mono/ros_ws/src/tauv_autonomy/config/pid_history.csv")
        self.last_csv_row = {}

        # Subscriptions
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(Odometry, '/desired_state', self.desired_state_callback, 10)
        self.create_subscription(PID, '/pid', self.pid_callback, 10)

        # Publisher
        self.thruster_pub = self.create_publisher(ThrusterSetpoint, 'thruster_forces', 10)
        self.wrench_pub = self.create_publisher(Wrench, '/cmd_wrench', 10)
        self.pid_gains_pub = self.create_publisher(PID, '/pid', 10)

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
            'x':     PIDController(kp=9.0, ki=0.1),
            'y':     PIDController(kp=9.0, ki=0.1),
            'z':     PIDController(kp=15.0, ki=0.1, kd=3.0, ff=-28),
            'roll':  PIDController(kp=9.0, ki=0.1, kd=2.0),
            'pitch': PIDController(kp=25.0, ki=0.1, kd=2.0),
            'yaw':   PIDController(kp=9.0, ki=0.1, kd=2.0)
        }

        self.load_pid_data()
        if self.tune:
            for axis in ['z', 'roll', 'pitch']:
                self.pid_vel[axis].kp = 0.0
                self.pid_vel[axis].ki = 0.0
                self.pid_vel[axis].kd = 0.0

        self.publish_current_pid_gains()

        # Control loop timer
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def odom_callback(self, msg):
        """Update current state from filtered odometry"""
        self.current_state = msg

    def desired_state_callback(self, msg):
        """Update the desired target state"""
        self.desired_state = msg

    def publish_current_pid_gains(self):
        msg = PID()

        # Helper to extract [kp, ki, kd, ff], defaulting to 0.0 if an attribute doesn't exist
        def get_pidf_array(pidcontroller):
            return [
                float(getattr(pidcontroller, 'kp', 0.0)),
                float(getattr(pidcontroller, 'ki', 0.0)),
                float(getattr(pidcontroller, 'kd', 0.0)),
                float(getattr(pidcontroller, 'ff', 0.0))
            ]

        # Populate Position PIDF values
        msg.pidf_x_pos = get_pidf_array(self.pid_pos['x'])
        msg.pidf_y_pos = get_pidf_array(self.pid_pos['y'])
        msg.pidf_z_pos = get_pidf_array(self.pid_pos['z'])
        msg.pidf_roll_pos = get_pidf_array(self.pid_pos['roll'])
        msg.pidf_pitch_pos = get_pidf_array(self.pid_pos['pitch'])
        msg.pidf_yaw_pos = get_pidf_array(self.pid_pos['yaw'])

        # Populate Velocity PIDF values
        msg.pidf_x_vel = get_pidf_array(self.pid_vel['x'])
        msg.pidf_y_vel = get_pidf_array(self.pid_vel['y'])
        msg.pidf_z_vel = get_pidf_array(self.pid_vel['z'])
        msg.pidf_roll_vel = get_pidf_array(self.pid_vel['roll'])
        msg.pidf_pitch_vel = get_pidf_array(self.pid_vel['pitch'])
        msg.pidf_yaw_vel = get_pidf_array(self.pid_vel['yaw'])

        # Publish the message
        self.pid_gains_pub.publish(msg)

    def pid_callback(self, msg):
        """
        Callback to dynamically update PID gains from an external publisher.
        Expects msg arrays to be formatted as [kp, ki, kd, ff].
        """
        # Helper function to safely unpack the array and update the controller
        def update_controller_gains(controller, gain_array):
            # Ensure the array has the expected 4 elements before updating
            if len(gain_array) >= 4:
                controller.kp = float(gain_array[0])
                controller.ki = float(gain_array[1])
                controller.kd = float(gain_array[2])
                controller.ff = float(gain_array[3])
            else:
                self.get_logger().warn("Received malformed PID array. Expected 4 elements.")

        try:
            # --- Update Position PIDs ---
            update_controller_gains(self.pid_pos['x'], msg.pidf_x_pos)
            update_controller_gains(self.pid_pos['y'], msg.pidf_y_pos)
            update_controller_gains(self.pid_pos['z'], msg.pidf_z_pos)
            update_controller_gains(self.pid_pos['roll'], msg.pidf_roll_pos)
            update_controller_gains(self.pid_pos['pitch'], msg.pidf_pitch_pos)
            update_controller_gains(self.pid_pos['yaw'], msg.pidf_yaw_pos)

            # --- Update Velocity PIDs ---
            update_controller_gains(self.pid_vel['x'], msg.pidf_x_vel)
            update_controller_gains(self.pid_vel['y'], msg.pidf_y_vel)
            update_controller_gains(self.pid_vel['z'], msg.pidf_z_vel)
            update_controller_gains(self.pid_vel['roll'], msg.pidf_roll_vel)
            update_controller_gains(self.pid_vel['pitch'], msg.pidf_pitch_vel)
            update_controller_gains(self.pid_vel['yaw'], msg.pidf_yaw_vel)

            self.get_logger().info("Successfully updated PID gains from external message.")

        except Exception as e:
            self.get_logger().error(f"Failed to update PID gains: {e}")

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

        # --- 2. CURRENT VELOCITY DATA IN WORLD FRAME ---
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
        
        # --- 2.5. DESIRED VELOCITY DATA IN WORLD FRAME ---
        # Assuming desired twist is in the desired body frame (standard ROS convention)
        R_des_body_to_world = quat_to_rot_matrix(des_orient_world)

        des_lin_vel_body = np.array([
            self.desired_state.twist.twist.linear.x,
            self.desired_state.twist.twist.linear.y,
            self.desired_state.twist.twist.linear.z
        ])
        des_ang_vel_body = np.array([
            self.desired_state.twist.twist.angular.x,
            self.desired_state.twist.twist.angular.y,
            self.desired_state.twist.twist.angular.z
        ])

        des_lin_vel_world = R_des_body_to_world @ des_lin_vel_body
        des_ang_vel_world = R_des_body_to_world @ des_ang_vel_body

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

        # --- 3.5. DEBUGGING ---
        # err_x_world = 0
        # err_y_world = 0
        # err_z_world = 0
        # err_roll_world = 0
        # err_pitch_world = 0
        # err_yaw_world = 0

        # --- 3.75. AUTO-TUNE FEEDFORWARD ---
        if self.tune:
            alpha = -0.5  # Velocity multiplier
            beta = 2.0   # Position error multiplier

            self.pid_vel['z'].ff += (beta * err_z_world) - (alpha * cur_lin_vel_world[2])
            self.pid_vel['roll'].ff += (beta * err_roll_world) - (alpha * cur_ang_vel_world[0])
            self.pid_vel['pitch'].ff += (beta * err_pitch_world) - (alpha * cur_ang_vel_world[1])

            self.publish_current_pid_gains()

        # --- 4. OUTER LOOP: POSITION -> VELOCITY ---
        max_lin = 10
        max_ang = 5

        # Include velocity feedforward directly into the commanded velocity calculations
        cmd_lin_vel_x_world = np.clip(self.pid_pos['x'].compute(err_x_world, 0.0, dt, cur_lin_vel_world[0]) + des_lin_vel_world[0], -max_lin, max_lin)
        cmd_lin_vel_y_world = np.clip(self.pid_pos['y'].compute(err_y_world, 0.0, dt, cur_lin_vel_world[1]) + des_lin_vel_world[1], -max_lin, max_lin)
        cmd_lin_vel_z_world = np.clip(self.pid_pos['z'].compute(err_z_world, 0.0, dt, cur_lin_vel_world[2]) + des_lin_vel_world[2], -max_lin, max_lin)

        cmd_ang_vel_roll_world  = np.clip(self.pid_pos['roll'].compute( err_roll_world,  0.0, dt, cur_ang_vel_world[0]) + des_ang_vel_world[0], -max_ang, max_ang)
        cmd_ang_vel_pitch_world = np.clip(self.pid_pos['pitch'].compute(err_pitch_world, 0.0, dt, cur_ang_vel_world[1]) + des_ang_vel_world[1], -max_ang, max_ang)
        cmd_ang_vel_yaw_world   = np.clip(self.pid_pos['yaw'].compute(  err_yaw_world,   0.0, dt, cur_ang_vel_world[2]) + des_ang_vel_world[2], -max_ang, max_ang)

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

    def load_pid_data(self):
        """Reads the CSV and applies the very last row to the controllers."""
        if not self.pid_file_path.exists():
            self.get_logger().info('No CSV history found. Using hardcoded defaults.')
            return

        try:
            with open(self.pid_file_path, mode='r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
                if not rows:
                    self.get_logger().info('CSV file is empty. Using hardcoded defaults.')
                    return
                
                # Grab the most recent entry
                self.last_csv_row = rows[-1]

            # Helper to safely parse strings back to floats
            def get_val(key, default_val):
                return float(self.last_csv_row.get(key, default_val))

            # Dynamically apply the loaded row to the dictionaries
            for loop_name, pid_dict in [('pos', self.pid_pos), ('vel', self.pid_vel)]:
                for axis, pid in pid_dict.items():
                    pid.kp = get_val(f'{loop_name}_{axis}_kp', pid.kp)
                    pid.ki = get_val(f'{loop_name}_{axis}_ki', pid.ki)
                    pid.kd = get_val(f'{loop_name}_{axis}_kd', pid.kd)
                    pid.ff = get_val(f'{loop_name}_{axis}_ff', pid.ff)
            
            self.get_logger().info(f"Loaded PID gains from previous run at: {self.last_csv_row.get('timestamp')}")
        
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV data: {e}')


    def save_pid_data(self):
        """Builds a flat row of all current gains and appends it to the CSV."""
        
        # Start the row with a timestamp
        row = {'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

        # Read live controller values
        for loop_name, pid_dict in [('pos', self.pid_pos), ('vel', self.pid_vel)]:
            for axis, pid in pid_dict.items():
                row[f'{loop_name}_{axis}_kp'] = pid.kp
                row[f'{loop_name}_{axis}_ki'] = pid.ki
                row[f'{loop_name}_{axis}_kd'] = pid.kd
                row[f'{loop_name}_{axis}_ff'] = pid.ff

        # FIX THE MISSING VALUES: If we are tuning, don't save the forced 0.0s. 
        # Copy the true gains from the previous row instead.
        if self.tune and self.last_csv_row:
            for axis in ['z', 'roll', 'pitch']:
                for gain in ['kp', 'ki', 'kd']:
                    col_name = f'vel_{axis}_{gain}'
                    if col_name in self.last_csv_row:
                        row[col_name] = self.last_csv_row[col_name]

        # Append to the file
        headers = list(row.keys())
        try:
            with open(self.pid_file_path, mode='a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=headers)
                
                # Write the header row only if this is a brand new file
                if not self.pid_file_path.exists():
                    self.get_logger().info('Creating new CSV file and writing header.')
                    writer.writeheader()
                    
                writer.writerow(row)
            self.get_logger().info('--- APPENDED NEW PID HISTORY ROW TO CSV ---')
        except Exception as e:
            self.get_logger().error(f'Failed to write to CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected. Shutting down...')
        node.save_pid_data()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()