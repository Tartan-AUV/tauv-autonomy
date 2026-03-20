#!/usr/bin/env python3
"""
Controller ROS2 node — 6-DOF PID feedback controller.

Pipeline:
  /odometry/filtered (nav_msgs/Odometry)  -- current state from EKF
  /desired_state     (geometry_msgs/PoseStamped)  -- desired position + orientation
    -> 6-DOF PID (x, y, z, roll, pitch, yaw)
    -> publish /cmd_wrench (geometry_msgs/WrenchStamped) in body frame (ENU/FLU)

Topics:
  Subscribes: /odometry/filtered  (nav_msgs/Odometry)
  Subscribes: /desired_state      (geometry_msgs/PoseStamped)
  Publishes:  cmd_wrench          (geometry_msgs/WrenchStamped)

Frame convention: wrench is expressed in body frame os/base_link (ENU/FLU: x=forward, y=left, z=up).
The EKF provides pose in ENU world frame; the error rotation produces ENU/FLU body-frame outputs directly.
Orientation error is computed as euler angle difference (roll/pitch/yaw).
"""

import math
from collections import deque
import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from ament_index_python.packages import get_package_share_directory
import yaml
from pathlib import Path


def _quat_to_euler(x, y, z, w):
    """Convert quaternion to (roll, pitch, yaw) in radians."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _angle_diff(a, b):
    """Shortest signed angular difference a - b, result in (-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        share_dir = Path(get_package_share_directory('tauv_autonomy'))
        yaml_path = share_dir / 'config' / 'controller_params.yaml'
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        cfg = data['controller']['ros__parameters']

        self._kp = cfg['kp']              # [kp_x, kp_y, kp_z, kp_roll, kp_pitch, kp_yaw]
        self._ki = cfg['ki']
        self._kd = cfg['kd']
        self._ff_bias = cfg.get('ff_bias', [0.0] * 6)   # constant feedforward per DOF
        self._kff = cfg.get('kff', [0.0] * 6)           # velocity feedforward gain per DOF
        self._max_wrench = cfg['max_wrench']
        self._integ_max_frac = float(cfg['integrator_max_fraction'])
        self._integ_max = [self._integ_max_frac * m for m in self._max_wrench]
        rate = float(cfg['rate'])
        self._rate = rate
        window_sec = float(cfg.get('integrator_window', 1.0))

        self._integ_history = [deque(maxlen=max(1, round(window_sec * rate))) for _ in range(6)]
        self._prev_errors = [0.0] * 6
        self._dt = 1.0 / rate

        self._current_odom: Odometry | None = None
        self._desired_pose: PoseStamped | None = None
        # Previous desired position in world frame — used to compute setpoint velocity for kff
        self._prev_des_pos = None   # (x, y, z)
        self._prev_des_rpy = None   # (roll, pitch, yaw)

        # Always starts disabled. Enable by publishing:
        #   ros2 topic pub --once /controller_enabled std_msgs/msg/Bool "data: true"
        # (waypoint_runner does this automatically when system_enable is received)
        self._enabled = False

        self._pub = self.create_publisher(WrenchStamped, 'cmd_wrench', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(PoseStamped, 'desired_state', self._desired_cb, 10)
        # Use regular (non-latched) QoS so stale True messages from previous runs
        # don't immediately re-enable the controller on startup.
        self.create_subscription(Bool, 'controller_enabled', self._enabled_cb, 10)

        # Declare ROS2 parameters so Foxglove Parameters panel can edit gains live.
        self.declare_parameter('kp',                       self._kp)
        self.declare_parameter('ki',                       self._ki)
        self.declare_parameter('kd',                       self._kd)
        self.declare_parameter('ff_bias',                  self._ff_bias)
        self.declare_parameter('kff',                      self._kff)
        self.declare_parameter('max_wrench',               self._max_wrench)
        self.declare_parameter('integrator_max_fraction',  self._integ_max_frac)
        self.declare_parameter('integrator_window',        window_sec)
        # Resolve symlink so the overrides file lands in the source tree, not the install dir.
        # With symlink-install, share_dir/config/controller_params.yaml is a symlink to src.
        # Writing directly to install/share would be wiped on rebuild.
        yaml_real = (share_dir / 'config' / 'controller_params.yaml').resolve()
        self._overrides_path = yaml_real.parent / 'controller_params_overrides.yaml'
        self._suppress_write = False

        # Load persisted overrides (written by _write_overrides on every param change).
        # Suppress the write-back triggered by set_parameters so we don't write on startup.
        if self._overrides_path.exists():
            with open(self._overrides_path) as f:
                overrides = yaml.safe_load(f) or {}
            if overrides:
                self._suppress_write = True
                self.set_parameters([
                    rclpy.parameter.Parameter(k, value=v) for k, v in overrides.items()
                ])
                self._suppress_write = False
                self.get_logger().info(
                    f'Loaded overrides from {self._overrides_path.name}: '
                    + ', '.join(overrides.keys())
                )
        else:
            self.get_logger().info('No overrides file — using controller_params.yaml defaults')

        self.add_on_set_parameters_callback(self._on_params_changed)

        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info('Controller initialized.')

    def _odom_cb(self, msg: Odometry):
        self._current_odom = msg

    def _desired_cb(self, msg: PoseStamped):
        self._desired_pose = msg

    def _on_params_changed(self, params):
        for p in params:
            if p.name in ('kp', 'ki', 'kd', 'ff_bias', 'kff'):
                if len(p.value) != 6:
                    return SetParametersResult(successful=False,
                        reason=f'{p.name} must have 6 elements')
                setattr(self, f'_{p.name}', list(p.value))
            elif p.name == 'max_wrench':
                if len(p.value) != 6:
                    return SetParametersResult(successful=False,
                        reason='max_wrench must have 6 elements')
                self._max_wrench = list(p.value)
                self._integ_max = [self._integ_max_frac * m for m in self._max_wrench]
            elif p.name == 'integrator_max_fraction':
                self._integ_max_frac = float(p.value)
                self._integ_max = [self._integ_max_frac * m for m in self._max_wrench]
            elif p.name == 'integrator_window':
                new_steps = max(1, round(float(p.value) * self._rate))
                self._integ_history = [deque(maxlen=new_steps) for _ in range(6)]
        if not self._suppress_write:
            self.get_logger().info(f'Params updated: {[p.name for p in params]}')
            self._write_overrides()
        return SetParametersResult(successful=True)

    def _write_overrides(self):
        try:
            data = {
                'kp':                      self._kp,
                'ki':                      self._ki,
                'kd':                      self._kd,
                'ff_bias':                 self._ff_bias,
                'kff':                     self._kff,
                'max_wrench':              self._max_wrench,
                'integrator_max_fraction': self._integ_max_frac,
                'integrator_window':       self._integ_history[0].maxlen / self._rate,
            }
            with open(self._overrides_path, 'w') as f:
                yaml.safe_dump(data, f)
            self.get_logger().info(f'Saved overrides → {self._overrides_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write overrides: {e}')

    def _enabled_cb(self, msg: Bool):
        self._enabled = msg.data
        if not self._enabled:
            # Reset integrators so there's no windup when re-enabled
            for d in self._integ_history:
                d.clear()
            self._prev_errors = [0.0] * 6
            self._prev_des_pos = None
            self._prev_des_rpy = None

    def _control_loop(self):
        if not self._enabled:
            return
        if self._current_odom is None or self._desired_pose is None:
            return

        # Current state
        cur_pos = self._current_odom.pose.pose.position
        cur_q = self._current_odom.pose.pose.orientation
        cur_roll, cur_pitch, cur_yaw = _quat_to_euler(cur_q.x, cur_q.y, cur_q.z, cur_q.w)

        # Desired state
        des_pos = self._desired_pose.pose.position
        des_q = self._desired_pose.pose.orientation
        des_roll, des_pitch, des_yaw = _quat_to_euler(des_q.x, des_q.y, des_q.z, des_q.w)

        # Position error in world frame, then rotate to body frame (NED)
        # For simplicity, use yaw rotation only (small angle / near-level assumption)
        dx_world = des_pos.x - cur_pos.x
        dy_world = des_pos.y - cur_pos.y
        dz = des_pos.z - cur_pos.z

        cos_yaw = math.cos(cur_yaw)
        sin_yaw = math.sin(cur_yaw)
        ex = cos_yaw * dx_world + sin_yaw * dy_world   # forward (ENU/FLU x)
        ey = -sin_yaw * dx_world + cos_yaw * dy_world  # left    (ENU/FLU y)
        ez = dz                                          # up      (ENU/FLU z)

        # Orientation error
        e_roll = _angle_diff(des_roll, cur_roll)
        e_pitch = _angle_diff(des_pitch, cur_pitch)
        e_yaw = _angle_diff(des_yaw, cur_yaw)

        errors = [ex, ey, ez, e_roll, e_pitch, e_yaw]

        # Velocity feedforward: derivative of desired setpoint, rotated to body NED frame
        cur_des_pos = (des_pos.x, des_pos.y, des_pos.z)
        cur_des_rpy = (des_roll, des_pitch, des_yaw)
        if self._prev_des_pos is None:
            des_vel_body = [0.0] * 6
        else:
            dvx_w = (cur_des_pos[0] - self._prev_des_pos[0]) / self._dt
            dvy_w = (cur_des_pos[1] - self._prev_des_pos[1]) / self._dt
            dvz_w = (cur_des_pos[2] - self._prev_des_pos[2]) / self._dt
            # Rotate world-frame velocity to body NED (same yaw rotation as position error)
            des_vel_body = [
                cos_yaw * dvx_w + sin_yaw * dvy_w,    # forward (NED x)
                -sin_yaw * dvx_w + cos_yaw * dvy_w,   # right   (NED y)
                dvz_w,                                  # down    (NED z)
                (cur_des_rpy[0] - self._prev_des_rpy[0]) / self._dt,  # roll rate
                (cur_des_rpy[1] - self._prev_des_rpy[1]) / self._dt,  # pitch rate
                _angle_diff(cur_des_rpy[2], self._prev_des_rpy[2]) / self._dt,  # yaw rate
            ]
        self._prev_des_pos = cur_des_pos
        self._prev_des_rpy = cur_des_rpy

        # PID + feedforward
        wrench = []
        for i in range(6):
            self._integ_history[i].append(errors[i] * self._dt)
            integral = max(-self._integ_max[i],
                           min(self._integ_max[i], sum(self._integ_history[i])))

            derivative = (errors[i] - self._prev_errors[i]) / self._dt
            self._prev_errors[i] = errors[i]

            feedforward = self._ff_bias[i] + self._kff[i] * des_vel_body[i]
            output = (feedforward
                      + self._kp[i] * errors[i]
                      + self._ki[i] * integral
                      + self._kd[i] * derivative)
            output = max(-self._max_wrench[i], min(self._max_wrench[i], output))
            wrench.append(output)

        out = WrenchStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'os/base_link'
        out.wrench.force.x = wrench[0]
        out.wrench.force.y = wrench[1]
        out.wrench.force.z = wrench[2]
        out.wrench.torque.x = wrench[3]
        out.wrench.torque.y = wrench[4]
        out.wrench.torque.z = wrench[5]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
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
