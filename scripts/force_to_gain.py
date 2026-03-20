#!/usr/bin/env python3
"""
ForceToGain ROS2 node.

Pipeline:
  thruster_forces (ThrusterSetpoint, N)
    -> force->RPM (thrust coefficients + handedness)
    -> RPM->gain (polynomial quadratic inversion, voltage-corrected)
    -> publish thruster_gains   (ThrusterGains, normalized [-1, 1])  -- sim + future
    -> publish thruster_setpoint (ThrusterSetpoint, .thrust=gains)   -- real hardware (can_driver)

Watchdog:
  If no /thruster_forces message arrives within WATCHDOG_TIMEOUT seconds, publishes zero gains
  with armed=False. This prevents stale thrust commands persisting when upstream nodes stop —
  e.g. raw_thrust finishes but PID doesn't take over, controller disabled, or node crash.

Topics:
  Subscribes: thruster_forces    (tauv_msgs/ThrusterSetpoint)
  Subscribes: esc_telemetry      (tauv_msgs/EscTelemetry)   -- for live voltage
  Publishes:  thruster_gains     (tauv_msgs/ThrusterGains)
  Publishes:  thruster_setpoint  (tauv_msgs/ThrusterSetpoint, .thrust = gains [-1,1])
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tauv_msgs.msg import EscTelemetry, ThrusterSetpoint, ThrusterGains
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

WATCHDOG_TIMEOUT = 1.0   # seconds — zero thrusters if no /thruster_forces within this window

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from forceOptimizer import load_thruster_config


class ForceToGain(Node):
    def __init__(self):
        super().__init__('force_to_gain')

        share_dir = Path(get_package_share_directory('tauv_autonomy'))
        yaml_path = share_dir / 'config' / 'thruster_params.yaml'
        self._cfg = load_thruster_config(yaml_path)

        self._right_handed = self._cfg['right_handed']
        self._K_F_fwd = self._cfg['K_F_fwd']
        self._K_F_rev = self._cfg['K_F_rev']
        self._v_bat = self._cfg['v_bat']
        self._gain_max = self._cfg['gain_max']

        self._esc_voltages: dict[int, float] = {}
        self._last_forces_time: float | None = None
        self._watchdog_fired = False
        self._controller_enabled = False   # watchdog only active when controller is enabled

        self._pub = self.create_publisher(ThrusterGains, 'thruster_gains', 10)
        self._setpoint_pub = self.create_publisher(ThrusterSetpoint, 'thruster_setpoint', 10)
        self._ctrl_pub = self.create_publisher(Bool, 'controller_enabled', 10)
        self.create_subscription(ThrusterSetpoint, 'thruster_forces', self._forces_cb, 10)
        self.create_subscription(EscTelemetry, 'esc_telemetry', self._telemetry_cb, 10)
        self.create_subscription(Bool, 'controller_enabled', self._ctrl_state_cb, 10)
        self.create_timer(0.1, self._watchdog_cb)   # 10 Hz

        self.get_logger().info('ForceToGain initialized.')

    def _telemetry_cb(self, msg: EscTelemetry):
        if msg.voltage > 0.0:
            self._esc_voltages[msg.id] = msg.voltage

    def _ctrl_state_cb(self, msg: Bool):
        self._controller_enabled = msg.data
        if not msg.data:
            # Disabled: clear watchdog so intentional silence doesn't trigger it
            self._last_forces_time = None
            self._watchdog_fired = False

    def _watchdog_cb(self):
        if not self._controller_enabled:
            return   # controller disabled — silence is expected
        if self._last_forces_time is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_forces_time > WATCHDOG_TIMEOUT:
            if not self._watchdog_fired:
                self.get_logger().error('=' * 60)
                self.get_logger().error('!!! WATCHDOG FIRED !!!')
                self.get_logger().error(
                    f'No /thruster_forces for >{WATCHDOG_TIMEOUT}s — '
                    'ZEROING THRUSTERS and DISABLING CONTROLLER')
                self.get_logger().error('=' * 60)
                self._watchdog_fired = True
                # Disable the controller so it stops producing stale cmd_wrench
                ctrl = Bool()
                ctrl.data = False
                self._ctrl_pub.publish(ctrl)
            self._publish_zero()

    def _publish_zero(self):
        stamp = self.get_clock().now().to_msg()
        out = ThrusterGains()
        out.header.stamp = stamp
        out.header.frame_id = 'os/base_link'
        out.gain = [0.0] * 8
        out.armed = False
        self._pub.publish(out)
        setpoint = ThrusterSetpoint()
        setpoint.header.stamp = stamp
        setpoint.header.frame_id = 'os/base_link'
        setpoint.thrust = [0.0] * 8
        setpoint.armed = False
        self._setpoint_pub.publish(setpoint)

    def _forces_cb(self, msg: ThrusterSetpoint):
        self._last_forces_time = self.get_clock().now().nanoseconds * 1e-9
        self._watchdog_fired = False
        voltage = self._get_voltage()
        gains = []
        for i in range(8):
            rpm = self._force_to_rpm(msg.thrust[i], i)
            gain_raw = self._rpm_to_gain(rpm, voltage)
            # Normalize to [-1, 1]
            gain_norm = gain_raw / float(self._gain_max)
            gain_norm = max(-1.0, min(1.0, gain_norm))
            gains.append(gain_norm)

        stamp = self.get_clock().now().to_msg()

        out = ThrusterGains()
        out.header.stamp = stamp
        out.header.frame_id = 'os/base_link'
        out.gain = [float(g) for g in gains]
        out.armed = msg.armed
        self._pub.publish(out)

        # Real hardware: can_driver subscribes to ThrusterSetpoint on thruster_setpoint,
        # treating .thrust as normalized throttle [-1, 1].
        setpoint = ThrusterSetpoint()
        setpoint.header.stamp = stamp
        setpoint.header.frame_id = 'os/base_link'
        setpoint.thrust = [float(g) for g in gains]
        setpoint.armed = msg.armed
        self._setpoint_pub.publish(setpoint)

    def _get_voltage(self) -> float:
        if self._esc_voltages:
            return sum(self._esc_voltages.values()) / len(self._esc_voltages)
        return self._v_bat

    def _force_to_rpm(self, force: float, thruster_idx: int) -> float:
        """Convert desired thruster force [N] to RPM.

        F = K_F * omega^2, omega = RPM * 2*pi/60
        => RPM = sqrt(|F| / K_F) * 60/(2*pi) * sign(F)
        Left-handed thrusters negate RPM.
        """
        K_F = self._K_F_fwd if force >= 0.0 else self._K_F_rev
        omega_abs = math.sqrt(abs(force) / K_F)
        rpm = math.copysign(omega_abs * 60.0 / (2.0 * math.pi), force)
        if not self._right_handed[thruster_idx]:
            rpm = -rpm
        return rpm

    def _rpm_to_gain(self, rpm: float, voltage: float) -> float:
        """Convert target RPM to DroneCAN gain using polynomial inversion.

        The gain->RPM polynomial is:
          RPM = a + b*gain + c*V + d*gain^2 + e*gain*V + f*V^2
              + g*gain^2*V + h*gain*V^2 + i*V^3

        Rearranged as quadratic in gain:
          (d + g*V)*gain^2 + (b + e*V + h*V^2)*gain
          + (a + c*V + f*V^2 + i*V^3 - RPM) = 0

        Result clamped to [-gain_max, gain_max].
        """
        if rpm == 0.0:
            return 0.0

        use_positive_side = (rpm > 0.0)
        poly = self._cfg['poly_pos'] if use_positive_side else self._cfg['poly_neg']
        rpm_abs = abs(rpm)
        V = voltage

        a = poly['intercept']
        b = poly['gain1']
        c = poly['voltage1']
        d = poly['gain2']
        e = poly['gain_voltage']
        f = poly['voltage2']
        g = poly['gain2_voltage']
        h = poly['gain_voltage2']
        k = poly['voltage3']

        A = d + g * V
        B = b + e * V + h * V * V
        C = a + c * V + f * V * V + k * V * V * V - rpm_abs

        gain_abs = self._solve_quadratic(A, B, C)
        gain_abs = max(0.0, min(float(self._gain_max), gain_abs))

        return gain_abs if use_positive_side else -gain_abs

    @staticmethod
    def _solve_quadratic(A: float, B: float, C: float) -> float:
        """Solve A*x^2 + B*x + C = 0, returning the positive root."""
        if abs(A) < 1e-12:
            if abs(B) < 1e-12:
                return 0.0
            return max(0.0, -C / B)

        discriminant = B * B - 4.0 * A * C
        if discriminant < 0.0:
            discriminant = 0.0

        sqrt_disc = math.sqrt(discriminant)
        x1 = (-B + sqrt_disc) / (2.0 * A)
        x2 = (-B - sqrt_disc) / (2.0 * A)

        candidates = [x for x in (x1, x2) if x >= 0.0]
        if not candidates:
            return 0.0
        return min(candidates)


def main(args=None):
    rclpy.init(args=args)
    node = ForceToGain()
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
