#!/usr/bin/env python3
"""
play_commands.py — Scripted thruster command sequence loaded from YAML.

Reads a sequence file and publishes each step at 10 Hz for its duration.
Supports two step types:
  type: wrench  — publishes WrenchStamped to /cmd_wrench (via thruster_controller TAM)
  type: rpm     — publishes ThrusterRPM directly to /thruster_rpm (bypasses controller)

Default sequence file: <tauv_autonomy share>/config/play_sequence.yaml
Override with ROS parameter: sequence_file:=/path/to/file.yaml

Usage (via launch file):
  ros2 launch tauv_sim thruster_test.launch.py play:=true

Usage (standalone):
  ros2 run tauv_autonomy play_commands.py
"""

import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from tauv_msgs.msg import ThrusterRPM
from ament_index_python.packages import get_package_share_directory

PUBLISH_HZ = 10.0


class PlayCommands(Node):
    def __init__(self):
        super().__init__('play_commands')

        share_dir = Path(get_package_share_directory('tauv_autonomy'))
        default_seq = str(share_dir / 'config' / 'play_sequence.yaml')

        self.declare_parameter('sequence_file', default_seq)

        self._wrench_pub = self.create_publisher(WrenchStamped, 'cmd_wrench', 10)
        self._rpm_pub = self.create_publisher(ThrusterRPM, 'thruster_rpm', 10)

    def run(self):
        seq_path = self.get_parameter('sequence_file').value
        self.get_logger().info(f'Loading sequence from: {seq_path}')

        with open(seq_path, 'r') as f:
            data = yaml.safe_load(f)

        startup_delay = float(data.get('startup_delay', 3.0))
        sequence = data['sequence']

        self.get_logger().info(
            f'Waiting {startup_delay:.1f}s for system to initialise...'
        )
        time.sleep(startup_delay)

        total = len(sequence)
        for i, step in enumerate(sequence):
            step_type = step['type']
            duration = float(step['duration'])

            if step_type == 'wrench':
                w = step['wrench']
                self.get_logger().info(
                    f'[{i+1}/{total}] wrench  '
                    f'[{w[0]:.1f}, {w[1]:.1f}, {w[2]:.1f}, '
                    f'{w[3]:.1f}, {w[4]:.1f}, {w[5]:.1f}]  '
                    f'{duration:.1f}s'
                )
                self._run_wrench_step(w[0], w[1], w[2], w[3], w[4], w[5], duration)

            elif step_type == 'rpm':
                r = step['rpm']
                self.get_logger().info(
                    f'[{i+1}/{total}] rpm     '
                    f'{[int(x) for x in r]}  {duration:.1f}s'
                )
                self._run_rpm_step(r, duration)

            else:
                self.get_logger().warn(f'[{i+1}/{total}] unknown type "{step_type}", skipping')

        # Teardown: zero everything
        self._publish_wrench(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._publish_rpm([0.0] * 8, armed=False)
        self.get_logger().info('Sequence complete.')

    # ------------------------------------------------------------------

    def _run_wrench_step(self, fx, fy, fz, tx, ty, tz, duration):
        interval = 1.0 / PUBLISH_HZ
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self._publish_wrench(fx, fy, fz, tx, ty, tz)
            time.sleep(interval)

    def _run_rpm_step(self, rpm_list, duration):
        interval = 1.0 / PUBLISH_HZ
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self._publish_rpm(rpm_list, armed=True)
            time.sleep(interval)

    def _publish_wrench(self, fx, fy, fz, tx, ty, tz):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'os/base_link'
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        msg.wrench.force.z = fz
        msg.wrench.torque.x = tx
        msg.wrench.torque.y = ty
        msg.wrench.torque.z = tz
        self._wrench_pub.publish(msg)

    def _publish_rpm(self, rpm_list, armed: bool):
        msg = ThrusterRPM()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'os/base_link'
        msg.rpm = [float(r) for r in rpm_list]
        msg.armed = armed
        self._rpm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlayCommands()
    try:
        node.run()
        rclpy.spin(node)  # stay alive so foxglove_bridge can introspect cleanly
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
