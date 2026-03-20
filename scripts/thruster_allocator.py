#!/usr/bin/env python3
"""
ThrusterAllocator ROS2 node.

Pipeline:
  cmd_wrench (WrenchStamped) -> TAM pseudoinverse -> per-thruster forces
    -> publish thruster_forces (ThrusterSetpoint, units = N)

Topics:
  Subscribes: cmd_wrench                  (geometry_msgs/WrenchStamped)
  Publishes:  thruster_forces             (tauv_msgs/ThrusterSetpoint)
              thruster_allocation_matrix      (std_msgs/Float64MultiArray, latched 6x8 row-major)
                  rows = [x, y, z, roll, pitch, yaw], cols = [blh, flu, blu, flh, brh, fru, bru, frh]
              thruster_allocation_matrix_str  (std_msgs/String, latched, human-readable table)
              thruster_pseudoinverse          (std_msgs/Float64MultiArray, latched 8x6 row-major)
                  rows = [blh, flu, blu, flh, brh, fru, bru, frh], cols = [x, y, z, roll, pitch, yaw]
              thruster_pseudoinverse_str      (std_msgs/String, latched, human-readable table)
              thruster_forces_str             (std_msgs/String, per-callback wrench + forces table)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout, String
from tauv_msgs.msg import ThrusterSetpoint
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from forceOptimizer import load_thruster_config, get_complete_tam, get_pseudoinverse, solve_thrusts

class ThrusterAllocator(Node):
    def __init__(self):
        super().__init__('thruster_allocator')

        self.declare_parameter('sim', False)
        sim = self.get_parameter('sim').get_parameter_value().bool_value

        share_dir = Path(get_package_share_directory('tauv_autonomy'))
        yaml_path = share_dir / 'config' / 'thruster_params.yaml'
        self._cfg = load_thruster_config(yaml_path)

        flip_key = 'flip_signs_sim' if sim else 'flip_signs_real'
        flip_signs = self._cfg[flip_key]
        self.get_logger().info(f'Using {flip_key}: {flip_signs}')

        self._A_plus = get_pseudoinverse(self._cfg, flip_signs)  # precomputed once
        self._flip_signs = flip_signs

        self._pub = self.create_publisher(ThrusterSetpoint, 'thruster_forces', 10)
        self.create_subscription(WrenchStamped, 'cmd_wrench', self._wrench_cb, 10)

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._tam_pub = self.create_publisher(
            Float64MultiArray, 'thruster_allocation_matrix', latched_qos
        )
        self._tam_str_pub = self.create_publisher(
            String, 'thruster_allocation_matrix_str', latched_qos
        )
        self._pinv_pub = self.create_publisher(
            Float64MultiArray, 'thruster_pseudoinverse', latched_qos
        )
        self._pinv_str_pub = self.create_publisher(
            String, 'thruster_pseudoinverse_str', latched_qos
        )
        self._forces_str_pub = self.create_publisher(String, 'thruster_forces_str', 10)
        self._publish_tam()
        self._publish_pseudoinverse()

        self.get_logger().info('ThrusterAllocator initialized.')

    def _publish_tam(self):
        A = get_complete_tam(self._cfg, self._flip_signs)  # shape (6, 8); rows=[x,y,z,roll,pitch,yaw], cols=thrusters
        rows, cols = A.shape

        mat_msg = Float64MultiArray()
        mat_msg.layout = MultiArrayLayout(
            dim=[
                MultiArrayDimension(label='dof',     size=rows, stride=rows * cols),
                MultiArrayDimension(label='thruster', size=cols, stride=cols),
            ],
            data_offset=0,
        )
        mat_msg.data = A.flatten().tolist()
        self._tam_pub.publish(mat_msg)

        dof_labels     = ['x    ', 'y    ', 'z    ', 'roll ', 'pitch', 'yaw  ']
        thruster_names = ['blh', 'flu', 'blu', 'flh', 'brh', 'fru', 'bru', 'frh']
        header = '       ' + '  '.join(f'{n:>7}' for n in thruster_names)
        lines = [header]
        for i, label in enumerate(dof_labels):
            row_str = '  '.join(f'{v:+7.4f}' for v in A[i])
            lines.append(f'{label}  {row_str}')
        str_msg = String()
        str_msg.data = '\n'.join(lines)
        self._tam_str_pub.publish(str_msg)

        self.get_logger().info(f'Published thruster_allocation_matrix ({rows}x{cols})')

    def _publish_pseudoinverse(self):
        P = self._A_plus  # shape (8, 6); rows=thrusters, cols=[x,y,z,roll,pitch,yaw]
        rows, cols = P.shape

        mat_msg = Float64MultiArray()
        mat_msg.layout = MultiArrayLayout(
            dim=[
                MultiArrayDimension(label='thruster', size=rows, stride=rows * cols),
                MultiArrayDimension(label='dof',      size=cols, stride=cols),
            ],
            data_offset=0,
        )
        mat_msg.data = P.flatten().tolist()
        self._pinv_pub.publish(mat_msg)

        thruster_names = ['blh', 'flu', 'blu', 'flh', 'brh', 'fru', 'bru', 'frh']
        dof_labels     = ['x    ', 'y    ', 'z    ', 'roll ', 'pitch', 'yaw  ']
        header = '         ' + '  '.join(f'{d:>7}' for d in dof_labels)
        lines = [header]
        for i, name in enumerate(thruster_names):
            row_str = '  '.join(f'{v:+7.4f}' for v in P[i])
            lines.append(f'{name:<7}  {row_str}')
        str_msg = String()
        str_msg.data = '\n'.join(lines)
        self._pinv_str_pub.publish(str_msg)

        self.get_logger().info(f'Published thruster_pseudoinverse ({rows}x{cols})')

    def _wrench_cb(self, msg: WrenchStamped):
        wrench = [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ]

        # cmd_wrench is in ENU/FLU body frame — controller outputs ENU directly.
        forces = solve_thrusts(wrench, self._A_plus)
        forces = np.where(np.abs(forces) < 1e-9, 0.0, forces)

        out = ThrusterSetpoint()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'os/base_link'
        out.thrust = [float(f) for f in forces]
        out.armed = True
        self._pub.publish(out)

        dof_labels     = ['x    ', 'y    ', 'z    ', 'roll ', 'pitch', 'yaw  ']
        thruster_names = ['blh', 'flu', 'blu', 'flh', 'brh', 'fru', 'bru', 'frh']
        wrench_lines = ['wrench (N / N·m):']
        for label, val in zip(dof_labels, wrench):
            wrench_lines.append(f'  {label}  {val:+.4f}')
        forces_header = '         ' + '  '.join(f'{n:>7}' for n in thruster_names)
        forces_row    = 'forces   ' + '  '.join(f'{f:+7.4f}' for f in forces)
        str_msg = String()
        str_msg.data = '\n'.join(wrench_lines + [forces_header, forces_row])
        self._forces_str_pub.publish(str_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocator()
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
