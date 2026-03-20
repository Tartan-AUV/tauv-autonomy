"""
pid_tuning_real.launch.py

PID tuning on the real Osprey AUV. Starts the tauv_core sensor converters,
EKF, full autonomy stack, waypoint runner, and Foxglove bridge.

Assumes hardware drivers are already running (separate terminal):
  ros2 launch tauv_launch sensors.launch.py

Usage:
  ros2 launch tauv_autonomy Osprey/launch/pid_tuning_real.launch.py [record:=true]

Tuning workflow:
  1. Edit tauv_autonomy/config/waypoints.yaml  — set waypoints that isolate one axis
  2. Edit tauv_autonomy/config/controller_params.yaml — adjust kp/ki/kd/ff_bias/kff
     OR edit live via Foxglove Parameters panel → /controller (no rebuild needed)
  3. colcon build && source install/setup.bash  (only needed for waypoints.yaml changes)
  4. ros2 launch tauv_autonomy Osprey/launch/pid_tuning_real.launch.py record:=true
  5. Review response in Foxglove (ws://localhost:8765)

Key Foxglove topics:
  /odometry/filtered   — EKF fused pose/velocity
  /desired_state       — active waypoint target from waypoint_runner
  /cmd_wrench          — PID output (ENU/FLU body frame)
  /thruster_forces     — per-thruster forces [N]
  /waypoint_status     — current waypoint index + error values
"""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    autonomy_share_dir = Path(get_package_share_directory('tauv_autonomy'))
    core_share_dir     = Path(get_package_share_directory('tauv_core'))

    osprey_launch_file = autonomy_share_dir / 'Osprey' / 'launch' / 'osprey.launch.py'
    ekf_file           = core_share_dir / 'config' / 'ekfFUNNY.yaml'

    timestamp  = datetime.now().strftime('%Y.%m.%d_%H.%M.%S')
    bag_output = Path('/tauv-mono/ros_ws') / 'bags' / f'pid_tuning_real_{timestamp}'

    return LaunchDescription([
        DeclareLaunchArgument(
            'record', default_value='false',
            description='Record all topics to a bag in ros_ws/bags/'
        ),

        LogInfo(msg='pid_tuning_real: ensure "ros2 launch tauv_launch sensors.launch.py" is running first.'),

        # ── tauv_core sensor converters ──────────────────────────────────────────
        # Convert hardware-specific message types → os/sensors/* (ENU/FLU convention)
        # for the EKF and autonomy stack.
        Node(
            package='tauv_core',
            executable='imu_converter',
            name='imu_converter',
            output='screen',
        ),
        Node(
            package='tauv_core',
            executable='depth_converter',
            name='depth_converter',
            output='screen',
        ),
        Node(
            package='tauv_core',
            executable='dvl_converter',
            name='dvl_converter',
            output='screen',
        ),

        # ── EKF ─────────────────────────────────────────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[str(ekf_file)],
            output='screen',
        ),

        # ── autonomy stack (sim:=false → real hardware sign conventions) ─────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(osprey_launch_file)),
            launch_arguments={'sim': 'false'}.items(),
        ),

        # ── waypoint runner ──────────────────────────────────────────────────────
        Node(
            package='tauv_autonomy',
            executable='waypoint_runner.py',
            name='waypoint_runner',
            output='screen',
        ),

        # ── optional bag recording ───────────────────────────────────────────────
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record')),
            cmd=[
                'ros2', 'bag', 'record',
                '-a',
                '-s', 'mcap',
                '--polling-interval', '1',
                '-o', str(bag_output),
            ],
            output='screen',
        ),

        # ── Foxglove bridge ──────────────────────────────────────────────────────
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765, 'address': '0.0.0.0'}],
        ),
    ])
