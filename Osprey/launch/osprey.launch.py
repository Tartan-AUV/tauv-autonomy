from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    autonomy_share_dir = Path(get_package_share_directory('tauv_autonomy'))
    urdf_file = autonomy_share_dir / 'Osprey' / 'urdf' / 'sim_model.urdf'

    with open(str(urdf_file), 'r') as f:
        robot_description = f.read()

    controller_params = str(autonomy_share_dir / 'config' / 'controller_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='Set true when running in simulation; selects sim thruster sign conventions'
        ),

        # Publishes TF tree from URDF:
        #   os/base_link -> imu_xsens_link  (xyz=-0.017 -0.111 0.047)
        #   os/base_link -> dvl_link         (xyz=0 -0.141 0.010)
        #   os/base_link -> depth_link        (xyz=0.064 -0.200 0.080)
        # Coordinate convention: sensor joint rotations are zeroed.
        # NED/FRD->ENU/FLU conversion is handled by tauv_sim bridges (sim)
        # or tauv_drivers (real robot) before data reaches this launch.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # 6-DOF PID controller:
        #   /odometry/filtered + /desired_state (PoseStamped) -> /cmd_wrench (NED body frame)
        Node(
            package='tauv_autonomy',
            executable='controller.py',
            name='controller',
            parameters=[controller_params],
            output='screen',
        ),

        # Thrust allocation:
        #   /cmd_wrench (ENU/FLU) -> TAM pseudoinverse -> /thruster_forces (ThrusterSetpoint, N)
        Node(
            package='tauv_autonomy',
            executable='thruster_allocator.py',
            name='thruster_allocator',
            parameters=[{'sim': LaunchConfiguration('sim')}],
            output='screen',
        ),

        # Force -> gain conversion:
        #   /thruster_forces + /esc_telemetry -> /thruster_gains (ThrusterGains, [-1,1])
        Node(
            package='tauv_autonomy',
            executable='force_to_gain.py',
            name='force_to_gain',
            output='screen',
        ),

        # TODO: path_planner_node
        # Suggested package: tauv_planner (TBD)
        # Subscribes to: /odometry/filtered (nav_msgs/Odometry)
        # Publishes to:  /desired_state (geometry_msgs/PoseStamped)
    ])
