"""
Standalone script (no ROS 2 required) for testing core.py:
builds a synthetic start/target VehicleState pair, generates a cubic
Hermite path, and plots position/velocity/acceleration profiles, the 3D
path, and a simulated Pure Pursuit run chasing it.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)

from core import (
    PurePursuitPlanner,
    VehicleState,
    quat_inverse,
    quat_multiply,
    quat_to_rot_vec,
    rot_vec_to_quat,
    quat_to_euler
)


def make_state(position, yaw_deg, linear_velocity, angular_velocity) -> VehicleState:
    """Builds a VehicleState from a position, a yaw angle (deg, about Z), and velocities"""
    yaw = np.radians(yaw_deg)
    orientation = np.array([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)])
    return VehicleState(
        position=np.array(position, dtype=float),
        orientation=orientation,
        linear_velocity=np.array(linear_velocity, dtype=float),
        angular_velocity=np.array(angular_velocity, dtype=float),
    )


def path_field(planner: PurePursuitPlanner, field_name: str) -> np.ndarray:
    """Stacks a per-axis VehicleState field (e.g. 'position') across the whole path"""
    return np.array([getattr(state, field_name) for state in planner.path])


def path_times(planner: PurePursuitPlanner) -> np.ndarray:
    """Reconstructs the exact sample times used inside generate_path()"""
    n = len(planner.path)
    return np.array([min(i * planner.resolution, planner.duration) for i in range(n)])


def main():
    start = make_state(position=[5.252564288914845, 4.5815351007211875, -1.0255774110218008], yaw_deg=-8.78,
                        linear_velocity=[0.2542218324099031, 0.015004018145670026, 0.16842912674738988], angular_velocity=[-0.05356658761106165, -0.2747009546492725, -0.07554932494740169])
    target = make_state(position=[14.440000000000001, 5.6, -1.0], yaw_deg=0.0,
                         linear_velocity=[0.0, 0.0, 0.0], angular_velocity=[0.0, 0.0, 0.0])

    planner = PurePursuitPlanner()
    planner.generate_path(start, target, speed_mps=0.4, res_sec=0.05, use_target_vel=True)

    t = path_times(planner)

    # --- Translational Data ---
    positions = path_field(planner, 'position')
    velocities = path_field(planner, 'linear_velocity')
    accelerations = path_field(planner, 'linear_acceleration')

    # --- Rotational Data ---
    orientations = path_field(planner, 'orientation')
    angular_velocities = path_field(planner, 'angular_velocity')
    angular_accelerations = path_field(planner, 'angular_acceleration')

    # Convert quaternions to euler (roll, pitch, yaw) and rates to degrees for plotting
    eulers = np.degrees(np.array([quat_to_euler(q) for q in orientations]))
    ang_vels_deg = np.degrees(angular_velocities)
    ang_accs_deg = np.degrees(angular_accelerations)

    # =========================================================================
    # 1. Translational Profiles (X, Y, Z)
    # =========================================================================
    trans_labels = ['X', 'Y', 'Z']
    fig_trans, axes_trans = plt.subplots(3, 3, figsize=(14, 9))
    for axis in range(3):
        axes_trans[0, axis].plot(t, positions[:, axis])
        axes_trans[0, axis].set_title(f'Position {trans_labels[axis]} [m]')

        axes_trans[1, axis].plot(t, velocities[:, axis])
        axes_trans[1, axis].set_title(f'Velocity {trans_labels[axis]} [m/s]')

        axes_trans[2, axis].plot(t, accelerations[:, axis])
        axes_trans[2, axis].set_title(f'Acceleration {trans_labels[axis]} [m/s^2]')

    for row in axes_trans:
        for ax in row:
            ax.set_xlabel('t [s]')
            ax.grid(True, alpha=0.3)

    fig_trans.suptitle('Cubic Hermite Trajectory - Translational Profiles')
    fig_trans.tight_layout()

    # =========================================================================
    # 2. Rotational Profiles (Roll, Pitch, Yaw)
    # =========================================================================
    rot_labels = ['Roll', 'Pitch', 'Yaw']
    fig_rot, axes_rot = plt.subplots(3, 3, figsize=(14, 9))
    for axis in range(3):
        axes_rot[0, axis].plot(t, eulers[:, axis])
        axes_rot[0, axis].set_title(f'{rot_labels[axis]} [deg]')

        axes_rot[1, axis].plot(t, ang_vels_deg[:, axis])
        axes_rot[1, axis].set_title(f'Angular Velocity {rot_labels[axis]} [deg/s]')

        axes_rot[2, axis].plot(t, ang_accs_deg[:, axis])
        axes_rot[2, axis].set_title(f'Angular Accel {rot_labels[axis]} [deg/s^2]')

    for row in axes_rot:
        for ax in row:
            ax.set_xlabel('t [s]')
            ax.grid(True, alpha=0.3)

    fig_rot.suptitle('Cubic Hermite Trajectory - Rotational Profiles')
    fig_rot.tight_layout()

    # =========================================================================
    # 3. 3D Path Plot & Pure Pursuit Simulation
    # =========================================================================
    fig3d = plt.figure(figsize=(7, 6))
    ax3d = fig3d.add_subplot(111, projection='3d')
    ax3d.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Planned cubic path')
    ax3d.scatter(*start.position, color='green', s=60, label='Start')
    ax3d.scatter(*target.position, color='red', s=60, label='Target')

    sim_planner = PurePursuitPlanner()
    sim_planner.generate_path(start, target, speed_mps=0.6, res_sec=0.05, use_target_vel=True)

    vehicle_pos = start.position.copy()
    vehicle_quat = start.orientation.copy()

    track_pos = [vehicle_pos.copy()]
    track_yaw = [np.degrees(quat_to_euler(vehicle_quat)[2])]

    dt = 0.1
    max_vehicle_speed = 0.4
    max_angular_speed = 0.5  # rad/s
    lookahead_dist = 1.0

    for _ in range(2000):
        current_sim_state = VehicleState(
            position=vehicle_pos,
            orientation=vehicle_quat,
            linear_velocity=np.zeros(3),
            angular_velocity=np.zeros(3)
        )

        setpoint = sim_planner.get_lookahead_setpoint(current_sim_state, lookahead_dist)
        if setpoint is None:
            break

        # --- Translational Update ---
        direction = setpoint.position - vehicle_pos
        dist = np.linalg.norm(direction)
        if dist > 1e-6:
            step = min(dist, max_vehicle_speed * dt)
            vehicle_pos = vehicle_pos + (direction / dist) * step

        # --- Rotational Update ---
        q_diff = quat_multiply(quat_inverse(vehicle_quat), setpoint.orientation)
        rot_vec = quat_to_rot_vec(q_diff)
        angle = np.linalg.norm(rot_vec)

        if angle > 1e-6:
            step_angle = min(angle, max_angular_speed * dt)
            delta_quat = rot_vec_to_quat((rot_vec / angle) * step_angle)
            vehicle_quat = quat_multiply(vehicle_quat, delta_quat)

            # Normalize to prevent floating point drift
            vehicle_quat = vehicle_quat / np.linalg.norm(vehicle_quat)

        track_pos.append(vehicle_pos.copy())
        track_yaw.append(np.degrees(quat_to_euler(vehicle_quat)[2]))

    track_pos = np.array(track_pos)
    ax3d.plot(track_pos[:, 0], track_pos[:, 1], track_pos[:, 2], '--', label='Simulated Pure Pursuit track')

    ax3d.set_xlabel('X [m]')
    ax3d.set_ylabel('Y [m]')
    ax3d.set_zlabel('Z [m] (negative = deeper)')
    ax3d.set_title('3D Cubic Hermite Path + Pure Pursuit Tracking')
    ax3d.legend()

    # =========================================================================
    # 4. Pure Pursuit Yaw Tracking Comparison
    # =========================================================================
    fig_yaw, ax_yaw = plt.subplots(figsize=(7, 4))

    sim_time = np.arange(len(track_yaw)) * dt

    # We can reuse the `eulers` array from above for the planned yaw
    ax_yaw.plot(t, eulers[:, 2], label='Planned Yaw', linewidth=2)
    ax_yaw.plot(sim_time, track_yaw, '--', label='Simulated Pure Pursuit Yaw', linewidth=2)
    ax_yaw.set_xlabel('Time [s]')
    ax_yaw.set_ylabel('Yaw [deg]')
    ax_yaw.set_title('Simulated Controller Heading (Yaw) Tracking')
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.legend()

    plt.show()


if __name__ == '__main__':
    main()