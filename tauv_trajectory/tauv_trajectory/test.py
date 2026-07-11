"""
Standalone script (no ROS 2 required) for testing core.py:
builds a synthetic start/target VehicleState pair, generates a cubic
Hermite path, and plots position/velocity/acceleration profiles, the 3D
path, and a simulated Pure Pursuit run chasing it.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)

from core import PurePursuitPlanner, VehicleState


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
    start = make_state(position=[0.0, 0.0, -5.0], yaw_deg=0.0,
                        linear_velocity=[0.3, 0.0, 0.0], angular_velocity=[0.0, 0.0, 0.0])
    target = make_state(position=[8.0, 4.0, -8.0], yaw_deg=90.0,
                         linear_velocity=[0.2, 0.2, 0.0], angular_velocity=[0.0, 0.0, 0.1])

    planner = PurePursuitPlanner()
    planner.generate_path(start, target, speed_mps=0.6, res_sec=0.05, use_target_vel=True)

    t = path_times(planner)
    positions = path_field(planner, 'position')
    velocities = path_field(planner, 'linear_velocity')
    accelerations = path_field(planner, 'linear_acceleration')

    # --- Per-axis position / velocity / acceleration profiles ---
    labels = ['X', 'Y', 'Z']
    fig, axes = plt.subplots(3, 3, figsize=(14, 9))
    for axis in range(3):
        axes[0, axis].plot(t, positions[:, axis])
        axes[0, axis].set_title(f'Position {labels[axis]} [m]')

        axes[1, axis].plot(t, velocities[:, axis])
        axes[1, axis].set_title(f'Velocity {labels[axis]} [m/s]')

        axes[2, axis].plot(t, accelerations[:, axis])
        axes[2, axis].set_title(f'Acceleration {labels[axis]} [m/s^2]')

    for row in axes:
        for ax in row:
            ax.set_xlabel('t [s]')
            ax.grid(True, alpha=0.3)

    fig.suptitle('Cubic Hermite Trajectory - Per-Axis Profiles (position + velocity matched at endpoints)')
    fig.tight_layout()

    # --- 3D path plot with a simulated Pure Pursuit run ---
    fig3d = plt.figure(figsize=(7, 6))
    ax3d = fig3d.add_subplot(111, projection='3d')
    ax3d.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Planned cubic path')
    ax3d.scatter(*start.position, color='green', s=60, label='Start')
    ax3d.scatter(*target.position, color='red', s=60, label='Target')

    sim_planner = PurePursuitPlanner()
    sim_planner.generate_path(start, target, speed_mps=0.6, res_sec=0.05, use_target_vel=True)

    vehicle_pos = start.position.copy()
    track = [vehicle_pos.copy()]
    dt = 0.1
    max_vehicle_speed = 0.6
    lookahead_dist = 1.0

    for _ in range(2000):
        setpoint = sim_planner.get_lookahead_setpoint(vehicle_pos, lookahead_dist)
        if setpoint is None:
            break
        direction = setpoint.position - vehicle_pos
        dist = np.linalg.norm(direction)
        if dist > 1e-6:
            step = min(dist, max_vehicle_speed * dt)
            vehicle_pos = vehicle_pos + direction / dist * step
        track.append(vehicle_pos.copy())

    track = np.array(track)
    ax3d.plot(track[:, 0], track[:, 1], track[:, 2], '--', label='Simulated Pure Pursuit track')

    ax3d.set_xlabel('X [m]')
    ax3d.set_ylabel('Y [m]')
    ax3d.set_zlabel('Z [m] (negative = deeper)')
    ax3d.set_title('3D Cubic Hermite Path + Pure Pursuit Tracking')
    ax3d.legend()

    plt.show()


if __name__ == '__main__':
    main()
