"""
This file contains the "core" of the trajectory planner, essentially handling 
all of the math behind generating trajectories.

--- TOP-DOWN MATHEMATICAL EXPLANATION ---

1. The Translational Math (Position, Velocity, Acceleration)
The goal is to find a path p(t) that fulfills 6 constraints: a starting position, 
velocity, and acceleration (p0, v0, a0), and an ending position, velocity, and 
acceleration (p1, v1, a1). Because 3D space is orthogonal, we calculate a separate 
quintic equation for the X axis, the Y axis, and the Z axis.

2. The Rotational Math (Lie Algebra / Rotation Vectors)
To prevent Gimbal Lock in 6-DOF maneuvers, we calculate the rotational difference 
between the start and target orientations. We map this difference from a Quaternion 
into a 3D Rotation Vector. Because it is a 3D vector, we can feed it directly into 
the same Quintic Matrix Solver used for translation, allowing us to perfectly 
match boundary angular velocities and accelerations. 

3. Path Tracking (Pure Pursuit)
Once the geometric path is generated, the planner acts as a "Virtual Donkey and 
Carrot". It finds the closest point on the curve to the vehicle (Cross-Track Error) 
and projects forward a set distance (Lookahead Distance) to generate a single 
target setpoint for a downstream cascade PID controller to chase.
"""

import math
from dataclasses import dataclass
from typing import List, Optional
import numpy as np

from utils import (
    _solve_quintic, _eval_quintic, _eval_quintic_first_derivative,
    _quat_inverse, _quat_multiply, _quat_to_rot_vec, _rot_vec_to_quat
)

@dataclass(frozen=True)
class QuinticBoundary:
    p0: float; v0: float; a0: float
    p1: float; v1: float; a1: float

@dataclass
class VehicleState:
    position: np.ndarray             # [x, y, z]
    orientation: np.ndarray          # [x, y, z, w]
    linear_velocity: np.ndarray      # [x, y, z]
    angular_velocity: np.ndarray     # [x, y, z]
    linear_acceleration: np.ndarray  # [x, y, z]
    angular_acceleration: np.ndarray # [x, y, z]


class PurePursuitPlanner:
    """Generates a dense geometric path and tracks it using a lookahead point."""
    
    def __init__(self):
        self._internal_path: List[VehicleState] = []

    def has_active_path(self) -> bool:
        return len(self._internal_path) > 0

    def clear_path(self) -> None:
        self._internal_path.clear()

    def generate_path(self, current_state: VehicleState, target_state: VehicleState, 
                      speed_mps: float, res_sec: float, use_target_vel: bool) -> None:
        """Calculates the dense geometric quintic path and stores it internally."""
        
        speed = max(speed_mps, 0.01)
        res = max(res_sec, 0.01)

        # 1. Determine Maneuver Duration
        distance = np.linalg.norm(target_state.position - current_state.position)
        duration = max(1.0, distance / speed)
        num_samples = int(math.ceil(duration / res)) + 1

        # --- 2. TRANSLATIONAL QUINTICS (XYZ) ---
        p0 = current_state.position
        p1 = target_state.position
        v0 = current_state.linear_velocity
        v1 = target_state.linear_velocity if use_target_vel else np.zeros(3)
        a0 = current_state.linear_acceleration
        a1 = target_state.linear_acceleration if use_target_vel else np.zeros(3)

        linear_coeffs = np.zeros((3, 6))
        for axis in range(3):
            bound = QuinticBoundary(p0[axis], v0[axis], a0[axis], p1[axis], v1[axis], a1[axis])
            linear_coeffs[axis, :] = _solve_quintic(bound, duration)

        # --- 3. ROTATIONAL QUINTICS (Rotation Vectors / Lie Algebra) ---
        q_start = current_state.orientation
        q_target = target_state.orientation
        
        # Calculate the rotational difference to target
        q_start_inv = _quat_inverse(q_start)
        q_diff = _quat_multiply(q_start_inv, q_target)
        
        # Convert that difference into a 3D Rotation Vector target
        r0 = np.zeros(3) 
        r1 = _quat_to_rot_vec(q_diff)

        omega0 = current_state.angular_velocity
        omega1 = target_state.angular_velocity if use_target_vel else np.zeros(3)
        alpha0 = current_state.angular_acceleration
        alpha1 = target_state.angular_acceleration if use_target_vel else np.zeros(3)

        angular_coeffs = np.zeros((3, 6))
        for axis in range(3):
            bound = QuinticBoundary(r0[axis], omega0[axis], alpha0[axis], r1[axis], omega1[axis], alpha1[axis])
            angular_coeffs[axis, :] = _solve_quintic(bound, duration)

        # --- 4. GENERATE DENSE PATH ---
        new_path = []
        for i in range(num_samples):
            t = min(i * res, duration)
            
            # Evaluate Position, Linear Velocity, Linear Acceleration
            pos = np.array([_eval_quintic(linear_coeffs[axis, :], t) for axis in range(3)])
            lin_vel = np.array([_eval_quintic_first_derivative(linear_coeffs[axis, :], t) for axis in range(3)])
            lin_acc = np.array([_eval_quintic_first_derivative(linear_coeffs[axis, :], t) for axis in range(3)]) 
            
            # Evaluate Rotation Vector and Angular Velocity
            rot_vec_t = np.array([_eval_quintic(angular_coeffs[axis, :], t) for axis in range(3)])
            ang_vel = np.array([_eval_quintic_first_derivative(angular_coeffs[axis, :], t) for axis in range(3)])
            ang_acc = np.zeros(3) 

            # Convert intermediate rotation vector to local quaternion, then multiply by absolute start
            q_local_t = _rot_vec_to_quat(rot_vec_t)
            quat = _quat_multiply(q_start, q_local_t)

            new_path.append(VehicleState(pos, quat, lin_vel, ang_vel, lin_acc, ang_acc))

        self._internal_path = new_path

    def get_lookahead_setpoint(self, current_pos: np.ndarray, lookahead_dist: float) -> Optional[VehicleState]:
        """Executes Pure Pursuit logic to find the next target setpoint on the curve."""
        if not self.has_active_path():
            return None

        # 1. Find Closest Point (Minimize Cross-Track Error)
        min_dist = float('inf')
        closest_idx = 0
        
        for i, state in enumerate(self._internal_path):
            dist = np.linalg.norm(state.position - current_pos)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # 2. Apply the Lookahead Distance
        target_idx = closest_idx
        accumulated_dist = 0.0
        
        while target_idx < len(self._internal_path) - 1 and accumulated_dist < lookahead_dist:
            step_dist = np.linalg.norm(self._internal_path[target_idx + 1].position - self._internal_path[target_idx].position)
            accumulated_dist += step_dist
            target_idx += 1

        # 3. Check Terminal Condition
        if target_idx == len(self._internal_path) - 1 and min_dist < 0.1:
            self.clear_path()
            return None

        return self._internal_path[target_idx]