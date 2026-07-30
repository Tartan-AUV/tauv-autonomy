"""
This is where all the trajectory magic happens. I basically separated out
all the ros2 stuff from here, so that test.py can run anywhere without the
full ecosystem. The actual ros2 entrypoint is ros2_interface.py.

Explanation:
We use a *cubic hermite* to create a path between two points. This lets us
specify a position and velocity at each endpoint. I originally considered
a quintic so that I could set acceleration too, but apparently it's overkill
and it also makes the path kinda warped. Our sub is overactuated so we can
separate each axis (x, y, z, roll, pitch, yaw). For each axis, we find a path
on t in [0, T]: p(t) = c0 + c1*t + c2*t^2 + c3*t^3. you can see that it has
4 coeffecients which is why we can specify a position and velocity at both
endpoints.
To avoid gimbal lock, we compute the rotational difference between the
start and target orientation as a quaternion, then map it into a 3D rotation
vector (SO(3) -> so(3), the log map ifykyk).
Once we have the path, we just run pure pursuit. The planner finds the
closest point on the path the vehicle, projects forward by some "lookahead
distance" and returns that point for the downstream controller to chase.

Path modes:
  generate_path()              -- original per-axis cubic Hermite in x/y/z.
  generate_straight_line_path() -- NEW: constrains translation to the direct
                                    line segment between start and target by
                                    solving a single cubic Hermite in scalar
                                    arc-length along that line, instead of
                                    three independent per-axis cubics. This
                                    avoids the case where a velocity component
                                    perpendicular to the direct path causes an
                                    axis (e.g. z) to bow arbitrarily far away
                                    from both endpoints before coming back.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np


def quat_inverse(q: np.ndarray) -> np.ndarray:
    """Inverse (= conjugate, for a unit quaternion) of a [x, y, z, w] quaternion"""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1 * q2, both in [x, y, z, w] order"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_to_rot_vec(q: np.ndarray) -> np.ndarray:
    """Log map SO(3) -> so(3): quaternion [x, y, z, w] -> 3D rotation vector"""
    if q[3] < 0:
        q = -q # pick the hemisphere with a non-negative scalar part

    angle = 2.0 * math.acos(max(min(q[3], 1.0), -1.0))
    s = math.sqrt(max(0.0, 1.0 - q[3] * q[3]))

    if s < 1e-6:
        return np.zeros(3)

    axis = q[:3] / s
    return axis * angle


def rot_vec_to_quat(v: np.ndarray) -> np.ndarray:
    """Exp map so(3) -> SO(3): 3D rotation vector -> quaternion [x, y, z, w]"""
    angle = np.linalg.norm(v)
    if angle < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])

    axis = v / angle
    s = math.sin(angle / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0)])


def quat_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """
    Quaternion [x, y, z, w] -> (roll, pitch, yaw) in radians, aerospace/ZYX
    convention. Used here only for small terminal-condition orientation
    errors, so gimbal lock near pitch = +/-90deg isn't a practical concern.
    """
    x, y, z, w = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


@dataclass(frozen=True)
class CubicBoundary:
    p0: float
    v0: float
    p1: float
    v1: float


def solve_cubic_hermite(boundary: CubicBoundary, duration: float) -> np.ndarray:
    """
    Closed-form solve for the 4 coefficients [c0, c1, c2, c3] of
    p(t) = c0 + c1*t + c2*t^2 + c3*t^3, t in [0, duration]
    matching p(0)=p0, p'(0)=v0, p(duration)=p1, p'(duration)=v1
    """
    T = max(duration, 1e-6) # woah safety

    c0 = boundary.p0
    c1 = boundary.v0

    dp = boundary.p1 - boundary.p0 - boundary.v0 * T
    dv = boundary.v1 - boundary.v0

    c2 = 3.0 * dp / T**2 - dv / T
    c3 = -2.0 * dp / T**3 + dv / T**2

    return np.array([c0, c1, c2, c3])


def eval_cubic(coeffs: np.ndarray, t: float) -> float:
    """Position at time t"""
    return coeffs[0] + coeffs[1] * t + coeffs[2] * t**2 + coeffs[3] * t**3


def eval_cubic_velocity(coeffs: np.ndarray, t: float) -> float:
    """First derivative (velocity) at time t"""
    return coeffs[1] + 2.0 * coeffs[2] * t + 3.0 * coeffs[3] * t**2


def eval_cubic_acceleration(coeffs: np.ndarray, t: float) -> float:
    """Second derivative (acceleration) at time t"""
    return 2.0 * coeffs[2] + 6.0 * coeffs[3] * t


@dataclass
class VehicleState:
    position: np.ndarray # [x, y, z]
    orientation: np.ndarray # [x, y, z, w]
    linear_velocity: np.ndarray # [x, y, z]
    angular_velocity: np.ndarray # [x, y, z]
    # Not used as a boundary condition by the cubic solver -- kept around so
    # this dataclass can still carry a feed-forward acceleration if a
    # downstream controller wants one (e.g. read off the generated path).
    linear_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))


class PurePursuitPlanner:
    """Generates a dense geometric path and tracks it via Pure Pursuit"""

    def __init__(self):
        self._internal_path: List[VehicleState] = []
        self._last_duration: float = 0.0
        self._last_res: float = 0.0

    def has_active_path(self) -> bool:
        return len(self._internal_path) > 0

    def clear_path(self) -> None:
        self._internal_path.clear()

    @property
    def path(self) -> List[VehicleState]:
        return list(self._internal_path)

    @property
    def duration(self) -> float:
        """Duration (s) of the most recently generated path"""
        return self._last_duration

    @property
    def resolution(self) -> float:
        """Sample spacing (s) of the most recently generated path"""
        return self._last_res

    def _compute_rotational_coeffs(self, current_state: VehicleState, target_state: VehicleState,
                                   duration: float, use_target_vel: bool) -> np.ndarray:
        """
        Shared by both path modes: cubic Hermite in the rotation-vector (Lie
        algebra) representation of the start->target orientation delta.
        """
        q_start = current_state.orientation
        q_target = target_state.orientation

        q_diff = quat_multiply(quat_inverse(q_start), q_target)
        r0 = np.zeros(3)
        r1 = quat_to_rot_vec(q_diff)

        # FORCE INITIAL AND FINAL ANGULAR VELOCITIES TO ZERO
        # This prevents the spline from "bowing" or overshooting if the sub
        # currently has angular momentum in the wrong direction.
        omega0 = np.zeros(3)
        omega1 = np.zeros(3) # Optionally keep use_target_vel logic here if you want it to exit with a specific spin

        angular_coeffs = np.zeros((3, 4))
        for axis in range(3):
            bound = CubicBoundary(r0[axis], omega0[axis], r1[axis], omega1[axis])
            angular_coeffs[axis, :] = solve_cubic_hermite(bound, duration)

        return angular_coeffs

    def generate_path(self, current_state: VehicleState, target_state: VehicleState,
    speed_mps: float, res_sec: float, use_target_vel: bool) -> None:
        """Calculates the dense geometric cubic Hermite path (independent per-axis) and stores it internally"""

        speed = max(speed_mps, 0.01)
        res = max(res_sec, 0.01)

        # 1. Determine maneuver duration from the translational distance and cruise speed.
        distance = np.linalg.norm(target_state.position - current_state.position)
        duration = max(1.0, distance / speed)
        num_samples = int(math.ceil(duration / res)) + 1

        # --- 2. Translational cubics (X, Y, Z) ---
        p0 = current_state.position
        p1 = target_state.position
        v0 = current_state.linear_velocity
        v1 = target_state.linear_velocity if use_target_vel else np.zeros(3)

        linear_coeffs = np.zeros((3, 4))
        for axis in range(3):
            bound = CubicBoundary(p0[axis], v0[axis], p1[axis], v1[axis])
            linear_coeffs[axis, :] = solve_cubic_hermite(bound, duration)

        # --- 3. Rotational cubics (rotation vector / Lie algebra) ---
        angular_coeffs = self._compute_rotational_coeffs(current_state, target_state, duration, use_target_vel)
        q_start = current_state.orientation

        # --- 4. Generate the dense path ---
        new_path = []
        for i in range(num_samples):
            t = min(i * res, duration)

            pos = np.array([eval_cubic(linear_coeffs[axis], t) for axis in range(3)])
            lin_vel = np.array([eval_cubic_velocity(linear_coeffs[axis], t) for axis in range(3)])
            lin_acc = np.array([eval_cubic_acceleration(linear_coeffs[axis], t) for axis in range(3)])

            rot_vec_t = np.array([eval_cubic(angular_coeffs[axis], t) for axis in range(3)])
            ang_vel = np.array([eval_cubic_velocity(angular_coeffs[axis], t) for axis in range(3)])
            ang_acc = np.array([eval_cubic_acceleration(angular_coeffs[axis], t) for axis in range(3)])

            q_local_t = rot_vec_to_quat(rot_vec_t)
            quat = quat_multiply(q_start, q_local_t)

            new_path.append(VehicleState(pos, quat, lin_vel, ang_vel, lin_acc, ang_acc))

        self._internal_path = new_path
        self._last_duration = duration
        self._last_res = res

    def generate_straight_line_path(self, current_state: VehicleState, target_state: VehicleState,
    speed_mps: float, res_sec: float, use_target_vel: bool) -> None:
        """
        Like generate_path(), but constrains translation to the straight-line
        segment between current_state.position and target_state.position.

        We solve a single cubic Hermite for scalar arc-length s(t) along the
        line, s in [0, distance]. Position at time t is always exactly
        current_state.position + s(t) * direction, which by construction
        can never leave the line in the perpendicular sense (this is what was
        happening before -- a start/target velocity with a component off the
        direct path could bow the per-axis cubic arbitrarily far away from
        both endpoints, e.g. the z-overshoot case we diagnosed).

        Note: s(t) is itself still a cubic Hermite, so if the *along-line*
        component of a boundary velocity is large relative to distance/duration,
        s(t) can still transiently dip below 0 or above `distance` before
        settling -- i.e. the vehicle could briefly pass the target and come
        back. That's a much more benign failure mode than leaving the line
        entirely. If you want strictly monotonic travel with no along-line
        overshoot either, this can be swapped for a clamped or smoothstep-based
        profile instead -- just ask.

        Rotation is handled identically to generate_path().
        """
        speed = max(speed_mps, 0.01)
        res = max(res_sec, 0.01)

        p0 = current_state.position
        p1 = target_state.position
        displacement = p1 - p0
        distance = float(np.linalg.norm(displacement))
        duration = max(1.0, distance / speed)
        num_samples = int(math.ceil(duration / res)) + 1

        direction = displacement / distance if distance > 1e-9 else np.zeros(3)

        # Project boundary linear velocities onto the line direction -- this
        # lets speed continue smoothly into/out of the segment without
        # introducing any perpendicular (off-line) motion.
        v0_along = float(np.dot(current_state.linear_velocity, direction))
        v1_along = float(np.dot(target_state.linear_velocity, direction)) if use_target_vel else 0.0

        arc_bound = CubicBoundary(0.0, v0_along, distance, v1_along)
        arc_coeffs = solve_cubic_hermite(arc_bound, duration)

        # --- Rotational cubics (rotation vector / Lie algebra), same as generate_path() ---
        angular_coeffs = self._compute_rotational_coeffs(current_state, target_state, duration, use_target_vel)
        q_start = current_state.orientation

        new_path = []
        for i in range(num_samples):
            t = min(i * res, duration)

            s = eval_cubic(arc_coeffs, t)
            s_dot = eval_cubic_velocity(arc_coeffs, t)
            s_ddot = eval_cubic_acceleration(arc_coeffs, t)

            pos = p0 + direction * s
            lin_vel = direction * s_dot
            lin_acc = direction * s_ddot

            rot_vec_t = np.array([eval_cubic(angular_coeffs[axis], t) for axis in range(3)])
            ang_vel = np.array([eval_cubic_velocity(angular_coeffs[axis], t) for axis in range(3)])
            ang_acc = np.array([eval_cubic_acceleration(angular_coeffs[axis], t) for axis in range(3)])

            q_local_t = rot_vec_to_quat(rot_vec_t)
            quat = quat_multiply(q_start, q_local_t)

            new_path.append(VehicleState(pos, quat, lin_vel, ang_vel, lin_acc, ang_acc))

        self._internal_path = new_path
        self._last_duration = duration
        self._last_res = res

    def get_lookahead_setpoint(self, current_state: VehicleState, lookahead_dist: float,
    xy_tol: float = 0.1, z_tol: float = 0.1,
    yaw_tol: float = math.radians(5.0),
    roll_pitch_tol: float = math.radians(5.0)) -> Optional[VehicleState]:
        """
        Runs Pure Pursuit: finds the closest path point, then projects forward
        by lookahead_dist. Once the projection reaches the end of the path,
        the maneuver is considered complete only once the vehicle is within
        all of: xy_tol (horizontal position, m), z_tol (vertical position, m),
        yaw_tol (rad), and roll_pitch_tol (rad, applied to both roll and pitch)
        of the target state.
        """
        if not self.has_active_path():
            return None

        current_pos = current_state.position

        # Find the closest point on the path (minimize cross-track error)
        min_dist = float('inf')
        closest_idx = 0
        for i, state in enumerate(self._internal_path):
            dist = np.linalg.norm(state.position - current_pos)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Project along the path by lookahead_dist
        target_idx = closest_idx
        accumulated_dist = 0.0
        while target_idx < len(self._internal_path) - 1 and accumulated_dist < lookahead_dist:
            step_dist = np.linalg.norm(
                self._internal_path[target_idx + 1].position - self._internal_path[target_idx].position
            )
            accumulated_dist += step_dist
            target_idx += 1

        # Check for terminal condition (reached the end of the path)
        if target_idx == len(self._internal_path) - 1:
            final_state = self._internal_path[-1]

            pos_err = final_state.position - current_pos
            xy_err = float(np.linalg.norm(pos_err[:2]))
            z_err = float(abs(pos_err[2]))

            q_err = quat_multiply(quat_inverse(current_state.orientation), final_state.orientation)
            roll_err, pitch_err, yaw_err = quat_to_euler(q_err)

            if (xy_err < xy_tol and z_err < z_tol
            and abs(yaw_err) < yaw_tol
            and abs(roll_err) < roll_pitch_tol
            and abs(pitch_err) < roll_pitch_tol):
                self.clear_path()
                return None

        lookahead_state = self._internal_path[target_idx]
        closest_state = self._internal_path[closest_idx]

        return VehicleState(
            position=lookahead_state.position,
            orientation=lookahead_state.orientation,
            linear_velocity=closest_state.linear_velocity,
            angular_velocity=closest_state.angular_velocity,
            linear_acceleration=closest_state.linear_acceleration,
            angular_acceleration=closest_state.angular_acceleration
        )