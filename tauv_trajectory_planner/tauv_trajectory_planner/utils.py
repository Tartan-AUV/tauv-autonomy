"""
This file contains helper functions for the trajectory generation.
"""

import math
import numpy as np

def _vector3_to_numpy(vec_msg) -> np.ndarray:
    return np.array([vec_msg.x, vec_msg.y, vec_msg.z], dtype=float)

def _quat_xyzw_to_numpy(quat_msg) -> np.ndarray:
    return np.array([quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w], dtype=float)

def _solve_quintic(boundary, duration: float) -> np.ndarray:
    """
    Solves for the 6 coefficients of a 1D quintic polynomial.
    Returns: [c0, c1, c2, c3, c4, c5] for the equation:
    p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
    """
    if duration <= 1e-6:
        return np.array([boundary.p0, boundary.v0, boundary.a0 / 2.0, 0.0, 0.0, 0.0])

    c0 = boundary.p0
    c1 = boundary.v0
    c2 = boundary.a0 / 2.0

    T = duration
    A = np.array([
        [T**3,     T**4,      T**5],
        [3 * T**2, 4 * T**3,  5 * T**4],
        [6 * T,    12 * T**2, 20 * T**3]
    ])
    
    b = np.array([
        boundary.p1 - (c0 + c1 * T + c2 * T**2),
        boundary.v1 - (c1 + 2 * c2 * T),
        boundary.a1 - 2 * c2
    ])
    
    try:
        x = np.linalg.solve(A, b)
        return np.array([c0, c1, c2, x[0], x[1], x[2]])
    except np.linalg.LinAlgError:
        return np.array([c0, c1, c2, 0.0, 0.0, 0.0])

def _eval_quintic(coeffs: np.ndarray, t: float) -> float:
    """Evaluates a quintic polynomial position at time t."""
    return coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5

def _eval_quintic_first_derivative(coeffs: np.ndarray, t: float) -> float:
    """Evaluates a quintic polynomial velocity (first derivative) at time t."""
    return coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2 + 4*coeffs[4]*t**3 + 5*coeffs[5]*t**4

def _quat_inverse(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]])

def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def _quat_to_rot_vec(q: np.ndarray) -> np.ndarray:
    """
    Converts a quaternion to a Rotation Vector (Axis-Angle) via taking the
    logarithm map: SO(3) --> so(3). 
    """
    if q[3] < 0: q = -q  # Quaternions double cover SO(3) so we choose a "hemisphere" 
        
    angle = 2 * math.acos(max(min(q[3], 1.0), -1.0))  # reverse w = cos(angle/2)
    s = math.sqrt(1.0 - q[3]*q[3])  # calculate sin(theta/2) since (x, y, z) = axis * sin(theta/2)
    
    if s < 1e-6:
        return np.zeros(3)
        
    axis = q[:3] / s
    return axis * angle

def _rot_vec_to_quat(v: np.ndarray) -> np.ndarray:
    """
    Converts a Rotation Vector back to a Quaternion via taking the expoenntial
    map: so(3) --> SO(3).
    """
    angle = np.linalg.norm(v)
    if angle < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
        
    axis = v / angle
    s = math.sin(angle / 2.0)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(angle / 2.0)])