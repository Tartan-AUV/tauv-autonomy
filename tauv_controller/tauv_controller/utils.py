import math
import numpy as np

def wrap_angle(angle):
    """Keeps an angle bound between -pi and pi radians."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def quat_to_euler(q):
    """Converts a geometry_msgs Quaternion to Roll, Pitch, Yaw."""
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp) # Use 90 degrees if out of bounds
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quat_to_rot_matrix(q):
    """Converts a quaternion to a 3x3 rotation matrix (Body to World)"""
    return np.array([
        [1 - 2 * (q.y**2  + q.z**2 ),     2 * (q.x*q.y - q.w*q.z),     2 * (q.x*q.z + q.w*q.y)],
        [    2 * (q.x*q.y + q.w*q.z), 1 - 2 * (q.x**2  + q.z**2 ),     2 * (q.y*q.z - q.w*q.x)],
        [    2 * (q.x*q.z - q.w*q.y),     2 * (q.y*q.z + q.w*q.x), 1 - 2 * (q.x**2  + q.y**2 )]
    ])