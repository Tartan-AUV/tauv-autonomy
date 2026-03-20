import numpy as np
import yaml
from pathlib import Path

'''
Thruster Allocation Matrix (TAM) for the Osprey AUV.

Motor ordering matches ThrusterSetpoint.msg (ground truth):
  idx 0 = BLH  (Back Left Horizontal)
  idx 1 = FLU  (Front Left Upward)
  idx 2 = BLU  (Back Left Upward)
  idx 3 = FLH  (Front Left Horizontal)
  idx 4 = BRH  (Back Right Horizontal)
  idx 5 = FRU  (Front Right Upward)
  idx 6 = BRU  (Back Right Upward)
  idx 7 = FRH  (Front Right Horizontal)

The TAM maps a desired wrench [Fx, Fy, Fz, tx, ty, tz] in body frame
to per-thruster forces. Body frame is ENU/FLU (x=forward, y=left, z=up).

Stonefish applies thrust along each thruster's local +X axis. Handedness
is handled entirely in force_to_gain.py (negates RPM for left-handed props),
so the TAM is handedness-agnostic.
'''


def _rpy_deg_to_rotation(roll_deg, pitch_deg, yaw_deg):
    '''Return 3x3 rotation matrix from intrinsic RPY angles (degrees).
    Applied as R = Rz(yaw) * Ry(pitch) * Rx(roll)
    '''
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r),  np.cos(r)]])
    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [0,          1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0,          0,          1]])
    return Rz @ Ry @ Rx


def load_thruster_config(yaml_path=None):
    '''Load thruster config from thruster_params.yaml.
    Returns the ros__parameters dict.
    '''
    if yaml_path is None:
        yaml_path = Path(__file__).parent / 'config' / 'thruster_params.yaml'
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['thruster_controller']['ros__parameters']


def get_complete_tam(cfg, flip_signs):
    '''Build the 6x8 Thruster Allocation Matrix from config.

    Args:
        cfg: dict from thruster_params.yaml ros__parameters
        flip_signs: list of 8 values (+1 or -1), one per thruster.
                    Use cfg['flip_signs_sim'] or cfg['flip_signs_real'].

    Returns:
        A: 6x8 numpy array. Column i = [force_dir; torque] for thruster i.
           Wrench convention: [Fx, Fy, Fz, tx, ty, tz] in body frame (ENU/FLU).
    '''
    # cad_R_body maps body->cad; body_R_cad = its inverse = transpose (orthogonal)
    cad_R_body = np.array(cfg['cad_R_body']).reshape(3, 3)
    body_R_cad = cad_R_body.T

    # CoM in body frame — moment arms must be relative to CoM, not base_link origin
    com_body = body_R_cad @ np.array(cfg['com_cad'])

    positions_cad = [np.array(p) for p in cfg['positions_cad']]
    orientations_rpy = cfg['orientations_rpy_deg']
    right_handed = cfg['right_handed']

    A = np.zeros((6, 8))

    for i in range(8):
        # Moment arm from CoM to thruster, in body frame
        r = body_R_cad @ positions_cad[i] - com_body

        # Stonefish applies thrust along the thruster's local +X axis.
        # thrust_dir_cad = R_cad_thruster @ [1, 0, 0]
        rpy = orientations_rpy[i]
        R_cad_thruster = _rpy_deg_to_rotation(rpy[0], rpy[1], rpy[2])
        thrust_dir_cad = R_cad_thruster @ np.array([1.0, 0.0, 0.0])

        # Convert thrust direction to body frame
        thrust_dir_body = body_R_cad @ thrust_dir_cad

        # Force contribution (top 3 rows)
        A[0:3, i] = thrust_dir_body * flip_signs[i]
        # Torque contribution: r x d (bottom 3 rows)
        A[3:6, i] = np.cross(r, thrust_dir_body) * flip_signs[i]

    return A


def get_pseudoinverse(cfg, flip_signs):
    '''Return the Moore-Penrose pseudoinverse of the TAM (8x6 matrix).

    Args:
        cfg: dict from thruster_params.yaml ros__parameters
        flip_signs: list of 8 values (+1 or -1) — use cfg['flip_signs_sim'] or cfg['flip_signs_real']

    Returns:
        A_plus: 8x6 numpy array (pseudoinverse of the 6x8 TAM)
    '''
    return np.linalg.pinv(get_complete_tam(cfg, flip_signs))


def solve_thrusts(wrench, A_plus):
    '''Compute per-thruster forces for a desired wrench using a pre-computed pseudoinverse.

    Args:
        wrench: array-like [Fx, Fy, Fz, tx, ty, tz] in body frame (ENU/FLU).
        A_plus: 8x6 pseudoinverse of the TAM, from get_pseudoinverse()

    Returns:
        forces: numpy array of 8 thruster forces [N]. Ordered by ThrusterSetpoint.msg index.
    '''
    return np.dot(A_plus, wrench)
