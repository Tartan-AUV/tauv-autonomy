import numpy as np
from enum import Enum

'''
Takes a net force/torque wrench in R^6 and breaks it into 8 motor forces.

Body frame: FLU (x = forward, y = left, z = up)
Wrench: [Fx, Fy, Fz, τ_roll, τ_pitch, τ_yaw]

Motors on sub (top-down view, front = up):

\\ ----------- //              Front diagonal motors: thrust has fwd + inward component
  |           |
 O|           |O
  |           |                Middle vertical motors: thrust upward (+Z)
 O|           |O
  |           |
// ----------- \\              Back diagonal motors: thrust has fwd + outward component

                               All motors reversible

Positive force command → forward component (horizontal) or upward (vertical).

Motor positions and orientations derived from Osprey YAML config:
  - positions_cad transformed to body frame via body_R_cad
  - positions are relative to centre of mass (COM)
  - thrust directions from RPY orientations (thrust = local +X of thruster frame)
  - FLH and FRH directions flipped so positive = forward for ALL horizontal thrusters

Motor ordering (matches ThrusterSetpoint.msg):
  FRH=0, FLH=1, BRH=2, BLH=3, FRV=4, FLV=5, BRV=6, BLV=7
'''

class Directions(Enum):
    FORWARD  = 0
    REVERSED = 1

class Motors(Enum):
    FRH = 0  # 1 → index 0
    FLH = 1  # 2 → index 1
    BRH = 2  # 3 → index 2
    BLH = 3  # 4 → index 3
    FRV = 4  # 5 → index 4
    FLV = 5  # 6 → index 5
    BRV = 6  # 7 → index 6
    BLV = 7  # 8 → index 7


# ---------------------------------------------------------------------------
# Real geometry from Osprey YAML config
# ---------------------------------------------------------------------------
# Positions in body frame (FLU) relative to COM, derived from:
#   positions_cad, com_cad = [0.005, -0.002, 0.059]
#   body_R_cad (x_fwd=cad_y, y_left=-cad_x, z_up=cad_z)
#
# Thrust directions in body frame (FLU):
#   Derived from orientations_rpy_deg, thrust axis = local +X of thruster frame,
#   FLH and FRH columns negated so positive command = forward for all horizontals.
#
# Horizontal thrusters at ~45° mounting angle:
#   FRH: fwd + left     FLH: fwd + right
#   BRH: fwd + right    BLH: fwd + left
#   (front thrusters splay inward, back thrusters splay outward → X-frame)
# ---------------------------------------------------------------------------

config = {
    "motors": [
        Motors.FRH,  # 0
        Motors.FLH,  # 1
        Motors.BRH,  # 2
        Motors.BLH,  # 3
        Motors.FRV,  # 4
        Motors.FLV,  # 5
        Motors.BRV,  # 6
        Motors.BLV,  # 7
    ],
    "angle": 45,
    "positions": {
        "L"  : 0.25,
        "HL" : 0.1,
        "W"  : 0.2,
        "H"  : 0.1,
    }
}


def get_complete_tam(config):
    """
    Builds the 6x8 TAM (A) in the body frame (FLU).

    Each column i is: [force_vector(3); torque_vector(3)] for motor i.
    Torque = position × force_direction (cross product).

    Positions are the real Osprey CAD positions transformed to body frame
    relative to the centre of mass.  Thrust directions use the YAML RPY
    orientations with per-thruster flips so positive = forward for all
    horizontal thrusters.
    """
    A = np.zeros((6, 8))

    t = np.radians(config["angle"])
    s, c = np.sin(t), np.cos(t)

    # -----------------------------------------------------------------
    # Builder: position (body FLU, relative to COM) and direction
    #
    # Positions from YAML (metres):
    #   Computed as body_R_cad @ (position_cad - com_cad)
    #
    # Directions from YAML RPY orientations (thrust = local +X):
    #   FLH, FRH flipped so positive command = forward (+Fx) for all.
    #   Result:  FRH/BLH → [+s, +c, 0] (fwd+left)
    #            FLH/BRH → [+s, -c, 0] (fwd+right)
    # -----------------------------------------------------------------
    motors_setup = []
    builder = {
        # Vertical thrusters (V in this file = U "upward" in YAML)
        #                    Position [x_fwd, y_left, z_up]        Direction
        Motors.FLV : ([ +0.0962, +0.2509, -0.0803],                [0, 0, 1]),
        Motors.FRV : ([ +0.0962, -0.2409, -0.0803],                [0, 0, 1]),
        Motors.BLV : ([ -0.0922, +0.2509, -0.0803],                [0, 0, 1]),
        Motors.BRV : ([ -0.0922, -0.2409, -0.0803],                [0, 0, 1]),

        # Horizontal thrusters (diagonal X-frame)
        Motors.FRH : ([ +0.2285, -0.2204, +0.0199],                [ s,  c, 0]),  # fwd + left
        Motors.FLH : ([ +0.2416, +0.2173, +0.0199],                [ s, -c, 0]),  # fwd + right
        Motors.BRH : ([ -0.2242, -0.2207, +0.0199],                [ s, -c, 0]),  # fwd + right
        Motors.BLH : ([ -0.2242, +0.2307, +0.0199],                [ s,  c, 0]),  # fwd + left
    }

    for motor in config["motors"]:
        motors_setup.append(builder[motor])

    for i, (pos, direct) in enumerate(motors_setup):
        r = np.array(pos)
        d = np.array(direct)

        # Force part (top 3 rows)
        A[0:3, i] = d
        # Torque part: r x d (bottom 3 rows)
        A[3:6, i] = np.cross(r, d)

    return A


def solve_thrusts(wrench, config):
    """
    Allocate a body-frame wrench [Fx, Fy, Fz, τ_roll, τ_pitch, τ_yaw]
    to 8 thrusters using the Moore-Penrose pseudo-inverse (minimum L2 norm).
    """
    A = get_complete_tam(config)

    A_plus = np.linalg.pinv(A)
    v = A_plus @ wrench
    return v


def solve_thrusts_clamped(wrench, config, max_force=None):
    """
    Same as solve_thrusts but with optional per-thruster saturation.
    If any thruster exceeds max_force, the entire solution is scaled
    down to preserve wrench direction.
    """
    A = get_complete_tam(config)
    A_plus = np.linalg.pinv(A)
    v = A_plus @ wrench

    if max_force is not None and max_force > 0:
        peak = np.max(np.abs(v))
        if peak > max_force:
            v *= max_force / peak

    return v


# --- Example Usage / Verification ---
if __name__ == "__main__":
    A = get_complete_tam(config)

    motor_order = [m.name for m in config["motors"]]
    dof_labels  = ["Fx", "Fy", "Fz", "τ_roll", "τ_pitch", "τ_yaw"]

    print("=" * 78)
    print("Thrust Allocation Matrix (body FLU frame)")
    print("  Positive command → forward (horiz) or up (vert)")
    print("=" * 78)
    header = "           " + "  ".join(f"{n:>8s}" for n in motor_order)
    print(header)
    for i, label in enumerate(dof_labels):
        row = f"  {label:>8s}  " + "  ".join(f"{A[i,j]:+8.4f}" for j in range(8))
        print(row)
    print(f"\n  Rank: {np.linalg.matrix_rank(A)} / 6")

    # --- Test wrenches ---
    tests = {
        "Pure surge  (10 N fwd)":       np.array([10,  0,  0,  0,  0,  0]),
        "Pure sway   (5 N left)":       np.array([ 0,  5,  0,  0,  0,  0]),
        "Pure heave  (10 N up)":        np.array([ 0,  0, 10,  0,  0,  0]),
        "Pure yaw    (2 N·m CCW)":      np.array([ 0,  0,  0,  0,  0,  2]),
        "Pure pitch  (2 N·m nose-up)":  np.array([ 0,  0,  0,  0,  2,  0]),
        "Pure roll   (2 N·m right)":    np.array([ 0,  0,  0,  2,  0,  0]),
        "Surge + yaw (10N, 2N·m)":      np.array([10,  0,  0,  0,  0,  2]),
    }

    for desc, wrench in tests.items():
        forces = solve_thrusts(wrench, config)
        achieved = A @ forces
        error = np.linalg.norm(wrench - achieved)

        print(f"\n{'─' * 78}")
        print(f"TEST: {desc}")
        print(f"  Wrench:   {wrench}")
        print(f"  Forces:   {np.round(forces, 4)}")
        print(f"  Achieved: {np.round(achieved, 4)}")
        print(f"  Error:    {error:.6f}")
