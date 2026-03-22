import numpy as np
from enum import Enum

'''
Takes a net force vector in R^3 and breaks it into motor componenets

Motors are as such on sub

\\ ----------- //              Front diagonally mounted motors propel water backwards (forwards thrust)
  |           |
 O|           |O
  |           |                Middle set of motors propel water down to generate upwards thrust
 O|           |O
  |           |
// ----------- \\              Back diagonally mounted motors propel water forwards (backwards thrust)

                               All motors reversible

We will implement two functions:

    linearToForce( x, y, z , config)         => Array of 8 motor force outputs
    torqueToForce( rol, pit, yaw, config )   => Array of 8 motor force outputs

    Note that config includes motor directions and orientations in the following structure
        config = {
            motors : [inorder array of motors],
            angle : mounting angle of the corner motors (cannot be 0 or 90, casues singlualr matrix loses a degree of freedom)
            positions: Dict of possitions relative to COM
        }
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
         "L" : 0.25,
        "HL" : 0.1,
         "W" : 0.2,
         "H" : 0.1,
    }
}


def get_complete_tam(config):
    """
    Builds the 6x8 TAM (A).
    Each column i is: [force_vector; torque_vector] for motor i.
    """
    # A is 6 rows (DOF) by 8 columns (Motors)
    A = np.zeros((6, 8))

    # We'll use your 45-degree logic for directions
    t = np.radians(config["angle"])
    s, c = np.sin(t), np.cos(t)

    # Positions relative to Center of Mass (x, y, z) in meters
    # Assume L, W, H are half-dimensions
    L = config["positions"]["L"]
    HL = config["positions"]["HL"]
    W = config["positions"]["W"]
    H = config["positions"]["H"]

    # Define each motor's [Position (r), Direction (d)]
    # Vertical motors point in +Z. Horizontal point diagonally.
    # Order: BLV, FLV, FRV, BRV, FLH, FRH, BLH, BRH
    #
    # SAMPLE
    # motors_setup = [
    #     # Pos (x, y, z)          Direction (dx, dy, dz)
    #     ([-HL, -W, -H],           [0, 0, 1]),  # BLV
    #     ([ HL, -W, -H],           [0, 0, 1]),  # FLV
    #     ([ HL,  W, -H],           [0, 0, 1]),  # FRV
    #     ([-HL,  W, -H],           [0, 0, 1]),  # BRV
    #     ([  L, -W,  0],           [s, c, 0]),  # FLH
    #     ([  L,  W,  0],          [-s, c, 0]),  # FRH
    #     ([ -L, -W,  0],           [s,-c, 0]),  # BLH
    #     ([ -L,  W,  0],          [-s,-c, 0]),  # BRH
    # ]

    motors_setup = []
    builder = {
    Motors.BLV : ([-HL,  W, -H],           [0, 0, 1]),
    Motors.FLV : ([ HL,  W, -H],           [0, 0, 1]),
    Motors.FRV : ([ HL, -W, -H],           [0, 0, 1]),
    Motors.BRV : ([-HL, -W, -H],           [0, 0, 1]),
    Motors.FLH : ([  L,  W,  0],           [ s,  c, 0]),
    Motors.FRH : ([  L, -W,  0],           [ s, -c, 0]),
    Motors.BLH : ([ -L,  W,  0],           [ s, -c, 0]),
    Motors.BRH : ([ -L, -W,  0],           [ s,  c, 0]),
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

    # print(A)

    return A

def solve_thrusts(wrench, config):
    A = get_complete_tam(config)

    # The Moore-Penrose Pseudo-inverse
    A_plus = np.linalg.pinv(A)

    # v = A+ * F
    v = A_plus @ wrench
    return v



# --- Example Usage ---
# Desired Wrench: 10N Forward (x), 2N-m Yaw (tau_z)
desired_wrench = np.array([10, 0, 0, 0, 0, 2])

motor_commands = solve_thrusts(desired_wrench, config)
print(f"Individual Motor Forces: \n{motor_commands}")
