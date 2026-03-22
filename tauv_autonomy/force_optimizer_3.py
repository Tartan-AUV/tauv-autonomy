import numpy as np

def build_tam():
    com = np.array([0.0, 0.0, 0.076])
    raw_pos = np.array([
        [ 0.227, -0.225,  0.079], # 0: FRH
        [ 0.240,  0.212,  0.079], # 1: FLH
        [-0.226, -0.226,  0.079], # 2: BRH
        [-0.226,  0.226,  0.079], # 3: BLH
        [ 0.094, -0.246, -0.021], # 4: FRV
        [ 0.094,  0.246, -0.021], # 5: FLV
        [-0.094, -0.246, -0.021], # 6: BRV
        [-0.094,  0.246, -0.021], # 7: BLV
    ])
    
    pos = raw_pos - com
    sin45 = np.sin(np.deg2rad(45))
    cos45 = np.cos(np.deg2rad(45))
    
    dirs = np.array([
        [-cos45, -sin45, 0], 
        [-cos45,  sin45, 0], 
        [ cos45, -sin45, 0], 
        [ cos45,  sin45, 0], 
        [     0,      0, -1], 
        [     0,      0, -1], 
        [     0,      0, -1], 
        [     0,      0, -1], 
    ])
    
    T = np.zeros((6, 8))
    for i in range(8):
        T[0:3, i] = dirs[i]
        T[3:6, i] = np.cross(pos[i], dirs[i])
        
    return T

# Calculate the matrix and inverse once so it doesn't recalculate on every loop
T = build_tam()
T_inv = np.linalg.pinv(T)

def solve_thrusts(wrench):
    """
    Takes a 6-element wrench array [Surge, Sway, Heave, Roll, Pitch, Yaw]
    and returns an 8-element array of individual thruster forces.
    """
    return T_inv @ np.array(wrench)

def format_actual_values(forces):
    """Helper function to format the float values cleanly"""
    # Formats to 3 decimal places with a fixed width for clean columns
    return ", ".join([f"{f:>6.3f}" for f in forces])

def main():
    test_cases = {
        "Forwards":      [1, 0, 0, 0, 0, 0],
        "Left":          [0, 1, 0, 0, 0, 0],
        "Forward Left":  [1, 1, 0, 0, 0, 0],
        "Up":            [0, 0, 1, 0, 0, 0],
        "Yaw (CCW)":     [0, 0, 0, 0, 0, 1]
    }
    
    print(f"{'Test Case':<15} | {'   FRH,    FLH,    BRH,    BLH,    FRV,    FLV,    BRV,    BLV'}")
    print("-" * 85)
    
    for name, wrench in test_cases.items():
        # Use your new method here
        thruster_forces = solve_thrusts(wrench)
        
        # Print the formatted numerical results
        print(f"{name:<15} | {format_actual_values(thruster_forces)}")

if __name__ == '__main__':
    main()