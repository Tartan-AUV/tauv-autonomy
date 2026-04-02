"""
Solve for gain given a desired force and a known voltage.

Chain:
  1. Force -> RPM  (invert the quadratic F(RPM))
  2. RPM, Voltage -> Gain  (invert the cubic polynomial RPM(gain, voltage))

Coefficients are loaded from motor_equations.yaml installed in the package
share directory (standard ROS 2 pattern).
"""

import os
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


def _load_equations(package_name: str = "tauv_autonomy",
                    filename: str = "motor_equations.yaml") -> dict:
    """Load motor equation coefficients from the installed YAML file."""
    share_dir = get_package_share_directory(package_name)
    yaml_path = os.path.join(share_dir, "config", filename)
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)


# ── Load coefficients once at import time ───────────────────────────────────
_EQ = _load_equations()

FORCE_FROM_RPM_POS = _EQ["force_from_rpm"]["positive"]
FORCE_FROM_RPM_NEG = _EQ["force_from_rpm"]["negative"]
FORCE_DEADBAND = float(_EQ["force_deadband"])


def force_to_rpm(force: float) -> float:
    """Invert F = a*RPM^2 + b*RPM + c  ->  RPM via np.roots."""
    if abs(force) < FORCE_DEADBAND:
        return 0.0
    if (force > 40):
        force = 40
    elif (force < -40):
        force = -40


    force = force / 9.81 # from N to kgf
    coefs = FORCE_FROM_RPM_POS if force >= 0 else FORCE_FROM_RPM_NEG
    a, b, c = coefs["a"], coefs["b"], coefs["c"]

    # a*RPM^2 + b*RPM + (c - F) = 0
    roots = np.roots([a, b, c - force])
    real_roots = roots[np.isreal(roots)].real

    if len(real_roots) == 0:
        return 0.0

    # Pick the physically meaningful root
    if force >= 0:
        return float(np.max(real_roots))
    else:
        return float(np.min(real_roots))


# ── quick demo ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Testing force_to_rpm with example forces and voltages:")
    test_cases = [
        (0, 16.0, "zero force"),
        (10, 16.0, "positive force"),
        (-10, 16.0, "negative force"),
        (50, 16.0, "large positive force"),
        (-50, 16.0, "large negative force"),
    ]
    for force, voltage, label in test_cases:
        print(f"\n--- {label}: F={force:.4f} N, V={voltage:.2f} V ---")
        rpm = force_to_rpm(force)
        print(f"  Solved RPM: {rpm:.2f}")
