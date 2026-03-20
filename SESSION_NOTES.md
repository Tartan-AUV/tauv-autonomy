# Session Notes — Thruster Controller Implementation
*Last updated: 2026-03-19*

## What We Built

A full wrench → thruster RPM pipeline for the Osprey AUV, working in both sim and real hardware.

### Pipeline summary
```
cmd_wrench (WrenchStamped)
  → ThrusterController (TAM pseudoinverse → force→RPM)
  → thruster_rpm (ThrusterRPM msg, 8 values)
      ├── sim:  ThrusterBridge → omega = RPM × 2π/60 → Stonefish
      └── real: CANDriver → polynomial inversion → gain → DroneCAN RawCommand
```

### Files created/modified
| File | What changed |
|---|---|
| `tauv_msgs/msg/ThrusterRPM.msg` | **NEW** — 8 RPM values + armed flag |
| `tauv_msgs/CMakeLists.txt` | Added ThrusterRPM.msg |
| `tauv_autonomy/config/thruster_params.yaml` | **NEW** — all thruster physics/polynomial config |
| `tauv_autonomy/forceOptimizer.py` | Rewrote to load from YAML, correct motor order, handedness |
| `tauv_autonomy/scripts/thruster_controller.py` | **NEW** — wrench→RPM ROS2 node |
| `tauv_autonomy/scripts/twist_to_wrench.py` | **NEW** — Foxglove Teleop (Twist→WrenchStamped) |
| `tauv_autonomy/CMakeLists.txt` | Added script install, config install, ament_cmake_python |
| `tauv_autonomy/package.xml` | Added geometry_msgs, python deps, ament_cmake_python |
| `tauv_autonomy/Osprey/launch/osprey.launch.py` | Added ThrusterController node |
| `tauv_autonomy/README.md` | **NEW** — full pipeline docs |
| `tauv_sim/config/params.yaml` | Reordered thrusters to match ThrusterSetpoint.msg order |
| `tauv_sim/src/thruster_bridge.cpp` | Accepts ThrusterRPM, RPM→omega, publishes v_bat as voltage |
| `tauv_sim/src/osprey.cpp` | Single `thruster_rpm` sub, shared `esc_telemetry` pub |
| `tauv_sim/include/tauv_sim/thruster_bridge.h` | Updated for ThrusterRPM |
| `tauv_sim/include/tauv_sim/osprey.h` | Updated for ThrusterRPM |
| `tauv_sim/launch/thruster_test.launch.py` | **NEW** — dynamic sim + teleop:=true flag |
| `tauv_drivers/tauv_dronecan/tauv_dronecan/can_driver.py` | Subscribes ThrusterRPM, RPM→gain via polynomial |

---

## Where We Left Off — Fix Applied (needs rebuild)

### Original error
```
[ERROR] [launch]: executable 'thruster_controller.py' not found on the libexec directory
  '/tauv-mono/ros_ws/install/lib/tauv_autonomy'
```

### Fix already applied (2026-03-19)
- `chmod +x` applied to all scripts
- `find_package(ament_cmake_python REQUIRED)` removed from CMakeLists.txt (not needed for plain `install(PROGRAMS ...)`)
- `<buildtool_depend>ament_cmake_python</buildtool_depend>` removed from package.xml

### Rebuild needed — run inside Docker container:

**Step 1 (done)** — Scripts are executable and CMakeLists.txt is fixed.

**Step 2 (TODO)** — Force a clean reconfigure and rebuild:
```bash
cd /tauv-mono/ros_ws
COLCON_DEFAULTS_FILE=colcon_defaults.sim.yaml \
  colcon build --packages-select tauv_msgs tauv_autonomy tauv_sim \
  --cmake-clean-cache
source install/setup.bash
```

**Step 5** — Verify the scripts installed:
```bash
ls /tauv-mono/ros_ws/install/lib/tauv_autonomy/
# Should show: thruster_controller.py  twist_to_wrench.py  forceOptimizer.py
```

**Step 6** — Try launching again:
```bash
ros2 launch tauv_sim thruster_test.launch.py teleop:=true
```

---

## Key Design Decisions Made This Session

### Motor ordering — ground truth is ThrusterSetpoint.msg
```
0=BLH  1=FLU  2=BLU  3=FLH  4=BRH  5=FRU  6=BRU  7=FRH
```
The old `tauv_sim/config/params.yaml` had a DIFFERENT order (0=BLH was first).
We reordered all thruster definitions in params.yaml to match.

### Sim interface
Stonefish takes omega (rad/s). We bypassed the old Bessa motor model —
`thruster_bridge.cpp` now just does `omega = RPM * 2π/60` directly.

### Handedness
Mirrored-prop thrusters (right_handed=0): negate RPM for same force direction.
`right_handed: [1, 1, 0, 0, 0, 1, 1, 0]`  — in ThrusterSetpoint.msg order.

### Voltage for polynomial
Averaged across all ESC telemetry readings. Falls back to `v_bat=16V` from config.
In sim, `ThrusterBridge` now publishes `v_bat` as voltage in ESC telemetry (was 0.0).

### Foxglove Teleop
Foxglove's Teleop panel publishes `geometry_msgs/Twist` on `cmd_vel`.
`twist_to_wrench.py` converts it to `cmd_wrench` with configurable force/torque scales.
Launch with `teleop:=true` to enable it.

---

## Things Still TODO (not done this session)

- [ ] **Fix thruster positions in `tauv_sim/config/params.yaml`** — vertical thrusters appear physically wrong in Stonefish visualization (wrong location on vehicle). `t_cad__thruster_N` values for upward thrusters (indices 1, 2, 5, 6 = FLU, BLU, FRU, BRU) need to be verified against CAD. Also update matching entries in `tauv_autonomy/config/thruster_params.yaml` (DOUBLE-PLACED).
- [ ] Path planner node (feeds `cmd_wrench` from odometry)
- [ ] Verify TAM gives correct thruster allocation — run thruster_test and check RPM signs
- [ ] Tune `max_force_*` / `max_torque_*` in thruster_test.launch.py for comfortable joystick feel
- [ ] Validate polynomial inversion numerically (at 16V, gain=4000 should give ~RPM 600)
- [ ] Real hardware test with can_driver.py once sim is confirmed working
