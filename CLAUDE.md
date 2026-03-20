# tauv_autonomy — Claude Notes

## Building

```bash
cd /tauv-mono/ros_ws
colcon build
source install/setup.bash
```

**Adding a new script:** list it in `CMakeLists.txt` under `install(PROGRAMS ...)` or `ros2 run` / the launch file won't find it.

**New scripts must be executable** or ROS2 will fail with "executable not found" even if the symlink exists:
```bash
chmod +x ros_ws/src/tauv_autonomy/scripts/your_script.py
```
This only needs to be done once — the symlink-install points directly at the source file so the bit persists across rebuilds.

## Package Purpose
Robot-agnostic autonomy stack for the Osprey AUV. Works identically with simulation (`tauv_sim`) and real hardware (`tauv_drivers`) — it doesn't know or care which is running.

## Architecture

```
SIM:   tauv_sim bridges → os/sensors/* ──────────────────┐
                                                           ↓
REAL:  tauv_drivers → raw topics                   osprey.launch.py
         ↓                                    ├── robot_state_publisher (URDF)
       tauv_core converters → os/sensors/* ──→├── ekf_filter_node (tauv_core config)
                                               ├── [TODO] controller
                                               └── [TODO] path_planner
```

## Sensor Topic Contract

Both sim and real publish to these topics before reaching tauv_autonomy:

| Topic | Type | Producer (sim) | Producer (real) |
|---|---|---|---|
| `os/sensors/depth` | nav_msgs/Odometry | pressure_sensor_bridge.cpp | tauv_core/depth_converter |
| `os/sensors/dvl` | geometry_msgs/TwistWithCovarianceStamped | dvl_bridge.cpp | tauv_core/dvl_converter |
| `os/sensors/imu_xsens` | sensor_msgs/Imu | imu_bridge.cpp | tauv_core/imu_converter |

All data arrives in **ENU/FLU convention** — the NED→ENU conversion is done in the sim bridges (C++) or implicitly by real hardware drivers.

## URDF (`Osprey/urdf/sim_model.urdf`)

The URDF defines the physical robot model with correct sensor positions. Root link is `os/base_link`.

**Sensor frame offsets from `os/base_link`:**
- `imu_xsens_link`: xyz = (-0.017, -0.111, +0.047) m
- `dvl_link`:       xyz = (0, -0.141, +0.010) m
- `depth_link`:     xyz = (+0.064, -0.200, +0.080) m

**Important:** Sensor joint rotations are zeroed in the URDF. The sim bridges already convert FRD→FLU in C++. Keeping the original CAD joint rotations would double-rotate EKF inputs via TF. If you update the URDF, keep sensor joint rpy="0 0 0".

## Launch Files

### `Osprey/launch/osprey.launch.py`
Starts `robot_state_publisher`, `controller`, `thruster_allocator`, and `force_to_gain`. Accepts a `sim` argument (default `false`) that is forwarded to `thruster_allocator` to enable NED→ENU wrench conversion. The EKF is launched from `tauv_sim/launch/state_estimator.launch.py` (sim) or should be added here for real-robot use.

**Real robot usage:**
```bash
# Terminal 1: hardware drivers
ros2 launch tauv_launch sensors.launch.py

# Terminal 2: autonomy
ros2 launch tauv_autonomy Osprey/launch/osprey.launch.py
# + EKF node separately (see tauv_core/config/ekfFUNNY.yaml)
```

**Sim usage:**
```bash
ros2 launch tauv_sim state_estimator.launch.py
# (includes osprey.launch.py automatically)
```

## Scripts (`scripts/`)

| Script | Role |
|---|---|
| `controller.py` | 6-DOF PID + feedforward: `/odometry/filtered` + `/desired_state` → `/cmd_wrench` (ENU/FLU body frame). Output = `ff_bias + kff*des_vel + P + I + D`. Always starts disabled (hardcoded `_enabled = False`) — no parameter, no race condition — set `false` in launch to start disabled (no race condition with `require_enable`). Subscribes to `controller_enabled` (std_msgs/Bool, **non-latched** QoS) — when False, stops publishing and resets integrators. Non-latched is intentional: prevents a stale True from a previous run being delivered on startup and overriding the hardcoded disabled state. Feedforward gains in `controller_params.yaml`. **Live gain tuning:** all gains (`kp`, `ki`, `kd`, `ff_bias`, `kff`, `max_wrench`, `integrator_max_fraction`, `integrator_window`) are declared as ROS2 parameters — edit live via Foxglove Parameters panel (Add Panel → Parameters → `/controller`). Changes take effect immediately and are written to `config/controller_params_overrides.yaml`, which is loaded on next startup. Delete that file to revert to `controller_params.yaml` defaults. Integrator uses a rolling window (default 1s, `integrator_window` param). |
| `ned_to_enu_wrench.py` | **Not used in pipeline.** Kept for reference. Controller already outputs ENU/FLU — adding this node would double-flip signs and reverse all commands. |
| `thruster_allocator.py` | TAM pseudoinverse: `/cmd_wrench` (ENU/FLU) → `/thruster_forces` (ThrusterSetpoint, N); `sim` bool param (default `false`) — selects `flip_signs_sim` vs `flip_signs_real` sign conventions only; latched on startup: `thruster_allocation_matrix` (Float64MultiArray 6×8), `thruster_allocation_matrix_str` (String), `thruster_pseudoinverse` (Float64MultiArray 8×6), `thruster_pseudoinverse_str` (String); per-callback: `thruster_forces_str` (String, wrench + forces table) |
| `force_to_gain.py` | Force→RPM→gain: `/thruster_forces` + `/esc_telemetry` → `/thruster_gains` (ThrusterGains, [-1,1]) + `/thruster_setpoint` (ThrusterSetpoint, `.thrust`=gains [-1,1] for real hardware `can_driver`). **Watchdog (1.0s):** subscribes to `controller_enabled` — watchdog is suppressed when controller is disabled (intentional silence, e.g. `pause` waypoint). When controller is enabled and no `/thruster_forces` arrives within `WATCHDOG_TIMEOUT`, publishes zero gains (`armed=False`) AND publishes `controller_enabled=False`. Logs at ERROR level. Receiving `controller_enabled=False` resets the watchdog timer so intentional silence (pause, disable, raw_thrust) doesn't trigger it. |
| `twist_to_wrench.py` | Foxglove Teleop adapter: `/cmd_vel` (Twist) → `/cmd_wrench` (WrenchStamped) |
| `play_commands.py` | Scripted test sequencer: publishes WrenchStamped commands to `/cmd_wrench`. Activated via `ros2 launch tauv_sim thruster_test.launch.py play:=true`. |
| `waypoint_runner.py` | Sequential waypoint follower for PID tuning. Reads `config/waypoints.yaml`. **Sequence always runs from start** — no startup gate. Controller starts disabled (hardcoded in controller.py); use `enable`/`disable` waypoints or publish to `/system_enable` (Bool) from Foxglove to arm/disarm. Eight waypoint types: **set_pose** `{x,y,z,yaw}` (all optional, default 0) — calls `/ekf_filter_node/set_pose` service to override EKF estimated position (fire-and-forget, add a pause after if needed); **pose** `{x,y,z,yaw}` — enable controller, publish desired_state, advance on arrival; **enable** — `controller_enabled=True`, advance immediately; **disable** — `controller_enabled=False`, advance immediately; **pause** `{duration}` — shortcut for zero raw_thrust: disables controller, actively publishes zero forces for N seconds, then re-enables and advances; **hold** `{duration}` (omit/-1=forever) — re-publish last pose, no controller state change; **raw_thrust** `{forces:[f0..f7], duration}` — disable controller, inject ThrusterSetpoint (N) directly to `thruster_forces`, re-enable after; **loop** `{to: idx, count: N}` (count omit/-1=infinite) — jump back to waypoint index. `/system_enable` topic directly calls `_set_controller(True/False)` — usable from Foxglove Publish panel. Publishes `desired_state`, `waypoint_path` (latched), `waypoint_status`, `controller_enabled`, `thruster_forces`. |

## Thruster Pipeline

```
controller.py  →  /cmd_wrench (ENU/FLU)
  →  thruster_allocator.py  →  /thruster_forces
  →  force_to_gain.py  →  /thruster_gains  →  sim (tauv_sim) or hardware (tauv_drivers)
```

- `/thruster_forces` (tauv_msgs/ThrusterSetpoint): per-thruster forces in N
- `/thruster_gains` (tauv_msgs/ThrusterGains): normalized gains in [-1, 1], same signal sent to both hardware and sim
- TAM is in **ENU/FLU body frame** (x=forward, y=left, z=up); controller outputs ENU/FLU directly — no translation node needed
- TAM logic lives in `forceOptimizer.py`, imported by `thruster_allocator.py`
- Torque arms are computed relative to the **CoM** (`com_cad` in `thruster_params.yaml`), not `os/base_link` origin; CoM sourced from `Osprey/urdf/sim_model.urdf` `<inertial><origin>`
- Per-thruster sign flips live in `flip_signs_sim` / `flip_signs_real` in `thruster_params.yaml`; `thruster_allocator` selects via `sim` ROS param (bool, default `false`) — pass `sim:=true` when launching in simulation
- Config for allocator/gain conversion: `config/thruster_params.yaml`
- Config for PID gains: `config/controller_params.yaml`

## TODO Placeholders (in osprey.launch.py)
- **Path planner** (`tauv_planner` — TBD): subscribes `/odometry/filtered`, publishes `/desired_state` (PoseStamped)
