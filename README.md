# tauv_autonomy — Thruster Test Quick Start

## 1. Start and enter the Docker container

From the repo root on your host machine:

```bash
cd tauv-mono/containers
docker compose up -d tauv-desktop
docker exec -it containers-tauv-desktop-1 bash
```

All subsequent commands run **inside the container**.

---

## 2. Build

Run once after any code change:

```bash
cd /tauv-mono/ros_ws
  colcon build 
source install/setup.bash
```

> If you get `executable not found` errors:
> ```bash
> chmod +x /tauv-mono/ros_ws/src/tauv_autonomy/scripts/*.py \
>           /tauv-mono/ros_ws/src/tauv_autonomy/forceOptimizer.py
> ```
> Then rebuild with `--cmake-clean-cache` appended to the build command.

---

## 3. Clean up stale shared memory (do this before every launch)

Prevents harmless-but-noisy FastDDS errors in the console:

```bash
sudo rm -f /dev/shm/fastrtps_port*
```

---

## 4. Launch the thruster test sim

```bash
ros2 launch tauv_sim thruster_test.launch.py teleop:=true
```

| Flag | Default | Effect |
|---|---|---|
| `teleop:=true` | false | Enables Foxglove joystick → thruster pipeline |
| `record:=true` | false | Records all topics to a bag in `ros_ws/bags/` |

Leave this terminal running. Open a second terminal into the container for any extra commands:

```bash
docker exec -it containers-tauv-desktop-1 bash
```

---

## 5. Connect Foxglove

Open **[Foxglove Studio](https://app.foxglove.dev)** in a browser (or the desktop app) and connect to:

```
ws://localhost:8765
```

---

## 6. Add a Teleop panel (joystick)

1. Click **Add panel** → **Teleop**
2. Set **Topic** to `/cmd_vel`
3. Leave **Message type** as `geometry_msgs/Twist`
4. Drag the on-screen joystick — the vehicle will move

The joystick maps to:

| Joystick axis | Body axis | Max |
|---|---|---|
| Left stick X | Strafe (Y force) | 5 N |
| Left stick Y | Forward (X force) | 5 N |
| Right stick Y | Heave (Z force) | 3 N |
| Right stick X | Yaw torque | 2 N·m |

---

## 7. Add a slider (publish a single wrench value)

To manually command one axis at a time using a slider:

1. Click **Add panel** → **Publish**
2. Set **Topic** to `/cmd_wrench`
3. Set **Message type** to `geometry_msgs/WrenchStamped`
4. In the message template, set the fields you don't need to `0`
5. Click the field you want to control (e.g. `wrench.force.z`) and click the **slider icon** next to it
6. Set the slider **min/max** (e.g. −10 to 10) and drag to command thrust

Example: to test heave only, set all fields to 0 except `wrench.force.z`, attach a slider to it, and slide up/down.

---

## 8. Verify it's working

In a second container terminal:

```bash
source /tauv-mono/ros_ws/install/setup.bash

# Should show non-zero RPM when joystick/slider is moved
ros2 topic echo /thruster_rpm

# Should reflect the joystick/slider position
ros2 topic echo /cmd_wrench
```

Motor index order on `/thruster_rpm`:

| Index | Thruster | Axis |
|---|---|---|
| 0 | BLH | horizontal |
| 1 | FLU | upward |
| 2 | BLU | upward |
| 3 | FLH | horizontal |
| 4 | BRH | horizontal |
| 5 | FRU | upward |
| 6 | BRU | upward |
| 7 | FRH | horizontal |

---

## 9. Manual wrench (terminal only, no Foxglove)

```bash
source /tauv-mono/ros_ws/install/setup.bash

# Push forward 3 N
ros2 topic pub --once /cmd_wrench geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: 'os/base_link'}, wrench: {force: {x: 3.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"

# Heave up 5 N
ros2 topic pub --once /cmd_wrench geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: 'os/base_link'}, wrench: {force: {x: 0.0, y: 0.0, z: 5.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"
```

---

## Signal flow

```
Foxglove Teleop panel  ──►  /cmd_vel (Twist)
                              │
                        twist_to_wrench.py
                              │
Foxglove Publish slider ──►  /cmd_wrench (WrenchStamped)
                              │
                       thruster_controller.py
                       (TAM pseudoinverse → force→RPM)
                              │
                         /thruster_rpm (ThrusterRPM[8])
                              │
                    tauv_sim thruster bridges
                              │
                      Stonefish physics engine
```

---

## Config

Thruster parameters (positions, gains, polynomial coefficients) live in:

```
tauv_autonomy/config/thruster_params.yaml
```

Joystick force/torque limits are set in `tauv_sim/launch/thruster_test.launch.py` under the `twist_to_wrench` node parameters.
