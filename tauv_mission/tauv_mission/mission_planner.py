"""
PLEASE WORK PLEASE WORK PLEASE WORK PLESE WORK

The mission planner that makes everything happen. I didn't have enough time to
actually figure out behavior trees (my fault for procrastinating), so I'm faking.
Basically there's a list of instructions. We repeatedly run one every tick until
it succeeds, then we move to the next. Yeah it's a fake sequence node with no
recovery behavior.

The mission plan itself now lives in a JSON file (built with the mission_gui
editor) instead of being hardcoded, so mission changes don't need a code push.

CV integration: move_to targets are no longer baked-in absolute poses. Each
move_to step is a relative offset (in the *task's* FLU frame, not the world
frame) from a named position in self.positions -- "gate", "bins", or the
always-present "start" (the origin, for absolute/manual positioning). Config
values seed self.positions on launch; cv_?_callback is meant to overwrite them
live as vision detects things, so move_to recomputes the absolute target from
whatever's currently in self.positions every tick.
"""

import json
import math
import os
import shlex
import signal
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String, Bool
from tauv_msgs.msg import Status


# ---------------------------------------------------------------------------
# Small quaternion helpers. No numpy dependency, matches the ZYX (yaw, pitch,
# roll) Euler convention the rest of this file already used for the
# Odometry message, just pulled out so compose_pose can reuse it.
# ---------------------------------------------------------------------------

def euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def quat_to_euler(w, x, y, z):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quat_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def rotate_vector_by_quat(q, v):
    qv = (0.0, v[0], v[1], v[2])
    r = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return (r[1], r[2], r[3])


def compose_pose(task_pose, relative_pose):
    """task_pose, relative_pose: (x, y, z, roll, pitch, yaw).
    relative_pose is expressed in the task's own FLU frame (X fwd, Y left,
    Z up) -- e.g. (-1, 0, 0, 0, 0, 0) means "1m behind the task, along
    whichever way the task is facing", not "1m in the -X direction of the
    world". Returns (global_position_xyz, global_orientation_quat_wxyz).
    """
    tx, ty, tz, troll, tpitch, tyaw = task_pose
    rx, ry, rz, rroll, rpitch, ryaw = relative_pose

    q_task = euler_to_quat(troll, tpitch, tyaw)
    q_rel = euler_to_quat(rroll, rpitch, ryaw)

    rel_world = rotate_vector_by_quat(q_task, (rx, ry, rz))
    global_pos = (tx + rel_world[0], ty + rel_world[1], tz + rel_world[2])
    global_quat = quat_multiply(q_task, q_rel)

    return global_pos, global_quat


def rotate_velocity(task_pose, velocity):
    """velocity: (vx, vy, vz, vroll, vpitch, vyaw), expressed in the task's
    FLU frame -- same convention as the position offset in compose_pose, so
    "1 m/s forward" means forward along whichever way the task is facing,
    not world +X. Rotates both the linear and angular parts by the task's
    orientation and returns a 6-tuple in world frame."""
    troll, tpitch, tyaw = task_pose[3], task_pose[4], task_pose[5]
    q_task = euler_to_quat(troll, tpitch, tyaw)

    linear = rotate_vector_by_quat(q_task, velocity[0:3])
    angular = rotate_vector_by_quat(q_task, velocity[3:6])
    return linear + angular


class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')

        # enum thing basically
        self.FAIL = 0
        self.SUCCESS = 1
        self.RUNNING = 2

        self.statuses = {}  # dictionary of all the statuses, updated as we go

        # wait steps are self-timed (no /mission/status message to wait on),
        # so they need their own bit of state: the deadline for whichever
        # wait step is currently running, or None if no wait is in progress.
        self._wait_deadline = None

        # cli steps launch a subprocess once and then poll /mission/status
        # for completion -- this holds the Popen handle for whichever cli
        # step is currently running, or None if nothing's been launched yet
        # for the current step.
        self._cli_process = None

        # Everything cli steps have launched, keyed by the launching step's
        # label. Kept even after the step succeeds (the process itself is
        # left running) so a later cli_stop step can find and kill it.
        # Entries are removed once cli_stop reaps them.
        self._cli_children = {}

        # Deadline for the cli_stop step currently in progress (SIGINT
        # sent, waiting for exit before escalating to SIGKILL), or None
        # when no stop is in progress.
        self._stop_deadline = None

        # subscibers and publishers and yeah
        self.status_sub = self.create_subscription(
            Status, '/mission/status', self.status_callback, 10)
        self.cv_banner_sub = self.create_subscription(
            PoseArray, '/vision/banner/targets', self.cv_banner_callback, 10)

        self.traj_pub = self.create_publisher(Odometry, '/odometry/target', 10)
        self.actuator_pub = self.create_publisher(String, '/servo/task', 10)
        self.watchdog_pub = self.create_publisher(Bool, '/watchdog/toggle', 10)

        # Mission plan is loaded from disk on launch, resolved against
        # this package's installed share directory. Point mission_file at
        # whatever the mission_gui editor saved, e.g.:
        #   ros2 run <pkg> mission_planner --ros-args -p mission_file:=finals.json

        self.declare_parameter('mission_file', 'example_mission.json')
        mission_file = self.get_parameter('mission_file').get_parameter_value().string_value
        missions_dir = Path(get_package_share_directory('tauv_mission')) / 'missions'
        mission_path = missions_dir / mission_file

        # Which config/<name>.json the mission was built against, recorded by
        # the mission_gui editor. Loaded (by load_mission) before positions so
        # the right file can be picked below.
        self._mission_positions_file = None
        self.mission_plan = self.load_mission(mission_path)
        self.current_step = 0

        # Positions file resolution: an explicit positions_file param wins,
        # then whatever the mission file says it was built against, then the
        # old default. The GUI stores the name without ".json", so accept both.
        self.declare_parameter('positions_file', '')
        positions_file = self.get_parameter('positions_file').get_parameter_value().string_value
        if not positions_file:
            positions_file = self._mission_positions_file or 'positions'
        if not positions_file.endswith('.json'):
            positions_file += '.json'
        config_dir = Path(get_package_share_directory('tauv_mission')) / 'config'
        self.positions = self.load_positions(config_dir / positions_file)

        self.timer = self.create_timer(1.0, self.tick)

    def load_positions(self, path: Path):
        """Reads a positions config (config/<name>.json) into
        {name: (x,y,z,roll,pitch,yaw)}.
        These are the starting values -- cv_callback overwrites entries live
        as detections come in. "start" is guaranteed to exist (defaults to
        the origin) even if it's missing from the file, since move_to steps
        with no more specific reference use it for absolute positioning."""
        positions = {}

        if not path.is_file():
            self.get_logger().error(f'Positions file not found: {path}. Starting with no known positions.')
        else:
            try:
                data = json.loads(path.read_text())
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Positions file {path} is not valid JSON ({e}). Starting empty.')
                data = {}

            for name, pose in data.items():
                try:
                    pose_t = tuple(float(v) for v in pose)
                except (TypeError, ValueError):
                    self.get_logger().error(f'Skipping malformed position "{name}": not a list of numbers')
                    continue
                if len(pose_t) != 6:
                    self.get_logger().error(f'Skipping malformed position "{name}": needs exactly 6 values')
                    continue
                positions[name] = pose_t

        if 'start' not in positions:
            self.get_logger().warn('No "start" entry in positions config -- defaulting it to the origin.')
            positions['start'] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        self.get_logger().info(f'Loaded {len(positions)} position(s) from {path}')
        return positions

    def load_mission(self, path: Path):
        """Reads a mission JSON file (the format mission_gui/app.py saves)
        and turns each step into an (fn, label, *args) tuple -- tick() pulls
        the label out for logging and passes the rest through as args."""
        if not path.is_file():
            self.get_logger().error(
                f'Mission file not found: {path}. Sub will stay idle (empty plan).')
            return []

        try:
            data = json.loads(path.read_text())
        except json.JSONDecodeError as e:
            self.get_logger().error(
                f'Mission file {path} is not valid JSON ({e}). Sub will stay idle (empty plan).')
            return []

        positions_file = data.get('positions_file')
        if isinstance(positions_file, str) and positions_file:
            self._mission_positions_file = positions_file

        plan = []
        for i, step in enumerate(data.get('steps', [])):
            step_type = step.get('type')
            label = step.get('label', f'step_{i}')

            if step_type == 'move_to':
                reference = step.get('reference') or 'start'
                try:
                    offset = tuple(float(v) for v in step['offset'])
                    vel = tuple(float(v) for v in step['target_velocity'])
                except (KeyError, TypeError, ValueError) as e:
                    self.get_logger().error(f'Skipping malformed move_to step "{label}" (#{i}): {e}')
                    continue
                if len(offset) != 6 or len(vel) != 6:
                    self.get_logger().error(
                        f'Skipping move_to step "{label}" (#{i}): offset/velocity need exactly 6 values each')
                    continue
                plan.append((self.move_to, label, reference, offset, vel))

            elif step_type == 'actuate':
                mechanism = step.get('mechanism')
                setpoint = step.get('setpoint')
                if not mechanism or not setpoint:
                    self.get_logger().error(
                        f'Skipping malformed actuate step "{label}" (#{i}): missing mechanism or setpoint')
                    continue
                plan.append((self.actuate, label, mechanism, setpoint))

            elif step_type == 'toggle_watchdog':
                enabled = step.get('enabled')
                if not isinstance(enabled, bool):
                    self.get_logger().error(
                        f'Skipping malformed toggle_watchdog step "{label}" (#{i}): "enabled" must be true/false')
                    continue
                plan.append((self.toggle_watchdog, label, enabled))

            elif step_type == 'cli':
                command = step.get('command')
                if not command or not isinstance(command, str):
                    self.get_logger().error(
                        f'Skipping malformed cli step "{label}" (#{i}): missing or empty "command"')
                    continue
                # Label is passed as an arg too: cli() files the launched
                # process under it so cli_stop steps can target it later.
                plan.append((self.cli, label, label, command))

            elif step_type == 'cli_stop':
                target = step.get('target')
                if not target or not isinstance(target, str):
                    self.get_logger().error(
                        f'Skipping malformed cli_stop step "{label}" (#{i}): '
                        f'missing "target" (the label of the cli step to stop)')
                    continue
                plan.append((self.cli_stop, label, target))

            elif step_type == 'wait':
                seconds = step.get('seconds')
                if not isinstance(seconds, (int, float)) or isinstance(seconds, bool) or seconds <= 0:
                    self.get_logger().error(
                        f'Skipping malformed wait step "{label}" (#{i}): "seconds" must be a positive number')
                    continue
                plan.append((self.wait, label, float(seconds)))

            else:
                self.get_logger().error(f'Skipping step "{label}" (#{i}): unknown type "{step_type}"')

        self.get_logger().info(f'Loaded {len(plan)} step(s) from {path}')
        return plan

    def status_callback(self, msg):
        try:
            task_id = msg.id
            status = msg.status
            self.statuses[task_id] = status
        except AttributeError:
            self.get_logger().info("Attribute error", throttle_duration_sec=2.0)

    def cv_banner_callback(self, msg):
        if len(msg.poses) < 5: return
        self.get_logger().info("Banner CV callback received", throttle_duration_sec=2.0)
        names = ['banner0', 'banner1', 'banner2', 'banner3', 'bannerCenter']
        for i in range(0, 5):
            banner = names[i]
            banner_pose = msg.poses[i]
            r, p, y = quat_to_euler(
                banner_pose.orientation.w, banner_pose.orientation.x, banner_pose.orientation.y, banner_pose.orientation.z)
            # self.get_logger().info(f"CV CALLBACK: {banner} pose: x={banner_pose.position.x:.2f}, y={banner_pose.position.y:.2f}, z={banner_pose.position.z:.2f}, roll={r:.2f}, pitch={p:.2f}, yaw={y:.2f}")
            self.positions[banner] = (banner_pose.position.x, banner_pose.position.y, banner_pose.position.z, r, p, y)

    def tick(self):
        # Check if done
        if self.current_step >= len(self.mission_plan):
            self.get_logger().info('Mission Complete! Sub is idle.', throttle_duration_sec=5.0)
            return

        # Grab the current task and execute it
        current_task = self.mission_plan[self.current_step]
        result = current_task[0](*current_task[2:])

        # Evaluate
        if result == self.SUCCESS:
            self.get_logger().info(f'Task [{current_task[1]}] SUCCESS!')
            self.current_step += 1
            self.statuses.clear()  # Wipe the statuses for the next task
            self._cli_process = None  # Let the next cli step launch fresh

            if self.current_step < len(self.mission_plan):
                self.get_logger().info(f"Next Task: {self.mission_plan[self.current_step][1]}")
        elif result == self.FAIL:
            # Deliberately no recovery/retry here. A task that can't succeed
            # means the sub sits here logging FAIL forever, which surfaces
            # it (same effect as a diver cutting power) and is much easier
            # to debug post-run than a silent retry/recovery path would be.
            self.get_logger().error(f'Task [{current_task[1]}] FAILED. NAURRR!!', throttle_duration_sec=2.0)
        elif result == self.RUNNING:
            self.get_logger().info(f'Task [{current_task[1]}] RUNNING!')

    def move_to(self, reference, relative_pose, target_velocity):
        task_pose = self.positions.get(reference)
        if task_pose is None:
            self.get_logger().error(
                f'move_to: unknown reference "{reference}" -- not in positions config or CV data.')
            return self.FAIL

        task_id = "trajectory"
        msg = Odometry()

        global_pos, global_quat = compose_pose(task_pose, relative_pose)
        world_velocity = rotate_velocity(task_pose, target_velocity)

        # Position (X: Forward, Y: Left, Z: Up)
        msg.pose.pose.position.x = float(global_pos[0])
        msg.pose.pose.position.y = float(global_pos[1])
        msg.pose.pose.position.z = float(global_pos[2])

        msg.pose.pose.orientation.w = float(global_quat[0])
        msg.pose.pose.orientation.x = float(global_quat[1])
        msg.pose.pose.orientation.y = float(global_quat[2])
        msg.pose.pose.orientation.z = float(global_quat[3])

        # Linear velocity -- target_velocity is given in the task's FLU
        # frame, same as the position offset, then rotated into world frame
        # here so it's actually consistent with where the target pose is.
        msg.twist.twist.linear.x = float(world_velocity[0])
        msg.twist.twist.linear.y = float(world_velocity[1])
        msg.twist.twist.linear.z = float(world_velocity[2])

        # Angular velocity -- same deal, rotated into world frame.
        msg.twist.twist.angular.x = float(world_velocity[3])
        msg.twist.twist.angular.y = float(world_velocity[4])
        msg.twist.twist.angular.z = float(world_velocity[5])

        self.traj_pub.publish(msg)

        return self.statuses.get(task_id, self.RUNNING)

    def actuate(self, mechanism, setpoint):
        task_id = mechanism
        msg = String()

        # (mechanism, setpoint) -> /servo/task payload.
        if mechanism == "torpedo":
            if setpoint == "RESET":
                msg.data = "reset_torpedo"
            elif setpoint == "SETPOINT_1":
                msg.data = "farnam"
            elif setpoint == "SETPOINT_2":
                msg.data = "fence"
            else:
                print("Illegal setpoint:", setpoint)
                return
        elif mechanism == "dropper":
            if setpoint == "RESET":
                msg.data = "reset_dropper"
            elif setpoint == "SETPOINT_1":
                msg.data = "trump"
            elif setpoint == "SETPOINT_2":
                msg.data = "biden"
            else:
                print("Illegal setpoint:", setpoint)
                return
        else:
            print("Illegal mechanism:", mechanism)
            return
        self.actuator_pub.publish(msg)

        status = self.statuses.get(task_id)
        return status

    def toggle_watchdog(self, enabled):
        task_id = "watchdog"

        msg = Bool()
        msg.data = bool(enabled)
        self.watchdog_pub.publish(msg)

        return self.statuses.get(task_id, self.RUNNING)

    def cli(self, label, command):
        # Actually runs the command (e.g. "ros2 launch ..."), not just
        # announces it. Launched once via Popen the first tick this step is
        # active -- not re-launched on every tick like move_to/actuate republish
        # -- then left running in the background. Completion is still driven by
        # /mission/status with id "command", since a lot of these commands (a
        # launch file, a long-running node) never exit on their own -- whatever
        # gets launched is expected to publish that status itself once it's
        # done/ready. A later cli_stop step can kill it by this step's label.
        task_id = "command"

        if self._cli_process is None:
            try:
                # start_new_session makes the child the leader of its own
                # process group, so cli_stop can signal the whole group
                # (ros2 launch spawns a subprocess per node) without the
                # signal ever reaching this planner.
                self._cli_process = subprocess.Popen(
                    shlex.split(command), start_new_session=True)
                self.get_logger().info(f'cli: launched "{command}" (pid {self._cli_process.pid})')
            except (OSError, ValueError) as e:
                self.get_logger().error(f'cli: failed to launch "{command}": {e}')
                return self.FAIL
            if label in self._cli_children:
                self.get_logger().warn(
                    f'cli: label "{label}" was already used by an earlier cli step -- '
                    f'cli_stop will only see this newer process.')
            self._cli_children[label] = self._cli_process
        else:
            # Reap the process if it already exited, so it doesn't sit
            # around as a zombie while we wait on /mission/status.
            self._cli_process.poll()

        return self.statuses.get(task_id, self.RUNNING)

    def cli_stop(self, target_label):
        # Kills whatever the cli step labeled `target_label` launched.
        # SIGINT to the process group first, so launch files get their
        # normal ctrl-C shutdown path; if the group is still alive after
        # the grace period, escalate to SIGKILL. Unlike cli, this step
        # completes on its own -- no /mission/status involved. SUCCESS
        # means the launched process is actually gone.
        proc = self._cli_children.get(target_label)
        if proc is None:
            self.get_logger().error(
                f'cli_stop: no process on record for cli step "{target_label}" -- '
                f'it never launched, or was already stopped.')
            return self.FAIL

        if proc.poll() is not None:
            del self._cli_children[target_label]
            self._stop_deadline = None
            self.get_logger().info(
                f'cli_stop: "{target_label}" (pid {proc.pid}) is gone (exit code {proc.returncode})')
            return self.SUCCESS

        now = self.get_clock().now()
        if self._stop_deadline is None:
            self._stop_deadline = now + Duration(seconds=10.0)
            self._signal_group(proc, signal.SIGINT, target_label)
        elif now >= self._stop_deadline:
            # Didn't die gracefully within the grace period. Push the
            # deadline out again so SIGKILL isn't re-sent every tick while
            # the kernel finishes tearing the group down.
            self._stop_deadline = now + Duration(seconds=10.0)
            self._signal_group(proc, signal.SIGKILL, target_label)

        return self.RUNNING

    def _signal_group(self, proc, sig, label):
        # cli launches with start_new_session=True, so the child's pid is
        # also its process group id.
        name = signal.Signals(sig).name
        try:
            os.killpg(proc.pid, sig)
            self.get_logger().info(f'cli_stop: sent {name} to "{label}" (pgid {proc.pid})')
        except ProcessLookupError:
            # Group already gone -- the next poll() will confirm and succeed.
            pass

    def wait(self, seconds):
        # No /mission/status message drives this one -- it's just the
        # node's own clock. Starts the deadline on first call, then holds
        # RUNNING until it's passed. Resets to None on completion so the
        # next wait step (if any) starts its own deadline fresh.
        now = self.get_clock().now()

        if self._wait_deadline is None:
            self._wait_deadline = now + Duration(seconds=seconds)
            self.get_logger().info(f'Waiting {seconds}s...')

        if now >= self._wait_deadline:
            self._wait_deadline = None
            return self.SUCCESS

        return self.RUNNING


def main(args=None):
    rclpy.init(args=args)
    planner = MissionPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
