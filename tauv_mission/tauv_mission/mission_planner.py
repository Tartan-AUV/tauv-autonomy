"""
PLEASE WORK PLEASE WORK PLEASE WORK PLESE WORK

The mission planner that makes everything happen. I didn't have enough time to
actually figure out behavior trees (my fault for procrastinating), so I'm faking.
Basically there's a list of instructions. We repeatedly run one every tick until
it succeeds, then we move to the next. Yeah it's a fake sequence node with no
recovery behavior.

The mission plan itself now lives in a JSON file (built with the mission_gui
editor) instead of being hardcoded, so mission changes don't need a code push.
"""

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String, Bool
from tauv_msgs.msg import Status


class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')

        # enum thing basically
        self.FAIL = 0
        self.SUCCESS = 1
        self.RUNNING = 2

        # Reserved for future use if cv_callback starts updating waypoints
        # live. Not used at plan-build time anymore -- the mission file
        # already has resolved absolute poses baked in.
        self.positions = {}
        self.statuses = {}  # dictionary of all the statuses, updated as we go

        # subscibers and publishers and yeah
        self.status_sub = self.create_subscription(
            Status, '/mission/status', self.status_callback, 10)
        self.cv_sub = self.create_subscription(
            PoseArray, '/cv/positions', self.cv_callback, 10)

        self.traj_pub = self.create_publisher(Odometry, '/odometry/target', 10)
        self.actuator_pub = self.create_publisher(String, '/servo/task', 10)
        self.watchdog_pub = self.create_publisher(Bool, '/watchdog/toggle', 10)

        # Mission plan is now loaded from disk on launch, resolved against
        # this package's installed share directory. Point mission_file at
        # whatever the mission_gui editor saved, e.g.:
        #   ros2 run <pkg> mission_planner --ros-args -p mission_file:=finals.json
        #
        # NOTE: this resolves to the *installed* share dir
        # (get_package_share_directory), not the mission_gui/missions/
        # folder the editor writes to directly. Make sure that folder is
        # installed into share (setup.py data_files / CMakeLists install)
        # or symlinked, or missions saved from the GUI won't show up here
        # until you rebuild.
        self.declare_parameter('mission_file', 'example_mission.json')
        mission_file = self.get_parameter('mission_file').get_parameter_value().string_value
        missions_dir = Path(get_package_share_directory('tauv_mission')) / 'missions'
        mission_path = missions_dir / mission_file
        self.mission_plan = self.load_mission(mission_path)
        self.current_step = 0

        self.timer = self.create_timer(0.1, self.tick)

    def load_mission(self, path: Path):
        """Reads a mission JSON file (the format mission_gui/app.py saves)
        and turns each step into an (fn, *args) tuple -- same shape tick()
        already expects, so nothing downstream had to change."""
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

        plan = []
        for i, step in enumerate(data.get('steps', [])):
            step_type = step.get('type')
            label = step.get('label', f'step_{i}')

            if step_type == 'move_to':
                try:
                    pose = tuple(float(v) for v in step['target_pose'])
                    vel = tuple(float(v) for v in step['target_velocity'])
                except (KeyError, TypeError, ValueError) as e:
                    self.get_logger().error(f'Skipping malformed move_to step "{label}" (#{i}): {e}')
                    continue
                if len(pose) != 6 or len(vel) != 6:
                    self.get_logger().error(
                        f'Skipping move_to step "{label}" (#{i}): pose/velocity need exactly 6 values each')
                    continue
                plan.append((self.move_to, pose, vel))

            elif step_type == 'actuate':
                mechanism = step.get('mechanism')
                setpoint = step.get('setpoint')
                if not mechanism or not setpoint:
                    self.get_logger().error(
                        f'Skipping malformed actuate step "{label}" (#{i}): missing mechanism or setpoint')
                    continue
                plan.append((self.actuate, mechanism, setpoint))

            elif step_type == 'toggle_watchdog':
                enabled = step.get('enabled')
                if not isinstance(enabled, bool):
                    self.get_logger().error(
                        f'Skipping malformed toggle_watchdog step "{label}" (#{i}): "enabled" must be true/false')
                    continue
                plan.append((self.toggle_watchdog, enabled))

            else:
                self.get_logger().error(f'Skipping step "{label}" (#{i}): unknown type "{step_type}"')

        self.get_logger().info(f'Loaded {len(plan)} step(s) from {path}')
        return plan

    def status_callback(self, msg):
        try:
            task_id = msg.data[0]
            status = msg.data[1]
            self.statuses[task_id] = status
        except AttributeError:
            pass

    def cv_callback(self, msg):
        pass

    def tick(self):
        # Check if done
        if self.current_step >= len(self.mission_plan):
            self.get_logger().info('Mission Complete! Sub is idle.', throttle_duration_sec=5.0)
            return

        # Grab the current task and execute it
        current_task = self.mission_plan[self.current_step]
        result = current_task[0](*current_task[1:])

        # Evaluate
        if result == self.SUCCESS:
            self.get_logger().info(f'Task [{current_task[0].__name__}] SUCCESS!')
            self.current_step += 1
            self.statuses.clear()  # Wipe the statuses for the next task
        elif result == self.FAIL:
            # Deliberately no recovery/retry here. A task that can't succeed
            # means the sub sits here logging FAIL forever, which surfaces
            # it (same effect as a diver cutting power) and is much easier
            # to debug post-run than a silent retry/recovery path would be.
            self.get_logger().error(f'Task [{current_task[0].__name__}] FAILED. NAURRR!!', throttle_duration_sec=2.0)
        elif result == self.RUNNING:
            pass

    def move_to(self, target_pose, target_velocity):
        task_id = "trajectory"

        msg = Odometry()

        # Position (X: Forward, Y: Left, Z: Up)
        msg.pose.pose.position.x = float(target_pose[0])
        msg.pose.pose.position.y = float(target_pose[1])
        msg.pose.pose.position.z = float(target_pose[2])

        # Angles (Roll, Pitch, Yaw)
        roll = float(target_pose[3])
        pitch = float(target_pose[4])
        yaw = float(target_pose[5])

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

        # Linear Velocity
        msg.twist.twist.linear.x = float(target_velocity[0])
        msg.twist.twist.linear.y = float(target_velocity[1])
        msg.twist.twist.linear.z = float(target_velocity[2])

        # Angular Velocity
        msg.twist.twist.angular.x = float(target_velocity[3])
        msg.twist.twist.angular.y = float(target_velocity[4])
        msg.twist.twist.angular.z = float(target_velocity[5])

        self.traj_pub.publish(msg)

        return self.statuses.get(task_id, self.RUNNING)

    def actuate(self, mechanism, setpoint):
        task_id = "actuator"
        msg = String()

        # TODO: fill in the actual /servo/task payload for each
        # (mechanism, setpoint) pair. Only the torpedo case had a value so
        # far (carried over as-is); everything else -- including what
        # "setpoint" should do to it -- is still a placeholder.
        if mechanism == "torpedo":
            if setpoint == "RESET":
                msg.data = "{data: 'reset_torpedo'}"
            elif setpoint == "SETPOINT 1":
                msg.data = "{data: 'trump'}"
            elif setpoint == "SETPOINT 2":
                msg.data = "{data: 'biden'}"
        elif mechanism == "dropper":
            if setpoint == "RESET":
                msg.data = "{data: 'reset_dropper'}"
            elif setpoint == "SETPOINT 1":
                msg.data = "{data: 'farnam'}"
            elif setpoint == "SETPOINT 2":
                msg.data = "{data: 'fence'}"

        self.actuator_pub.publish(msg)

        return self.statuses.get(task_id, self.RUNNING)

    def toggle_watchdog(self, enabled):
        task_id = "watchdog"

        msg = Bool()
        msg.data = bool(enabled)
        self.watchdog_pub.publish(msg)

        return self.statuses.get(task_id, self.RUNNING)


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