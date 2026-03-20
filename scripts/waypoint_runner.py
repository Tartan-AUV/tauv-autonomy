#!/usr/bin/env python3
"""
Waypoint runner — steps through a list of commands for PID tuning and testing.

The sequence always starts immediately. The controller starts disabled (hardcoded
in controller.py). Use enable/disable waypoints, or publish to /system_enable from
Foxglove, to arm/disarm thrusters at any point.

Waypoint types (set via 'type' field, default is 'pose'):

  pose       {x, y, z, yaw}
               Publish desired_state and advance when pos_err < arrival_radius
               AND yaw_err < yaw_threshold, then hold for hold_duration seconds.
               Does NOT automatically enable the controller — add an enable
               waypoint before pose waypoints to arm the vehicle.

  hold       {duration}  (optional, omit or -1 = hold forever)
               Re-publish the last pose waypoint as desired_state.
               Does NOT change controller state.

  set_pose   {x, y, z, yaw}  (all optional, default 0.0)
               Call /ekf_filter_node/set_pose to override the EKF's estimated
               position. Useful at sequence start to zero out accumulated drift.
               Fire-and-forget — advances immediately. Add a short pause after
               if the next waypoint needs the new pose to be applied first.

  enable     (no fields)
               Publish controller_enabled = True and advance immediately.
               Also triggered externally via /system_enable Bool(data=true).

  disable    (no fields)
               Publish controller_enabled = False and advance immediately.
               Also triggered externally via /system_enable Bool(data=false).

  pause      {duration}
               Shortcut for raw_thrust with all-zero forces for 'duration' seconds.
               Disables controller, actively commands zero thrust (no coast drift),
               then re-enables and advances.

  raw_thrust {forces: [f0..f7], duration}
               Disable controller, publish ThrusterSetpoint (N) directly to
               thruster_forces for 'duration' seconds, then re-enable controller.
               Thruster order: [blh, flu, blu, flh, brh, fru, bru, frh]

  loop       {to: <index>}  (optional count: N, omit or -1 = infinite)
               Jump back to waypoint at index 'to'. Counts down 'count' passes
               through the loop entry (not iterations from 'to'), then falls
               through. Great for repeated step-response runs.

External control (e.g. from Foxglove Publish panel):
  Enable:   publish Bool(data=true)  to /system_enable
  Disable:  publish Bool(data=false) to /system_enable

Example PID step-response sequence:
  waypoints:
    - {type: enable}                           # arm thrusters
    - {x: 0.0, y: 0.0, z: -2.0, yaw: 0.0}   # descend to start
    - {type: hold, duration: 2.0}             # settle
    - {x: 1.0, y: 0.0, z: -2.0, yaw: 0.0}   # step in x
    - {type: hold, duration: 2.0}             # observe response
    - {type: loop, to: 1, count: 5}           # repeat 5 times

Topics:
  Subscribes: /odometry/filtered    (nav_msgs/Odometry)
              /system_enable        (std_msgs/Bool) — directly enables/disables controller
  Publishes:  desired_state         (geometry_msgs/PoseStamped)
              waypoint_path         (nav_msgs/Path, latched — pose waypoints only)
              waypoint_status       (std_msgs/String — live state string)
              controller_enabled    (std_msgs/Bool, latched)
              thruster_forces       (tauv_msgs/ThrusterSetpoint — raw_thrust only)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from tauv_msgs.msg import ThrusterSetpoint
from robot_localization.srv import SetPose
from ament_index_python.packages import get_package_share_directory
import yaml
from pathlib import Path as FilePath
from enum import Enum, auto


def _quat_from_yaw(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _quat_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a, b) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class State(Enum):
    MOVING = auto()
    HOLDING = auto()
    DONE = auto()


class WaypointRunner(Node):
    def __init__(self):
        super().__init__('waypoint_runner')

        share_dir = FilePath(get_package_share_directory('tauv_autonomy'))
        yaml_path = share_dir / 'config' / 'waypoints.yaml'
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        cfg = data['waypoint_runner']
        self._arrival_radius = float(cfg.get('arrival_radius', 0.05))  # [m]
        self._yaw_threshold  = float(cfg.get('yaw_threshold',  0.10))  # [rad]
        self._hold_duration  = float(cfg.get('hold_duration',  0.5))   # [s]
        self._startup_delay  = float(cfg.get('startup_delay',  0.0))   # [s]
        rate = float(cfg.get('rate', 10.0))

        self._waypoints = cfg['waypoints']
        self._wp_index = 0
        self._state = State.MOVING
        self._hold_start: float | None = None
        self._start_time: float | None = None

        self._last_pose_wp = None        # most recent pose wp — re-published by 'hold'
        self._loop_counts: dict[int, int] = {}  # wp_index -> remaining passes
        self._set_pose_wait_start: float | None = None  # for service timeout

        self._current_odom: Odometry | None = None
        self._last_countdown_log: float | None = None   # for startup delay countdown

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pose_pub   = self.create_publisher(PoseStamped,     'desired_state',     10)
        self._path_pub   = self.create_publisher(Path,            'waypoint_path',     latched)
        self._status_pub = self.create_publisher(String,          'waypoint_status',   10)
        self._ctrl_pub   = self.create_publisher(Bool,            'controller_enabled', latched)
        self._thrust_pub = self.create_publisher(ThrusterSetpoint,'thruster_forces',   10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        # Latched so a Foxglove enable published before node start is still received on connect.
        self.create_subscription(Bool, '/system_enable', self._enable_cb, latched)
        self._set_pose_client = self.create_client(SetPose, '/ekf_filter_node/set_pose')
        self.create_timer(1.0 / rate, self._tick)

        self._publish_path()
        self.get_logger().info(
            f'WaypointRunner: {len(self._waypoints)} steps, '
            f'arrival_radius={self._arrival_radius}m, hold={self._hold_duration}s'
        )

    # ── helpers ────────────────────────────────────────────────────────────────

    def _set_controller(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self._ctrl_pub.publish(msg)

    def _publish_path(self):
        """Latched path of pose-type waypoints only, for Foxglove 3D view."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for wp in self._waypoints:
            if wp.get('type', 'pose') != 'pose':
                continue
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(wp.get('x', 0.0))
            pose.pose.position.y = float(wp.get('y', 0.0))
            pose.pose.position.z = float(wp.get('z', 0.0))
            qx, qy, qz, qw = _quat_from_yaw(float(wp.get('yaw', 0.0)))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            msg.poses.append(pose)
        self._path_pub.publish(msg)

    def _publish_pose(self, wp):
        """Publish desired_state. Omitted fields use current odom so the controller
        sees zero error for those DOFs and doesn't fight them."""
        cur = self._current_odom
        cur_pos = cur.pose.pose.position if cur is not None else None
        cur_q   = cur.pose.pose.orientation if cur is not None else None
        cur_yaw = _quat_to_yaw(cur_q.x, cur_q.y, cur_q.z, cur_q.w) if cur_q is not None else 0.0

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(wp['x']) if 'x' in wp else (cur_pos.x if cur_pos is not None else 0.0)
        msg.pose.position.y = float(wp['y']) if 'y' in wp else (cur_pos.y if cur_pos is not None else 0.0)
        msg.pose.position.z = float(wp['z']) if 'z' in wp else (cur_pos.z if cur_pos is not None else 0.0)
        yaw = float(wp['yaw']) if 'yaw' in wp else cur_yaw
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._pose_pub.publish(msg)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _pos_error(self, wp) -> float:
        if self._current_odom is None:
            return float('inf')
        p = self._current_odom.pose.pose.position
        axes = []
        if 'x' in wp: axes.append((float(wp['x']) - p.x) ** 2)
        if 'y' in wp: axes.append((float(wp['y']) - p.y) ** 2)
        if 'z' in wp: axes.append((float(wp['z']) - p.z) ** 2)
        return math.sqrt(sum(axes)) if axes else 0.0

    def _yaw_error(self, wp) -> float:
        if 'yaw' not in wp or self._current_odom is None:
            return 0.0
        q = self._current_odom.pose.pose.orientation
        return abs(_angle_diff(float(wp['yaw']), _quat_to_yaw(q.x, q.y, q.z, q.w)))

    def _advance(self):
        self._wp_index += 1
        self._state = State.MOVING
        self._hold_start = None

    def _enable_cb(self, msg: Bool):
        """Direct controller arm/disarm from /system_enable (e.g. Foxglove Publish panel)."""
        state = 'enabled' if msg.data else 'disabled'
        self.get_logger().info(f'Controller {state} via /system_enable')
        self._set_controller(msg.data)

    def _odom_cb(self, msg: Odometry):
        self._current_odom = msg

    # ── main tick ──────────────────────────────────────────────────────────────

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._start_time is None:
            self._start_time = now
        if now - self._start_time < self._startup_delay:
            remaining = self._startup_delay - (now - self._start_time)
            self._publish_status(f'STARTUP DELAY {remaining:.1f}s remaining')
            # Log countdown once per second
            if self._last_countdown_log is None or now - self._last_countdown_log >= 1.0:
                self.get_logger().info(f'Starting in {remaining:.0f}s...')
                self._last_countdown_log = now
            return
        if self._state == State.DONE:
            return
        if self._wp_index >= len(self._waypoints):
            self.get_logger().info('All waypoints complete.')
            self._state = State.DONE
            return

        wp = self._waypoints[self._wp_index]
        wp_type = wp.get('type', 'pose')

        # ── pose ──────────────────────────────────────────────────────────────
        if wp_type == 'pose':
            self._set_controller(True)
            self._publish_pose(wp)
            self._last_pose_wp = wp

            pos_err = self._pos_error(wp)
            yaw_err = self._yaw_error(wp)
            self._publish_status(
                f'POSE {self._wp_index}/{len(self._waypoints)-1} '
                f'pos_err={pos_err:.3f}m yaw_err={yaw_err:.3f}rad '
                f'[{self._state.name}]'
            )

            if self._state == State.MOVING:
                if pos_err < self._arrival_radius and yaw_err < self._yaw_threshold:
                    self.get_logger().info(
                        f'[{self._wp_index}] arrived pos={pos_err:.3f}m '
                        f'yaw={yaw_err:.3f}rad — holding {self._hold_duration}s'
                    )
                    self._state = State.HOLDING
                    self._hold_start = now
            elif self._state == State.HOLDING:
                if now - self._hold_start >= self._hold_duration:
                    self.get_logger().info(f'[{self._wp_index}] hold done → advancing')
                    self._advance()

        # ── hold ──────────────────────────────────────────────────────────────
        elif wp_type == 'hold':
            duration = float(wp.get('duration', -1))
            target = self._last_pose_wp
            if target is None:
                self.get_logger().warn('hold: no previous pose waypoint — skipping')
                self._advance()
                return

            self._set_controller(True)
            self._publish_pose(target)

            if self._state == State.MOVING:
                label = f'{duration:.1f}s' if duration >= 0 else 'forever'
                self.get_logger().info(f'[{self._wp_index}] hold at last pose ({label})')
                self._state = State.HOLDING
                self._hold_start = now

            elapsed = now - self._hold_start
            self._publish_status(
                f'HOLD {self._wp_index}/{len(self._waypoints)-1} '
                f'elapsed={elapsed:.1f}s'
                + (f'/{duration:.1f}s' if duration >= 0 else ' (forever)')
            )

            if duration >= 0 and self._state == State.HOLDING:
                if elapsed >= duration:
                    self.get_logger().info(f'[{self._wp_index}] hold done → advancing')
                    self._advance()

        # ── set_pose ──────────────────────────────────────────────────────────
        elif wp_type == 'set_pose':
            x   = float(wp.get('x',   0.0))
            y   = float(wp.get('y',   0.0))
            z   = float(wp.get('z',   0.0))
            yaw = float(wp.get('yaw', 0.0))

            if not self._set_pose_client.service_is_ready():
                if self._set_pose_wait_start is None:
                    self._set_pose_wait_start = now
                waited = now - self._set_pose_wait_start
                self._publish_status(
                    f'SET_POSE {self._wp_index}/{len(self._waypoints)-1} '
                    f'— waiting for /ekf_filter_node/set_pose ({waited:.1f}s)...'
                )
                if waited > 3.0:
                    self.get_logger().warn(
                        f'[{self._wp_index}] set_pose: service not available after 3s — skipping'
                    )
                    self._set_pose_wait_start = None
                    self._advance()
                return
            self._set_pose_wait_start = None

            req = SetPose.Request()
            req.pose = PoseWithCovarianceStamped()
            req.pose.header.stamp = self.get_clock().now().to_msg()
            req.pose.header.frame_id = 'odom'
            req.pose.pose.pose.position.x = x
            req.pose.pose.pose.position.y = y
            req.pose.pose.pose.position.z = z
            qx, qy, qz, qw = _quat_from_yaw(yaw)
            req.pose.pose.pose.orientation.x = qx
            req.pose.pose.pose.orientation.y = qy
            req.pose.pose.pose.orientation.z = qz
            req.pose.pose.pose.orientation.w = qw
            self._set_pose_client.call_async(req)
            self.get_logger().info(
                f'[{self._wp_index}] set_pose x={x} y={y} z={z} yaw={yaw} → advancing'
            )
            self._advance()

        # ── enable ────────────────────────────────────────────────────────────
        elif wp_type == 'enable':
            self.get_logger().info(f'[{self._wp_index}] enable controller → advancing')
            self._set_controller(True)
            self._advance()

        # ── disable ───────────────────────────────────────────────────────────
        elif wp_type == 'disable':
            self.get_logger().info(f'[{self._wp_index}] disable controller → advancing')
            self._set_controller(False)
            self._advance()

        # ── pause ─────────────────────────────────────────────────────────────
        elif wp_type == 'pause':
            # Shortcut for raw_thrust with all-zero forces
            duration = float(wp['duration'])
            self._set_controller(False)
            thrust_msg = ThrusterSetpoint()
            thrust_msg.header.stamp = self.get_clock().now().to_msg()
            thrust_msg.header.frame_id = 'os/base_link'
            thrust_msg.thrust = [0.0] * 8
            thrust_msg.armed = True
            self._thrust_pub.publish(thrust_msg)

            if self._state == State.MOVING:
                self.get_logger().info(
                    f'[{self._wp_index}] pause (zero thrust) for {duration}s'
                )
                self._state = State.HOLDING
                self._hold_start = now

            elapsed = now - self._hold_start
            self._publish_status(
                f'PAUSE {self._wp_index}/{len(self._waypoints)-1} '
                f'elapsed={elapsed:.1f}s/{duration:.1f}s'
            )

            if self._state == State.HOLDING and elapsed >= duration:
                self.get_logger().info(f'[{self._wp_index}] pause done → advancing')
                self._set_controller(True)
                self._advance()

        # ── raw_thrust ────────────────────────────────────────────────────────
        elif wp_type == 'raw_thrust':
            self._set_controller(False)
            thrust_msg = ThrusterSetpoint()
            thrust_msg.header.stamp = self.get_clock().now().to_msg()
            thrust_msg.header.frame_id = 'os/base_link'
            thrust_msg.thrust = [float(f) for f in wp['forces']]
            thrust_msg.armed = True
            self._thrust_pub.publish(thrust_msg)

            duration = float(wp.get('duration', self._hold_duration))

            if self._state == State.MOVING:
                self.get_logger().info(
                    f'[{self._wp_index}] raw_thrust {wp["forces"]} for {duration}s'
                )
                self._state = State.HOLDING
                self._hold_start = now

            elapsed = now - self._hold_start
            self._publish_status(
                f'RAW_THRUST {self._wp_index}/{len(self._waypoints)-1} '
                f'elapsed={elapsed:.1f}s/{duration:.1f}s'
            )

            if self._state == State.HOLDING and elapsed >= duration:
                self.get_logger().info(f'[{self._wp_index}] raw_thrust done → advancing')
                self._advance()
                self._set_controller(True)

        # ── loop ──────────────────────────────────────────────────────────────
        elif wp_type == 'loop':
            target_index = int(wp['to'])
            count = int(wp.get('count', -1))   # -1 = infinite

            if self._wp_index not in self._loop_counts:
                # First time hitting this loop instruction — initialise counter
                self._loop_counts[self._wp_index] = count

            remaining = self._loop_counts[self._wp_index]

            if remaining == 0:
                # Counter exhausted — fall through
                self.get_logger().info(
                    f'[{self._wp_index}] loop exhausted → advancing'
                )
                del self._loop_counts[self._wp_index]
                self._advance()
            else:
                if remaining > 0:
                    self._loop_counts[self._wp_index] -= 1
                    self.get_logger().info(
                        f'[{self._wp_index}] loop → {target_index} '
                        f'({remaining - 1 if remaining > 0 else "∞"} remaining)'
                    )
                else:
                    self.get_logger().info(
                        f'[{self._wp_index}] loop → {target_index} (infinite)'
                    )
                self._wp_index = target_index
                self._state = State.MOVING
                self._hold_start = None

            self._publish_status(
                f'LOOP {self._wp_index}/{len(self._waypoints)-1} '
                f'to={target_index} remaining={remaining}'
            )

        else:
            self.get_logger().warn(f'Unknown waypoint type "{wp_type}" — skipping')
            self._advance()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
