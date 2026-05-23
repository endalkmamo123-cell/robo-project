#!/usr/bin/env python3
"""
Autonomous Room Explorer + Cleaner  v3
========================================
Phase 1 — EXPLORE (wall-following, explorer owns /cmd_vel)
Phase 2 — CLEAN   (A* or Dijkstra plans path, DWA or APF drives the robot)

How the algorithms are used
────────────────────────────
  Explorer publishes /goal_pose  →  A* (or Dijkstra) computes the path
                                    and publishes it to /astar/path (or /dijkstra/path)
  Explorer follows the waypoints →  but yields to DWA or APF for
                                    obstacle avoidance (reads /apf_active or /dwa_active flag)

Run order (Terminal 3):
  ros2 run cleaning_robot_algorithms vacuum &
  ros2 run cleaning_robot_algorithms astar &      # or dijkstra
  ros2 run cleaning_robot_algorithms dwa &        # or apf
  ros2 run cleaning_robot_algorithms explorer
"""

import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import String

TARGETS = [
    ('dust_1',  -5.0,  1.0),
    ('dust_2',   3.0, -2.0),
    ('dust_3',  -2.0, -5.0),
    ('trash_1', -6.0, -5.0),
    ('trash_2',  4.0,  6.0),
    ('trash_3',  8.0, -2.0),
]

EXPLORE_TIME     = 90.0
OBSTACLE_DIST    = 0.50
SIDE_CLEAR       = 0.60
FORWARD_SPEED    = 0.28
TURN_SPEED       = 1.0
WAYPOINT_TOL     = 0.35   # metres — close enough to a waypoint
GOAL_TOLERANCE   = 0.45   # metres — close enough to final target
GOAL_TIMEOUT     = 60.0
STUCK_TIMEOUT    = 3.5
STUCK_MOVE_THR   = 0.05
MAP_READY_CELLS  = 300


class AutonomousExplorer(Node):

    def __init__(self):
        super().__init__('autonomous_explorer')

        # ── Subscriptions ──────────────────────────────────────────────
        self.scan_sub    = self.create_subscription(LaserScan,     '/scan',         self._scan_cb,  10)
        self.odom_sub    = self.create_subscription(Odometry,      '/odom',         self._odom_cb,  10)
        self.map_sub     = self.create_subscription(OccupancyGrid, '/map',          self._map_cb,   1)
        self.status_sub  = self.create_subscription(String,        '/vacuum/status',self._status_cb,10)
        # Subscribe to BOTH planner outputs — whichever is running will publish
        self.astar_sub   = self.create_subscription(Path, '/astar/path',    self._path_cb, 1)
        self.dijkstra_sub= self.create_subscription(Path, '/dijkstra/path', self._path_cb, 1)

        # ── Publications ───────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(Twist,       '/cmd_vel',   10)
        self.goal_pub   = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ── State ──────────────────────────────────────────────────────
        self.scan       = None
        self.rx = -5.0; self.ry = 0.0; self.ryaw = 0.0
        self.map_known  = 0
        self.collected  = 0
        self.targets    = list(TARGETS)

        self.phase       = 'explore'
        self.goal_idx    = 0
        self.goal_start  = None

        # Path following
        self.current_path     = []   # list of (wx, wy) waypoints from A*/Dijkstra
        self.waypoint_idx     = 0
        self.path_requested   = False
        self.last_goal_pub    = 0.0
        self.local_ctrl_active = False  # True when DWA/APF is handling cmd_vel

        # Stuck detection
        self.last_check_pos   = (-5.0, 0.0)
        self.last_check_time  = time.time()
        self.escaping         = False
        self.escape_start     = 0.0
        self.escape_phase     = 0

        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            'Explorer v3 started.\n'
            '  Phase 1: autonomous wall-following exploration (90s)\n'
            '  Phase 2: A*/Dijkstra plans path to each target,\n'
            '           DWA/APF handles local obstacle avoidance\n'
            '  Make sure these nodes are also running:\n'
            '    ros2 run cleaning_robot_algorithms astar  (or dijkstra)\n'
            '    ros2 run cleaning_robot_algorithms dwa    (or apf)\n'
            '    ros2 run cleaning_robot_algorithms vacuum'
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _scan_cb(self, msg):  self.scan = msg

    def _odom_cb(self, msg):
        self.rx   = msg.pose.pose.position.x
        self.ry   = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.ryaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                               1 - 2*(q.y*q.y + q.z*q.z))

    def _map_cb(self, msg):
        self.map_known = sum(1 for c in msg.data if c >= 0)

    def _status_cb(self, msg):
        try:
            self.collected = int(msg.data.split('Collected:')[1].split('/')[0].strip())
        except Exception:
            pass

    def _path_cb(self, msg):
        """Receive planned path from A* or Dijkstra."""
        if not msg.poses:
            self.get_logger().warn('Received empty path from planner')
            return
        self.current_path   = [(p.pose.position.x, p.pose.position.y)
                                for p in msg.poses]
        self.waypoint_idx   = 0
        self.path_requested = False
        src = 'A*' if '/astar' in (msg.header.frame_id or '') else 'planner'
        self.get_logger().info(
            f'Path received: {len(self.current_path)} waypoints from {src}')

    # ── Main tick ────────────────────────────────────────────────────────

    def _tick(self):
        if self.scan is None:
            return

        if self.escaping:
            self._do_escape()
            return

        elapsed = time.time() - self.start_time

        if self.phase == 'explore':
            if self.map_known >= MAP_READY_CELLS and elapsed >= EXPLORE_TIME:
                self.get_logger().info(
                    f'Map ready ({self.map_known} cells known). '
                    'Switching to CLEAN phase. A*/Dijkstra will plan routes.')
                self.phase      = 'clean'
                self.goal_start = time.time()
                self.last_check_pos  = (self.rx, self.ry)
                self.last_check_time = time.time()
            else:
                self._check_stuck()
                self._explore()

        elif self.phase == 'clean':
            if self.goal_idx >= len(self.targets):
                self._stop()
                self.phase = 'done'
                self.get_logger().info(
                    f'ALL {len(TARGETS)} targets visited! '
                    f'Collected={self.collected}/{len(TARGETS)}. Room is clean.')
                return
            self._check_stuck()
            self._clean_phase()

    # ── Phase 1: wall-following exploration ─────────────────────────────

    def _explore(self):
        scan = self.scan
        ranges = scan.ranges
        n = len(ranges)

        def sector_min(s, e):
            vals = []
            for deg in range(s, e):
                i = int((math.radians(deg % 360) - scan.angle_min)
                        / scan.angle_increment)
                if 0 <= i < n:
                    r = ranges[i]
                    if not(math.isinf(r) or math.isnan(r)) and r > 0.01:
                        vals.append(r)
            return min(vals) if vals else 10.0

        front  = min(sector_min(350,360), sector_min(0,10))
        fl     = sector_min(10,40)
        fr     = sector_min(320,350)
        right  = sector_min(255,285)

        cmd = Twist()
        if front < OBSTACLE_DIST or fl < OBSTACLE_DIST or fr < OBSTACLE_DIST:
            cmd.linear.x  = 0.0
            cmd.angular.z = TURN_SPEED
        elif right > SIDE_CLEAR * 1.6:
            cmd.linear.x  = FORWARD_SPEED
            cmd.angular.z = -TURN_SPEED * 0.5
        elif right < SIDE_CLEAR * 0.5:
            cmd.linear.x  = FORWARD_SPEED * 0.8
            cmd.angular.z =  TURN_SPEED * 0.4
        else:
            cmd.linear.x  = FORWARD_SPEED
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    # ── Phase 2: use A*/Dijkstra path + DWA/APF obstacle avoidance ──────

    def _clean_phase(self):
        name, tx, ty = self.targets[self.goal_idx]
        dist_to_target = math.hypot(self.rx - tx, self.ry - ty)

        # ── Reached target ─────────────────────────────────────────────
        if dist_to_target <= GOAL_TOLERANCE:
            self.get_logger().info(
                f'Reached {name} (dist={dist_to_target:.2f}m)')
            self._advance_target()
            return

        # ── Timeout ────────────────────────────────────────────────────
        if time.time() - self.goal_start > GOAL_TIMEOUT:
            self.get_logger().warn(
                f'Timeout on {name} at dist={dist_to_target:.2f}m. Skipping.')
            self._advance_target()
            return

        # ── Request a path from A*/Dijkstra if not already done ────────
        if not self.current_path and not self.path_requested:
            self._publish_goal(tx, ty, name)
            return   # wait for path to arrive

        # ── If still waiting for path, drive directly (fallback) ───────
        # Request a path from A*/Dijkstra if not already done
        if not self.current_path and not self.path_requested:
            self._publish_goal(tx, ty, name)
            return

        # If waiting too long for path → give up, use APF fallback
        if not self.current_path and self.path_requested:
            wait_time = time.time() - self.last_goal_pub
            if wait_time < 5.0:
                return   # still waiting
            else:
                # APF is already running and has the goal — just let it drive
                self.get_logger().info(
                    f'No path from planner for {name} — APF driving directly',
                    throttle_duration_sec=3.0)
                # Don't publish cmd_vel — APF owns it now
                return

        # ── Follow the planned path waypoint by waypoint ───────────────
        # DWA/APF is running separately and handling obstacle avoidance
        # via /cmd_vel. Explorer only adjusts heading toward next waypoint
        # when the path is clear — DWA/APF overrides when obstacles appear.
        self._follow_path(tx, ty, name, dist_to_target)

    def _publish_goal(self, tx, ty, name):
        """Send goal to A* or Dijkstra planner."""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = tx
        msg.pose.position.y = ty
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.path_requested  = True
        self.last_goal_pub   = time.time()
        self.get_logger().info(
            f'Published /goal_pose for {name} at ({tx},{ty}) '
            f'→ waiting for A*/Dijkstra path...')

    def _follow_path(self, tx, ty, name, dist_to_target):
        """
        Follow waypoints from A*/Dijkstra.
        Explorer steers toward each waypoint.
        DWA/APF (running in parallel) handles obstacle avoidance.
        """
        if self.waypoint_idx >= len(self.current_path):
            # Exhausted all waypoints but not at target — drive direct
            self._drive_direct(tx, ty, name, dist_to_target)
            return

        wx, wy = self.current_path[self.waypoint_idx]
        dist_to_wp = math.hypot(self.rx - wx, self.ry - wy)

        # Advance to next waypoint if close enough
        if dist_to_wp < WAYPOINT_TOL:
            self.waypoint_idx += 1
            self.get_logger().info(
                f'Waypoint {self.waypoint_idx}/{len(self.current_path)} reached',
                throttle_duration_sec=1.0)
            return

        # Check if path ahead is clear
        scan = self.scan
        ranges = scan.ranges
        n = len(ranges)
        front_min = 10.0
        for deg in range(340, 380):
            i = int((math.radians(deg % 360) - scan.angle_min)
                    / scan.angle_increment)
            if 0 <= i < n:
                r = ranges[i]
                if not(math.isinf(r) or math.isnan(r)) and r > 0.01:
                    front_min = min(front_min, r)

        angle_to_wp  = math.atan2(wy - self.ry, wx - self.rx)
        angle_error  = self._wrap(angle_to_wp - self.ryaw)

        cmd = Twist()

        if front_min < OBSTACLE_DIST:
            # Obstacle — DWA/APF is already publishing avoidance commands
            # Explorer yields: publish zero so DWA/APF commands dominate
            # (last publisher wins in ROS2, so we just stop publishing)
            self.get_logger().info(
                f'Obstacle at {front_min:.2f}m — DWA/APF handling avoidance',
                throttle_duration_sec=1.5)
            return   # do NOT publish — let DWA/APF take over
        else:
            # Path clear — explorer steers toward next waypoint
            if abs(angle_error) > 0.35:
                cmd.linear.x  = FORWARD_SPEED * 0.4
                cmd.angular.z = TURN_SPEED * 0.8 * (1.0 if angle_error > 0 else -1.0)
            else:
                cmd.linear.x  = FORWARD_SPEED
                cmd.angular.z = max(-0.8, min(0.8, angle_error * 1.5))
            self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Following path to {name} | wp {self.waypoint_idx+1}/{len(self.current_path)} '
            f'| target dist={dist_to_target:.2f}m | cleaned={self.collected}/{len(TARGETS)}',
            throttle_duration_sec=2.0)

    def _drive_direct(self, tx, ty, name, dist):
        """Fallback: drive directly to target when no path available."""
        scan = self.scan
        ranges = scan.ranges
        n = len(ranges)
        front_min = 10.0
        for deg in range(340, 380):
            i = int((math.radians(deg % 360) - scan.angle_min)
                    / scan.angle_increment)
            if 0 <= i < n:
                r = ranges[i]
                if not(math.isinf(r) or math.isnan(r)) and r > 0.01:
                    front_min = min(front_min, r)

        angle_to_goal = math.atan2(ty - self.ry, tx - self.rx)
        angle_error   = self._wrap(angle_to_goal - self.ryaw)
        cmd = Twist()

        if front_min < OBSTACLE_DIST:
            cmd.linear.x  = 0.0
            cmd.angular.z = TURN_SPEED if angle_error >= 0 else -TURN_SPEED
        elif abs(angle_error) > 0.4:
            cmd.linear.x  = FORWARD_SPEED * 0.3
            cmd.angular.z = TURN_SPEED * 0.9 * (1.0 if angle_error > 0 else -1.0)
        else:
            cmd.linear.x  = FORWARD_SPEED
            cmd.angular.z = max(-1.0, min(1.0, angle_error * 1.5))
        self.cmd_pub.publish(cmd)

    def _advance_target(self):
        self.goal_idx   += 1
        self.goal_start  = time.time()
        self.current_path = []
        self.waypoint_idx = 0
        self.path_requested = False
        self.last_check_pos  = (self.rx, self.ry)
        self.last_check_time = time.time()
        self._stop()

    # ── Stuck detection + escape ─────────────────────────────────────────

    def _check_stuck(self):
        now = time.time()
        if now - self.last_check_time >= STUCK_TIMEOUT:
            moved = math.hypot(self.rx - self.last_check_pos[0],
                               self.ry - self.last_check_pos[1])
            self.last_check_pos  = (self.rx, self.ry)
            self.last_check_time = now
            if moved < STUCK_MOVE_THR:
                self.get_logger().warn(
                    f'STUCK (moved {moved:.3f}m). Escaping...')
                # Reset path so we re-request after escape
                self.current_path   = []
                self.path_requested = False
                self.escaping     = True
                self.escape_start = now
                self.escape_phase = 0

    def _do_escape(self):
        now  = time.time()
        elapsed = now - self.escape_start
        cmd = Twist()
        if self.escape_phase == 0:
            if elapsed < 1.5:
                cmd.linear.x  = -FORWARD_SPEED
                cmd.angular.z =  TURN_SPEED * 0.5
            else:
                self.escape_phase = 1
                self.escape_start = now
        if self.escape_phase == 1:
            if elapsed < 2.0:
                cmd.linear.x  = 0.0
                cmd.angular.z = TURN_SPEED * 1.2
            else:
                self.escaping = False
                self.last_check_pos  = (self.rx, self.ry)
                self.last_check_time = time.time()
                self.get_logger().info('Escape done. Resuming.')
        self.cmd_pub.publish(cmd)

    # ── Helpers ──────────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.3)

    @staticmethod
    def _wrap(a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a


def main():
    rclpy.init()
    node = AutonomousExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()