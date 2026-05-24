#!/usr/bin/env python3
"""
Dynamic Window Approach — local obstacle avoidance.
Only activates when the forward path is blocked; stays silent otherwise
so the Explorer can drive unimpeded.
Goal-aware: subscribes to /goal_pose and /odom.
"""
import rclpy, math, time, json
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

LOG = '/tmp/local_planner_log.jsonl'

class DWAController(Node):
    def __init__(self):
        super().__init__('dwa_controller')
        self.scan_sub = self.create_subscription(LaserScan,   '/scan',      self._scan_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)
        self.odom_sub = self.create_subscription(Odometry,    '/odom',      self._odom_cb, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        self.max_v      = 0.55   # m/s
        self.max_w      = 1.4    # rad/s
        self.safe_r     = 0.28   # reject trajectory if clearance < this (≈ robot radius + margin)
        self.activate_r = 0.38   # only publish when forward dist < this

        self.goal = None
        self.rx = 0.0; self.ry = 0.0; self.ryaw = 0.0
        self.cycles = 0
        self.get_logger().info("DWA controller ready — publishes to /cmd_vel")

    def _goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def _odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.ryaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def _scan_cb(self, msg):
        # Only take over when forward path is actually blocked
        if self._arc_clearance(msg, 0.0, arc_deg=20) > self.activate_r:
            return  # clear ahead — stay silent, let Explorer drive

        t0 = time.time()
        best_v, best_w, best_score = 0.0, self.max_w, -1e9
        for v in [round(i*0.055, 3) for i in range(11)]:   # 0.0 .. 0.55
            for w in [round(i*0.28 - 1.4, 3) for i in range(11)]:  # -1.4 .. 1.4
                score = self._score(v, w, msg)
                if score > best_score:
                    best_score = score; best_v = v; best_w = w

        # If every trajectory is blocked, stay silent — let Explorer steer
        if best_score < -100:
            return

        cmd = Twist()
        cmd.linear.x  = best_v
        cmd.angular.z = best_w
        self.cmd_pub.publish(cmd)

        self.cycles += 1
        if self.cycles % 50 == 0:
            dt = time.time() - t0
            entry = {"algorithm": "DWA", "cycle": self.cycles,
                     "cycle_ms": round(dt*1000, 2),
                     "v": best_v, "w": best_w, "score": round(best_score, 3)}
            with open(LOG, 'a') as f:
                f.write(json.dumps(entry) + '\n')
            self.get_logger().info(f"DWA cycle {self.cycles}: v={best_v} w={best_w}")

    def _arc_clearance(self, scan, heading_offset, arc_deg=25):
        """Min lidar distance in an arc centred on heading_offset (robot frame, radians)."""
        n = len(scan.ranges)
        min_dist = 10.0
        for deg in range(-arc_deg, arc_deg + 1):
            angle = math.radians(deg) + heading_offset
            i = round((angle - scan.angle_min) / scan.angle_increment)
            if 0 <= i < n:
                r = scan.ranges[i]
                if not (math.isinf(r) or math.isnan(r)) and r > 0.01:
                    min_dist = min(min_dist, r)
        return min_dist

    def _score(self, v, w, scan):
        # Check clearance in the direction this (v, w) pair would face after 0.5 s
        future_heading = w * 0.5
        clearance = self._arc_clearance(scan, future_heading, arc_deg=25)
        if clearance < self.safe_r:
            return -1000.0

        heading_score   = v * 2.0
        clearance_score = min(clearance, 2.0) * 0.5

        # Prefer turning toward the current goal
        goal_score = 0.0
        if self.goal is not None:
            goal_angle = math.atan2(self.goal[1] - self.ry, self.goal[0] - self.rx)
            future_yaw = self.ryaw + future_heading
            angle_diff = abs(self._wrap(goal_angle - future_yaw))
            goal_score = (math.pi - angle_diff) / math.pi * 2.0

        return heading_score + clearance_score + goal_score

    @staticmethod
    def _wrap(a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a


def main():
    rclpy.init()
    rclpy.spin(DWAController())
    rclpy.shutdown()
