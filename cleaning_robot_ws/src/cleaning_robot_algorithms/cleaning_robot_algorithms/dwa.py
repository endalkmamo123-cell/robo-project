#!/usr/bin/env python3
"""
Dynamic Window Approach — local obstacle avoidance.
Reads /scan (from ros_gz_bridge), publishes /cmd_vel.
Logs cycle time and chosen velocity every 50 cycles.
"""
import rclpy, math, time, json
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

LOG = '/tmp/local_planner_log.jsonl'

class DWAController(Node):
    def __init__(self):
        super().__init__('dwa_controller')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        # DWA parameters
        self.max_v   = 0.25   # m/s
        self.max_w   = 1.0    # rad/s
        self.safe_r  = 0.35   # metres clearance
        self.goal_x  = 3.0   # simple forward goal
        self.goal_y  = 0.0
        self.cycles  = 0
        self.get_logger().info("DWA controller ready — publishes to /cmd_vel")

    def _scan_cb(self, msg):
        t0 = time.time()
        best_v, best_w, best_score = 0.0, 0.0, -1e9
        # Sample velocity space (dynamic window)
        for v in [round(i*0.05,2) for i in range(6)]:            # 0.0 .. 0.25
            for w in [round(i*0.2-1.0,2) for i in range(11)]:   # -1.0 .. 1.0
                score = self._score(v, w, msg)
                if score > best_score:
                    best_score = score; best_v = v; best_w = w

        cmd = Twist()
        cmd.linear.x  = best_v
        cmd.angular.z = best_w
        self.cmd_pub.publish(cmd)

        self.cycles += 1
        if self.cycles % 50 == 0:
            dt = time.time() - t0
            entry = {"algorithm":"DWA","cycle":self.cycles,
                     "cycle_ms":round(dt*1000,2),
                     "v":best_v,"w":best_w,"score":round(best_score,3)}
            with open(LOG, 'a') as f:
                    f.write(json.dumps(entry) + '\n')
            self.get_logger().info(f"DWA cycle {self.cycles}: v={best_v} w={best_w}")

    def _score(self, v, w, scan):
        # Clearance: reject if any obstacle inside safe radius
        min_r = min(
            (r for r in scan.ranges if not math.isinf(r) and not math.isnan(r) and r > 0.01),
            default=10.0)
        if min_r < self.safe_r:
            return -1000.0
        heading_score  = v * 2.0            # prefer forward motion
        clearance_score= min_r * 0.5        # prefer distance from obstacles
        return heading_score + clearance_score

def main():
    rclpy.init()
    rclpy.spin(DWAController())
    rclpy.shutdown()

