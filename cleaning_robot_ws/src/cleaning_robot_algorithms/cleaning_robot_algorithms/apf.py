#!/usr/bin/env python3
"""
Artificial Potential Field — local obstacle avoidance.
Attractive force toward goal_pose, repulsive from /scan obstacles.
"""
import rclpy, math, time, json
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

LOG = '/tmp/local_planner_log.jsonl'

class APFController(Node):
    def __init__(self):
        super().__init__('apf_controller')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 1)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        # APF gains
        self.k_att  = 0.8    # attractive gain
        self.k_rep  = 0.6    # repulsive gain
        self.d0     = 0.9    # obstacle influence radius (m)
        self.max_v  = 0.25
        self.goal   = None
        self.cycles = 0
        self.get_logger().info("APF controller ready")

    def _goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"APF goal set: {self.goal}")

    def _scan_cb(self, msg):
        # Always drive toward (3,0) if no explicit goal
        gx, gy = self.goal if self.goal else (3.0, 0.0)
        t0 = time.time()

        # Attractive force (robot frame: goal direction)
        fx = self.k_att * gx   # simplified: goal in robot frame
        fy = self.k_att * gy

        # Repulsive force from each scan beam
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r <= 0.01 or r > self.d0:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            ox = r * math.cos(angle)
            oy = r * math.sin(angle)
            factor = self.k_rep * (1.0/r - 1.0/self.d0) / (r**2 + 1e-6)
            dist   = math.hypot(ox, oy) + 1e-9
            fx -= factor * ox / dist
            fy -= factor * oy / dist

        # Convert net force to velocity command
        force_mag = math.hypot(fx, fy)
        cmd = Twist()
        cmd.linear.x  = min(force_mag * 0.3, self.max_v)
        cmd.angular.z = max(min(math.atan2(fy, fx), 1.5), -1.5)
        self.cmd_pub.publish(cmd)

        self.cycles += 1
        if self.cycles % 50 == 0:
            dt = time.time() - t0
            entry = {"algorithm":"APF","cycle":self.cycles,
                     "cycle_ms":round(dt*1000,2),
                     "fx":round(fx,3),"fy":round(fy,3),
                     "v":round(cmd.linear.x,3),"w":round(cmd.angular.z,3)}
            with open(LOG, 'a') as f:
                    f.write(json.dumps(entry) + '\n')

def main():
    rclpy.init()
    rclpy.spin(APFController())
    rclpy.shutdown()