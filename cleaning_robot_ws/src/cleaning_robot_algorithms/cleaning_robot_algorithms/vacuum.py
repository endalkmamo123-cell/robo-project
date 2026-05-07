#!/usr/bin/env python3
"""
Vacuum collector node.
Monitors robot position via /odom and checks proximity to
dust/trash targets defined in the office world.
When the robot drives within SUCTION_RADIUS of a target,
it 'collects' it and logs the event.
"""
import rclpy, math, json
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

SUCTION_RADIUS = 0.35   # metres — how close robot must be to collect
LOG = '/tmp/vacuum_log.jsonl'

# All cleaning targets from the world file
TARGETS = {
    'dust_1':  (-5.0,  1.0),
    'dust_2':  ( 3.0, -2.0),
    'dust_3':  (-2.0, -5.0),
    'trash_1': (-6.0, -5.0),
    'trash_2': ( 4.0,  6.0),
    'trash_3': ( 8.0, -2.0),
}

class VacuumCollector(Node):
    def __init__(self):
        super().__init__('vacuum_collector')
        self.odom_sub     = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.status_pub   = self.create_publisher(String, '/vacuum/status', 10)
        self.remaining    = dict(TARGETS)   # targets not yet collected
        self.collected    = []
        self.get_logger().info(
            f"Vacuum ready — tracking {len(self.remaining)} targets")

    def _odom_cb(self, msg):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y

        for name, (tx, ty) in list(self.remaining.items()):
            dist = math.hypot(rx - tx, ry - ty)
            if dist <= SUCTION_RADIUS:
                self._collect(name, dist)

        # Publish live status
        status = String()
        status.data = (
            f"Collected: {len(self.collected)}/{len(TARGETS)} | "
            f"Remaining: {list(self.remaining.keys())}"
        )
        self.status_pub.publish(status)

    def _collect(self, name, dist):
        self.collected.append(name)
        del self.remaining[name]
        entry = {
            "event": "collected",
            "target": name,
            "distance_m": round(dist, 3),
            "total_collected": len(self.collected),
            "remaining": len(self.remaining)
        }
        self.get_logger().info(
            f"VACUUMED: {name} (dist={dist:.2f}m) | "
            f"{len(self.collected)}/{len(TARGETS)} done")
        with open(LOG, 'a') as f:
            f.write(json.dumps(entry) + '\n')

        if not self.remaining:
            self.get_logger().info("ALL TARGETS COLLECTED — room is clean!")

def main():
    rclpy.init()
    rclpy.spin(VacuumCollector())
    rclpy.shutdown()
