#!/usr/bin/env python3
"""
Vacuum collector node.
Monitors robot position via /odom and checks proximity to
dust/trash targets defined in the office world.
When the robot drives within SUCTION_RADIUS of a target,
it 'collects' it — logging the event AND calling the Gazebo
delete service so the entity visually disappears.
"""
import rclpy, math, json, threading
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ros_gz_interfaces.srv import DeleteEntity

# Must be >= GOAL_TOLERANCE in explorer.py (0.45 m) so the vacuum always
# triggers before the explorer considers itself "arrived" and stops.
SUCTION_RADIUS = 0.50   # metres
WORLD_NAME     = 'office_world'
LOG            = '/tmp/vacuum_log.jsonl'

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

        cb = ReentrantCallbackGroup()

        self.odom_sub   = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10, callback_group=cb)
        self.status_pub = self.create_publisher(String, '/vacuum/status', 10)

        self._delete_cli = self.create_client(
            DeleteEntity,
            f'/world/{WORLD_NAME}/remove',
            callback_group=cb,
        )

        self.remaining = dict(TARGETS)
        self.collected = []

        # Wait for the Gazebo delete service in a background thread so node
        # startup is not blocked — deletion will silently skip if not ready.
        threading.Thread(target=self._await_service, daemon=True).start()

        self.get_logger().info(f'Vacuum ready — tracking {len(self.remaining)} targets')

    def _await_service(self):
        if self._delete_cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().info('[gz] Entity delete service ready')
        else:
            self.get_logger().warn(
                '[gz] Entity delete service not available — items will not disappear')

    def _odom_cb(self, msg):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y

        for name, (tx, ty) in list(self.remaining.items()):
            if math.hypot(rx - tx, ry - ty) <= SUCTION_RADIUS:
                self._collect(name, rx, ry)

        status = String()
        status.data = (
            f'Collected: {len(self.collected)}/{len(TARGETS)} | '
            f'Remaining: {list(self.remaining.keys())}'
        )
        self.status_pub.publish(status)

    def _collect(self, name, rx, ry):
        self.collected.append(name)
        del self.remaining[name]

        self.get_logger().info(
            f'VACUUMED: {name} at robot ({rx:.2f}, {ry:.2f}) | '
            f'{len(self.collected)}/{len(TARGETS)} done')

        entry = {
            'event': 'collected', 'target': name,
            'robot_x': round(rx, 3), 'robot_y': round(ry, 3),
            'total_collected': len(self.collected),
            'remaining': len(self.remaining),
        }
        with open(LOG, 'a') as f:
            f.write(json.dumps(entry) + '\n')

        self._delete_entity(name)

        if not self.remaining:
            self.get_logger().info('ALL TARGETS COLLECTED — room is clean!')

    def _delete_entity(self, name):
        """Async delete of a Gazebo model via the bridged remove service."""
        if not self._delete_cli.service_is_ready():
            self.get_logger().warn(f'[gz] Delete service not ready — {name} stays visible')
            return

        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = 2  # Entity.MODEL

        future = self._delete_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_delete_done(name, f))

    def _on_delete_done(self, name, future):
        try:
            result = future.result()
            if result and result.success:
                self.get_logger().info(f'[gz] {name} removed from simulation ✓')
            else:
                self.get_logger().warn(f'[gz] {name} removal returned failure')
        except Exception as e:
            self.get_logger().warn(f'[gz] {name} removal error: {e}')


def main():
    rclpy.init()
    # MultiThreadedExecutor lets the service call run while odom callbacks continue
    from rclpy.executors import MultiThreadedExecutor
    node = VacuumCollector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
