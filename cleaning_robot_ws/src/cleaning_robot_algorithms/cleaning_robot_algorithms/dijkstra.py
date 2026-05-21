#!/usr/bin/env python3
import rclpy, heapq, time, json
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
LOG = '/tmp/algo_results.jsonl'

class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__('dijkstra_planner')
        self.map_sub  = self.create_subscription(OccupancyGrid, '/map',  self._map_cb,  1)
        self.odom_sub = self.create_subscription(Odometry,      '/odom', self._odom_cb, 10)
        self.path_pub = self.create_publisher(Path,        '/dijkstra/path', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 1)
        self._map = None
        self.rx = 0.0
        self.ry = 0.0
        self.get_logger().info("Dijkstra planner ready — using robot odom position as start")

    def _map_cb(self, msg):  self._map = msg

    def _odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y

    def _goal_cb(self, goal):
        if not self._map: return
        info = self._map.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        W, H = info.width, info.height
        data = self._map.data

        def world_to_cell(wx, wy):
            return (int((wx - ox) / res), int((wy - oy) / res))
        def cell_to_world(cx, cy):
            return (ox + cx*res + res/2, oy + cy*res + res/2)
        def free(cx, cy):
            if cx < 0 or cy < 0 or cx >= W or cy >= H: return False
            return data[cy*W + cx] < 50

        # Use robot's actual current position as start
        start = world_to_cell(self.rx, self.ry)
        end   = world_to_cell(goal.pose.position.x, goal.pose.position.y)

        self.get_logger().info(
            f"Dijkstra: from robot pos ({self.rx:.1f},{self.ry:.1f}) "
            f"cell={start} → goal cell={end}")

        t0 = time.time()
        dist_map = {start: 0}
        came_from = {}
        pq = [(0, start)]
        visited = 0

        # Clamp end cell to map bounds — handles targets outside explored area
        end = (max(0, min(W-1, end[0])), max(0, min(H-1, end[1])))
        
        # Also ensure end cell is free — find nearest free cell if not
        if not free(*end):
            best = None
            best_dist = 1e9
            for dx in range(-5, 6):
                for dy in range(-5, 6):
                    nc = (end[0]+dx, end[1]+dy)
                    if free(*nc):
                        d = abs(dx)+abs(dy)
                        if d < best_dist:
                            best_dist = d
                            best = nc
            if best:
                self.get_logger().info(f'Goal clamped: {end} → {best}')
                end = best
            else:
                self.get_logger().warn('No free cell near goal')
                return

        while pq:
            d, cur = heapq.heappop(pq)
            visited += 1
            if cur == end:
                path_cells = []
                node = cur
                while node in came_from:
                    path_cells.append(node); node = came_from[node]
                path_cells.reverse()
                dt = time.time() - t0
                ros_path = Path()
                ros_path.header.frame_id = 'map'
                ros_path.header.stamp = self.get_clock().now().to_msg()
                for (cx, cy) in path_cells:
                    ps = PoseStamped()
                    ps.header = ros_path.header
                    wx, wy = cell_to_world(cx, cy)
                    ps.pose.position.x = wx
                    ps.pose.position.y = wy
                    ps.pose.orientation.w = 1.0
                    ros_path.poses.append(ps)
                self.path_pub.publish(ros_path)
                result = {"algorithm":"Dijkstra","path_cells":len(path_cells),
                          "path_distance_m":round(len(path_cells)*res,3),
                          "time_s":round(dt,4),"nodes_visited":visited}
                self.get_logger().info(str(result))
                with open(LOG,'a') as f: f.write(json.dumps(result)+'\n')
                return

            for dx, dy in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nb = (cur[0]+dx, cur[1]+dy)
                if not free(*nb): continue
                nd = d + (1.414 if abs(dx)+abs(dy)==2 else 1.0)
                if nd < dist_map.get(nb, 1e18):
                    dist_map[nb] = nd; came_from[nb] = cur
                    heapq.heappush(pq, (nd, nb))

        self.get_logger().warn(
            f"Dijkstra: no path found from {start} to {end} "
            f"(visited {visited} nodes, map has {W}x{H} cells, "
            f"{sum(1 for c in data if c==0)} free cells)")

def main():
    rclpy.init()
    rclpy.spin(DijkstraPlanner())
    rclpy.shutdown()
