#!/usr/bin/env python3
"""
Pedestrian obstacle-avoidance controller.

Drives person_1 and person_2 (physics-based VelocityControl models in Gazebo)
along patrol routes using a potential-field algorithm:
  - Goal attraction  : pulls each person toward their next waypoint
  - Wall repulsion   : pushes away from every wall segment in the room
  - Furniture        : pushes away from furniture bounding boxes
  - Mutual repulsion : persons keep distance from each other

Because the Gazebo models have real collision geometry, the physics engine
also prevents penetration — the potential field just steers them smoothly.

Topics
------
Subscriptions : /model/person_{1,2}/odometry  (nav_msgs/Odometry)
Publications  : /model/person_{1,2}/cmd_vel   (geometry_msgs/Twist)
"""

import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── Patrol waypoints (each person loops through these in order) ────────────
# person_1: east-west patrol through the doorway gap (y = -2, gap y ∈ [-3,-1])
# person_2: north-south patrol in the right zone at x = 3 (clear of furniture)
WAYPOINTS = {
    'person_1': [
        (-5.5, -2.0),   # left zone start
        (-0.5, -2.0),   # approach doorway
        ( 0.5, -2.0),   # through doorway
        ( 6.0, -2.0),   # right zone far end
        ( 0.5, -2.0),   # back through doorway
        (-0.5, -2.0),
    ],
    'person_2': [
        (3.0,  6.0),    # north end of right zone
        (3.0, -3.0),    # south end (clear of reception desk at y=-4)
    ],
}

# ── Room wall segments  (x1, y1, x2, y2) ──────────────────────────────────
WALLS = [
    (-10.0, -7.5,  10.0, -7.5),   # south outer wall
    (-10.0,  7.5,  10.0,  7.5),   # north outer wall
    (-10.0, -7.5, -10.0,  7.5),   # west outer wall
    ( 10.0, -7.5,  10.0,  7.5),   # east outer wall
    (   0.0, -1.0,   0.0,  7.5),  # divider left  (gap is y ∈ [-3, -1])
    (   0.0, -7.5,   0.0, -3.0),  # divider right
]

# ── Furniture as axis-aligned bounding boxes (xmin, xmax, ymin, ymax) ─────
FURNITURE = [
    (-7.8, -6.2,  4.6,  5.4),  # desk_1
    (-4.8, -3.2,  4.6,  5.4),  # desk_2
    (-7.8, -6.2, -2.4, -1.6),  # desk_3
    (-6.6, -5.4, -2.9, -2.5),  # chair_3 (nearest to person_1 path)
    (-9.3, -8.7, -0.3,  0.3),  # cabinet_1
    (-9.3, -8.7,  0.7,  1.3),  # cabinet_2
    ( 4.5,  7.5,  2.3,  3.7),  # conference table
    ( 4.6,  5.0,  2.75, 3.25), # mchair_1
    ( 5.75, 6.25, 4.25, 4.75), # mchair_2
    ( 6.95, 7.45, 2.75, 3.25), # mchair_3
    ( 4.75, 7.25, -4.35,-3.65),# reception desk
    ( 6.4,  8.6, -6.9, -6.1),  # sofa
    ( 7.0,  8.0, -5.45,-4.95), # coffee table
]

# ── Tuning parameters ─────────────────────────────────────────────────────
MAX_V          = 0.7    # m/s  — forward speed cap
MAX_W          = 1.8    # rad/s — turning speed cap
K_GOAL         = 1.4    # goal attraction gain
K_WALL         = 0.8    # wall / furniture repulsion gain
K_PERSON       = 1.0    # inter-person repulsion gain
D0             = 1.2    # influence radius for walls/furniture (m)
D0_PERSON      = 1.5    # influence radius for the other person (m)
WAYPOINT_TOL   = 0.45   # m — advance waypoint when this close
DT             = 0.1    # s — control period


# ── Geometry helpers ──────────────────────────────────────────────────────

def _dist_to_segment(px, py, x1, y1, x2, y2):
    """Signed distance and outward unit normal from point to line segment."""
    dx, dy = x2 - x1, y2 - y1
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        d = math.hypot(px - x1, py - y1)
        nx, ny = (px - x1, py - y1)
    else:
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))
        cx, cy = x1 + t * dx, y1 + t * dy
        d = math.hypot(px - cx, py - cy)
        nx, ny = px - cx, py - cy
    mag = math.hypot(nx, ny)
    if mag > 1e-9:
        nx, ny = nx / mag, ny / mag
    else:
        nx, ny = 0.0, 1.0
    return d, nx, ny


def _dist_to_aabb(px, py, xmin, xmax, ymin, ymax):
    """Distance and outward normal from point to AABB."""
    cx_box = 0.5 * (xmin + xmax)
    cy_box = 0.5 * (ymin + ymax)
    cpx = max(xmin, min(xmax, px))
    cpy = max(ymin, min(ymax, py))
    if cpx == px and cpy == py:   # point is inside box
        dx = min(px - xmin, xmax - px)
        dy = min(py - ymin, ymax - py)
        if dx < dy:
            return 0.0, (-1.0 if px < cx_box else 1.0), 0.0
        else:
            return 0.0, 0.0, (-1.0 if py < cy_box else 1.0)
    d = math.hypot(px - cpx, py - cpy)
    nx = (px - cpx) / d
    ny = (py - cpy) / d
    return d, nx, ny


# ── Controller node ───────────────────────────────────────────────────────

class PedestrianController(Node):

    def __init__(self):
        super().__init__('pedestrian_controller')

        self._pubs   = {}   # name → Twist publisher
        self._poses  = {}   # name → (x, y, yaw)
        self._wp_idx = {}   # name → current waypoint index
        self._got_odom = {} # name → bool

        for name, wps in WAYPOINTS.items():
            self._pubs[name] = self.create_publisher(
                Twist, f'/model/{name}/cmd_vel', 10)
            self._poses[name] = (wps[0][0], wps[0][1], 0.0)
            self._wp_idx[name] = 0
            self._got_odom[name] = False

            self.create_subscription(
                Odometry,
                f'/model/{name}/odometry',
                lambda msg, n=name: self._odom_cb(msg, n),
                10,
            )

        self.create_timer(DT, self._tick)
        self.get_logger().info(
            'Pedestrian controller ready — waiting for odometry...')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg, name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._poses[name] = (x, y, yaw)
        if not self._got_odom[name]:
            self._got_odom[name] = True
            self.get_logger().info(f'{name} odometry active at ({x:.1f},{y:.1f})')

    # ── Main tick ─────────────────────────────────────────────────────────

    def _tick(self):
        for name in WAYPOINTS:
            if self._got_odom[name]:
                self._step(name)

    def _step(self, name):
        px, py, pyaw = self._poses[name]
        wps = WAYPOINTS[name]
        idx = self._wp_idx[name]
        gx, gy = wps[idx]

        # Advance waypoint when close enough
        if math.hypot(px - gx, py - gy) < WAYPOINT_TOL:
            idx = (idx + 1) % len(wps)
            self._wp_idx[name] = idx
            gx, gy = wps[idx]
            self.get_logger().info(
                f'{name} → waypoint {idx} ({gx:.1f},{gy:.1f})',
                throttle_duration_sec=2.0)

        # ── Goal attraction ───────────────────────────────────────────────
        dx, dy = gx - px, gy - py
        d_goal = math.hypot(dx, dy) + 1e-9
        fx = K_GOAL * dx / d_goal
        fy = K_GOAL * dy / d_goal

        # ── Wall repulsion ────────────────────────────────────────────────
        for seg in WALLS:
            d_w, nx, ny = _dist_to_segment(px, py, *seg)
            if 1e-6 < d_w < D0:
                mag = K_WALL * (1.0 / d_w - 1.0 / D0) / (d_w ** 2)
                fx += mag * nx
                fy += mag * ny

        # ── Furniture repulsion ───────────────────────────────────────────
        for box in FURNITURE:
            d_b, nx, ny = _dist_to_aabb(px, py, *box)
            if d_b < D0:
                d_b = max(d_b, 1e-6)
                mag = K_WALL * (1.0 / d_b - 1.0 / D0) / (d_b ** 2)
                fx += mag * nx
                fy += mag * ny

        # ── Inter-person repulsion ────────────────────────────────────────
        for other, (ox, oy, _) in self._poses.items():
            if other == name:
                continue
            d_p = math.hypot(px - ox, py - oy)
            if 1e-6 < d_p < D0_PERSON:
                mag = K_PERSON * (1.0 / d_p - 1.0 / D0_PERSON) / (d_p ** 2)
                fx += mag * (px - ox) / d_p
                fy += mag * (py - oy) / d_p

        # ── Force → velocity command ──────────────────────────────────────
        target_angle = math.atan2(fy, fx)
        angle_err    = self._wrap(target_angle - pyaw)
        force_mag    = math.hypot(fx, fy)

        cmd = Twist()
        # Slow forward while turning sharply, full speed when aligned
        cmd.linear.x  = min(force_mag * 0.4, MAX_V) * max(0.1, 1.0 - abs(angle_err) / math.pi)
        cmd.angular.z = max(-MAX_W, min(MAX_W, angle_err * 2.5))
        self._pubs[name].publish(cmd)

    @staticmethod
    def _wrap(a):
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a


def main():
    rclpy.init()
    node = PedestrianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for pub in node._pubs.values():
            pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()
