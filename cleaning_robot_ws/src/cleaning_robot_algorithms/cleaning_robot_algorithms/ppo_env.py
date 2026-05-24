#!/usr/bin/env python3
"""
Gymnasium environment wrapping Gazebo Harmonic via ROS2 topics.
Used only during training (ppo_train.py). Not a ROS2 node entry point.

Prerequisites:
  - sim.launch.py must be running
  - pip install stable-baselines3[extra] gymnasium
"""
import math
import subprocess
import threading
import time

import gymnasium as gym
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

SUCTION_RADIUS = 0.35   # metres — must match vacuum.py
MAX_RANGE      = 10.0   # LiDAR max range (metres)
ROOM_DIAG      = 25.0   # normaliser for goal distance (~sqrt(20²+15²))
LIDAR_SAMPLES  = 36     # downsample 360 → 36 (every 10th beam)
OBS_DIM        = LIDAR_SAMPLES + 4   # 36 LiDAR + dist + angle + v + w = 40
MAX_STEPS      = 500
COLLISION_DIST = 0.15   # metres — min LiDAR reading before episode ends

SPAWN_X, SPAWN_Y, SPAWN_Z = -5.0, 0.0, 0.1

# All cleaning targets — must match room.sdf and vacuum.py
TARGETS = {
    'dust_1':  (-5.0,  1.0),
    'dust_2':  ( 3.0, -2.0),
    'dust_3':  (-2.0, -5.0),
    'trash_1': (-6.0, -5.0),
    'trash_2': ( 4.0,  6.0),
    'trash_3': ( 8.0, -2.0),
}


class _GazeboRosNode(Node):
    """Internal ROS2 node spun in a background thread."""

    def __init__(self):
        super().__init__('ppo_env_node')
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_scan  = None
        self.latest_odom  = None
        self.current_v    = 0.0
        self.current_w    = 0.0
        self._scan_event  = threading.Event()

    def _scan_cb(self, msg):
        self.latest_scan = msg
        self._scan_event.set()

    def _odom_cb(self, msg):
        self.latest_odom = msg
        self.current_v   = msg.twist.twist.linear.x
        self.current_w   = msg.twist.twist.angular.z

    def publish_cmd(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def wait_for_scan(self, timeout: float = 2.0) -> bool:
        got = self._scan_event.wait(timeout)
        self._scan_event.clear()
        return got


class CleaningRobotEnv(gym.Env):
    """
    Observation (40-dim Box):
      [0:36]  36 LiDAR beams, every 10th of 360, clipped to [0,10] normalised → [0,1]
      [36]    distance to nearest uncollected target, normalised by ROOM_DIAG
      [37]    angle to nearest uncollected target, normalised by π
      [38]    current linear velocity, normalised by 0.25
      [39]    current angular velocity, normalised by 1.0

    Action (2-dim Box):
      [0]  linear velocity  ∈ [0.0, 0.25]
      [1]  angular velocity ∈ [−1.0, 1.0]
    """

    metadata = {'render_modes': []}

    def __init__(self):
        super().__init__()

        self.observation_space = gym.spaces.Box(
            low=0.0, high=1.0, shape=(OBS_DIM,), dtype=np.float32)
        self.action_space = gym.spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([0.25,  1.0], dtype=np.float32),
            dtype=np.float32)

        # Start rclpy + spin in background thread
        if not rclpy.ok():
            rclpy.init()
        self._node = _GazeboRosNode()
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

        self._remaining   = {}
        self._step_count  = 0
        self._prev_dist   = 0.0

    # ------------------------------------------------------------------
    # Gym interface
    # ------------------------------------------------------------------

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # Reset Gazebo world (repositions all entities)
        subprocess.run(
            ['gz', 'service', '-s', '/world/default/reset',
             '--reqtype', 'gz.msgs.Empty',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000',
             '--req', '{}'],
            capture_output=True)

        time.sleep(1.5)   # wait for Gazebo to settle

        self._remaining  = dict(TARGETS)
        self._step_count = 0

        # Drain stale scan, then wait for a fresh one
        self._node._scan_event.clear()
        self._node.wait_for_scan(timeout=5.0)

        obs = self._get_obs()
        self._prev_dist = self._dist_to_nearest(self._robot_pos())
        return obs, {}

    def step(self, action):
        self._node.publish_cmd(float(action[0]), float(action[1]))

        # Wait for next LiDAR tick (≈100 ms at 10 Hz)
        self._node.wait_for_scan(timeout=2.0)

        self._step_count += 1
        rx, ry = self._robot_pos()
        min_scan = self._min_scan()

        reward = 0.0
        terminated = False
        truncated   = False

        # Progress toward nearest uncollected target
        curr_dist = self._dist_to_nearest(rx, ry)
        reward   += 1.0 * (self._prev_dist - curr_dist)
        self._prev_dist = curr_dist

        # Collection check
        for name, (tx, ty) in list(self._remaining.items()):
            if math.hypot(rx - tx, ry - ty) <= SUCTION_RADIUS:
                del self._remaining[name]
                reward += 5.0
                self._node.get_logger().info(f"PPO collected {name}")

        if not self._remaining:
            reward     += 20.0
            terminated  = True

        # Collision penalty
        if min_scan < COLLISION_DIST:
            reward    -= 5.0
            terminated = True

        # Time penalty
        reward -= 0.01

        # Step limit
        if self._step_count >= MAX_STEPS:
            truncated = True

        obs = self._get_obs()
        info = {'step': self._step_count, 'collected': 6 - len(self._remaining)}
        return obs, reward, terminated, truncated, info

    def close(self):
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_obs(self) -> np.ndarray:
        obs = np.zeros(OBS_DIM, dtype=np.float32)

        # LiDAR (36 beams)
        if self._node.latest_scan is not None:
            ranges = np.array(self._node.latest_scan.ranges, dtype=np.float32)
            ranges = np.nan_to_num(ranges, nan=MAX_RANGE, posinf=MAX_RANGE)
            downsampled = ranges[::10][:LIDAR_SAMPLES]
            obs[:LIDAR_SAMPLES] = np.clip(downsampled, 0, MAX_RANGE) / MAX_RANGE

        # Goal info
        rx, ry = self._robot_pos()
        if self._remaining:
            tx, ty = next(iter(self._remaining.values()))
            dist  = math.hypot(rx - tx, ry - ty)
            angle = math.atan2(ty - ry, tx - rx)
            obs[LIDAR_SAMPLES]     = min(dist / ROOM_DIAG, 1.0)
            obs[LIDAR_SAMPLES + 1] = angle / math.pi   # [-1, 1]

        # Velocity (already normalised by construction)
        obs[LIDAR_SAMPLES + 2] = self._node.current_v / 0.25
        obs[LIDAR_SAMPLES + 3] = self._node.current_w / 1.0

        assert obs.shape == (OBS_DIM,), f"Obs shape mismatch: {obs.shape}"
        return obs

    def _robot_pos(self):
        if self._node.latest_odom is None:
            return SPAWN_X, SPAWN_Y
        p = self._node.latest_odom.pose.pose.position
        return p.x, p.y

    def _dist_to_nearest(self, rx=None, ry=None):
        if rx is None:
            rx, ry = self._robot_pos()
        if not self._remaining:
            return 0.0
        return min(math.hypot(rx - tx, ry - ty)
                   for tx, ty in self._remaining.values())

    def _min_scan(self) -> float:
        if self._node.latest_scan is None:
            return MAX_RANGE
        ranges = [r for r in self._node.latest_scan.ranges
                  if not math.isinf(r) and not math.isnan(r) and r > 0.01]
        return min(ranges, default=MAX_RANGE)
