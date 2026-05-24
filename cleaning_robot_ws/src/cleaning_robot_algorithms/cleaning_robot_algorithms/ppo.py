#!/usr/bin/env python3
"""
PPO local planner — inference node.
Loads a trained PPO model and publishes /cmd_vel from /scan + /odom.
Drop-in replacement for dwa.py and apf.py.

Requires a trained model at /tmp/ppo_model/ppo_cleaning_robot.zip.
Train first with:  python3 ppo_train.py
"""
import math
import time
import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from stable_baselines3 import PPO as SB3PPO

LOG            = '/tmp/local_planner_log.jsonl'
MODEL_PATH     = '/tmp/ppo_model/ppo_cleaning_robot'
MAX_RANGE      = 10.0
ROOM_DIAG      = 25.0
LIDAR_SAMPLES  = 36
OBS_DIM        = LIDAR_SAMPLES + 4
SUCTION_RADIUS = 0.35

TARGETS = {
    'dust_1':  (-5.0,  1.0),
    'dust_2':  ( 3.0, -2.0),
    'dust_3':  (-2.0, -5.0),
    'trash_1': (-6.0, -5.0),
    'trash_2': ( 4.0,  6.0),
    'trash_3': ( 8.0, -2.0),
}


class PPOController(Node):

    def __init__(self):
        super().__init__('ppo_controller')

        self.get_logger().info(f"Loading PPO model from {MODEL_PATH} ...")
        self.model = SB3PPO.load(MODEL_PATH)
        self.get_logger().info("PPO model loaded — subscribing to /scan")

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, 1)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        self._latest_odom = None
        self._current_v   = 0.0
        self._current_w   = 0.0
        self._goal        = None          # (gx, gy) from /goal_pose if set
        self.remaining    = dict(TARGETS)
        self.cycles       = 0

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        self._latest_odom = msg
        self._current_v   = msg.twist.twist.linear.x
        self._current_w   = msg.twist.twist.angular.z

    def _goal_cb(self, msg):
        self._goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"PPO goal updated: {self._goal}")

    def _scan_cb(self, msg):
        t0  = time.time()
        obs = self._build_obs(msg)

        action, _ = self.model.predict(obs, deterministic=True)
        v = float(np.clip(action[0], 0.0,  0.25))
        w = float(np.clip(action[1], -1.0,  1.0))

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        # Check cleaning targets
        rx, ry = self._robot_pos()
        for name, (tx, ty) in list(self.remaining.items()):
            if math.hypot(rx - tx, ry - ty) <= SUCTION_RADIUS:
                del self.remaining[name]
                self.get_logger().info(
                    f"PPO collected {name} | {6 - len(self.remaining)}/6 done")

        self.cycles += 1
        if self.cycles % 50 == 0:
            dt    = time.time() - t0
            entry = {
                'algorithm': 'PPO',
                'cycle':     self.cycles,
                'cycle_ms':  round(dt * 1000, 2),
                'v':         round(v, 3),
                'w':         round(w, 3),
                'collected': 6 - len(self.remaining),
            }
            with open(LOG, 'a') as f:
                f.write(json.dumps(entry) + '\n')
            self.get_logger().info(
                f"PPO cycle {self.cycles}: v={v:.3f} w={w:.3f} "
                f"collected={6 - len(self.remaining)}/6")

    # ------------------------------------------------------------------
    # Observation builder — must match ppo_env.py exactly
    # ------------------------------------------------------------------

    def _build_obs(self, scan_msg) -> np.ndarray:
        obs = np.zeros(OBS_DIM, dtype=np.float32)

        # LiDAR (36 beams, every 10th of 360)
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=MAX_RANGE, posinf=MAX_RANGE)
        downsampled = ranges[::10][:LIDAR_SAMPLES]
        obs[:LIDAR_SAMPLES] = np.clip(downsampled, 0, MAX_RANGE) / MAX_RANGE

        # Goal info — use /goal_pose if set, otherwise nearest uncollected target
        rx, ry = self._robot_pos()
        if self._goal:
            tx, ty = self._goal
        elif self.remaining:
            tx, ty = next(iter(self.remaining.values()))
        else:
            tx, ty = rx, ry   # all done

        dist  = math.hypot(rx - tx, ry - ty)
        angle = math.atan2(ty - ry, tx - rx)
        obs[LIDAR_SAMPLES]     = min(dist / ROOM_DIAG, 1.0)
        obs[LIDAR_SAMPLES + 1] = angle / math.pi

        # Velocity
        obs[LIDAR_SAMPLES + 2] = self._current_v / 0.25
        obs[LIDAR_SAMPLES + 3] = self._current_w / 1.0

        return obs

    def _robot_pos(self):
        if self._latest_odom is None:
            return -5.0, 0.0
        p = self._latest_odom.pose.pose.position
        return p.x, p.y


def main():
    rclpy.init()
    rclpy.spin(PPOController())
    rclpy.shutdown()
