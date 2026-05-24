"""
Full algorithm stack — runs SLAM + every algorithm node together.

Requires Terminal 1: ros2 launch cleaning_robot_gazebo sim.launch.py

SLAM             : slam_toolbox (publishes /map — required by A* and Dijkstra)
Global planners  : A* + Dijkstra (both active, Explorer uses whichever responds first)
Local planner    : DWA (default) or APF — only one at a time to avoid /cmd_vel conflict
Support nodes    : vacuum collector + autonomous explorer

Usage (Terminal 2):
  ros2 launch cleaning_robot_algorithms full_stack.launch.py             # DWA local planner
  ros2 launch cleaning_robot_algorithms full_stack.launch.py local:=apf  # APF local planner

After the run (Terminal 3):
  ros2 run cleaning_robot_algorithms benchmark
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg  = get_package_share_directory('cleaning_robot_nav')
    params   = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')

    local_arg = DeclareLaunchArgument(
        'local', default_value='dwa',
        description='Local planner to activate: dwa or apf')

    return LaunchDescription([
        local_arg,

        # ── SLAM (publishes /map — A* and Dijkstra wait for this) ────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),

        # ── Global planners ──────────────────────────────────────────────
        Node(
            package='cleaning_robot_algorithms',
            executable='astar',
            name='astar_planner',
            output='screen',
        ),
        Node(
            package='cleaning_robot_algorithms',
            executable='dijkstra',
            name='dijkstra_planner',
            output='screen',
        ),

        # ── Local planner (one at a time) ────────────────────────────────
        # DWA only activates when forward path is blocked; APF is always-on.
        # Running both simultaneously would cause /cmd_vel conflicts.
        Node(
            package='cleaning_robot_algorithms',
            executable=LaunchConfiguration('local'),
            name='local_planner',
            output='screen',
        ),

        # ── Vacuum collector ─────────────────────────────────────────────
        Node(
            package='cleaning_robot_algorithms',
            executable='vacuum',
            name='vacuum_collector',
            output='screen',
        ),

        # ── Autonomous explorer ──────────────────────────────────────────
        Node(
            package='cleaning_robot_algorithms',
            executable='explorer',
            name='autonomous_explorer',
            output='screen',
        ),
    ])
