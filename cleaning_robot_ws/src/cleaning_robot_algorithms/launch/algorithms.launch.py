"""
Launch A* + Dijkstra planners and ONE local planner (dwa or apf).
Usage:
  ros2 launch cleaning_robot_algorithms algorithms.launch.py local:=dwa
  ros2 launch cleaning_robot_algorithms algorithms.launch.py local:=apf
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    local_arg = DeclareLaunchArgument(
        'local', default_value='dwa',
        description='Local planner: dwa or apf')

    return LaunchDescription([
        local_arg,

        Node(package='cleaning_robot_algorithms',
             executable='astar',
             name='astar_planner',
             output='screen'),

        Node(package='cleaning_robot_algorithms',
             executable='dijkstra',
             name='dijkstra_planner',
             output='screen'),

        # Switch between dwa and apf at the command line
        Node(package='cleaning_robot_algorithms',
             executable=LaunchConfiguration('local'),
             name='local_planner',
             output='screen'),
    ])