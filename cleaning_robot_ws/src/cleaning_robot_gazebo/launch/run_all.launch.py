"""
Single-command full stack launcher.

Starts Gazebo, the robot, the ROS-Gazebo bridge, RViz, SLAM, both global
planners (A* + Dijkstra), one local planner, the vacuum collector, and the
autonomous explorer — everything in one call.

Usage:
  ros2 launch cleaning_robot_gazebo run_all.launch.py
  ros2 launch cleaning_robot_gazebo run_all.launch.py local:=apf

After the run — view benchmark results in a separate terminal:
  ros2 run cleaning_robot_algorithms benchmark
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package paths ────────────────────────────────────────────────────
    desc_pkg  = get_package_share_directory('cleaning_robot_description')
    gz_pkg    = get_package_share_directory('cleaning_robot_gazebo')
    nav_pkg   = get_package_share_directory('cleaning_robot_nav')

    urdf_file  = os.path.join(desc_pkg, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(gz_pkg,   'worlds', 'room.sdf')
    nav_params = os.path.join(nav_pkg,  'config', 'nav2_params.yaml')

    robot_desc = Command(['xacro ', urdf_file])

    # ── Launch argument ──────────────────────────────────────────────────
    local_arg = DeclareLaunchArgument(
        'local', default_value='dwa',
        description='Local planner: dwa (obstacle-triggered) or apf (always-on)')

    # ════════════════════════════════════════════════════════════════════
    # PHASE 1 — Simulation (start immediately)
    # ════════════════════════════════════════════════════════════════════

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'{world_file} -r'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', 'cleaning_robot',
            '-topic', '/robot_description',
            '-x', '-5.0', '-y', '0.0', '-z', '0.1'
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # ── Cleaning robot ───────────────────────────────────────────
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/office_world/remove'
            '@ros_gz_interfaces/srv/DeleteEntity'
            '@gz.msgs.Entity@gz.msgs.Boolean',
            # ── Pedestrian models — cmd_vel (ROS→Gz) + odometry (Gz→ROS) ─
            '/model/person_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/person_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/person_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/person_2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # ════════════════════════════════════════════════════════════════════
    # PHASE 2 — SLAM  (delayed 5 s — waits for Gazebo + bridge + /scan)
    # ════════════════════════════════════════════════════════════════════

    slam = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[nav_params, {'use_sim_time': True}],
            )
        ]
    )

    # ════════════════════════════════════════════════════════════════════
    # PHASE 3 — Algorithms (delayed 8 s — SLAM needs a head-start)
    # ════════════════════════════════════════════════════════════════════

    algorithms = TimerAction(
        period=8.0,
        actions=[
            # Global planners — both run, Explorer uses whichever replies first
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

            # Local planner — DWA (obstacle-triggered) or APF (always-on).
            # Only one runs at a time to avoid /cmd_vel conflicts.
            Node(
                package='cleaning_robot_algorithms',
                executable=LaunchConfiguration('local'),
                name='local_planner',
                output='screen',
            ),

            # Vacuum collector
            Node(
                package='cleaning_robot_algorithms',
                executable='vacuum',
                name='vacuum_collector',
                output='screen',
            ),

            # Autonomous explorer — Phase 1: wall-follow, Phase 2: clean all targets
            Node(
                package='cleaning_robot_algorithms',
                executable='explorer',
                name='autonomous_explorer',
                output='screen',
            ),

            # Pedestrian controller — potential-field obstacle avoidance for
            # person_1 and person_2 (physics models, cannot pass through walls)
            Node(
                package='cleaning_robot_algorithms',
                executable='pedestrian_controller',
                name='pedestrian_controller',
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        local_arg,
        # Phase 1 — immediate
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz,
        # Phase 2 — +5 s
        slam,
        # Phase 3 — +8 s
        algorithms,
    ])
