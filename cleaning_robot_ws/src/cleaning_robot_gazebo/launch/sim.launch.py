import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    pkg_cleaning_robot_gazebo = get_package_share_directory('cleaning_robot_gazebo')
    pkg_cleaning_robot_desc = get_package_share_directory('cleaning_robot_description')

    world_path = os.path.join(pkg_cleaning_robot_gazebo, 'worlds', 'room.sdf')
    urdf_path = os.path.join(pkg_cleaning_robot_desc, 'urdf', 'robot.urdf.xacro')

    # Process the URDF file
    doc = xacro.process_file(urdf_path)
    robot_description = {'robot_description': doc.toxml()}

    # 1. Start Gazebo in headless mode (-s) for testing
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 3. Spawn Robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', doc.toxml(),
            '-name', 'cleaning_robot',
            '-allow_renaming', 'true',
            '-z', '0.1'
        ]
    )

    # 4. Bridge ROS 2 and Gazebo Topics
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge
    ])
