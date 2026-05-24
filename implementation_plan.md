# Integrate Kuka YouBot and Living Room World for Autonomous Cleaning

This plan outlines the steps to change the robot model to the Kuka YouBot, load the fuel `living_room` world, and implement a basic autonomous cleaning and trash-collecting behavior.

## User Review Required

> [!WARNING]
> The Kuka YouBot has mecanum wheels. In Gazebo Sim, simulating mecanum wheels can be complex. For this project, we can either:
> 1. Use the `gz::sim::systems::MecanumDrive` plugin (recommended, but requires proper wheel definitions).
> 2. Use a standard `DiffDrive` plugin applied to the front and rear wheels as a simplification.
> *I will proceed with the MecanumDrive plugin, but if issues arise, we can simplify.*

> [!IMPORTANT]
> The exact logic for "picking trash and cleaning dust" will be implemented as a high-level ROS 2 Python node using Nav2 waypoints. Real physical manipulation (using the YouBot arm to pick things up) is an extremely advanced task. For this project, I will simulate it by having the robot navigate to predefined trash locations and simply "clearing" them or acknowledging them via logs while following a coverage pattern for dust.

## Proposed Changes

### Gazebo Simulation & Robot Model

#### [MODIFY] `src/Kuka YouBot/model.sdf`
- Add the `gz::sim::systems::MecanumDrive` plugin so the robot can move.
- Add the `gz::sim::systems::Sensors` and odometry plugins so ROS 2 gets laser scan and pose data.
- Ensure the `base_laser_front` is active and publishing to `/scan`.

#### [NEW] Move `Kuka YouBot`
- Move the `Kuka YouBot` directory into `cleaning_robot_gazebo/models/` for proper ROS 2 packaging.

#### [MODIFY] `src/cleaning_robot_gazebo/launch/sim.launch.py`
- Change the world argument to use `living_room_world/living_room.sdf`.
- Remove the `xacro` processing and directly spawn the `Kuka YouBot/model.sdf`.
- Update the `ros_gz_bridge` topics to match the YouBot's sensors (e.g., cmd_vel, scan, odom, tf).

### Navigation & Cleaning Logic

#### [MODIFY] `src/cleaning_robot_nav/config/nav2_params.yaml`
- Adjust the robot footprint to match the Kuka YouBot dimensions.
- Tune the local trajectory planner for a mecanum/omnidirectional base if possible, or fallback to differential constraints.

#### [NEW] `src/cleaning_robot_nav/cleaning_robot_nav/cleaner_node.py`
- Create a Python node that uses `nav2_simple_commander`.
- Define a set of waypoints covering the `living_room` (dust cleaning).
- Define specific waypoints where "trash" is located. The node will navigate to these points, pause (simulating picking up), and continue.

#### [MODIFY] `src/cleaning_robot_nav/CMakeLists.txt` & `package.xml`
- Register the new Python node as an executable so it can be launched via `ros2 run`.

## Verification Plan

### Automated Tests
1. Build the workspace: `colcon build --symlink-install`
2. Launch the simulation: `ros2 launch cleaning_robot_gazebo sim.launch.py`
   - Verify the Kuka YouBot spawns in the `living_room` without falling through the floor.
3. Launch Nav2: `ros2 launch cleaning_robot_nav nav.launch.py`
   - Verify AMCL/mapping initializes and the costmap reflects the room.

### Manual Verification
1. Run the cleaning script: `ros2 run cleaning_robot_nav cleaner_node`
2. Observe the robot moving autonomously through the living room, pausing at designated "trash" spots.
