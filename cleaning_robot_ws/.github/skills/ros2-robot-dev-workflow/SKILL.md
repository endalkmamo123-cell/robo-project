---
name: ros2-robot-dev-workflow
description: "Multi-step ROS2 cleaning robot development workflow. Use when: modifying robot URDF → integrating ML algorithms → building with colcon → launching Gazebo simulation → testing navigation with Nav2. Includes URDF validation, environment/world setup (SDF configuration), build verification, and simulation debugging."
---

# ROS2 Cleaning Robot Development Workflow

This skill guides you through the complete development cycle for the cleaning robot project, from URDF modifications through simulation testing and navigation validation.

## Workflow Overview

```
1. URDF Definition & Validation
   ↓
2. Machine Learning Integration
   ↓
3. Build & Compilation (colcon)
   ↓
4. Gazebo Environment Setup & World Configuration
   ↓
5. Launch Simulation
   ↓
6. Nav2 Navigation Testing
   ↓
7. Debugging & Iteration
```

---

## Step 1: URDF Definition & Validation

**File**: `src/cleaning_robot_description/urdf/robot.urdf.xacro`

### Checklist
- [ ] Robot physical dimensions defined (base, wheels, sensors)
- [ ] Joint definitions: fixed, revolute, or prismatic types
- [ ] Collision and visual geometries match
- [ ] Inertia values are realistic for physics simulation
- [ ] Mesh files referenced correctly (if using STL/DAE)

### Commands
```bash
# Validate URDF syntax
xacro src/cleaning_robot_description/urdf/robot.urdf.xacro > /tmp/robot.urdf

# Check for errors
xmllint --noout /tmp/robot.urdf

# View URDF tree structure
urdf_to_graphiz /tmp/robot.urdf  # Creates PDF visualization
```

### Common Issues
- **Misspelled joint/link names**: Cross-reference in transmission definitions
- **Wrong coordinate frames**: Z-up convention in ROS/Gazebo
- **Missing inertia block**: Causes simulation instability
- **Incorrect parent-child relationships**: Review link/joint tree

---

## Step 2: Machine Learning Integration

**Integration Points**: Object detection (YOLO), sensor processing, decision-making

### Setup
- [ ] YOLO ROS2 node created/configured
- [ ] Image topics connected from Gazebo camera sensor
- [ ] ML model weights downloaded and verified
- [ ] Output topics (detections) configured for navigation stack

### ROS2 Integration
```bash
# Check topic connectivity
ros2 topic list
ros2 topic echo /robot_camera/image_raw  # Camera feed
ros2 topic echo /detections               # YOLO output
```

### Validation
- [ ] Camera publishes at expected rate (typically 30 Hz)
- [ ] YOLO inference time acceptable (< 100ms for real-time)
- [ ] Detection output compatible with Nav2/planning nodes

---

## Step 3: Build with Colcon

**Workspace**: `cleaning_robot_ws/`

### Build Command
```bash
cd cleaning_robot_ws
colcon build --symlink-install
```

### Debug Flags
```bash
# Verbose output for error diagnosis
colcon build --event-handlers console_direct+

# Build specific package
colcon build --packages-select cleaning_robot_description

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Build Validation
- [ ] No compilation errors
- [ ] All dependencies resolved
- [ ] CMakeLists.txt and package.xml correctly formatted

### Troubleshooting
- **Missing dependencies**: Run `rosdep install`
- **CMake errors**: Check CMakeLists.txt syntax and include paths
- **Python issues**: Verify setup.py or pyproject.toml

---

## Step 4: Gazebo Environment & World Setup

**Files**: 
- `src/cleaning_robot_gazebo/worlds/room.sdf` (environment definition)
- `src/cleaning_robot_gazebo/launch/sim.launch.py`

### World Configuration (SDF)
- [ ] Ground plane defined with correct friction
- [ ] Walls positioned correctly (common issue: walls in wrong locations)
- [ ] Lighting configured for camera sensors
- [ ] Physics engine settings (real-time factor, step size)

### Validation Checklist
```bash
# Validate SDF syntax
gz sdf -c worlds/room.sdf

# Test world loads (Gazebo Garden/Humble)
gz sim -v4 worlds/room.sdf
```

### Common SDF Issues (Pain Points)
- **Walls in wrong positions**: Review `<pose>` and `<size>` tags
- **Collision geometry mismatch**: Ensure collision `<geometry>` matches visual
- **Missing friction/contact properties**: Add `<friction>` and `<restitution>` tags
- **Physics instability**: Adjust `<max_step_size>` and `<real_time_factor>`

### SDF Structure Example
```xml
<world name="room">
  <physics>
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  
  <model name="walls">
    <pose>0 0 0 0 0 0</pose>
    <link name="north_wall">
      <collision><geometry><box><size>10 0.1 2</size></box></geometry></collision>
      <visual><geometry><box><size>10 0.1 2</size></box></geometry></visual>
    </link>
  </model>
</world>
```

---

## Step 5: Launch Simulation

**File**: `src/cleaning_robot_gazebo/launch/sim.launch.py`

### Launch Command
```bash
cd cleaning_robot_ws
source install/setup.bash
ros2 launch cleaning_robot_gazebo sim.launch.py
```

### Launch Script Checklist
- [ ] Gazebo spawn command includes correct model path
- [ ] Robot spawn pose reasonable (inside environment, not inside walls)
- [ ] All sensor plugins configured (camera, lidar, etc.)
- [ ] Joint state publisher running
- [ ] TF (transform) tree complete

### Diagnostics
```bash
# Check running nodes
ros2 node list

# View TF tree
ros2 run tf2_tools view_frames.py

# Monitor topics
rqt_graph  # Visualize node/topic graph
```

---

## Step 6: Nav2 Navigation Testing

**File**: `src/cleaning_robot_nav/config/nav2_params.yaml`

### Pre-Flight Checks
- [ ] nav2_params.yaml configured for your robot footprint
- [ ] Costmap layers: static/dynamic maps properly setup
- [ ] Planner parameters tuned (inflation radius, etc.)
- [ ] Controller parameters match robot dynamics

### Nav2 Launch
```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/path/to/map.yaml
```

### Navigation Testing
```bash
# Send goal via ROS2 CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}}"

# Or use RViz GUI (easier)
ros2 launch nav2_bringup rviz_launch.py
# Click "2D Goal Pose" button and set target
```

### Validation
- [ ] Robot path planning succeeds
- [ ] Navigation follows planned trajectory
- [ ] Costmap updates from sensor data
- [ ] Recoveries triggered appropriately (stuck detection)

---

## Step 7: Debugging & Iteration

### Common Issues & Solutions

| Issue | Diagnosis | Fix |
|-------|-----------|-----|
| Robot doesn't move | Check `/cmd_vel` topic; verify controller running | Ensure control node launched; check velocity limits |
| Walls appear in wrong place | View world in Gazebo GUI | Edit `<pose>` and `<size>` in room.sdf |
| Navigation fails | Check costmap visualization in RViz | Verify map frame, sensor topics, Nav2 params |
| Physics unstable (robot bouncing) | Simulation runs in fast-forward | Reduce `<max_step_size>` in SDF; check inertia values |
| ML/YOLO not detecting objects | Echo `/detections` topic | Verify camera feed; check YOLO config; ensure model weights loaded |
| TF tree incomplete | `ros2 run tf2_tools view_frames.py` | Add missing static transforms in launch file |

### Debugging Tools
```bash
# Log everything to rosbag for replay
ros2 bag record -a

# Play back recording
ros2 bag play rosbag2_folder/ --clock

# RViz for visualization
ros2 run rviz2 rviz2 -d config.rviz

# Console output with timestamps
rqt_console
```

---

## Iteration Cycle

When debugging or adding features:

1. **Identify**: Use diagnostic tools (RViz, topic echo, logs)
2. **Locate**: Find relevant configuration file or source code
3. **Modify**: Make minimal changes (URDF, SDF, params, or code)
4. **Rebuild**: If code changes: `colcon build --packages-select [pkg]`
5. **Relaunch**: Kill current sim; run `sim.launch.py` again
6. **Validate**: Use checklist or test scenario from appropriate step

---

## Quick Reference: File Locations

| Purpose | File |
|---------|------|
| Robot definition | `src/cleaning_robot_description/urdf/robot.urdf.xacro` |
| Simulation world | `src/cleaning_robot_gazebo/worlds/room.sdf` |
| Launch script | `src/cleaning_robot_gazebo/launch/sim.launch.py` |
| Navigation config | `src/cleaning_robot_nav/config/nav2_params.yaml` |
| Workspace build | Root: `cleaning_robot_ws/` |

---

## When to Use This Skill

Invoke this skill when:
- ✅ Starting a development session on the cleaning robot
- ✅ Troubleshooting simulation behavior or robot movement
- ✅ Modifying URDF or environment (world) configuration
- ✅ Integrating new sensors or ML algorithms
- ✅ Debugging navigation failures
- ✅ Validating the complete pipeline end-to-end

**Not for**: General ROS2 tutorials, new package creation, or dependency installation (use rosdep/apt for that).

---

## Next Steps to Customize

1. **Create a launch checklist** (`.github/skills/ros2-robot-dev-workflow/launch-checklist.md`) for pre-simulation validation
2. **Build a YOLO integration guide** (`.github/skills/ros2-robot-dev-workflow/ml-integration.md`) with specific YOLO node setup
3. **Document SDF troubleshooting** (`.github/skills/ros2-robot-dev-workflow/sdf-debugging.md`) with visual examples of common wall/environment issues
