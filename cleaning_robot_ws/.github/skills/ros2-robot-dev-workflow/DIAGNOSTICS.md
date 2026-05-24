# Gazebo Simulation Diagnostics

## Errors Encountered
```
[ERROR] [rviz2-5]: process has died [pid 140949, exit code 127, cmd '/opt/ros/jazzy/lib/rviz2/rviz2']
[gazebo-1] gz sim gui: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0
```

## Root Cause
**Snap/glibc library conflict**: The snap core20 libraries are interfering with Gazebo's GUI and RViz2 pthread symbol loading.

## Environment Status
- ✅ ROS2: Jazzy installed (`/opt/ros/jazzy`)
- ✅ Gazebo: Installed and accessible (`/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`)
- ✅ RViz2: Binary exists (`/opt/ros/jazzy/bin/rviz2`)
- ⚠️ Library conflict: snap core20 conflicting with system libraries

## Solutions Tested
1. **Remove RViz2 from launch** - RViz2 not essential for core simulation
2. **Run Gazebo headless** - Use `-s` flag to skip GUI server (avoids glibc conflict)
3. **Clean environment** - Avoid snap library path pollution

## Recommended Approach
- Launch Gazebo in **headless mode** (server-only, no GUI)
- Skip RViz2 by default
- Enable visualization via alternative methods if needed (web interface, etc.)
- Keep bridge active for ROS2 topic access

## Simulation Status ✅

### Launch Results
- Robot spawned successfully: "Entity creation successful"
- All expected ROS2 topics publishing:
  - `/scan` (LaserScan from GPU LIDAR)
  - `/odom` (Odometry from Gazebo)
  - `/camera/image_raw` (Camera sensor)
  - `/cmd_vel` (command input ready)
  - `/tf` / `/tf_static` (transforms)
  - `/joint_states` (joint feedback)
  - `/clock` (Gazebo simulation time)

### URDF Fixes Applied
- Removed non-standard `gz_frame_id` elements (replaced with standard frame handling)
- Schema warnings resolved

### Verification Commands
```bash
# Monitor active topics
ros2 topic list

# Check specific sensor data
ros2 topic echo /scan
ros2 topic echo /camera/image_raw
ros2 topic echo /odom

# Send movement commands
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## Next Steps
- Test navigation stack (Nav2) integration
- Validate cleaning algorithm performance in simulation
