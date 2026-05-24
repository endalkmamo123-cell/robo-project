#!/usr/bin/env bash
# Single-command launcher for the cleaning robot simulation.
# Must be run from a terminal OUTSIDE VS Code (e.g. GNOME Terminal / Konsole),
# or VS Code's snap-injected GTK variables are unset here automatically.
#
# Usage:
#   ./run_all.sh          # DWA local planner (default)
#   ./run_all.sh apf      # APF local planner
#   ./run_all.sh dwa      # explicit DWA

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_PLANNER="${1:-dwa}"

echo "=========================================="
echo "  Cleaning Robot — Full Stack Launcher"
echo "  Local planner : $LOCAL_PLANNER"
echo "=========================================="

# ── Strip VS Code snap environment that breaks Gazebo/RViz2 ────────────────
# VS Code (snap) injects GTK paths pointing to /snap/core20/libpthread.so.0
# which is missing __libc_pthread_init and causes a symbol lookup error.
unset GTK_PATH
unset GTK_EXE_PREFIX
unset GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULEDIR
unset GDK_PIXBUF_MODULE_FILE
unset GIO_MODULE_DIR
unset GIO_LAUNCHED_DESKTOP_FILE
unset GSETTINGS_SCHEMA_DIR
unset LOCPATH
unset SNAP
unset SNAP_ARCH
unset SNAP_COMMON
unset SNAP_CONTEXT
unset SNAP_COOKIE
unset SNAP_DATA
unset SNAP_EUID
unset SNAP_INSTANCE_NAME
unset SNAP_LAUNCHER_ARCH_TRIPLET
unset SNAP_NAME
unset SNAP_REVISION
unset SNAP_USER_COMMON
unset SNAP_USER_DATA
unset SNAP_VERSION

# ── Source ROS 2 + workspace ────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

echo ""
echo "Launching..."
echo "  Phase 0s  : Gazebo, robot, bridge, RViz2"
echo "  Phase +5s : slam_toolbox  (provides /map)"
echo "  Phase +8s : astar + dijkstra + $LOCAL_PLANNER + vacuum + explorer"
echo ""
echo "After the robot finishes navigating, open a new terminal and run:"
echo "  source $SCRIPT_DIR/install/setup.bash"
echo "  ros2 run cleaning_robot_algorithms benchmark"
echo ""

exec ros2 launch cleaning_robot_gazebo run_all.launch.py "local:=$LOCAL_PLANNER"
