#!/bin/bash
# Build livox_ros_driver2 for ROS2 without wiping the workspace.
# Usage: run from the workspace root, or anywhere — the script handles paths.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DRIVER_DIR="$(cd "$SCRIPT_DIR/../livox_ros_driver2" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Setup: copy ROS2-specific package.xml and launch files
cp -f "$DRIVER_DIR/package_ROS2.xml" "$DRIVER_DIR/package.xml"
cp -rf "$DRIVER_DIR/launch_ROS2/" "$DRIVER_DIR/launch/"

# Build
cd "$WORKSPACE_ROOT"
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

# Clean up temporary launch/ copy
rm -rf "$DRIVER_DIR/launch/"
