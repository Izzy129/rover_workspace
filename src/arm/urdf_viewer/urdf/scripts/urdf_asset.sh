#!/bin/bash
# Replaces all package://assets/ mesh paths with package://urdf_viewer/urdf/assets/
# in urdf/robot.urdf.
#
# Usage: Run from the urdf_viewer package root, or anywhere — the script
#        locates robot.urdf relative to its own location.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
URDF_FILE="$SCRIPT_DIR/../robot.urdf"

onshape-to-robot $SCRIPT_DIR/../

if [ ! -f "$URDF_FILE" ]; then
    echo "Error: robot.urdf not found at $URDF_FILE"
    exit 1
fi

sed -i 's|package://assets/|package://urdf_viewer/urdf/assets/|g' "$URDF_FILE"
echo "Updated mesh paths in $URDF_FILE"
