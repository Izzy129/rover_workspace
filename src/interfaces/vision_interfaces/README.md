# vision_interfaces

Custom ROS2 message definitions for the vision subsystem.

## Messages

### ArucoMarkers.msg
Array of detected ArUco markers with their IDs and poses.

**Fields:**
- `std_msgs/Header header` - Timestamp and frame
- `int32[] marker_ids` - Array of detected marker IDs
- `geometry_msgs/Pose[] poses` - Corresponding pose for each marker

### KeyboardKeys.msg
Keyboard key positions detected from ArUco markers.

**Fields:**
- `std_msgs/Header header` - Timestamp and frame
- `geometry_msgs/Pose[] key_poses` - Position and orientation of each key
- `string[] key_names` - Corresponding key labels

## Dependencies

- `geometry_msgs`
- `std_msgs`

## Build

```bash
colcon build --packages-select vision_interfaces
```
