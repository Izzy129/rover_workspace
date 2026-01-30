# aruco_detection

ArUco marker detection and pose estimation for visual localization.
## URC Marker ID/dimensions
We have 11 ArUco tags/IDs
- `0`: Start post for Auto Nav mission (*0.2m x 0.2m with 0.025m white border*)
- `1`: Post 1 for Auto Nav mission (*0.2m x 0.2m with 0.025m white border*)
- `2`: Post 2 for Auto Nav mission (*0.2m x 0.2m with 0.025m white border)*
- `1`: Top left corner of keyboard for Equipment Servicing mission (*0.02m x 0.02m, with 0.005m white border*)
- `4`: Top right corner of keyboard for Equipment Servicing mission (*0.02m x 0.02m, with 0.005m white border*)
- `3`: Bottom right corner of keyboard for Equipment Servicing mission (*0.02m x 0.02m, with 0.005m white border*)
- `2`: Bottom left corner of keyboard for Equipment Servicing mission (*0.02m x 0.02m, with 0.005m white border*)
- `4`: Top left corner of USB-C port for Equipment Servicing mission (*0.01m x 0.01m, with 0.0025m white border*)
- `1`: Top right corner of USB-C port for Equipment Servicing mission (*0.01m x 0.01m, with 0.0025m white border*)
- `3`: Bottom right corner of USB-C port for Equipment Servicing mission (*0.01m x 0.01m, with 0.0025m white border*)
- `2`: Bottom left corner of USB-C port for Equipment Servicing mission (*0.01m x 0.01m, with 0.0025m white border*)

## Executables

### aruco_node
Detects ArUco markers in camera images and publishes their IDs and poses.

**Subscribed Topics:**
- `/image_raw` (sensor_msgs/Image) - Input camera image
- `/camera_info` (sensor_msgs/CameraInfo) - Camera calibration

**Published Topics:**
- `/aruco_markers` (vision_interfaces/ArucoMarkers) - Detected markers with IDs and poses
- `/aruco_poses` (geometry_msgs/PoseArray) - Poses for RViz visualization
- `/aruco_detection/image` (sensor_msgs/Image) - Annotated image with detected markers

**Parameters:**
- `marker_size` (float, default: 0.0625) - Physical marker size in meters
- `aruco_dictionary_id` (string, default: "DICT_4X4_50") - ArUco dictionary
- `smoothing_alpha` (float, default: 0.7) - Pose smoothing factor (0.0-1.0)

### aruco_generate_marker
Command-line tool to generate ArUco marker images.

**Usage:**
```bash
ros2 run aruco_detection aruco_generate_marker <marker_id> <output_file.png>
```

## Launch

```bash
# Launch with RViz
ros2 launch aruco_detection aruco_detection.launch.py use_rviz:=true

# Launch without RViz
ros2 launch aruco_detection aruco_detection.launch.py use_rviz:=false smoothing_alpha:=0.8
```

## Dependencies

- rclpy
- sensor_msgs
- geometry_msgs
- cv_bridge
- tf2_ros
- vision_interfaces
- python3-opencv
- python3-numpy

## Author

Nathan Sprague
