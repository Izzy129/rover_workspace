# Vision Subsystem

This folder houses the vision pipeline for the rover providing camera capture, ArUco marker detection, and keyboard pose estimation for the University Rover Challenge (URC).

**TODO: Object detection subsystem** 

## ROS Architecture

```
camera_publisher
    ↓ /image_raw, /camera_info
aruco_detection
    ↓ /aruco_markers
keyboard_detector
    ↓ /keyboard_keys, /keyboard_center
```

## Packages

### camera_publisher
Camera abstraction layer providing a unified interface for webcam and RealSense cameras.

- **Executables:** `webcam_publisher`, `realsense_publisher`
- **Publishes:** Camera images and calibration info
- **TF:** Publishes static transform `map` → `camera_link`

**TODO:** Adapt for ArduCams once they arrive

### aruco_detection
Detects ArUco markers in camera images and estimates their 6DOF poses.

- **Executables:** `aruco_node`, `aruco_generate_marker`
- **Subscribes:** `/image_raw`, `/camera_info`
- **Publishes:** `/aruco_markers`, `/aruco_poses` (for RViz)
- **Features:** Configurable marker size, dictionary, and pose smoothing

### keyboard_detector
Computes keyboard and individual key poses from four corner ArUco markers.

- **Executables:** `keyboard_detection`, `keyboard_visualizer`
- **Subscribes:** `/aruco_markers`
- **Publishes:** `/keyboard_center`, `/keyboard_keys`, `/keyboard_pose` (for RViz)
- **Configuration:** `config/keys.yaml` defines keyboard layout

### vision_bringup
Subsystem-level launch orchestration that brings up the complete vision pipeline.

- **Launch:** `vision_bringup.launch.py`
- **Parameters:** Camera type, stream type, RViz, visualizer, smoothing
- **Dependencies:** Launches all three vision packages with parameter forwarding

## For New Developers

### First-Time Setup

1. **Install dependencies** (no pip install needed - uses system packages):
   ```bash
   cd ~/rover_workspace  # or wherever your workspace is
   rosdep install --from-paths src/vision --ignore-src -r -y
   ```

2. **Build the vision packages**:
   ```bash
   colcon build --packages-up-to vision_bringup
   source install/setup.bash
   ```

3. **Test with webcam**:
   ```bash
   ros2 launch vision_bringup vision_bringup.launch.py
   ```

4. **(Optional) Install RealSense SDK** if using RealSense cameras:
   - See [Intel RealSense installation guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

**Note:** We follow standard ROS2 practices - all Python dependencies are managed via rosdep/apt, not pip/venv.

## Quick Start

### Launch Full Vision System

```bash
# With webcam and RViz
ros2 launch vision_bringup vision_bringup.launch.py

# With RealSense depth camera
ros2 launch vision_bringup vision_bringup.launch.py \
    camera_type:=realsense \
    stream_type:=depth

# With keyboard visualizer
ros2 launch vision_bringup vision_bringup.launch.py \
    visualize_keyboard:=true
```

### Launch Individual Packages

```bash
# Camera only
ros2 launch camera_publisher camera_publisher.launch.py camera_type:=webcam

# ArUco detection only (requires camera running in separate terminal)
ros2 launch aruco_detection aruco_detection.launch.py use_rviz:=false

# Keyboard detection only (requires camera and ArUco running in separate terminals)
ros2 launch keyboard_detector keyboard_detector.launch.py
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/image_raw` | sensor_msgs/Image | Raw camera feed (~30 Hz) |
| `/camera_info` | sensor_msgs/CameraInfo | Camera calibration |
| `/aruco_markers` | vision_interfaces/ArucoMarkers | Detected markers with IDs and poses |
| `/aruco_poses` | geometry_msgs/PoseArray | Marker poses for RViz |
| `/aruco_detection/image` | sensor_msgs/Image | Annotated image with markers |
| `/keyboard_center` | geometry_msgs/PoseStamped | Keyboard center pose |
| `/keyboard_keys` | vision_interfaces/KeyboardKeys | Individual key poses and labels |
| `/keyboard_pose` | geometry_msgs/PoseArray | Key poses for RViz |
| `/tf_static` | tf2_msgs/TFMessage | Static transforms (map → camera_link) |

## Build

```bash
# Build all vision packages
colcon build --packages-up-to vision_bringup

# Build specific package
colcon build --packages-select camera_publisher
colcon build --packages-select aruco_detection
colcon build --packages-select keyboard_detector
```

## Configuration

### ArUco Markers
- Dictionary: DICT_4X4_50 (default)
- Marker size: 62.5mm (configurable in `aruco_detection/config/aruco_parameters.yaml`)

### Keyboard Layout
- Edit `keyboard_detector/config/keys.yaml` to define key positions
- Positions are relative to keyboard center in meters

### Camera Calibration
- Webcam: Auto-calibration from OpenCV
- RealSense: Uses built-in calibration from camera firmware

## Visualization

### RViz
Launch with `use_rviz:=true` to visualize:
- Camera image
- Detected ArUco markers (axes)
- Marker poses (PoseArray)
- Keyboard key poses (PoseArray)
- TF tree

### OpenCV Keyboard Visualizer
Launch with `visualize_keyboard:=true` to see:
- Real-time keyboard detection overlay
- Key positions and labels
- Keyboard center marker

## Testing

```bash
# Verify package executables
ros2 pkg executables camera_publisher
ros2 pkg executables aruco_detection
ros2 pkg executables keyboard_detector

# Check topic flow
ros2 topic list
ros2 topic hz /image_raw
ros2 topic echo /aruco_markers

# Verify transforms
ros2 run tf2_ros tf2_echo map camera_link
```

## Package Structure

```
src/vision/
├── camera_publisher/
│   ├── camera_publisher/       # Python package
│   ├── config/                 # Camera parameters
│   ├── launch/                 # Launch files
│   └── README.md
├── aruco_detection/
│   ├── aruco_detection/        # Python package
│   ├── config/                 # ArUco parameters, RViz config
│   ├── launch/                 # Launch files
│   └── README.md
├── keyboard_detector/
│   ├── keyboard_detector/      # Python package
│   ├── config/                 # Keyboard layout
│   ├── launch/                 # Launch files
│   └── README.md
├── vision_bringup/
│   ├── launch/                 # Subsystem launch
│   └── README.md
└── README.md                   # This file
```

## Related Packages

- `vision_interfaces/` - Custom message definitions (ArucoMarkers, KeyboardKeys)

## Authors

- Camera and keyboard packages: Rice Robotics Club (R-owl-vers)
- ArUco detection: Nathan Sprague (original), adapted for URC
