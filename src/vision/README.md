# Vision Subsystem

This folder houses the vision pipeline for the rover providing camera capture, ArUco marker detection, keyboard pose estimation, and YOLO object detection for the University Rover Challenge (URC).

## ROS Architecture

```
camera_publisher
    ↓ /camera_{left,front,right}/image_raw, /camera_info
aruco_detection
    ↓ /aruco_markers
keyboard_detector
    ↓ /keyboard_keys, /keyboard_center

camera_publisher
    ↓ /camera_{left,front,right}/image_raw
object_detection
    ↓ /camera_{left,front,right}/object_detection/image_raw

lidar_publisher
    livox_ros_driver2 → /livox/lidar (PointCloud2)
        ↓
    pointcloud_to_laserscan → /lidar/scan (LaserScan)
        ↓
    Navigation system
```

## Packages

### camera_publisher
Camera abstraction layer providing a unified interface for USB cameras and RealSense cameras.

- **Executables:** `camera_publisher`, `realsense_publisher`
- **Publishes:** Camera images and calibration info
- **TF:** Publishes static transform `map` → `camera_link`

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

### object_detection
YOLO-based object detection across the left, front, and right cameras.

- **Executables:** `detector`
- **Subscribes:** `/camera_{left,front,right}/image_raw`
- **Publishes:** `/camera_{left,front,right}/object_detection/image_raw` (annotated images with bounding boxes)

### lidar_publisher
Launch orchestration for the Livox MID-360 LiDAR with pointcloud-to-laserscan conversion.

- **Launches:** `livox_ros_driver2_node`, `pointcloud_to_laserscan_node`
- **Publishes:** `/livox/lidar` (PointCloud2), `/lidar/scan` (LaserScan)
- **Configuration:** `config/MID360_config.json` (network), `config/laserscan_params.yaml` (scan params)
- **Build script:** `build_livox_driver.sh` (builds the Livox driver with correct ROS2 cmake args)

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
   rosdep install --from-paths src --ignore-src -r -y
   ```

   **Note:** Use `src` not `src/vision` so rosdep can find `vision_interfaces` in `src/interfaces/`

2. **Build the vision packages**:
   ```bash
   colcon build --packages-up-to vision_bringup
   source install/setup.bash
   ```

3. **Test with webcam**:
   ```bash
   ros2 launch vision_bringup vision_bringup.launch.py
   ```

4. **(Optional) Install Livox SDK 2** if using the Livox MID-360 LiDAR:
   - Follow the [Livox SDK 2 installation instructions](https://github.com/Livox-SDK/Livox-SDK2#2-installation)
   - Then build the Livox ROS2 driver: `./src/vision/lidar_publisher/build_livox_driver.sh`
   - Install the laserscan converter: `sudo apt install ros-jazzy-pointcloud-to-laserscan`

5. **(Optional) Install RealSense SDK** if using RealSense cameras:
   - See [Intel RealSense installation guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

**Notes:**
- We follow standard ROS2 practices - all Python dependencies are managed via rosdep/apt, not pip/venv
- **Exception:** OpenCV 4.8+ is required for ArUco detection. If `python3-opencv` from apt is too old, use: `pip install opencv-contrib-python>=4.8.0`
- **Exception:** `ultralytics` is not on apt — install via: `pip install ultralytics --break-system-packages`
- `ros2_numpy` is vendored as a git submodule at `src/vision/ros2_numpy`. After cloning the repo, run: `git submodule update --init --recursive`

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

# LiDAR (Livox MID-360 → LaserScan)
ros2 launch lidar_publisher lidar_publisher.launch.py
```

TODO: add YOLO detection to subsystem launch?

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera_left/image_raw` | sensor_msgs/Image | Left ArduCam raw camera feed (30 Hz) |
| `/camera_left/camera_info` | sensor_msgs/CameraInfo | Left ArduCam calibration data |
| `/camera_front/image_raw` | sensor_msgs/Image | Front ArduCam raw camera feed (30 Hz) |
| `/camera_front/camera_info` | sensor_msgs/CameraInfo | Front ArduCam calibration data |
| `/camera_right/image_raw` | sensor_msgs/Image | Right ArduCam raw camera feed (30 Hz) |
| `/camera_right/camera_info` | sensor_msgs/CameraInfo | Right ArduCam calibration data |
| `/aruco_markers` | vision_interfaces/ArucoMarkers | Detected markers with IDs and poses |
| `/aruco_poses` | geometry_msgs/PoseArray | Marker poses for RViz |
| `/aruco_detection/image` | sensor_msgs/Image | Annotated image with markers |
| `/keyboard_center` | geometry_msgs/PoseStamped | Keyboard center pose (used for debugging) |
| `/keyboard_keys` | vision_interfaces/KeyboardKeys | Individual key poses and labels |
| `/keyboard_pose` | geometry_msgs/PoseArray | Key poses for RViz |
| `/camera_left/object_detection/image_raw` | sensor_msgs/Image | Left camera with YOLO bounding boxes |
| `/camera_front/object_detection/image_raw` | sensor_msgs/Image | Front camera with YOLO bounding boxes |
| `/camera_right/object_detection/image_raw` | sensor_msgs/Image | Right camera with YOLO bounding boxes |
| `/livox/lidar` | sensor_msgs/PointCloud2 | Livox MID-360 3D point cloud (10 Hz) |
| `/lidar/scan` | sensor_msgs/LaserScan | 2D laser scan from point cloud (10 Hz) |
| `/tf_static` | tf2_msgs/TFMessage | Static transforms (map → camera_link) |

## Build

```bash
# Build all vision packages
colcon build --packages-up-to vision_bringup

# Build specific package
colcon build --packages-select camera_publisher
colcon build --packages-select aruco_detection
colcon build --packages-select keyboard_detector
colcon build --packages-select object_detection
```

## Configuration

### ArUco Markers
- Dictionary: DICT_4X4_50 (default for URC)
- Marker size: 62.5mm (configurable in `aruco_detection/config/aruco_parameters.yaml`)

### Keyboard Layout
- Edit `keyboard_detector/config/keys.yaml` to define key positions
- Positions are relative to keyboard center in meters

### Camera Calibration
- ArduCams: Auto-calibration from OpenCV
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
ros2 pkg executables object_detection

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
├── lidar_publisher/
│   ├── config/                 # MID360 network config, laserscan params
│   ├── launch/                 # Launch files
│   ├── build_livox_driver.sh   # Livox driver build script
│   └── README.md
├── object_detection/
│   ├── object_detection/       # Python package
│   └── README.md
├── ros2_numpy/                 # Git submodule (ROS2 numpy bridge)
├── livox_ros_driver2/          # Git submodule (Livox driver)
├── Livox-SDK2/                 # Git submodule (Livox SDK, install systemwide)
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
