# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is the ROS2 workspace for the Rice Robotics Club rover project, designed for the University Rover Challenge (URC). The repository is organized into **subsystem packages** and **organizational packages**:

**Subsystems** (functionality):
- `vision/` - Camera capture, ArUco detection, keyboard pose estimation (currently implemented), object detection (coming soon)
- `navigation/` - Path planning and localization (coming soon)
- `arm/` - Robotic arm control (coming soon)
- `communications/` - Radio and network communication (coming soon)

**Organizational** (infrastructure):
- `interfaces/` - Custom ROS2 message/service definitions
- `launch/` - System-level launch orchestration
- `config/` - System-wide configuration files
- `description/` - URDF robot models and descriptions

## Development Environment

### Docker Devcontainer
This workspace uses a Docker-based development environment:
- **ROS Distribution**: ROS2 Jazzy
- **ROS_DOMAIN_ID**: 42
- **Base image**: `ros:jazzy-ros-base`
- **GUI Support**: X11 forwarding enabled for RViz and OpenCV visualization
- **Network**: Host networking (`--net=host`) for ROS discovery

To start: Open in VSCode and click "Reopen in Container" when prompted.

### OpenCV Requirement
The vision subsystem requires OpenCV 4.8+ for the ArUco API. The Dockerfile installs this via pip with `--break-system-packages` since Ubuntu's `python3-opencv` package may be too old.

## Build Commands

### Full Workspace Build
```bash
# Install all dependencies from package.xml files
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

### Building Specific Subsystems
```bash
# Build vision subsystem (builds all dependencies including vision_interfaces)
colcon build --packages-up-to vision_bringup

# Build only one package
colcon build --packages-select camera_publisher

# Clean build (if needed)
rm -rf build/ install/ log/
colcon build
```

**Important**: Always use `--packages-up-to` when building a subsystem's bringup package to ensure all dependencies are built.

## Running the Vision System

### Launch Full Vision Pipeline
```bash
# Default: webcam + RViz
ros2 launch vision_bringup vision_bringup.launch.py

# With RealSense depth camera
ros2 launch vision_bringup vision_bringup.launch.py camera_type:=realsense stream_type:=depth

# With keyboard visualizer
ros2 launch vision_bringup vision_bringup.launch.py visualize_keyboard:=true

# Disable pose smoothing
ros2 launch vision_bringup vision_bringup.launch.py use_smoothing:=false
```

### Launch Individual Packages
```bash
# Camera only
ros2 launch camera_publisher camera_publisher.launch.py camera_type:=webcam

# ArUco detection (requires camera running)
ros2 launch aruco_detection aruco_detection.launch.py

# Keyboard detection (requires camera + aruco running)
ros2 launch keyboard_detector keyboard_detector.launch.py
```

## Testing and Debugging

### Verify Package Installation
```bash
# List executables in a package
ros2 pkg executables camera_publisher
ros2 pkg executables aruco_detection
ros2 pkg executables keyboard_detector

# Check if package is found
ros2 pkg prefix vision_bringup
```

### Monitor Topics
```bash
# List all active topics
ros2 topic list

# Check message rate
ros2 topic hz /image_raw
ros2 topic hz /aruco_markers

# Echo topic content
ros2 topic echo /aruco_markers
ros2 topic echo /keyboard_keys

# Show topic info
ros2 topic info /image_raw
```

### Verify TF Transforms
```bash
# Check if transform exists
ros2 run tf2_ros tf2_echo map camera_link

# View full TF tree
ros2 run tf2_tools view_frames
```

### Linting (Test Subsystem)
```bash
# Run Python linters
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Architecture: Vision Subsystem

### Data Flow
```
camera_publisher
  ↓ /image_raw (sensor_msgs/Image, ~30 Hz)
  ↓ /camera_info (sensor_msgs/CameraInfo)
aruco_detection
  ↓ /aruco_markers (vision_interfaces/ArucoMarkers)
keyboard_detector
  ↓ /keyboard_keys (vision_interfaces/KeyboardKeys)
  ↓ /keyboard_center (geometry_msgs/PoseStamped)
```

### Package Responsibilities

**camera_publisher**: Hardware abstraction layer
- Provides unified interface for USB cameras (`camera_publisher`) and RealSense (`realsense_publisher`)
- Publishes static TF: `map` → `camera_link`
- Configuration in `config/camera_parameters.yaml`

**aruco_detection**: Computer vision node
- Subscribes to camera feed, detects ArUco markers (DICT_4X4_50)
- Estimates 6DOF pose for each marker
- Publishes both custom message (`/aruco_markers`) and RViz-compatible poses
- Optional pose smoothing via exponential moving average
- Configuration: `config/aruco_parameters.yaml` (marker size, dictionary)

**keyboard_detector**: Higher-level perception
- Computes keyboard plane from 4 corner ArUco markers (IDs 0-3)
- Calculates individual key positions based on layout in `config/keys.yaml`
- Publishes keyboard center pose and per-key poses with labels

**vision_bringup**: Launch orchestration
- Brings up complete vision pipeline with parameter forwarding
- Manages RViz configuration and visualization nodes

### Custom Messages

Custom messages are defined in `src/interfaces/vision_interfaces/`:

**ArucoMarkers.msg**:
- `std_msgs/Header header`
- `int32[] marker_ids`
- `geometry_msgs/Pose[] poses`

**KeyboardKeys.msg**:
- `std_msgs/Header header`
- `geometry_msgs/Pose[] key_poses`
- `string[] key_names`

**When adding new custom messages**:
1. Add `.msg` file to `src/interfaces/<subsystem>_interfaces/msg/`
2. Update `CMakeLists.txt` to include the message
3. Rebuild: `colcon build --packages-select <subsystem>_interfaces`
4. Rebuild dependent packages: `colcon build --packages-up-to <dependent_package>`

## ROS2 Best Practices for This Workspace

1. **No Python Virtual Environments**: All dependencies managed via rosdep and system packages (apt)
2. **Use rosdep**: Add dependencies to `package.xml` and run `rosdep install`
3. **Exception for OpenCV**: Use pip with `--break-system-packages` only for OpenCV 4.8+
4. **Package Naming**: `<subsystem>_<component>` (e.g., `vision_bringup`, `arm_controller`)
5. **Custom Messages**: Place in `src/interfaces/<subsystem>_interfaces/`
6. **Launch Files**: Individual packages have their own launch files; bringup packages orchestrate subsystems

## Adding New Subsystems

When implementing navigation, arm, or communications subsystems, follow the vision pattern:

1. Create component packages in `src/<subsystem>/`
2. Create custom messages in `src/interfaces/<subsystem>_interfaces/`
3. Add individual launch files to each component package
4. Create a `<subsystem>_bringup` package for orchestration
5. Update rosdep: `rosdep install --from-paths src --ignore-src -r -y`
6. Build with: `colcon build --packages-up-to <subsystem>_bringup`

## Common Issues

**OpenCV ArUco API errors**: Ensure OpenCV >= 4.8 is installed. Check with:
```bash
python3 -c "import cv2; print(cv2.__version__)"
```

**Camera not found**:
- Check permissions: `ls -l /dev/video*`
- Add user to video group: `sudo usermod -aG video $USER`

**ROS nodes can't find each other**:
- Verify `ROS_DOMAIN_ID=42` is set
- Check network: `ros2 node list`, `ros2 topic list`

**Build fails with "package not found"**:
- Ensure dependencies are built first: use `--packages-up-to` instead of `--packages-select`
- Run rosdep: `rosdep install --from-paths src --ignore-src -r -y`

## Key Files for Navigation

- `/src/vision/README.md` - Detailed vision subsystem documentation
- `/.devcontainer/Dockerfile` - Development environment setup
- `/src/interfaces/vision_interfaces/README.md` - Custom message definitions
