# keyboard_detector

Keyboard pose estimation from ArUco markers for the University Rover Challenge (URC).

Detects four corner ArUco markers to establish a keyboard reference frame, then computes individual key positions based on the configured keyboard layout.

## Executables

### keyboard_detection
Computes keyboard and key poses from detected ArUco markers.

**Subscribed Topics:**
- `/aruco_markers` (vision_interfaces/ArucoMarkers) - Detected ArUco markers

**Published Topics:**
- `/keyboard_center` (geometry_msgs/PoseStamped) - Center pose of keyboard
- `/keyboard_keys` (vision_interfaces/KeyboardKeys) - Individual key poses
- `/keyboard_pose` (geometry_msgs/PoseArray) - Key poses for RViz visualization

**Configuration:**
- `config/keys.yaml` - Keyboard layout with key positions relative to keyboard center

### keyboard_visualizer
OpenCV-based visualization showing keyboard and key detection in real-time.

**Subscribed Topics:**
- `/image_raw` (sensor_msgs/Image) - Camera image
- `/keyboard_center` (geometry_msgs/PoseStamped) - Keyboard center
- `/keyboard_keys` (vision_interfaces/KeyboardKeys) - Key poses

## Launch

```bash
# Launch keyboard detection only
ros2 launch keyboard_detector keyboard_detector.launch.py

# Launch with OpenCV visualizer
ros2 launch keyboard_detector keyboard_detector.launch.py visualize_keyboard:=true
```

## Configuration

Edit `config/keys.yaml` to define keyboard layout:
```yaml
keys:
  - name: "A"
    x: 0.05
    y: 0.03
  - name: "B"
    x: 0.10
    y: 0.03
  ...
```

## Dependencies

- rclpy
- geometry_msgs
- tf2_ros
- vision_interfaces
- python3-numpy
- python3-opencv
- python3-yaml
- python3-transforms3d
