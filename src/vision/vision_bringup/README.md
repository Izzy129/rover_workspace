# vision_bringup

Vision subsystem launch orchestration for the rover.

This package provides a unified launch interface that brings up the complete vision pipeline: camera, ArUco detection, and keyboard detection.

## Launch

### Full Vision System

```bash
ros2 launch vision_bringup vision_bringup.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `camera_type` | `webcam` | Camera type: `webcam` or `realsense` |
| `stream_type` | `color` | RealSense stream: `color`, `depth`, `infra1`, `infra2` |
| `use_rviz` | `true` | Start RViz for visualization |
| `visualize_keyboard` | `false` | Start OpenCV keyboard visualizer |
| `smoothing_alpha` | `0.7` | ArUco pose smoothing (0.0-1.0) |

### Example Usage

```bash
# Webcam with RViz
ros2 launch vision_bringup vision_bringup.launch.py camera_type:=webcam use_rviz:=true

# RealSense depth stream with keyboard visualizer
ros2 launch vision_bringup vision_bringup.launch.py \
    camera_type:=realsense \
    stream_type:=depth \
    visualize_keyboard:=true

# Webcam without visualization
ros2 launch vision_bringup vision_bringup.launch.py \
    camera_type:=webcam \
    use_rviz:=false
```

## System Architecture

```
camera_publisher → aruco_detection → keyboard_detector
                         ↓
                      RViz (optional)
                         ↓
                  keyboard_visualizer (optional)
```

## Published Topics

- `/image_raw` - Camera feed
- `/camera_info` - Camera calibration
- `/aruco_markers` - Detected markers with poses
- `/aruco_poses` - PoseArray for RViz
- `/aruco_detection/image` - Annotated image
- `/keyboard_center` - Keyboard center pose
- `/keyboard_keys` - Individual key poses
- `/keyboard_pose` - PoseArray for RViz
- `/tf`, `/tf_static` - Transform tree

## Dependencies

This package depends on:
- camera_publisher
- aruco_detection
- keyboard_detector
- vision_interfaces
