# camera_publisher

Camera abstraction layer providing a unified interface for ArduCam and RealSense cameras for the Rover. 

## Executables

### camera_publisher
Publishes images from USB cameras.

**Published Topics:**
- `/image_raw` (sensor_msgs/Image) - Raw camera image at ~30 Hz
- `/camera_info` (sensor_msgs/CameraInfo) - Camera calibration info

### realsense_publisher
Publishes images from Intel RealSense depth cameras.

**Published Topics:**
- `/image_raw` (sensor_msgs/Image) - Selected stream (color/depth/infrared)
- `/camera_info` (sensor_msgs/CameraInfo) - Camera calibration info

**Parameters:**
- `stream_type` (string, default: "color") - Stream type: color, depth, infra1, infra2

## Launch

```bash
# Launch with webcam
ros2 launch camera_publisher camera_publisher.launch.py camera_type:=webcam

# Launch with RealSense (color stream)
ros2 launch camera_publisher camera_publisher.launch.py camera_type:=realsense stream_type:=color

# Launch with RealSense (depth stream)
ros2 launch camera_publisher camera_publisher.launch.py camera_type:=realsense stream_type:=depth

# Launch with RealSense (left infrared stream)
ros2 launch camera_publisher camera_publisher.launch.py camera_type:=realsense stream_type:=infra1

# Launch with RealSense (right infrared stream)
ros2 launch camera_publisher camera_publisher.launch.py camera_type:=realsense stream_type:=infra2
```

## Static Transform

Publishes static TF: `map` → `camera_link`

## Dependencies

- rclpy
- sensor_msgs
- tf2_ros
- cv_bridge
- python3-opencv
- python3-numpy
