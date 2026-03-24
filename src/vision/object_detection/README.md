# object_detection

YOLO-based object detection across all three rover cameras.

## Executables

### `detector`

Runs YOLO inference on images from the left, front, and right cameras. Only runs inference when a subscriber is listening to the output topic.

**Subscribed Topics:**
- `/camera_left/image_raw` (sensor_msgs/Image)
- `/camera_front/image_raw` (sensor_msgs/Image)
- `/camera_right/image_raw` (sensor_msgs/Image)

**Published Topics:**
- `/camera_left/object_detection/image_raw` (sensor_msgs/Image) - Annotated image with bounding boxes
- `/camera_front/object_detection/image_raw` (sensor_msgs/Image) - Annotated image with bounding boxes
- `/camera_right/object_detection/image_raw` (sensor_msgs/Image) - Annotated image with bounding boxes

## Run

```bash
ros2 run object_detection detector
```

## Model

The YOLO model (`best.pt`) is bundled with the package. It is loaded from the package share directory at runtime.

## Dependencies

- rclpy
- sensor_msgs
- python3-numpy
- ros2_numpy (vendored at `src/vision/ros2_numpy`)
- ultralytics (installed via pip)
