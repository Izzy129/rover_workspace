#!/usr/bin/env python3

"""
Camera Publisher Node

This node captures images from multiple USB cameras (webcams, V4L2-compatible devices) and publishes them to ROS2 topics.

Published Topics:
    /camera_left/image_raw (sensor_msgs.msg.Image)
    /camera_left/camera_info (sensor_msgs.msg.CameraInfo)
    /camera_front/image_raw (sensor_msgs.msg.Image)
    /camera_front/camera_info (sensor_msgs.msg.CameraInfo)
    /camera_right/image_raw (sensor_msgs.msg.Image)
    /camera_right/camera_info (sensor_msgs.msg.CameraInfo)
"""

import threading

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.srv import SetCameraInfo

# Camera configuration
CAMERA_CONFIGS = [
    {'name': 'left', 'device_index': 0, 'frame_id': 'left_camera_link'},
    {'name': 'front', 'device_index': 4, 'frame_id': 'front_camera_link'},
    {'name': 'right', 'device_index': 8, 'frame_id': 'right_camera_link'},
]
PUBLISH_RATE = 30.0  # Hz
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# All 3 cameras are the same model so they share one calibration file
_share = get_package_share_directory('camera_publisher')
CALIBRATION_YAML = f'{_share}/config/calibration/ardu_camera.yaml'


class UsbCameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # CV Bridge
        self.bridge = CvBridge()

        # Storage for cameras and publishers
        self.cameras = {}
        self.image_pubs = {}
        self.info_pubs = {}
        self.camera_infos = {}

        # ROS image storage 
        self.ros_images = {}
        self.ros_image_locks = {}

        # Initialize all cameras
        for config in CAMERA_CONFIGS:
            name = config['name']
            device_index = config['device_index']
            frame_id = config['frame_id']

            self.get_logger().info(f"Initializing {name} camera (device {device_index})...")

            # Open camera
            # Gstreamer pipeline
            pipeline = (
                f"v4l2src device=/dev/video{device_index} ! "
                f"image/jpeg, width={CAMERA_WIDTH}, height={CAMERA_HEIGHT}, framerate={CAMERA_FPS}/1 ! "
                "jpegdec ! "
                "nvvidconv ! "
                "video/x-raw, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! "
                "appsink drop=1"
            )
            self.get_logger().info("HIIII THIS IS PIPELINEEE")
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
            if not cap.isOpened():
                self.get_logger().error(f'Cannot open {name} camera at device {device_index}')
                raise RuntimeError(f'Failed to open {name} camera at device {device_index}')

            self.cameras[name] = cap

            # Create publishers
            self.image_pubs[name] = self.create_publisher(Image, f'/camera_{name}/image_raw', 10)
            self.info_pubs[name] = self.create_publisher(CameraInfo, f'/camera_{name}/camera_info', 10)

            # Load camera info from calibration YAML
            camera_info = self._load_camera_info(CALIBRATION_YAML, frame_id)
            self.camera_infos[name] = camera_info

            # Frame buffer and lock for this camera
            self.ros_images[name] = None
            self.ros_image_locks[name] = threading.Lock()
            # Dedicated capture thread so cap.read() never blocks the ROS executor
            t = threading.Thread(target=self._capture_loop, args=(name,), daemon=True)
            t.start()

            self.get_logger().info(f"{name} camera initialized successfully")

        # Per-camera timers to publish at configured rate
        for config in CAMERA_CONFIGS:
            name = config['name']
            self.create_timer(1.0 / PUBLISH_RATE, lambda n=name: self.publish_camera(n))

        self.get_logger().info(f'Camera publisher started with {len(self.cameras)} cameras at {PUBLISH_RATE} Hz')

    def _load_camera_info(self, yaml_path, frame_id):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = data['image_width']
        info.height = data['image_height']
        info.distortion_model = data['distortion_model']
        info.k = data['camera_matrix']['data']
        info.d = data['distortion_coefficients']['data']
        info.r = data['rectification_matrix']['data']
        info.p = data['projection_matrix']['data']
        return info

    def _capture_loop(self, name):
        """Captures frames and pre-converts to ROS Image in background thread."""
        cap = self.cameras[name]
        frame_id = next(c['frame_id'] for c in CAMERA_CONFIGS if c['name'] == name)
        while rclpy.ok():
            ret, frame = cap.read()
            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                ros_image.header.frame_id = frame_id
                with self.ros_image_locks[name]:
                    self.ros_images[name] = ros_image
            else:
                self.get_logger().warn(
                    f'Failed to capture frame from {name} camera',
                    throttle_duration_sec=5.0
                )

    def publish_camera(self, name):
        with self.ros_image_locks[name]:
            ros_image = self.ros_images[name]

        if ros_image is None:
            return

        ros_image.header.stamp = self.get_clock().now().to_msg()
        self.image_pubs[name].publish(ros_image)

        self.camera_infos[name].header.stamp = ros_image.header.stamp
        self.info_pubs[name].publish(self.camera_infos[name])
    
    def destroy_node(self):
        for name, cap in self.cameras.items():
            self.get_logger().info(f'Releasing {name} camera')
            cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
