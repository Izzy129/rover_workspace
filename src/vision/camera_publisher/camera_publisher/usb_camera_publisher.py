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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

# Camera configuration
CAMERA_CONFIGS = [
    {'name': 'left', 'device_index': 2, 'frame_id': 'left_camera_link'},
    {'name': 'front', 'device_index': 0, 'frame_id': 'front_camera_link'},
    {'name': 'right', 'device_index': 6, 'frame_id': 'right_camera_link'},
]
PUBLISH_RATE = 30.0  # Hz


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

        # Initialize all cameras
        for config in CAMERA_CONFIGS:
            name = config['name']
            device_index = config['device_index']
            frame_id = config['frame_id']

            self.get_logger().info(f"Initializing {name} camera (device {device_index})...")

            # Open camera
            cap = cv2.VideoCapture(device_index)
            if not cap.isOpened():
                self.get_logger().error(f'Cannot open {name} camera at device {device_index}')
                # Fail entire node if any camera fails to open
                raise RuntimeError(f'Failed to open {name} camera at device {device_index}')

            self.cameras[name] = cap

            # Create publishers
            self.image_pubs[name] = self.create_publisher(Image, f'/camera_{name}/image_raw', 10)
            self.info_pubs[name] = self.create_publisher(CameraInfo, f'/camera_{name}/camera_info', 10)

            # Set camera info (basic defaults for USB camera)
            camera_info = CameraInfo()
            camera_info.header.frame_id = frame_id
            camera_info.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            camera_info.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            # Simple pinhole model
            camera_info.distortion_model = 'plumb_bob'
            # Focal length approximation (adjust as needed)
            fx = fy = min(camera_info.width, camera_info.height) * 0.8
            cx = camera_info.width / 2.0
            cy = camera_info.height / 2.0
            camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion

            self.camera_infos[name] = camera_info
            self.get_logger().info(f"{name} camera initialized successfully")

        # Timer to publish at configured rate
        self.timer = self.create_timer(1.0/PUBLISH_RATE, self.timer_callback)

        self.get_logger().info(f'Camera publisher started with {len(self.cameras)} cameras at {PUBLISH_RATE} Hz')

    def timer_callback(self):
        # Get timestamp for all cameras
        timestamp = self.get_clock().now().to_msg()

        # Read and publish from all cameras
        for name, cap in self.cameras.items():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f'Failed to capture frame from {name} camera')
                continue

            # Convert to ROS Image
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.stamp = timestamp
            ros_image.header.frame_id = self.camera_infos[name].header.frame_id

            # Publish image
            self.image_pubs[name].publish(ros_image)

            # Publish camera info
            self.camera_infos[name].header.stamp = timestamp
            self.info_pubs[name].publish(self.camera_infos[name])

    def destroy_node(self):
        # Release all cameras
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