#!/usr/bin/env python3

"""
RealSense Publisher Node

This node subscribes to Intel RealSense D435i camera streams and republishes
them to standardized topics for compatibility with the ArUco detection node.

You can select which stream to use (color, depth, infrared) via the 'stream_type' parameter.

Published Topics:
    /image_raw (sensor_msgs.msg.Image)
    /camera_info (sensor_msgs.msg.CameraInfo)

Subscribed Topics (based on stream_type parameter):
    /camera/color/image_raw (sensor_msgs.msg.Image) - when stream_type='color'
    /camera/color/camera_info (sensor_msgs.msg.CameraInfo)
    /camera/depth/image_rect_raw (sensor_msgs.msg.Image) - when stream_type='depth'
    /camera/depth/camera_info (sensor_msgs.msg.CameraInfo)
    /camera/infra1/image_rect_raw (sensor_msgs.msg.Image) - when stream_type='infra1'
    /camera/infra1/camera_info (sensor_msgs.msg.CameraInfo)
    /camera/infra2/image_rect_raw (sensor_msgs.msg.Image) - when stream_type='infra2'
    /camera/infra2/camera_info (sensor_msgs.msg.CameraInfo)

Parameters:
    stream_type (string): Type of stream to use - 'color', 'depth', 'infra1', or 'infra2'
                          Default: 'color'
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')

        # Declare and get parameter for stream type
        self.declare_parameter('stream_type', 'color')
        self.stream_type = self.get_parameter('stream_type').get_parameter_value().string_value

        # Validate stream type
        valid_streams = ['color', 'depth', 'infra1', 'infra2']
        if self.stream_type not in valid_streams:
            self.get_logger().error(
                f"Invalid stream_type '{self.stream_type}'. Must be one of {valid_streams}"
            )
            return

        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()

        # Create publishers for standardized topics
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)

        # Determine input topics based on stream type
        # Note: ROS2 Jazzy uses /camera/realsense2_camera/ namespace
        if self.stream_type == 'color':
            image_topic = '/camera/realsense2_camera/color/image_raw'
            info_topic = '/camera/realsense2_camera/color/camera_info'
        elif self.stream_type == 'depth':
            image_topic = '/camera/realsense2_camera/depth/image_rect_raw'
            info_topic = '/camera/realsense2_camera/depth/camera_info'
        elif self.stream_type == 'infra1':
            image_topic = '/camera/realsense2_camera/infra1/image_rect_raw'
            info_topic = '/camera/realsense2_camera/infra1/camera_info'
        else:  # infra2
            image_topic = '/camera/realsense2_camera/infra2/image_rect_raw'
            info_topic = '/camera/realsense2_camera/infra2/camera_info'

        # Create subscribers to RealSense topics
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            info_topic,
            self.info_callback,
            10
        )

        self.get_logger().info(
            f'RealSense publisher started - Using {self.stream_type} stream'
        )
        self.get_logger().info(f'  Image topic: {image_topic}')
        self.get_logger().info(f'  Info topic: {info_topic}')
        self.get_logger().info(f'  Publishing to: /image_raw and /camera_info')

    def image_callback(self, msg):
        """Republish image from RealSense to standardized topic with color conversion"""
        # RealSense publishes color images in RGB8 format, but OpenCV/ArUco expects BGR8
        if self.stream_type == 'color' and msg.encoding == 'rgb8':
            try:
                # Convert RGB to BGR
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                msg_out = self.bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
                msg_out.header = msg.header
                
                self.image_pub.publish(msg_out)
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')
        else:
            # For depth/infrared streams, pass through as-is
            self.image_pub.publish(msg)

    def info_callback(self, msg):
        """Republish camera info from RealSense to standardized topic"""
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
