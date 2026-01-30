#!/usr/bin/env python3

"""
Keyboard Visualizer Node

Subscribes to:
    /keyboard_keys (aruco_detection_interfaces/KeyboardKeys) - all key poses
    /keyboard_center (geometry_msgs/PoseStamped) - keyboard center position
    /image_raw (sensor_msgs/Image) - raw camera image
    camera_info (sensor_msgs/CameraInfo) - camera intrinsics

Displays:
    CV window with keyboard center (green) and individual key poses (blue dots) overlaid on camera image
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from aruco_detection_interfaces.msg import KeyboardKeys
from cv_bridge import CvBridge
import cv2
import numpy as np


class KeyboardVisualizer(Node):
    def __init__(self):
        super().__init__('keyboard_visualizer')

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Store camera info and keyboard data
        self.camera_info = None
        self.intrinsic_mat = None
        self.keyboard_center_3d = None  # (x, y, z) in camera frame
        self.key_poses = {}  # {key_name: (x, y, z)} in camera frame

        # Subscribe to camera info
        self.info_sub = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.info_callback,
            qos_profile_sensor_data
        )

        # Subscribe to raw image
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Subscribe to keyboard center
        self.center_sub = self.create_subscription(
            PoseStamped,
            '/keyboard_center',
            self.center_callback,
            10
        )

        # Subscribe to keyboard keys
        self.keys_sub = self.create_subscription(
            KeyboardKeys,
            '/keyboard_keys',
            self.keys_callback,
            10
        )

        self.get_logger().info('Keyboard visualizer started. Waiting for camera info and keyboard data...')

    def info_callback(self, info_msg):
        """Callback for camera info - extract intrinsic matrix"""
        self.camera_info = info_msg
        self.intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
        # Only need to get this once
        self.destroy_subscription(self.info_sub)
        self.get_logger().info('Camera info received')

    def center_callback(self, center_msg):
        """Callback for keyboard center - store the 3D position"""
        self.keyboard_center_3d = (
            center_msg.pose.position.x,
            center_msg.pose.position.y,
            center_msg.pose.position.z
        )

    def keys_callback(self, keys_msg):
        """Callback for keyboard keys - store all key poses"""
        self.key_poses = {}
        for i, key_name in enumerate(keys_msg.key_names):
            pose = keys_msg.poses[i]
            self.key_poses[key_name] = (
                pose.position.x,
                pose.position.y,
                pose.position.z
            )

    def image_callback(self, img_msg):
        """Callback for image - draw the keyboard center on the image"""
        if self.camera_info is None or self.intrinsic_mat is None:
            return

        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(img_msg)
        output = cv_image.copy()

        # If we have a keyboard center, project it to image coordinates and draw it
        if self.keyboard_center_3d is not None:
            x, y, z = self.keyboard_center_3d

            # Project 3D point to 2D image coordinates
            # Using pinhole camera model: u = fx * X/Z + cx, v = fy * Y/Z + cy
            fx = self.intrinsic_mat[0, 0]
            fy = self.intrinsic_mat[1, 1]
            cx = self.intrinsic_mat[0, 2]
            cy = self.intrinsic_mat[1, 2]

            if z > 0:  # Only project if in front of camera
                u = int(fx * x / z + cx)
                v = int(fy * y / z + cy)

                # Draw a circle at the keyboard center
                cv2.circle(output, (u, v), 15, (0, 255, 0), 3)  # Green circle
                cv2.circle(output, (u, v), 3, (0, 0, 255), -1)  # Red center dot

                # Draw crosshair
                cv2.line(output, (u - 25, v), (u + 25, v), (0, 255, 0), 2)
                cv2.line(output, (u, v - 25), (u, v + 25), (0, 255, 0), 2)

                # Draw text label
                label = f"Keyboard Center ({x:.2f}, {y:.2f}, {z:.2f})"
                cv2.putText(output, label, (u + 20, v - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Draw individual key poses
        if self.key_poses:
            # Extract camera intrinsics (reuse from above)
            fx = self.intrinsic_mat[0, 0]
            fy = self.intrinsic_mat[1, 1]
            cx = self.intrinsic_mat[0, 2]
            cy = self.intrinsic_mat[1, 2]

            for key_name, (x, y, z) in self.key_poses.items():
                if z > 0:  # Only project if in front of camera
                    # Project 3D point to 2D (reuse same formula)
                    u = int(fx * x / z + cx)
                    v = int(fy * y / z + cy)

                    # Draw small blue dot for each key
                    cv2.circle(output, (u, v), 5, (0, 255, 0), -1)  # Blue filled circle

        # Display the image with keyboard center overlay (resize to fit screen)
        display_img = cv2.resize(output, (0, 0), fx=0.50, fy=0.50)
        cv2.imshow("Keyboard Center Visualization", display_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
