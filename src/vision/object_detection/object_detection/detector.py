"""
This node runs YOLO object detection on images from multiple cameras and publishes annotated results.

Subscriptions:
    /camera_left/image_raw (sensor_msgs.msg.Image)
    /camera_front/image_raw (sensor_msgs.msg.Image)
    /camera_right/image_raw (sensor_msgs.msg.Image)

Published Topics:
    /camera_left/object_detection/image_raw (sensor_msgs.msg.Image)
        Debug image with detected objects and bounding boxes
    /camera_front/object_detection/image_raw (sensor_msgs.msg.Image)
        Debug image with detected objects and bounding boxes
    /camera_right/object_detection/image_raw (sensor_msgs.msg.Image)
        Debug image with detected objects and bounding boxes
"""
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
import os

from ultralytics import YOLO
import ros2_numpy as rnp

class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection")

        model_path = os.path.join(get_package_share_directory('object_detection'), 'best.pt')
        self.yolo = YOLO(model_path)
        
        
        self.camera_names = ['left', 'front', 'right']
        self.detected_image_pubs = {}

        for camera in self.camera_names:
            self.create_subscription(
                Image,
                f'/camera_{camera}/image_raw',
                lambda msg, cam=camera: self.image_callback(msg, cam),
                qos_profile_sensor_data
            )

            self.detected_image_pubs[camera] = self.create_publisher(
                Image,
                f"/camera_{camera}/object_detection/image_raw",
                10
            )

    def image_callback(self, img_msg, camera):
        array = rnp.numpify(img_msg)

        # YOLO is costly, only run if needed (i.e. detected image topic has subscribers)
        if self.detected_image_pubs[camera].get_subscription_count() > 0:
            det_result = self.yolo(array)
            det_annotated = det_result[0].plot(show=False)
            self.detected_image_pubs[camera].publish(rnp.msgify(Image, det_annotated, encoding="bgr8"))
            cv2.imshow(f"{camera.capitalize()} Camera YOLO Detections", det_annotated)
            cv2.waitKey(1)

def main():
    rclpy.init()
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
