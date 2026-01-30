"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (aruco_detection_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from aruco_detection_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.marker_size = 0.02 # meters
        self.get_logger().info(f"Marker size: {self.marker_size}")

        self.marker_family = 'DICT_4X4_50'
        self.get_logger().info(f"Marker type: {self.marker_family}")

        image_topic = '/image_raw'
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = 'camera_info'
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.declare_parameter("smoothing_alpha", 0.7)
        self.smoothing_alpha = self.get_parameter("smoothing_alpha").get_parameter_value().double_value
        self.get_logger().info(f"Smoothing alpha: {self.smoothing_alpha}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)
        self.image_pub = self.create_publisher(Image, "aruco_detection/image", 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        dictionary_id = cv2.aruco.__getattribute__(self.marker_family)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        self.bridge = CvBridge()
        
        # Dictionary to store the previous smoothed pose for each marker ID
        # Format: {marker_id: (position_np_array, orientation_quaternion_np_array)}
        self.pose_tracker = {}

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg)
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = self.detector.detectMarkers(cv_image)
        output = cv_image.copy()
        cv2.aruco.drawDetectedMarkers(output, corners, marker_ids)
        

        if marker_ids is not None:
            for i, marker_id in enumerate(marker_ids):
                # Get the 2D image points (detected corners)
                image_points = corners[i][0].astype(np.float32)
                
                # Define 3D object points (marker corners in marker coordinate system)
                half_size = self.marker_size / 2.0
                object_points = np.array([
                    [-half_size, half_size, 0],
                    [half_size, half_size, 0],
                    [half_size, -half_size, 0],
                    [-half_size, -half_size, 0]
                ], dtype=np.float32)
                
                # Solve PnP to get pose
                success, rvec, tvec = cv2.solvePnP(
                    object_points, image_points, 
                    self.intrinsic_mat, self.distortion
                )
                
                if success:
                    # Draw coordinate axes on the marker
                    cv2.drawFrameAxes(output, self.intrinsic_mat, self.distortion, 
                                    rvec, tvec, self.marker_size * 1.5, 2)
                    
                    pose = Pose()
                    
                    # Current raw pose
                    curr_pos = np.array([tvec[0][0], tvec[1][0], tvec[2][0]])
                    
                    rot_matrix = np.eye(4)
                    rot_matrix[0:3, 0:3] = cv2.Rodrigues(rvec)[0]
                    curr_quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                    # Apply smoothing if available
                    key = marker_id[0]
                    if key in self.pose_tracker:
                        prev_pos, prev_quat = self.pose_tracker[key]
                        
                        # Smooth position (linear interpolation)
                        alpha = self.smoothing_alpha
                        new_pos = alpha * curr_pos + (1.0 - alpha) * prev_pos
                        
                        # Smooth orientation (SLERP)
                        new_quat = tf_transformations.quaternion_slerp(prev_quat, curr_quat, alpha)
                        
                        self.pose_tracker[key] = (new_pos, new_quat)
                        
                        pose.position.x = new_pos[0]
                        pose.position.y = new_pos[1]
                        pose.position.z = new_pos[2]
                        pose.orientation.x = new_quat[0]
                        pose.orientation.y = new_quat[1]
                        pose.orientation.z = new_quat[2]
                        pose.orientation.w = new_quat[3]
                    else:
                        # First time seeing this marker, no smoothing
                        self.pose_tracker[key] = (curr_pos, curr_quat)
                        
                        pose.position.x = curr_pos[0]
                        pose.position.y = curr_pos[1]
                        pose.position.z = curr_pos[2]
                        pose.orientation.x = curr_quat[0]
                        pose.orientation.y = curr_quat[1]
                        pose.orientation.z = curr_quat[2]
                        pose.orientation.w = curr_quat[3]

                    pose_array.poses.append(pose)
                    markers.poses.append(pose)
                    markers.marker_ids.append(marker_id[0])

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

        # Publish annotated image for RViz visualization
        output_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        output_msg.header = img_msg.header
        self.image_pub.publish(output_msg)

        # Display the image with markers and axes
        cv2.imshow("Detected Markers", output)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
