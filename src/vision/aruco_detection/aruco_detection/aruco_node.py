"""
This node locates Aruco AR markers in images from multiple cameras and publishes their ids and poses.

Subscriptions:
   /camera_left/image_raw (sensor_msgs.msg.Image)
   /camera_left/camera_info (sensor_msgs.msg.CameraInfo)
   /camera_front/image_raw (sensor_msgs.msg.Image)
   /camera_front/camera_info (sensor_msgs.msg.CameraInfo)
   /camera_right/image_raw (sensor_msgs.msg.Image)
   /camera_right/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /camera_left/aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)
    /camera_front/aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)
    /camera_right/aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /camera_left/aruco_markers (vision_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding marker ids
    /camera_front/aruco_markers (vision_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding marker ids
    /camera_right/aruco_markers (vision_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding marker ids

    /camera_left/aruco_detection/image (sensor_msgs.msg.Image)
       Debug image with detected markers and coordinate axes
    /camera_front/aruco_detection/image (sensor_msgs.msg.Image)
       Debug image with detected markers and coordinate axes
    /camera_right/aruco_detection/image (sensor_msgs.msg.Image)
       Debug image with detected markers and coordinate axes

Parameters:
    marker_size - size of the markers in meters (default 0.02)
    aruco_dictionary_id - dictionary that was used to generate markers (default DICT_4X4_50)
    camera_frames - mapping of camera names to frame IDs
    frame_skip - frame skipping interval (0 = process all frames)
    smoothing_alpha - pose smoothing factor (0.7 default)
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
from vision_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Check OpenCV version - require 4.7+ for ArucoDetector API
def check_opencv_version():
    cv_version = tuple(map(int, cv2.__version__.split('.')[:2]))
    if cv_version < (4, 7):
        raise ImportError(
            f"OpenCV {cv2.__version__} detected. This package requires OpenCV >= 4.7.0 for ArUco detection.\n"
            f"Install with: pip install opencv-contrib-python>=4.8.0"
        )

check_opencv_version()


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.camera_names = ['left', 'front', 'right']

        # Declare individual camera frame parameters
        self.declare_parameter("camera_frames.left", "left_camera_link")
        self.declare_parameter("camera_frames.front", "front_camera_link")
        self.declare_parameter("camera_frames.right", "right_camera_link")

        # frame skip parameter
        self.declare_parameter("frame_skip", 0)
        self.frame_skip = self.get_parameter("frame_skip").get_parameter_value().integer_value
        self.get_logger().info(f"Frame skip: {self.frame_skip}")

        # smoothing parameter
        self.declare_parameter("smoothing_alpha", 0.7)
        self.smoothing_alpha = self.get_parameter("smoothing_alpha").get_parameter_value().double_value
        self.get_logger().info(f"Smoothing alpha: {self.smoothing_alpha}")

        # get camera frames from parameters
        self.camera_frames = {}
        for camera in self.camera_names:
            param_name = f"camera_frames.{camera}"
            self.camera_frames[camera] = self.get_parameter(param_name).get_parameter_value().string_value
            if not self.camera_frames[camera]:
                self.get_logger().error(f"Missing camera_frame for {camera}")
                raise ValueError(f"camera_frames.{camera} not defined in parameters")

        self.get_logger().info(f"Camera frames: {self.camera_frames}")

        self.marker_size = 0.02 # meters
        self.get_logger().info(f"Marker size: {self.marker_size}")

        self.marker_family = 'DICT_4X4_50'
        self.get_logger().info(f"Marker type: {self.marker_family}")

        dictionary_id = cv2.aruco.__getattribute__(self.marker_family)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        self.bridge = CvBridge()

        # Per-camera state storage
        self.camera_infos = {camera: None for camera in self.camera_names}
        self.intrinsic_mats = {camera: None for camera in self.camera_names}
        self.distortions = {camera: None for camera in self.camera_names}
        self.pose_trackers = {camera: {} for camera in self.camera_names}  # {camera: {marker_id: (pos, quat)}}
        self.frame_counters = {camera: 0 for camera in self.camera_names}
        self.info_subs = {}  # Store info subscriptions to destroy after first message

        # Create subscriptions and publishers for each camera
        for camera in self.camera_names:
            image_topic = f'/camera_{camera}/image_raw'
            info_topic = f'/camera_{camera}/camera_info'

            self.get_logger().info(f"Subscribing to {camera} camera:")
            self.get_logger().info(f"  Image topic: {image_topic}")
            self.get_logger().info(f"  Info topic: {info_topic}")

            # Subscribe to camera info (will be destroyed after first message)
            self.info_subs[camera] = self.create_subscription(
                CameraInfo, 
                info_topic, 
                lambda msg, cam=camera: self.info_callback(msg, cam),
                qos_profile_sensor_data
            )

            # Subscribe to raw image
            self.create_subscription(
                Image, 
                image_topic, 
                lambda msg, cam=camera: self.image_callback(msg, cam),
                qos_profile_sensor_data
            )

        # Create publishers for each camera
        self.poses_pubs = {}
        self.markers_pubs = {}
        self.image_pubs = {}

        for camera in self.camera_names:
            self.poses_pubs[camera] = self.create_publisher(
                PoseArray, f"/camera_{camera}/aruco_poses", 10
            )
            self.markers_pubs[camera] = self.create_publisher(
                ArucoMarkers, f"/camera_{camera}/aruco_markers", 10
            )
            self.image_pubs[camera] = self.create_publisher(
                Image, f"/camera_{camera}/aruco_detection/image", 10
            )

        self.get_logger().info(f"Multi-camera ArUco node initialized for {len(self.camera_names)} cameras")

    def info_callback(self, info_msg, camera):
        """Store camera info for a specific camera and unsubscribe."""
        self.camera_infos[camera] = info_msg
        self.intrinsic_mats[camera] = np.reshape(np.array(info_msg.k), (3, 3))
        self.distortions[camera] = np.array(info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_subs[camera])
        self.get_logger().info(f"Camera info received for {camera} camera")

    def image_callback(self, img_msg, camera):
        """Process image from a specific camera and detect ArUco markers."""
        # Check if camera info has been received
        if self.camera_infos[camera] is None:
            self.get_logger().warn(f"No camera info received for {camera} camera yet!")
            return

        # Frame skipping logic
        self.frame_counters[camera] += 1
        if self.frame_skip > 0 and self.frame_counters[camera] % (self.frame_skip + 1) != 0:
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg)
        markers = ArucoMarkers()
        pose_array = PoseArray()
        # Set frame_id from camera configuration
        markers.header.frame_id = self.camera_frames[camera]
        pose_array.header.frame_id = self.camera_frames[camera]
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
                    self.intrinsic_mats[camera], self.distortions[camera]
                )
                
                if success:
                    # Draw coordinate axes on the marker
                    cv2.drawFrameAxes(output, self.intrinsic_mats[camera], self.distortions[camera], 
                                    rvec, tvec, self.marker_size * 1.5, 2)
                    
                    pose = Pose()
                    
                    # Current raw pose
                    curr_pos = np.array([tvec[0][0], tvec[1][0], tvec[2][0]])
                    
                    rot_matrix = np.eye(4)
                    rot_matrix[0:3, 0:3] = cv2.Rodrigues(rvec)[0]
                    curr_quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                    # Apply smoothing if available
                    key = marker_id[0]
                    if key in self.pose_trackers[camera]:
                        prev_pos, prev_quat = self.pose_trackers[camera][key]
                        
                        # Smooth position (linear interpolation)
                        alpha = self.smoothing_alpha
                        new_pos = alpha * curr_pos + (1.0 - alpha) * prev_pos
                        
                        # Smooth orientation (SLERP)
                        new_quat = tf_transformations.quaternion_slerp(prev_quat, curr_quat, alpha)
                        
                        self.pose_trackers[camera][key] = (new_pos, new_quat)
                        
                        pose.position.x = new_pos[0]
                        pose.position.y = new_pos[1]
                        pose.position.z = new_pos[2]
                        pose.orientation.x = new_quat[0]
                        pose.orientation.y = new_quat[1]
                        pose.orientation.z = new_quat[2]
                        pose.orientation.w = new_quat[3]
                    else:
                        # First time seeing this marker, no smoothing
                        self.pose_trackers[camera][key] = (curr_pos, curr_quat)
                        
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

            self.poses_pubs[camera].publish(pose_array)
            self.markers_pubs[camera].publish(markers)

        # Publish annotated image for RViz visualization
        output_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        output_msg.header = img_msg.header
        self.image_pubs[camera].publish(output_msg)

        # Display the image with markers and axes
        window_name = f"ArUco - {camera.capitalize()} Camera"
        cv2.imshow(window_name, output)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
