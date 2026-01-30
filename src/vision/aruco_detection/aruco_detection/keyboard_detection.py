#!/usr/bin/env python3

"""
1. get 4 aruco markers, compute geometric center
2. load coords from yaml -> keyboard frame -> convert to camera frame
3. publish to /key_poses
"""


import rclpy
from rclpy.node import Node
from aruco_detection_interfaces.msg import ArucoMarkers, KeyboardKeys
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import yaml
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import tf_transformations


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')

        package_share = get_package_share_directory('aruco_detection')
        yaml_path = os.path.join(package_share, 'config', 'keys.yaml')
        
        self.get_logger().info(f'Loading keys from: {yaml_path}')

        try:
            # open r -> read 
            with open(yaml_path, 'r') as f:
                yaml_data = yaml.safe_load(f)
                self.key_coords = yaml_data['keys']

            self.get_logger().info(f'Loaded {len(self.key_coords)} keys from YAML')
        except Exception as e:
            self.get_logger().error(f'Failed to load keys.yaml: {e}')
            self.key_coords = {}

        # subscribe to aruco_markers topic to get aruco poses
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10
        )

        # create keyboard center publisher
        self.center_publisher = self.create_publisher(
            PoseStamped,
            '/keyboard_center',
            10
        )

        # create keyboard keys publisher
        self.keys_publisher = self.create_publisher(
            KeyboardKeys,
            '/keyboard_keys',
            10
        )
        
        # create keyboard pose publisher (for rviz)
        self.keyboard_pose_publisher = self.create_publisher(
            PoseArray,
            '/keyboard_pose',
            10
        )

        # marker IDs for the keyboard corners (see github readme)
        self.target_ids = [1, 4, 3, 2]

        self.get_logger().info('Keyboard detection node started. Listening for markers 1 (top-left), 4 (top-right), 3 (bottom-right), 2 (bottom-left)...')

    def create_transform_matrix(self, pose):
        """
        Convert geometry_msgs/Pose to 4x4 homogeneous transform matrix.

        Args:
            pose: geometry_msgs/Pose object

        Returns:
            4x4 numpy array representing the homogeneous transform
        """
        # get quaternion
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        # make 4x4 rotation transform matrix 
        transform = tf_transformations.quaternion_matrix(q)

        # something chat made :sob:
        transform[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]

        return transform

    def transform_keys_to_camera_frame(self, keyboard_center_pose):
        """
        Transform all key coordinates from keyboard frame to camera frame.

        Args:
            keyboard_center_pose: geometry_msgs/Pose of keyboard center in camera frame

        Returns:
            dict: {key_name: geometry_msgs/Pose in camera frame}
        """
        # get 4x4 transform matrix for keyboard center
        T_cam_from_kbd = self.create_transform_matrix(keyboard_center_pose)

        key_poses = {}
        # chat cooked here...
        for key_name, key_data in self.key_coords.items():
            # Create homogeneous point [x, y, 0, 1] in keyboard frame
            point_kbd = np.array([key_data['x']*6.7, key_data['y']*6.7, 0.0, 1.0])

            # Transform to camera frame
            point_cam = T_cam_from_kbd @ point_kbd

            # Create Pose message
            pose = Pose()
            pose.position.x = float(point_cam[0])
            pose.position.y = float(point_cam[1])
            pose.position.z = float(point_cam[2])

            # Use keyboard's orientation (all keys have same orientation)
            pose.orientation = keyboard_center_pose.orientation

            key_poses[key_name] = pose

        return key_poses

    def aruco_callback(self, msg):
        """
        Callback for ArucoMarkers messages.
        Filters for markers with IDs 1, 4, 3, 2 and computes their geometric center.
        """
        # Find poses for target marker IDs and build a dict
        target_poses_dict = {}

        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id in self.target_ids:
                if i < len(msg.poses):
                    target_poses_dict[marker_id] = msg.poses[i]
                    self.get_logger().debug(f'Found marker {marker_id} at index {i}')

        if not target_poses_dict:
            self.get_logger().debug('No target markers (1, 4, 3, 2) detected in this frame')
            return
        if len(target_poses_dict) < 4:
            self.get_logger().info(f'Only {len(target_poses_dict)} target markers detected; need all 4 to compute keyboard center')
            # TODO: use partial info to estimate center if we have at least 2 markers
            return


        # compute geometric center yahhhh
        target_poses = list(target_poses_dict.values())
        sum_x = sum(pose.position.x for pose in target_poses)
        sum_y = sum(pose.position.y for pose in target_poses)
        sum_z = sum(pose.position.z for pose in target_poses)

        center_x = sum_x / len(target_poses)
        center_y = sum_y / len(target_poses)
        center_z = sum_z / len(target_poses)

        # make keyboard center pose
        keyboard_center_pose = Pose()
        keyboard_center_pose.position.x = center_x
        keyboard_center_pose.position.y = center_y
        keyboard_center_pose.position.z = center_z

        # quaternions and orientations type shit
        keyboard_center_pose.orientation.x = sum(pose.orientation.x for pose in target_poses) / 4
        keyboard_center_pose.orientation.y = sum(pose.orientation.y for pose in target_poses) / 4
        keyboard_center_pose.orientation.z = sum(pose.orientation.z for pose in target_poses) / 4
        keyboard_center_pose.orientation.w = sum(pose.orientation.w for pose in target_poses) / 4

        # publish keyboard center (not rly needed anymore)
        center_msg = PoseStamped()
        center_msg.header = msg.header  # header contains frame; we want to keep it in camera frame
        center_msg.pose = keyboard_center_pose
        self.center_publisher.publish(center_msg)

        # transform key poses (YAML coords are in keyboard frame) to camera frame
        key_poses_dict = self.transform_keys_to_camera_frame(keyboard_center_pose)

        # publish key poses as PoseArray for RViz visualization
        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.poses = list(key_poses_dict.values())
        self.keyboard_pose_publisher.publish(pose_array)

        # build KeyboardKeys message
        keys_msg = KeyboardKeys()
        keys_msg.header = msg.header  # same frame as aruco markers' frame (camera frame)

        for key_name, pose in key_poses_dict.items():
            keys_msg.key_names.append(key_name)
            keys_msg.poses.append(pose)

        
        self.keys_publisher.publish(keys_msg)

        self.get_logger().info(
            f'Keyboard center: ({center_x:.3f}, {center_y:.3f}, {center_z:.3f}) | '
            f'Published {len(key_poses_dict)} key poses in camera frame'
        )


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
