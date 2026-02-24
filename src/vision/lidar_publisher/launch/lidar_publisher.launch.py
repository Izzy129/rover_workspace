import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_publisher')
    config_path = os.path.join(pkg_share, 'config')

    user_config_path = os.path.join(config_path, 'MID360_config.json')
    laserscan_params_path = os.path.join(config_path, 'laserscan_params.yaml')

    # Livox MID-360 driver node
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0},         # PointCloud2 (PointXYZRTL)
            {'multi_topic': 0},
            {'data_src': 0},            # live LiDAR
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'user_config_path': user_config_path},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ],
    )

    # Convert PointCloud2 to LaserScan
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/lidar/scan'),
        ],
        parameters=[laserscan_params_path],
    )

    return LaunchDescription([
        livox_driver,
        pointcloud_to_laserscan,
    ])
