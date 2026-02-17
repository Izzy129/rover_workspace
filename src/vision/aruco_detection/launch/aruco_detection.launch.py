import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )

    smoothing_alpha_arg = DeclareLaunchArgument(
        'smoothing_alpha',
        default_value='0.7',
        description='Smoothing factor for marker poses (0.0 = infinite smoothing, 1.0 = no smoothing)'
    )

    frame_skip_arg = DeclareLaunchArgument(
        'frame_skip',
        default_value='0',
        description='Frame skip interval (0=process all frames, 1=every other frame, 2=every third frame, etc.)'
    )

    # Get configuration file paths
    aruco_params = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'aruco_parameters.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'aruco_rviz.rviz'
    )

    # ArUco detection node
    aruco_node = Node(
        package='aruco_detection',
        executable='aruco_node',
        name='aruco_node',
        output='screen',
        parameters=[
            aruco_params,
            {
                'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
                'frame_skip': LaunchConfiguration('frame_skip')
            }
        ]
    )

    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        smoothing_alpha_arg,
        frame_skip_arg,
        aruco_node,
        rviz_node,
    ])
