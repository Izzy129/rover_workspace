from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare launch arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='webcam',
        description='Type of camera to use: webcam or realsense'
    )

    stream_type_arg = DeclareLaunchArgument(
        'stream_type',
        default_value='color',
        description='Type of stream for RealSense: color, depth, infra1, infra2'
    )

    # Get launch configurations
    camera_type = LaunchConfiguration('camera_type')
    stream_type = LaunchConfiguration('stream_type')

    # Static transform: map -> camera_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link']
    )

    # Webcam publisher node
    webcam_node = Node(
        package='camera_publisher',
        executable='webcam_publisher',
        name='webcam_publisher',
        condition=IfCondition(
            PythonExpression(["'", camera_type, "' == 'webcam'"])
        )
    )

    # RealSense publisher node
    realsense_node = Node(
        package='camera_publisher',
        executable='realsense_publisher',
        name='realsense_publisher',
        parameters=[{
            'stream_type': stream_type
        }],
        condition=IfCondition(
            PythonExpression(["'", camera_type, "' == 'realsense'"])
        )
    )

    return LaunchDescription([
        camera_type_arg,
        stream_type_arg,
        static_tf_node,
        webcam_node,
        realsense_node,
    ])
