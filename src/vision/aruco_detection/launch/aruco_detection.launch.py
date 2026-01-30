import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )

    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='webcam',
        description='Camera type to use: webcam or realsense'
    )

    stream_type_arg = DeclareLaunchArgument(
        'stream_type',
        default_value='color',
        description='RealSense stream type: color, depth, infra1, or infra2'
    )

    visualize_keyboard_arg = DeclareLaunchArgument(
        'visualize_keyboard',
        default_value='false',
        description='Start keyboard center visualizer'
    )

    smoothing_alpha_arg = DeclareLaunchArgument(
        'smoothing_alpha',
        default_value='0.7',
        description='Smoothing factor for marker poses (0.0 = infinite smoothing, 1.0 = no smoothing)'
    )

    # Get configuration file paths
    aruco_params = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'aruco_parameters.yaml'
    )

    realsense_params = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'realsense_camera.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'aruco_rviz.rviz'
    )

    # Static transform: map -> camera_link
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link']
    )

    # Webcam publisher node (only if camera_type == 'webcam')
    webcam_publisher_node = Node(
        package='aruco_detection',
        executable='webcam_publisher',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_type'), "' == 'webcam'"])
        )
    )

    # RealSense camera node (only if camera_type == 'realsense')
    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        parameters=[realsense_params],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_type'), "' == 'realsense'"])
        )
    )

    # RealSense publisher wrapper (only if camera_type == 'realsense')
    realsense_publisher_node = Node(
        package='aruco_detection',
        executable='realsense_publisher',
        parameters=[{'stream_type': LaunchConfiguration('stream_type')}],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_type'), "' == 'realsense'"])
        )
    )

    # ArUco detection node
    aruco_node = Node(
        package='aruco_detection',
        executable='aruco_node',
        parameters=[
            aruco_params,
            {'smoothing_alpha': LaunchConfiguration('smoothing_alpha')}
        ]
    )

    # Keyboard detection node
    keyboard_node = Node(
        package='aruco_detection',
        executable='keyboard_detection'
    )

    # Keyboard visualizer node (optional)
    keyboard_visualizer_node = Node(
        package='aruco_detection',
        executable='keyboard_visualizer',
        condition=IfCondition(LaunchConfiguration('visualize_keyboard'))
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
        # Launch arguments
        use_rviz_arg,
        camera_type_arg,
        stream_type_arg,
        visualize_keyboard_arg,
        smoothing_alpha_arg,
        # Nodes
        static_transform_node,
        webcam_publisher_node,
        realsense_camera_node,
        realsense_publisher_node,
        aruco_node,
        rviz_node,
        keyboard_node,
        keyboard_visualizer_node
    ])
