from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
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

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )

    visualize_keyboard_arg = DeclareLaunchArgument(
        'visualize_keyboard',
        default_value='false',
        description='Start OpenCV keyboard visualizer'
    )

    smoothing_alpha_arg = DeclareLaunchArgument(
        'smoothing_alpha',
        default_value='0.7',
        description='ArUco pose smoothing factor (0.0-1.0)'
    )

    # Get launch configurations
    camera_type = LaunchConfiguration('camera_type')
    stream_type = LaunchConfiguration('stream_type')
    use_rviz = LaunchConfiguration('use_rviz')
    visualize_keyboard = LaunchConfiguration('visualize_keyboard')
    smoothing_alpha = LaunchConfiguration('smoothing_alpha')

    # Include camera_publisher launch
    camera_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('camera_publisher'),
                         'launch', 'camera_publisher.launch.py')
        ]),
        launch_arguments={
            'camera_type': camera_type,
            'stream_type': stream_type,
        }.items()
    )

    # Include aruco_detection launch
    aruco_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('aruco_detection'),
                         'launch', 'aruco_detection.launch.py')
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'smoothing_alpha': smoothing_alpha,
        }.items()
    )

    # Include keyboard_detector launch
    keyboard_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('keyboard_detector'),
                         'launch', 'keyboard_detector.launch.py')
        ]),
        launch_arguments={
            'visualize_keyboard': visualize_keyboard,
        }.items()
    )

    return LaunchDescription([
        camera_type_arg,
        stream_type_arg,
        use_rviz_arg,
        visualize_keyboard_arg,
        smoothing_alpha_arg,
        camera_publisher_launch,
        aruco_detection_launch,
        keyboard_detector_launch,
    ])
