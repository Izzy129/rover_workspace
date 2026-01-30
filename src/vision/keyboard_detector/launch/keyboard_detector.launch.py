from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    visualize_keyboard_arg = DeclareLaunchArgument(
        'visualize_keyboard',
        default_value='false',
        description='Start OpenCV keyboard visualizer'
    )

    # Get launch configurations
    visualize_keyboard = LaunchConfiguration('visualize_keyboard')

    # Keyboard detection node
    keyboard_detection_node = Node(
        package='keyboard_detector',
        executable='keyboard_detection',
        name='keyboard_detection',
        output='screen'
    )

    # Keyboard visualizer node (optional)
    keyboard_visualizer_node = Node(
        package='keyboard_detector',
        executable='keyboard_visualizer',
        name='keyboard_visualizer',
        output='screen',
        condition=IfCondition(visualize_keyboard)
    )

    return LaunchDescription([
        visualize_keyboard_arg,
        keyboard_detection_node,
        keyboard_visualizer_node,
    ])
