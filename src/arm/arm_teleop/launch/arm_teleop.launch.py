import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('arm_teleop')
    params_file = os.path.join(pkg_path, 'config', 'teleop_params.yaml')

    keyboard_teleop = Node(
        package='arm_teleop',
        executable='keyboard_teleop.py',
        name='arm_keyboard_teleop',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        keyboard_teleop,
    ])
