import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    control_config = os.path.join(
        get_package_share_directory('genesis_arm_control'),
        'config',
        'joint_limits.yaml'
    )

    return LaunchDescription([
        # Genesis simulation bridge (dev machine only)
        # Uses genesis_bridge_venv wrapper to run with venv Python,
        # avoiding system coverage/numba incompatibility.
        Node(
            package='genesis_arm_bridge',
            executable='genesis_bridge_venv',
            name='genesis_bridge_node',
            output='screen',
        ),

        # Arm controller with joint limits
        Node(
            package='genesis_arm_control',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[control_config],
        ),

        # Keyboard teleop (evdev-based, no dedicated terminal needed)
        Node(
            package='genesis_arm_teleop',
            executable='keyboard_teleop_venv',
            name='keyboard_teleop_node',
            output='screen',
        ),
    ])
