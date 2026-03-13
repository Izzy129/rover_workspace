from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Genesis simulation bridge only (for testing sim in isolation)
        # Uses genesis_bridge_venv wrapper to run with venv Python.
        Node(
            package='genesis_arm_bridge',
            executable='genesis_bridge_venv',
            name='genesis_bridge_node',
            output='screen',
        ),
    ])
