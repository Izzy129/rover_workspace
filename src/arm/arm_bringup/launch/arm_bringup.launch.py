import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Launch sim driver (set false for real hardware)')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2')

    # Package paths
    urdf_viewer_path = get_package_share_directory('urdf_viewer')
    arm_bringup_path = get_package_share_directory('arm_bringup')
    arm_teleop_path = get_package_share_directory('arm_teleop')
    params_file = os.path.join(arm_teleop_path, 'config', 'teleop_params.yaml')

    # Load URDF
    urdf_file = os.path.join(urdf_viewer_path, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # Sim driver (passthrough commands -> joint states)
    sim_driver = Node(
        package='arm_teleop',
        executable='sim_driver.py',
        name='arm_sim_driver',
        parameters=[params_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )

    # RViz2
    rviz_config = os.path.join(arm_bringup_path, 'rviz', 'arm_teleop.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_sim_arg,
        use_rviz_arg,
        robot_state_publisher,
        sim_driver,
        rviz,
    ])
