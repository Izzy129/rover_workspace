import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    gui = LaunchConfiguration('gui').perform(context)

    # Process URDF/xacro file
    if urdf_file.endswith('.xacro'):
        robot_description_content = xacro.process_file(urdf_file).toxml()
    else:
        with open(urdf_file, 'r') as f:
            robot_description_content = f.read()

    nodes = []

    # Robot State Publisher
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    ))

    # Joint State Publisher (with or without GUI)
    if gui.lower() == 'true':
        nodes.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ))
    else:
        nodes.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ))

    # RViz2 (no config file — configure displays manually, then save from RViz)
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    ))

    return nodes


def generate_launch_description():
    # Default URDF: robot.urdf bundled with this package
    default_urdf = os.path.join(
        get_package_share_directory('urdf_viewer'),
        'urdf', 'robot.urdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=default_urdf,
            description='Absolute path to a .urdf or .urdf.xacro file to visualize',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch joint_state_publisher_gui for interactive joint control',
        ),
        OpaqueFunction(function=launch_setup),
    ])
