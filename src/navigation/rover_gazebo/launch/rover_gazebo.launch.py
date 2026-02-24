import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop', default_value='false',
        description='Launch teleop_twist_keyboard in this terminal (use_teleop:=true)')

    # Package paths
    rover_gazebo_path = get_package_share_directory('rover_gazebo')
    rover_description_path = get_package_share_directory('rover_description')

    # Gazebo resolves package:// URIs as model:// and searches GZ_SIM_RESOURCE_PATH.
    # The rover URDF uses package://assets/... so we point GZ_SIM_RESOURCE_PATH at
    # the rover_description share directory, making model://assets/X.stl resolve to
    # <rover_description_share>/assets/X.stl
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        rover_description_path
    )

    # Process xacro -> URDF at launch time
    xacro_file = os.path.join(rover_gazebo_path, 'urdf', 'rover_gazebo.urdf.xacro')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Robot state publisher (uses sim time from Gazebo)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    # Launch Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn the rover into Gazebo from the /robot_description topic.
    # Spawn well above ground (1.5 m) so physics isn't fighting ground collisions
    # while the controllers are activating.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-topic', '/robot_description',
            '-z', '1.5',
        ],
        output='screen',
    )

    # Bridge /clock from Gazebo -> ROS2 (required for use_sim_time)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    # Spawn diff_drive_controller after joint_state_broadcaster is active
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    # Start controllers sequentially: joint_state_broadcaster first, then diff_drive
    start_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # Keyboard teleop — publishes to /diff_drive_controller/cmd_vel
    # Disabled by default: run in a separate terminal with use_teleop:=true,
    # or manually: ros2 run teleop_twist_keyboard teleop_twist_keyboard
    #              --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel
    teleop = TimerAction(
        period=6.0,
        actions=[Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_teleop')),
        )]
    )

    return LaunchDescription([
        use_teleop_arg,
        gz_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        clock_bridge,
        joint_state_broadcaster_spawner,
        start_diff_drive_controller,
        teleop,
    ])
