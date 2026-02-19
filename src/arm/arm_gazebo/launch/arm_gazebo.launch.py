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
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2')

    # Package paths
    arm_gazebo_path = get_package_share_directory('arm_gazebo')
    arm_teleop_path = get_package_share_directory('arm_teleop')
    arm_bringup_path = get_package_share_directory('arm_bringup')

    # Gazebo resolves package:// URIs as model:// and searches GZ_SIM_RESOURCE_PATH.
    # Point it at the share parent so model://urdf_viewer/urdf/assets/*.stl resolves.
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('urdf_viewer'))
    )

    # Process xacro -> URDF at launch time
    xacro_file = os.path.join(arm_gazebo_path, 'urdf', 'arm_gazebo.urdf.xacro')
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

    # Spawn the robot into Gazebo from the /robot_description topic
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'arm',
            '-topic', '/robot_description',
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

    # Spawn joint_state_broadcaster after robot is spawned
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

    # Spawn velocity controller after joint_state_broadcaster is active
    arm_vel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_vel_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    # Start controllers sequentially: joint_state_broadcaster first, then velocity controller
    start_arm_vel_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_vel_controller_spawner],
        )
    )

    # Gazebo bridge node translates /arm_joint_commands -> /arm_vel_controller/commands
    # Start with a small delay to allow the velocity controller to come up first
    gazebo_bridge = TimerAction(
        period=5.0,
        actions=[Node(
            package='arm_teleop',
            executable='gazebo_bridge.py',
            name='gazebo_bridge',
            parameters=[os.path.join(arm_teleop_path, 'config', 'teleop_params.yaml')],
            output='screen',
        )]
    )

    # RViz2
    rviz_config = os.path.join(arm_bringup_path, 'rviz', 'arm_teleop.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        gz_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        clock_bridge,
        joint_state_broadcaster_spawner,
        start_arm_vel_controller,
        gazebo_bridge,
        rviz,
    ])
