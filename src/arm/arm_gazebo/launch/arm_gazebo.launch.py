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

    # Toolbox model: cube base + two rectangular posts + horizontal cylinder handle
    # Inertia computed for base box (0.14x0.10x0.08, 2 kg) — dominant mass contributor:
    #   Ixx = m/12*(y²+z²) = 2/12*(0.01+0.0064)  = 0.002733
    #   Iyy = m/12*(x²+z²) = 2/12*(0.0196+0.0064) = 0.004333
    #   Izz = m/12*(x²+y²) = 2/12*(0.0196+0.01)   = 0.004933
    # Surface params on every collision: high friction (mu=1.0) + contact stiffness/damping
    # prevent the object phasing through the gripper.
    _surface = """<surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          <contact><ode><kp>1e6</kp><kd>100</kd><max_vel>0.1</max_vel><min_depth>0.001</min_depth></ode></contact>
        </surface>"""
    toolbox_sdf = f"""<sdf version='1.7'>
  <model name='toolbox'>
    <static>false</static>
    <link name='body'>
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.002733</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.004333</iyy><iyz>0</iyz>
          <izz>0.004933</izz>
        </inertia>
      </inertial>
      <collision name='base_col'>
        <pose>0 0 0.04 0 0 0</pose>
        <geometry><box><size>0.14 0.10 0.08</size></box></geometry>
        {_surface}
      </collision>
      <visual name='base_vis'>
        <pose>0 0 0.04 0 0 0</pose>
        <geometry><box><size>0.14 0.10 0.08</size></box></geometry>
        <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
      </visual>
      <collision name='left_post_col'>
        <pose>0 -0.04 0.12 0 0 0</pose>
        <geometry><box><size>0.025 0.025 0.08</size></box></geometry>
        {_surface}
      </collision>
      <visual name='left_post_vis'>
        <pose>0 -0.04 0.12 0 0 0</pose>
        <geometry><box><size>0.025 0.025 0.08</size></box></geometry>
        <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
      </visual>
      <collision name='right_post_col'>
        <pose>0 0.04 0.12 0 0 0</pose>
        <geometry><box><size>0.025 0.025 0.08</size></box></geometry>
        {_surface}
      </collision>
      <visual name='right_post_vis'>
        <pose>0 0.04 0.12 0 0 0</pose>
        <geometry><box><size>0.025 0.025 0.08</size></box></geometry>
        <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
      </visual>
      <collision name='handle_col'>
        <pose>0 0 0.16 1.5708 0 0</pose>
        <geometry><cylinder><radius>0.015</radius><length>0.10</length></cylinder></geometry>
        {_surface}
      </collision>
      <visual name='handle_vis'>
        <pose>0 0 0.16 1.5708 0 0</pose>
        <geometry><cylinder><radius>0.015</radius><length>0.10</length></cylinder></geometry>
        <material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""

    spawn_toolbox = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'toolbox',
            '-x', '0.6', '-y', '0.0', '-z', '0.05',
            '-string', toolbox_sdf,
        ],
        output='screen',
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
        spawn_toolbox,
        clock_bridge,
        joint_state_broadcaster_spawner,
        start_arm_vel_controller,
        gazebo_bridge,
        rviz,
    ])
