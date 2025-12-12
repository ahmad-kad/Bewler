#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='autonomy_simulation').find('autonomy_simulation')
    launch_dir = os.path.join(pkg_share, 'launch')
    world_dir = os.path.join(pkg_share, 'worlds')
    model_dir = os.path.join(pkg_share, 'models')

    # Launch configuration variables
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_gps = LaunchConfiguration('enable_gps')
    sensor_noise_level = LaunchConfiguration('sensor_noise_level')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='urc_desert_terrain',
        description='Full path to world model file to load'
    )

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial x position of the robot'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial y position of the robot'
    )

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.0',
        description='Initial z position of the robot'
    )

    declare_yaw_position_cmd = DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='Initial yaw of the robot'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_enable_gps_cmd = DeclareLaunchArgument(
        'enable_gps',
        default_value='true',
        description='Enable GPS sensor if true'
    )

    declare_sensor_noise_level_cmd = DeclareLaunchArgument(
        'sensor_noise_level',
        default_value='medium',
        description='Sensor noise level: low, medium, high'
    )

    # Specify the actions
    start_gazebo_server_cmd = Node(
        package='gazebo_ros',
        executable='gzserver',
        name='gazebo',
        output='screen',
        arguments=[os.path.join(world_dir, world + '.world')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    start_gazebo_client_cmd = Node(
        package='gazebo_ros',
        executable='gzclient',
        name='gazebo_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(os.path.join(model_dir, 'rover2025_gazebo.urdf'), 'r').read()
        }]
    )

    # Spawn the robot
    start_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rover2025',
            '-file', os.path.join(model_dir, 'rover2025_gazebo.urdf'),
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        output='screen'
    )

    # Sensor noise configuration node
    start_sensor_config_cmd = Node(
        package='autonomy_simulation',
        executable='sensor_noise_config',
        name='sensor_noise_config',
        output='screen',
        parameters=[{
            'sensor_noise_level': sensor_noise_level,
            'enable_gps': enable_gps,
            'use_sim_time': use_sim_time
        }]
    )

    # Static transform publishers for sensor frames
    start_camera_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0.3', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )

    start_lidar_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0', '0', '0.25', '0', '0', '0', 'base_link', 'lidar_link'],
        output='screen'
    )

    start_imu_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_publisher',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )

    start_gps_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_tf_publisher',
        arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', 'gps_link'],
        output='screen',
        condition=IfCondition(enable_gps)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_yaw_position_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_enable_gps_cmd)
    ld.add_action(declare_sensor_noise_level_cmd)

    # Add any conditioned actions
    ld.add_action(DeclareLaunchArgument('gui', default_value='true'))

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_spawner_cmd)
    ld.add_action(start_sensor_config_cmd)
    ld.add_action(start_camera_tf_cmd)
    ld.add_action(start_lidar_tf_cmd)
    ld.add_action(start_imu_tf_cmd)
    ld.add_action(start_gps_tf_cmd)

    return ld
