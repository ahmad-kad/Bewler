"""
ROS 2 launch file for Pi 5 "Brain" system.
Includes calibration service, SLAM, and navigation nodes.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Pi 5 brain system."""

    # Declare launch arguments
    calibration_dir_arg = DeclareLaunchArgument(
        'calibration_dir',
        default_value='calibrations',
        description='Directory containing camera calibration files'
    )

    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM system'
    )

    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Enable navigation system'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base link frame ID'
    )

    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='42',
        description='ROS domain ID for network isolation'
    )

    # Include calibration service launch
    calibration_service_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('urc_camera'), '/launch/calibration_service.launch.py'
        ]),
        launch_arguments={
            'calibration_dir': LaunchConfiguration('calibration_dir'),
            'ros_domain_id': LaunchConfiguration('ros_domain_id'),
        }.items()
    )

    # SLAM Toolbox node (for mapping and localization)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': LaunchConfiguration('odom_frame'),
            'map_frame': LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'scan_topic': '/scan',  # Assuming LIDAR scan topic
            'mode': 'mapping',  # Can be 'mapping' or 'localization'
        }],
        condition=IfCondition(LaunchConfiguration('enable_slam')),
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Nav2 bringup (for autonomous navigation)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': '',  # No predefined map
            'use_sim_time': 'false',
            'params_file': FindPackageShare('urc_navigation') + '/config/nav2_params.yaml',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_navigation'))
    )

    # TF authority node (manages coordinate frame transforms)
    tf_authority_node = Node(
        package='urc_navigation',
        executable='tf_authority_node',
        name='tf_authority',
        parameters=[{
            'map_frame': LaunchConfiguration('map_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Log launch configuration
    log_launch = LogInfo(
        msg=[
            'Launching Pi 5 Brain system with:\n',
            '  Calibration directory: ', LaunchConfiguration('calibration_dir'), '\n',
            '  SLAM enabled: ', LaunchConfiguration('enable_slam'), '\n',
            '  Navigation enabled: ', LaunchConfiguration('enable_navigation'), '\n',
            '  Map frame: ', LaunchConfiguration('map_frame'), '\n',
            '  Odom frame: ', LaunchConfiguration('odom_frame'), '\n',
            '  Base frame: ', LaunchConfiguration('base_frame'), '\n',
            '  ROS Domain ID: ', LaunchConfiguration('ros_domain_id')
        ]
    )

    # Create launch description
    ld = LaunchDescription([
        # Launch arguments
        calibration_dir_arg,
        enable_slam_arg,
        enable_navigation_arg,
        map_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        ros_domain_id_arg,

        # Log configuration
        log_launch,

        # Include calibration service
        calibration_service_launch,

        # Navigation stack
        nav2_launch,

        # SLAM system
        slam_node,

        # TF authority
        tf_authority_node,
    ])

    return ld
