"""
ROS 2 launch file for calibration service on Pi 5.
Provides camera intrinsics to distributed Pi Zero 2 W nodes.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for calibration service."""

    # Declare launch arguments
    calibration_dir_arg = DeclareLaunchArgument(
        'calibration_dir',
        default_value='calibrations',
        description='Directory containing camera calibration files'
    )

    use_custom_interface_arg = DeclareLaunchArgument(
        'use_custom_interface',
        default_value='true',
        description='Use custom GetCalibration service interface if available'
    )

    default_camera_arg = DeclareLaunchArgument(
        'default_camera',
        default_value='camera',
        description='Default camera name for Trigger service fallback'
    )

    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='42',
        description='ROS domain ID for network isolation'
    )

    # Calibration service node
    calibration_service_node = Node(
        package='urc_camera',
        executable='calibration_service',
        name='calibration_service',
        parameters=[{
            'calibration_dir': LaunchConfiguration('calibration_dir'),
            'use_custom_interface': LaunchConfiguration('use_custom_interface'),
            'default_camera': LaunchConfiguration('default_camera'),
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Log launch configuration
    log_launch = LogInfo(
        msg=[
            'Launching calibration service with:\n',
            '  Calibration directory: ', LaunchConfiguration('calibration_dir'), '\n',
            '  Custom interface: ', LaunchConfiguration('use_custom_interface'), '\n',
            '  Default camera: ', LaunchConfiguration('default_camera'), '\n',
            '  ROS Domain ID: ', LaunchConfiguration('ros_domain_id')
        ]
    )

    # Create launch description
    ld = LaunchDescription([
        # Launch arguments
        calibration_dir_arg,
        use_custom_interface_arg,
        default_camera_arg,
        ros_domain_id_arg,

        # Log configuration
        log_launch,

        # Service node
        calibration_service_node,
    ])

    return ld
