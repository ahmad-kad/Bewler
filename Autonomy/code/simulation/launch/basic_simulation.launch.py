#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    """Launch basic simulation without Gazebo - focuses on robot state and visualization."""

    # Package directories
    pkg_simulation = get_package_share_directory("autonomy_simulation")
    pkg_description = get_package_share_directory("autonomy_simulation")  # Using simulation package for URDF

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    robot_name = LaunchConfiguration("robot_name", default="rover2025")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )

    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="rover2025", description="Robot name")

    # Robot description
    robot_description_content = Command(
        [
            "xacro ",
            os.path.join(pkg_description, "urdf", "rover2025_enhanced.urdf.xacro"),
        ]
    )

    robot_description = {
        "robot_description": robot_description_content,
        "use_sim_time": use_sim_time,
    }

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[
            ("robot_description", "robot_description"),
            ("joint_states", "joint_states"),
        ],
    )

    # Joint State Publisher (for testing without Gazebo)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "rate": 30}],
    )

    # Sensor simulator (provides GPS, IMU, camera data)
    sensor_simulator = Node(
        package="autonomy_simulation",
        executable="sensor_simulator.py",
        name="sensor_simulator",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "gps_noise_std": 3.0,
                "imu_noise_std": 0.01,
                "camera_width": 640,
                "camera_height": 480,
                "camera_fps": 30.0,
            }
        ],
    )

    # Launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_name)

    # Add nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(sensor_simulator)

    return ld
