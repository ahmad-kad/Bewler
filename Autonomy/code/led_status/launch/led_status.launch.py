#!/usr/bin/env python3


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package="autonomy_led_status").find("autonomy_led_status")

    # Declare launch arguments
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="Log level for LED status node"
    )

    declare_hardware_mode_cmd = DeclareLaunchArgument(
        "hardware_mode",
        default_value="simulation",
        description="Hardware mode: simulation or real",
    )

    # LED Status Node
    led_status_node = Node(
        package="autonomy_led_status",
        executable="led_status_node",
        name="led_status_node",
        output="screen",
        parameters=[
            {
                "hardware_mode": LaunchConfiguration("hardware_mode"),
                "log_level": LaunchConfiguration("log_level"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_hardware_mode_cmd)

    # Add the nodes
    ld.add_action(led_status_node)

    return ld
