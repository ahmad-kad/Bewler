"""
Launch file for WebSocket Sensor Bridge Node.

Launches the sensor bridge with configuration.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for sensor bridge."""
    # Get package directory
    sensor_bridge_dir = get_package_share_directory('sensor_bridge')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(
            sensor_bridge_dir, "config", "sensor_bridge.yaml"
        ),
        description="Path to sensor bridge configuration file",
    )

    websocket_url_arg = DeclareLaunchArgument(
        "websocket_url",
        default_value="ws://localhost:8080",
        description="WebSocket URL for sensor data source",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )

    # Sensor bridge node
    sensor_bridge_node = Node(
        package="sensor_bridge",
        executable="sensor_bridge_node.py",
        name="websocket_sensor_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "websocket_url": LaunchConfiguration("websocket_url"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        emulate_tty=True,
    )

    return LaunchDescription([
        # Arguments
        config_file_arg,
        websocket_url_arg,
        log_level_arg,
        # Nodes
        sensor_bridge_node,
    ])
