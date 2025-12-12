"""
Launch file for state machine director node.

Launches the state machine director with configuration parameters.
Optionally launches LED controller alongside for integrated operation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for state machine."""
    # Get package directories
    state_machine_dir = get_package_share_directory("autonomy_state_machine")

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(state_machine_dir, "config", "state_machine_config.yaml"),
        description="Path to state machine configuration file",
    )

    launch_led_controller_arg = DeclareLaunchArgument(
        "launch_led_controller",
        default_value="false",
        description="Whether to launch LED controller alongside state machine",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )

    # State machine director node
    state_machine_node = Node(
        package="autonomy_state_machine",
        executable="state_machine_director",
        name="state_machine_director",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        emulate_tty=True,
    )

    # Optional: Launch LED controller (if package exists)
    # led_controller_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('autonomy_led_status'),
    #             'launch',
    #             'led_controller.launch.py'
    #         )
    #     ),
    #     condition=IfCondition(LaunchConfiguration('launch_led_controller'))
    # )

    return LaunchDescription(
        [
            # Arguments
            config_file_arg,
            launch_led_controller_arg,
            log_level_arg,
            # Nodes
            state_machine_node,
            # Uncomment to include LED controller
            # led_controller_launch,
        ]
    )
