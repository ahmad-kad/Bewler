#!/usr/bin/env python3

"""
Launch file for Digital Twin system.

This launch file starts the digital twin manager and related components.

PLACEHOLDER VALUES: Configuration uses placeholder values for development.
"""


from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Generate launch description for digital twin system."""

    # Package directories
    pkg_digital_twin = get_package_share_directory("autonomy_simulation")  # PLACEHOLDER: Would be separate package
    pkg_simulation = get_package_share_directory("autonomy_simulation")

    # Launch arguments with PLACEHOLDER values
    sync_rate = LaunchConfiguration("sync_rate", default="10.0")  # PLACEHOLDER
    prediction_horizon = LaunchConfiguration("prediction_horizon", default="5.0")  # PLACEHOLDER
    twin_fidelity = LaunchConfiguration("twin_fidelity", default="0.85")  # PLACEHOLDER

    declare_sync_rate = DeclareLaunchArgument(
        "sync_rate",
        default_value="10.0",
        description="Digital twin synchronization rate (Hz) - PLACEHOLDER value",
    )

    declare_prediction_horizon = DeclareLaunchArgument(
        "prediction_horizon",
        default_value="5.0",
        description="Prediction time horizon (seconds) - PLACEHOLDER value",
    )

    declare_twin_fidelity = DeclareLaunchArgument(
        "twin_fidelity",
        default_value="0.85",
        description="Model fidelity score (0-1) - PLACEHOLDER value",
    )

    # Digital Twin Manager Node
    digital_twin_manager = Node(
        package="autonomy_simulation",  # PLACEHOLDER: Would be separate digital twin package
        executable="digital_twin_manager.py",
        name="digital_twin_manager",
        output="screen",
        parameters=[
            {
                "sync_rate": sync_rate,
                "prediction_horizon": prediction_horizon,
                "twin_fidelity": twin_fidelity,
                "health_check_interval": 1.0,  # PLACEHOLDER
                "max_sync_delay": 0.1,  # PLACEHOLDER
                "use_sim_time": LaunchConfiguration("use_sim_time", default="true"),
            }
        ],
        remappings=[
            # PLACEHOLDER: Topic remappings for digital twin interface
            ("/digital_twin/state", "/digital_twin/state"),
            ("/digital_twin/prediction", "/digital_twin/prediction"),
            ("/digital_twin/sync_status", "/digital_twin/sync_status"),
            ("/digital_twin/health", "/digital_twin/health"),
        ],
    )

    # PLACEHOLDER: Additional digital twin nodes would be added here
    # twin_monitor = Node(package='digital_twin', executable='twin_monitor', ...)
    # twin_predictor = Node(package='digital_twin', executable='twin_predictor', ...)
    # twin_validator = Node(package='digital_twin', executable='twin_validator', ...)

    # RViz configuration for digital twin visualization (PLACEHOLDER)
    rviz_config = PathJoinSubstitution(
        [
            pkg_simulation,
            "rviz",
            "digital_twin.rviz",  # PLACEHOLDER: Would need separate RViz config
        ]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_digital_twin",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time", default="true")}],
        output="screen",
    )

    # Launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_sync_rate)
    ld.add_action(declare_prediction_horizon)
    ld.add_action(declare_twin_fidelity)

    # Add nodes
    ld.add_action(digital_twin_manager)
    ld.add_action(rviz)

    return ld
