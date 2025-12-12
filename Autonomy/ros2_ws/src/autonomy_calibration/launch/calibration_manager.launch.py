#!/usr/bin/env python3
"""
Launch file for calibration subsystem
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for calibration nodes."""

    # Get package share directory
    pkg_dir = get_package_share_directory("autonomy_calibration")

    # Configuration file path
    config_file = os.path.join(pkg_dir, "config", "calibration.yaml")

    return LaunchDescription(
        [
            # Calibration Manager Node
            Node(
                package="autonomy_calibration",
                executable="calibration_manager",
                name="calibration_manager",
                parameters=[config_file],
                output="screen",
            ),
            # Optional: Add calibration service node if needed
            # Node(
            #     package='autonomy_calibration',
            #     executable='calibration_service',
            #     name='calibration_service',
            #     parameters=[config_file],
            #     output='screen',
            # ),
        ]
    )
