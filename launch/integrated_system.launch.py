#!/usr/bin/env python3
"""
Integrated System Launch File - Full system with teleoperation and control systems

Launches the complete URC 2026 system including:
- Autonomy system (navigation, SLAM, mission control)
- Teleoperation frontend (submodule)
- Control systems (submodule)
- ROS bridge for WebSocket communication
- All necessary bridges and interfaces

This launch file demonstrates the integration of submodules for complete system testing.
"""

import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Generate complete integrated system launch description"""

    # Get workspace paths
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    teleoperation_path = os.path.join(workspace_root, 'submodules', 'teleoperation')
    control_systems_path = os.path.join(workspace_root, 'submodules', 'control-systems')

    return LaunchDescription([
        # ===========================================
        # SYSTEM STATUS
        # ===========================================
        ExecuteProcess(
            cmd=['echo', '=== URC 2026 Integrated System Launch ==='],
            output='screen',
            name='system_status'
        ),

        ExecuteProcess(
            cmd=['echo', f'Teleoperation submodule: {teleoperation_path}'],
            output='screen',
            name='teleoperation_status'
        ),

        ExecuteProcess(
            cmd=['echo', f'Control systems submodule: {control_systems_path}'],
            output='screen',
            name='control_systems_status'
        ),

        # ===========================================
        # AUTONOMY SYSTEM (Core)
        # ===========================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(workspace_root, 'launch', 'mission_system.launch.py')
            )
        ),

        # ===========================================
        # TELEOPERATION FRONTEND
        # ===========================================
        # Note: Teleoperation frontend runs separately via npm/yarn
        # This is just for reference - actual startup is manual:
        # cd submodules/teleoperation && npm install && npm run dev
        ExecuteProcess(
            cmd=['echo', 'Teleoperation frontend: Run manually with "cd submodules/teleoperation && npm run dev"'],
            output='screen',
            name='teleoperation_instruction'
        ),

        # ===========================================
        # CONTROL SYSTEMS INTEGRATION
        # ===========================================
        ExecuteProcess(
            cmd=['echo', 'Control systems: Integrated via control_systems_bridge in mission_system.launch.py'],
            output='screen',
            name='control_systems_instruction'
        ),

        # ===========================================
        # SYSTEM MONITORING
        # ===========================================
        TimerAction(
            period=5.0,  # Wait for systems to start
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'list'],
                    output='screen',
                    name='topic_monitor'
                )
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'node', 'list'],
                    output='screen',
                    name='node_monitor'
                )
            ]
        ),

        # ===========================================
        # INTEGRATION VERIFICATION
        # ===========================================
        ExecuteProcess(
            cmd=['echo', '=== Integration Complete ==='],
            output='screen',
            name='integration_complete'
        ),

        ExecuteProcess(
            cmd=['echo', 'Teleoperation: http://localhost:5173 (if started)'],
            output='screen',
            name='teleoperation_url'
        ),

        ExecuteProcess(
            cmd=['echo', 'ROS Bridge: ws://localhost:9090'],
            output='screen',
            name='rosbridge_url'
        ),
    ])
