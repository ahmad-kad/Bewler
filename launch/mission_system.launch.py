#!/usr/bin/env python3
"""
Mission System Launch File - Integrated with teleoperation and control systems submodules

Launches:
- Mission Executor (autonomy system)
- WebSocket Mission Bridge (ROS <-> WebSocket communication)
- ROS Bridge Server (for teleoperation frontend)
- Control Systems Bridge (for drive/arm control via submodules)
- Teleoperation integration (references submodule)
"""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    """Generate launch description for integrated mission system"""

    # Get workspace paths for submodules
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    teleoperation_path = os.path.join(workspace_root, 'submodules', 'teleoperation')
    control_systems_path = os.path.join(workspace_root, 'submodules', 'control-systems')

    return LaunchDescription([
        # ===========================================
        # MISSION EXECUTOR (Autonomy Core)
        # ===========================================
        Node(
            package='autonomy_pkg',  # Adjust package name as needed
            executable='mission_executor',
            name='mission_executor',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),

        # ===========================================
        # WEBSOCKET MISSION BRIDGE
        # ===========================================
        Node(
            package='autonomy_pkg',  # Adjust package name as needed
            executable='websocket_mission_bridge',
            name='websocket_mission_bridge',
            output='screen',
            parameters=[
                {'websocket_port': 9090},
                {'mission_update_rate': 10.0}
            ]
        ),

        # ===========================================
        # ROS BRIDGE SERVER (for Teleoperation)
        # ===========================================
        Node(
            package='rosbridge_server',
            executable='rosbridge_server',
            name='rosbridge_server',
            output='screen',
            parameters=[
                {'port': 9090},
                {'address': '0.0.0.0'},
                {'retry_startup_delay': 5.0}
            ]
        ),

        # ===========================================
        # CONTROL SYSTEMS BRIDGE (Drive/Arm Control)
        # ===========================================
        Node(
            package='autonomy_pkg',  # Adjust package name as needed
            executable='control_systems_bridge',
            name='control_systems_bridge',
            output='screen',
            parameters=[
                {'can_interface': 'can0'},
                {'control_loop_rate': 100.0},
                {'submodule_path': control_systems_path}
            ]
        ),

        # ===========================================
        # TELEOPERATION STATUS MONITOR
        # ===========================================
        ExecuteProcess(
            cmd=['echo', f'Teleoperation submodule path: {teleoperation_path}'],
            output='screen',
            name='teleoperation_info'
        ),

        ExecuteProcess(
            cmd=['echo', f'Control systems submodule path: {control_systems_path}'],
            output='screen',
            name='control_systems_info'
        ),
    ])