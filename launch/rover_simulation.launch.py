#!/usr/bin/env python3
"""
Complete Rover Simulation Launch File - Production-ready testing environment

Launches full simulation stack for testing URC 2026 rover systems:
- Gazebo simulation world
- TurtleBot3 with navigation and SLAM
- Custom mission executor
- SLAM data bridge
- WebSocket bridge for commands

This simulates the production rover environment for comprehensive testing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate complete simulation launch description"""

    # Get package paths
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_navigation2 = get_package_share_directory('turtlebot3_navigation2')
    turtlebot3_cartographer = get_package_share_directory('turtlebot3_cartographer')

    # Define paths for custom packages (adjust as needed)
    autonomy_pkg = os.path.expanduser('~/urc-machiato-2026')  # Adjust path

    return LaunchDescription([
        # ===========================================
        # GAZEBO SIMULATION WORLD
        # ===========================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
        ),

        # ===========================================
        # NAVIGATION STACK
        # ===========================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_navigation2, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': os.path.join(turtlebot3_navigation2, 'config', 'nav2_params.yaml'),
                'map_subscribe_transient_local': 'true'
            }.items()
        ),

        # ===========================================
        # SLAM (CARTOGRAPHER)
        # ===========================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_cartographer, 'launch', 'cartographer.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
        ),

        # ===========================================
        # CUSTOM MISSION EXECUTOR
        # ===========================================
        Node(
            package='autonomy_pkg',  # Adjust package name
            executable='mission_executor',
            name='mission_executor',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
            remappings=[
                ('/odom', '/odom'),  # Use simulated odometry
                ('/imu/data', '/imu'),  # Remap to TurtleBot IMU
                ('/gps/fix', '/gps/fix')  # GPS if available
            ]
        ),

        # ===========================================
        # SLAM DATA BRIDGE
        # ===========================================
        Node(
            package='autonomy_pkg',  # Adjust package name
            executable='slam_data_bridge',
            name='slam_data_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'imu_frame': 'imu_link'},
                {'gps_frame': 'gps_link'},
                {'base_frame': 'base_link'},
                {'map_frame': 'map'},
                {'odom_frame': 'odom'}
            ],
            remappings=[
                ('/imu/data', '/imu'),  # Subscribe to TurtleBot IMU
                ('/odom', '/odom'),  # Subscribe to TurtleBot odometry
                ('/slam/pose', '/pose'),  # Subscribe to SLAM pose
            ]
        ),

        # ===========================================
        # WEBSOCKET MISSION BRIDGE
        # ===========================================
        Node(
            package='autonomy_pkg',  # Adjust package name
            executable='websocket_mission_bridge',
            name='websocket_mission_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),

        # ===========================================
        # RVIZ VISUALIZATION (Optional)
        # ===========================================
        TimerAction(
            period=5.0,  # Wait for systems to start
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(turtlebot3_bringup, 'launch', 'rviz2.launch.py')
                    )
                )
            ]
        ),

        # ===========================================
        # STATUS LOGGING
        # ===========================================
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/mission/status', '--once'],
            output='screen',
            name='status_monitor'
        ),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/frontend/map_data', '--once'],
            output='screen',
            name='map_monitor'
        )
    ])
