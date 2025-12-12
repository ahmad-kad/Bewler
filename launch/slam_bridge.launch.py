#!/usr/bin/env python3
"""
SLAM Bridge Launch File - WebSocket to SLAM Integration

Launches all components needed for SLAM data routing:
- SLAM Data Bridge (ROS2 node)
- WebSocket Mission Bridge (if needed)
- Mock Motor Controller (for testing)
- Mission Executor (for waypoint missions)
"""

from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    """Generate launch description for SLAM bridge testing"""

    return LaunchDescription([
        # SLAM Data Bridge - Main component
        Node(
            package='autonomy_pkg',  # Adjust package name as needed
            executable='slam_data_bridge',
            name='slam_data_bridge',
            output='screen',
            parameters=[
                {
                    'imu_frame': 'imu_link',
                    'gps_frame': 'gps_link',
                    'base_frame': 'base_link',
                    'map_frame': 'map',
                    'odom_frame': 'odom'
                }
            ]
        ),

        # Mission Executor for waypoint testing
        Node(
            package='autonomy_pkg',  # Adjust package name as needed
            executable='mission_executor',
            name='mission_executor',
            output='screen',
            parameters=[]
        ),

        # WebSocket Mission Bridge for command routing
        Node(
            package='autonomy_pkg',  # Adjust package name as needed
            executable='websocket_mission_bridge',
            name='websocket_mission_bridge',
            output='screen',
            parameters=[]
        ),

        # Mock Motor Controller for odometry
        Node(
            package='autonomy_pkg',  # Adjust package name as needed
            executable='mock_motor_controller',
            name='mock_motor_controller',
            output='screen',
            parameters=[]
        ),

        # Optional: ROS2 SLAM node (comment out if using external SLAM)
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[
        #         # SLAM configuration parameters
        #     ]
        # ),
    ])
