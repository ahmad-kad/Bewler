#!/usr/bin/env python3
"""
System Integration Launch File

This launch file coordinates all autonomy subsystems for integrated operation.
Shows how separate packages communicate through ROS2 topics, services, and actions.
"""

import os

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Generate integrated system launch description."""

    # Launch arguments for system configuration
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='autonomy',
        description='Top-level namespace for all nodes'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )

    enable_vision_arg = DeclareLaunchArgument(
        'enable_vision', default_value='true',
        description='Enable computer vision subsystem'
    )

    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam', default_value='true',
        description='Enable SLAM subsystem'
    )

    # Package shares for finding config files
    state_pkg = FindPackageShare('autonomy_state_management')
    nav_pkg = FindPackageShare('autonomy_navigation')
    slam_pkg = FindPackageShare('autonomy_slam')
    vision_pkg = FindPackageShare('autonomy_computer_vision')
    typing_pkg = FindPackageShare('autonomy_autonomous_typing')
    led_pkg = FindPackageShare('autonomy_led_status')

    # Common parameters
    common_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'namespace': LaunchConfiguration('namespace'),
    }

    # ==========================================
    # STATE MANAGEMENT (Central Coordinator)
    # ==========================================
    state_management_node = Node(
        package='autonomy_state_management',
        executable='state_management_node',
        name='state_management_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            common_params,
            {'mission_duration_limit': 1800.0},
            {'health_check_interval': 1.0},
        ],
        remappings=[
            # Standardize all topics to /autonomy namespace
            ('mission_status', '/autonomy/mission_status'),
            ('system_mode', '/autonomy/system_mode'),
            ('emergency_stop', '/autonomy/emergency_stop'),
            ('waypoint_goal', '/autonomy/waypoint_goal'),
            ('autonomy_health', '/autonomy/autonomy_health'),
            ('performance_metrics', '/autonomy/performance_metrics'),
            ('start_mission', '/autonomy/start_mission'),
            ('stop_mission', '/autonomy/stop_mission'),
            ('configure_mission', '/autonomy/configure_mission'),
            ('switch_mode', '/autonomy/switch_mode'),
            ('reset_autonomy', '/autonomy/reset_autonomy'),
            ('get_subsystem_status', '/autonomy/get_subsystem_status'),
            ('simulate_target_reached', '/autonomy/simulate_target_reached'),
        ],
        output='screen',
        emulate_tty=True,
    )

    # ==========================================
    # NAVIGATION SUBSYSTEM
    # ==========================================
    navigation_group = GroupAction([
        PushRosNamespace('navigation'),
        Node(
            package='autonomy_navigation',
            executable='navigation_node',
            name='navigation_node',
            parameters=[
                common_params,
                {'update_rate': 10.0},
                {'waypoint_tolerance': 2.0},
                PathJoinSubstitution([nav_pkg, 'config', 'navigation.yaml']),
            ],
            remappings=[
                # Standardize all navigation topics to /autonomy namespace
                ('navigation/status', '/autonomy/navigation/status'),
                ('navigation/current_waypoint', '/autonomy/navigation/current_waypoint'),
                ('navigation/waypoint_reached', '/autonomy/navigation/waypoint_reached'),
                ('cmd_vel', '/autonomy/cmd_vel'),
                ('gnss/fix', '/autonomy/gnss/fix'),
                ('imu/data', '/autonomy/imu/data'),
                ('wheel/odom', '/autonomy/wheel/odom'),
                ('navigation/navigate_to_waypoint', '/autonomy/navigation/navigate_to_waypoint'),
                ('navigation/stop', '/autonomy/navigation/stop'),
                ('navigation/get_current_waypoint', '/autonomy/navigation/get_current_waypoint'),
                ('navigate_to_pose', '/autonomy/navigate_to_pose'),
            ],
            output='screen',
        ),
        # GNSS processor (if available)
        Node(
            package='autonomy_navigation',
            executable='gnss_processor',
            name='gnss_processor',
            parameters=[common_params],
            remappings=[
                ('gnss/fix', '/autonomy/gnss/fix'),
            ],
        ),
    ])

    # ==========================================
    # SLAM SUBSYSTEM (Conditional)
    # ==========================================
    slam_group = GroupAction([
        PushRosNamespace('slam'),
        Node(
            package='autonomy_slam',
            executable='slam_node',
            name='slam_node',
            parameters=[
                common_params,
                PathJoinSubstitution([slam_pkg, 'config', 'slam.yaml']),
            ],
            remappings=[
                # Standardize all SLAM topics to /autonomy namespace
                ('slam/pose', '/autonomy/slam/pose'),
                ('slam/odom', '/autonomy/slam/odom'),
                ('slam/map', '/autonomy/slam/map'),
                ('slam/status', '/autonomy/slam/status'),
                ('gps/fix', '/autonomy/gnss/fix'),
                ('imu/data', '/autonomy/imu/data'),
                ('wheel/odom', '/autonomy/wheel/odom'),
            ],
            output='screen',
        ),
    ], condition=IfCondition(LaunchConfiguration('enable_slam')))

    # ==========================================
    # COMPUTER VISION SUBSYSTEM (Conditional)
    # ==========================================
    vision_group = GroupAction([
        PushRosNamespace('vision'),
        Node(
            package='autonomy_computer_vision',
            executable='computer_vision_node',
            name='computer_vision_node',
            parameters=[
                common_params,
                PathJoinSubstitution([vision_pkg, 'config', 'vision.yaml']),
            ],
            remappings=[
                # Standardize all vision topics to /autonomy namespace
                ('vision/detections', '/autonomy/vision/detections'),
                ('vision/aruco_markers', '/autonomy/vision/aruco_markers'),
                ('vision/status', '/autonomy/vision/status'),
                # Camera inputs (from sensors)
                ('camera/image_raw', '/autonomy/camera/image_raw'),
                ('camera/depth/image_raw', '/autonomy/camera/depth/image_raw'),
                ('camera/info', '/autonomy/camera/camera_info'),
                ('mast_camera/command', '/autonomy/mast_camera/command'),
            ],
            output='screen',
        ),
    ], condition=IfCondition(LaunchConfiguration('enable_vision')))

    # ==========================================
    # AUTONOMOUS TYPING SUBSYSTEM
    # ==========================================
    typing_group = GroupAction([
        PushRosNamespace('typing'),
        Node(
            package='autonomy_autonomous_typing',
            executable='autonomous_typing_node',
            name='autonomous_typing_node',
            parameters=[
                common_params,
                PathJoinSubstitution([typing_pkg, 'config', 'typing.yaml']),
            ],
            remappings=[
                # Standardize all typing topics to /autonomy namespace
                ('typing/status', '/autonomy/typing/status'),
                ('typing/target_pose', '/autonomy/typing/target_pose'),
                ('perform_typing', '/autonomy/perform_typing'),
            ],
            output='screen',
        ),
    ])

    # ==========================================
    # LED STATUS SUBSYSTEM
    # ==========================================
    led_group = GroupAction([
        PushRosNamespace('led'),
        Node(
            package='autonomy_led_status',
            executable='led_status_node',
            name='led_status_node',
            parameters=[common_params],
            remappings=[
                # Standardize all LED topics to /autonomy namespace
                ('led/status', '/autonomy/led/status'),
                ('led/command', '/autonomy/led/command'),
                ('system_mode', '/autonomy/system_mode'),
                ('mission_status', '/autonomy/mission_status'),
            ],
            output='screen',
        ),
    ])

    # ==========================================
    # INTEGRATION EVENT HANDLERS
    # ==========================================

    # System startup sequence
    startup_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=state_management_node,
            on_start=[
                LogInfo(msg='🚀 Autonomy system starting...'),
                TimerAction(
                    period=2.0,
                    actions=[LogInfo(msg='All subsystems initialized')]
                )
            ]
        )
    )

    # Emergency shutdown on state management failure
    emergency_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=state_management_node,
            on_exit=[
                LogInfo(msg='❌ State management failed - emergency shutdown'),
                # Could add emergency stop actions here
            ]
        )
    )

    # ==========================================
    # LAUNCH DESCRIPTION
    # ==========================================
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(namespace_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(enable_vision_arg)
    ld.add_action(enable_slam_arg)

    # Add event handlers
    ld.add_action(startup_handler)
    ld.add_action(emergency_handler)

    # Add subsystem groups (order matters for dependencies)
    ld.add_action(state_management_node)  # Start first
    ld.add_action(led_group)              # Simple, start early
    ld.add_action(navigation_group)       # Core functionality
    ld.add_action(slam_group)             # Depends on sensors
    ld.add_action(vision_group)           # Depends on cameras
    ld.add_action(typing_group)           # Depends on vision/navigation

    return ld


# Alternative: Modular launch inclusion
def create_subsystem_launch(subsystem_name: str):
    """Create launch inclusion for individual subsystems."""
    pkg_name = f'autonomy_{subsystem_name}'

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare(pkg_name),
            f'/launch/{subsystem_name}.launch.py'
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
