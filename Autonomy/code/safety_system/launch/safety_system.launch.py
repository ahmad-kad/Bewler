"""
Launch file for the complete URC 2026 Safety System.

Launches all safety components: watchdog, redundant monitor, emergency coordinator, and dashboard.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Generate launch description for safety system."""

    # Get package directories
    safety_system_dir = get_package_share_directory('autonomy_safety_system')
    state_management_dir = get_package_share_directory('autonomy_state_machine')

    # Declare launch arguments
    enable_safety_watchdog = DeclareLaunchArgument(
        'enable_safety_watchdog',
        default_value='true',
        description='Enable the safety watchdog system'
    )

    enable_redundant_monitor = DeclareLaunchArgument(
        'enable_redundant_monitor',
        default_value='true',
        description='Enable the redundant safety monitor'
    )

    enable_emergency_coordinator = DeclareLaunchArgument(
        'enable_emergency_coordinator',
        default_value='true',
        description='Enable the emergency response coordinator'
    )

    enable_safety_dashboard = DeclareLaunchArgument(
        'enable_safety_dashboard',
        default_value='true',
        description='Enable the safety monitoring dashboard'
    )

    enable_integration_tester = DeclareLaunchArgument(
        'enable_integration_tester',
        default_value='false',
        description='Enable the safety integration tester (for testing only)'
    )

    safety_namespace = DeclareLaunchArgument(
        'safety_namespace',
        default_value='safety',
        description='Namespace for safety system components'
    )

    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for safety components'
    )

    # Safety Watchdog Node
    safety_watchdog_node = Node(
        package='autonomy_safety_system',
        executable='safety_watchdog',
        name='safety_watchdog',
        namespace=LaunchConfiguration('safety_namespace'),
        parameters=[{
            'watchdog_levels': ['HEARTBEAT', 'STATE_TRANSITIONS', 'SUBSYSTEM_HEALTH', 'SENSOR_INTEGRITY'],
            'heartbeat_timeout': 5.0,
            'state_transition_timeout': 30.0,
            'subsystem_health_timeout': 10.0,
            'sensor_integrity_timeout': 2.0,
            'battery_critical_threshold': 10.0,
            'temperature_critical_threshold': 85.0,
            'enable_emergency_stop': True
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('enable_safety_watchdog'))
    )

    # Redundant Safety Monitor Node
    redundant_monitor_node = Node(
        package='autonomy_safety_system',
        executable='redundant_safety_monitor',
        name='redundant_safety_monitor',
        namespace=LaunchConfiguration('safety_namespace'),
        parameters=[{
            'imu_accel_max': 50.0,
            'imu_gyro_max': 20.0,
            'battery_critical': 10.0,
            'battery_warning': 20.0,
            'temperature_critical': 85.0,
            'temperature_warning': 70.0,
            'velocity_max': 5.0,
            'communication_timeout': 5.0
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('enable_redundant_monitor'))
    )

    # Emergency Response Coordinator Node
    emergency_coordinator_node = Node(
        package='autonomy_safety_system',
        executable='emergency_response_coordinator',
        name='emergency_response_coordinator',
        namespace=LaunchConfiguration('safety_namespace'),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('enable_emergency_coordinator'))
    )

    # Safety Dashboard Node
    safety_dashboard_node = Node(
        package='autonomy_safety_system',
        executable='safety_dashboard',
        name='safety_dashboard',
        namespace=LaunchConfiguration('safety_namespace'),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('enable_safety_dashboard'))
    )

    # Safety Integration Tester Node (for testing only)
    integration_tester_node = Node(
        package='autonomy_safety_system',
        executable='safety_integration_tester',
        name='safety_integration_tester',
        namespace=LaunchConfiguration('safety_namespace'),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('enable_integration_tester'))
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(enable_safety_watchdog)
    ld.add_action(enable_redundant_monitor)
    ld.add_action(enable_emergency_coordinator)
    ld.add_action(enable_safety_dashboard)
    ld.add_action(enable_integration_tester)
    ld.add_action(safety_namespace)
    ld.add_action(log_level)

    # Add nodes
    ld.add_action(safety_watchdog_node)
    ld.add_action(redundant_monitor_node)
    ld.add_action(emergency_coordinator_node)
    ld.add_action(safety_dashboard_node)
    ld.add_action(integration_tester_node)

    return ld
