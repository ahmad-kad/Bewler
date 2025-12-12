#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Launch full simulation with Gazebo (when available) and autonomy systems."""

    # Package directories
    pkg_simulation = get_package_share_directory("autonomy_simulation")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_gazebo = LaunchConfiguration(
        "use_gazebo", default="false"
    )  # Disabled by default for ARM64
    world_file = LaunchConfiguration("world_file", default="mars_desert.world")
    robot_name = LaunchConfiguration("robot_name", default="rover2025")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )

    declare_use_gazebo = DeclareLaunchArgument(
        "use_gazebo",
        default_value="false",
        description="Use Gazebo simulation (requires Gazebo installation)",
    )

    declare_world_file = DeclareLaunchArgument(
        "world_file",
        default_value="mars_desert.world",
        description="World file to load",
    )

    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="rover2025", description="Robot name"
    )

    # Robot description
    robot_description_content = Command(
        [
            "xacro ",
            os.path.join(pkg_simulation, "urdf", "rover2025_enhanced.urdf.xacro"),
        ]
    )

    robot_description = {
        "robot_description": robot_description_content,
        "use_sim_time": use_sim_time,
    }

    # Gazebo simulation (conditional)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(use_gazebo),
        launch_arguments={
            "world": PathJoinSubstitution([pkg_simulation, "worlds", world_file]),
            "verbose": "true",
        }.items(),
    )

    # Basic simulation (when Gazebo not available)
    basic_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_simulation, "launch", "basic_simulation.launch.py")]
        ),
        condition=IfCondition(PythonExpression(["not ", use_gazebo])),
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            robot_name,
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.5",
        ],
        condition=IfCondition(use_gazebo),
        output="screen",
    )

    # Digital Twin System (PLACEHOLDER integration)
    digital_twin = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    pkg_simulation, "digitaltwins", "launch", "digital_twin.launch.py"
                )
            ]
        ),
        launch_arguments={
            "sync_rate": "10.0",  # PLACEHOLDER
            "prediction_horizon": "5.0",  # PLACEHOLDER
            "twin_fidelity": "0.85",  # PLACEHOLDER
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # RViz for visualization
    rviz_config = os.path.join(pkg_simulation, "rviz", "simulation.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_gazebo)
    ld.add_action(declare_world_file)
    ld.add_action(declare_robot_name)

    # Add simulation components
    ld.add_action(gazebo)
    ld.add_action(basic_sim)
    ld.add_action(spawn_robot)

    # Add digital twin (PLACEHOLDER)
    ld.add_action(digital_twin)

    # Add visualization
    ld.add_action(rviz)

    return ld
