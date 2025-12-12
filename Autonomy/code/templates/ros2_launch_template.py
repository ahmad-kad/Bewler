"""
URC 2026 Autonomy System - ROS 2 Launch Template

Template for creating ROS 2 launch files to start multiple nodes and systems.

Author: [Your Name]
Date: [Date]
"""

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """
    Generate launch description for the [system name] system.

    This launch file starts all the nodes required for [brief description].
    """

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument("namespace", default_value="", description="Namespace for all nodes")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="Logging level",
    )

    # Configuration file paths
    pkg_share = FindPackageShare("your_package")
    config_dir = PathJoinSubstitution([pkg_share, "config"])
    launch_dir = PathJoinSubstitution([pkg_share, "launch"])

    # RViz configuration
    rviz_config = PathJoinSubstitution([config_dir, "your_config.rviz"])

    # Create nodes
    nodes = create_nodes()

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(namespace_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(log_level_arg)

    # Add nodes
    for node in nodes:
        ld.add_action(node)

    # Add event handlers
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=nodes[0],  # First node
                on_start=[
                    LogInfo(msg="System starting..."),
                ],
            )
        )
    )

    return ld


def create_nodes():
    """Create all the nodes for this system."""
    nodes = []

    # Common parameters
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    namespace = LaunchConfiguration("namespace")

    # Example node 1 - Core processing node
    core_node = Node(
        package="your_package",
        executable="core_node",
        name="core_processor",
        namespace=namespace,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"log_level": log_level},
            # Add your parameters here
            {"param1": 1.0},
            {"param2": "value"},
        ],
        remappings=[
            # Add topic remappings if needed
            # ('input_topic', 'remapped_input'),
        ],
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level],
    )
    nodes.append(core_node)

    # Example node 2 - Sensor processing
    sensor_node = Node(
        package="your_package",
        executable="sensor_processor",
        name="sensor_processor",
        namespace=namespace,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"sensor_timeout": 1.0},
            {"filter_enabled": True},
        ],
        output="screen",
        emulate_tty=True,
    )
    nodes.append(sensor_node)

    # Example node 3 - Control node
    control_node = Node(
        package="your_package",
        executable="controller",
        name="controller",
        namespace=namespace,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"control_rate": 10.0},
            {"max_velocity": 1.0},
        ],
        output="screen",
        emulate_tty=True,
    )
    nodes.append(control_node)

    # RViz visualization (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        arguments=["-d", rviz_config],
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_sim_time),  # Only in real operation
    )
    nodes.append(rviz_node)

    return nodes


# Alternative: Using OpaqueFunction for dynamic node creation
def create_dynamic_nodes(context):
    """Create nodes dynamically based on launch arguments."""
    nodes = []

    namespace = context.launch_configurations["namespace"]
    use_sim_time = context.launch_configurations["use_sim_time"]

    # Dynamic node creation based on conditions
    if use_sim_time == "true":
        # Add simulation-specific nodes
        sim_node = Node(
            package="your_package",
            executable="simulator",
            name="simulator",
            namespace=namespace,
        )
        nodes.append(sim_node)

    return nodes


def generate_launch_description_with_opaque():
    """Alternative launch description using OpaqueFunction."""
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(DeclareLaunchArgument("namespace", default_value=""))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))

    # Opaque function for dynamic node creation
    ld.add_action(OpaqueFunction(function=create_dynamic_nodes))

    return ld


# Example: Including other launch files
def create_included_launch():
    """Example of including other launch files."""
    pkg_share = FindPackageShare("your_package")
    other_launch = PathJoinSubstitution([pkg_share, "launch", "other_launch.py"])

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch),
        launch_arguments={
            "namespace": "subsystem",
            "use_sim_time": "false",
        }.items(),
    )

    return included_launch


# Example: Grouped actions with conditions
def create_conditional_group():
    """Example of conditional node groups."""
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Group of nodes that only run in simulation
    sim_group = GroupAction(
        [
            PushRosNamespace("simulation"),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", "robot", "-file", "robot.urdf"],
            ),
        ],
        condition=IfCondition(use_sim_time),
    )

    return sim_group
