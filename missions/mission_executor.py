#!/usr/bin/env python3
"""
Mission Executor - Core mission execution with integrated health monitoring

Handles mission planning, execution, and coordination using composition
with specialized components for data processing and health monitoring.

Author: URC 2026 Autonomy Team
"""

# Standard library
import json
import math
import os
import time
import uuid
from typing import Any, Dict, List, Optional

import rclpy
import structlog
import yaml

# ROS2 messages
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Float32, Float32MultiArray, String

# Configuration and validation
from .config_validator import validate_startup_configuration
from .emergency_response_coordinator import EmergencyResponseCoordinator, EmergencyType

# Non-invasive monitoring
from .monitoring_system import (
    MonitoringConfig,
    MonitoringEvent,
    SamplingRate,
    get_monitor,
    record_detection,
    record_emergency,
    record_failure,
)
from .system_health_monitor import SystemHealthMonitor

# Specialized components
from .teleoperation_data_processor import TeleoperationDataProcessor

logger = structlog.get_logger(__name__)


class MissionExecutor(Node):
    """Core mission execution system with integrated health monitoring and safety.

    The MissionExecutor serves as the central coordinator for autonomous mission
    execution on the URC 2026 Mars rover. It integrates multiple specialized
    subsystems to provide robust, safe, and reliable mission execution.

    Key Responsibilities:
        - Mission planning and execution orchestration
        - Real-time navigation and waypoint following
        - Integration with health monitoring systems
        - Emergency response coordination and safety
        - Teleoperation data processing and validation

    Architecture:
        Uses composition pattern with specialized components:
        - TeleoperationDataProcessor: Validates and filters teleoperation sensor data
        - SystemHealthMonitor: Monitors thermal, battery, and motor health
        - EmergencyResponseCoordinator: Handles emergency situations and safety protocols

    Monitoring:
        Integrates non-invasive monitoring system for performance tracking
        and event-driven data collection without impacting real-time performance.

    Attributes:
        data_processor (TeleoperationDataProcessor): Handles teleoperation data processing
        health_monitor (SystemHealthMonitor): System health monitoring
        emergency_coordinator (EmergencyResponseCoordinator): Emergency handling
        monitor (NonInvasiveMonitor): Performance and event monitoring
    """

    def __init__(self) -> None:
        """Initialize the Mission Executor with all subsystems.

        Sets up ROS2 interfaces, QoS profiles, mission state, and all specialized
        components including health monitoring and emergency response systems.
        Performs configuration validation at startup.
        """
        super().__init__("mission_executor")

        # Setup phases
        self._setup_qos_profiles()
        self._setup_ros_interfaces()
        self._setup_mission_state()
        self._setup_components()
        self._setup_monitoring()
        self._setup_message_pooling()

        self.get_logger().info(
            "Mission Executor initialized with integrated health monitoring"
        )

    def _setup_qos_profiles(self) -> None:
        """Setup ROS QoS profiles for different message types."""
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

    def _setup_ros_interfaces(self) -> None:
        """Setup ROS subscribers, publishers, and services."""
        # Core mission subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, self.sensor_qos
        )
        self.cmd_sub = self.create_subscription(
            String, "/mission/commands", self.command_callback, self.command_qos
        )

        # Teleoperation data subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/teleoperation/joint_states",
            self.joint_state_callback,
            self.sensor_qos,
        )
        self.chassis_vel_sub = self.create_subscription(
            TwistStamped,
            "/teleoperation/chassis_velocity",
            self.chassis_velocity_callback,
            self.sensor_qos,
        )
        self.motor_temp_sub = self.create_subscription(
            Float32MultiArray,
            "/teleoperation/motor_temperatures",
            self.motor_temp_callback,
            self.sensor_qos,
        )
        self.system_status_sub = self.create_subscription(
            BatteryState,
            "/teleoperation/system_status",
            self.system_status_callback,
            self.sensor_qos,
        )

        # Publishers
        self.status_pub = self.create_publisher(String, "/mission/status", self.status_qos)
        self.progress_pub = self.create_publisher(
            Float32, "/mission/progress", self.status_qos
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", self.command_qos)

        # Health check service for container orchestration
        from std_srvs.srv import Trigger
        self.health_srv = self.create_service(
            Trigger, "/mission/health_check", self.health_check_callback
        )

        # Execution timer
        self.execution_timer = self.create_timer(0.1, self.execution_step)

    def _setup_mission_state(self) -> None:
        """Initialize mission state variables."""
        self.current_position: Optional[object] = None
        self.current_pose = None
        self.waypoints: List[Dict[str, float]] = []
        self.current_waypoint_index = 0
        self.mission_active = False
        self.mission_completed = False
        self.mission_failed = False

    def _setup_components(self) -> None:
        """Initialize specialized components and configuration."""
        # Load configuration
        self.config = self.load_config()

        # Initialize specialized components
        self.data_processor = TeleoperationDataProcessor(self.config)
        self.health_monitor = SystemHealthMonitor(self.config)
        self.emergency_coordinator = EmergencyResponseCoordinator(self.config)

        # Connect emergency callbacks
        self._setup_emergency_callbacks()

    def _setup_monitoring(self) -> None:
        """Initialize non-invasive monitoring system."""
        monitoring_config = MonitoringConfig(
            sampling_rate=SamplingRate.MEDIUM,  # Configurable based on environment
            max_buffer_size=500,  # Limit memory usage
            enable_async_processing=True
        )
        self.monitor = get_monitor()
        self.monitor.update_config(monitoring_config)

    def _setup_message_pooling(self) -> None:
        """Setup reusable message objects for efficiency."""
        self.reusable_twist = Twist()
        self.reusable_status = String()
        self.reusable_progress = Float32()

    def _setup_emergency_callbacks(self) -> None:
        """Setup emergency response callbacks"""
        self.emergency_coordinator.register_emergency_callback(
            EmergencyType.THERMAL_OVERLOAD, self._handle_thermal_emergency
        )
        self.emergency_coordinator.register_emergency_callback(
            EmergencyType.BATTERY_CRITICAL, self._handle_battery_emergency
        )

    def joint_state_callback(self, msg: JointState) -> None:
        """Handle joint state data from teleoperation monitoring"""
        if self.data_processor.process_joint_state_data(msg):
            # Update health monitoring
            motor_healthy = self.health_monitor.monitor_motor_health(
                self.data_processor.latest_motor_data
            )

            if not motor_healthy:
                self.get_logger().warn("Motor health issues detected")

    def chassis_velocity_callback(self, msg: TwistStamped) -> None:
        """Handle chassis velocity data from teleoperation monitoring"""
        self.data_processor.process_chassis_velocity_data(msg)

    def motor_temp_callback(self, msg: Float32MultiArray) -> None:
        """Handle motor temperature data from teleoperation monitoring"""
        if self.data_processor.process_motor_temperature_data(msg):
            # Update thermal monitoring
            thermal_status = self.health_monitor.monitor_thermal_limits(
                self.data_processor.motor_temperatures
            )

            # Apply thermal speed adjustments
            if thermal_status["action"] in ["reduce_speed", "shutdown"]:
                self._apply_speed_adjustment(thermal_status["speed_factor"])

    def system_status_callback(self, msg: BatteryState) -> None:
        """Handle system status data from teleoperation monitoring"""
        if self.data_processor.process_system_status_data(msg):
            # Update battery monitoring
            battery_status = self.health_monitor.monitor_system_health(
                self.data_processor.system_status
            )

            # Apply battery conservation measures
            if battery_status["action"] in ["conserve_energy", "return_to_base"]:
                self._apply_speed_adjustment(battery_status["speed_factor"])
                if battery_status["action"] == "return_to_base":
                    self._initiate_return_to_base()

    def odom_callback(self, msg: Odometry) -> None:
        """Handle odometry data"""
        self.current_position = msg.pose.pose.position
        self.current_pose = msg.pose.pose

    def command_callback(self, msg: String) -> None:
        """Handle mission commands"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get("command", "")
            params = command_data.get("params", {})

            self.dispatch_command(command, params)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid command JSON: {msg.data}")

    def dispatch_command(self, command: str, params: Dict[str, Any]) -> None:
        """Dispatch mission commands to appropriate handler methods.

        Args:
            command: The command name (e.g., 'start_mission', 'stop_mission')
            params: Command parameters as a dictionary
        """
        handlers = {
            "start_mission": lambda: self.start_waypoint_mission(
                params.get("waypoints", [])
            ),
            "stop_mission": lambda: self.stop_mission(),
            "pause_mission": lambda: self.pause_mission(),
            "resume_mission": lambda: self.resume_mission(),
        }

        handler = handlers.get(command)
        if handler:
            handler()
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def start_waypoint_mission(self, waypoints: List[Dict[str, float]]) -> None:
        """Start autonomous waypoint navigation mission.

        Initializes and begins execution of a waypoint-based navigation mission.
        Records the mission start event for monitoring purposes.

        Args:
            waypoints: List of waypoint dictionaries containing 'x', 'y', 'heading' coordinates
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.mission_active = True
        self.mission_completed = False
        self.mission_failed = False

        # Monitor mission start (non-invasive)
        record_detection(
            "mission_executor",
            {
                "event": "mission_started",
                "mission_type": "waypoint_navigation",
                "waypoint_count": len(waypoints),
                "timestamp": time.time()
            }
        )

        self.publish_status("mission_started")
        self.get_logger().info(
            f"Started waypoint mission with {len(waypoints)} waypoints"
        )

    def stop_mission(self) -> None:
        """Stop current mission"""
        self.mission_active = False
        self.publish_zero_velocity()
        self.publish_status("mission_stopped")
        self.get_logger().info("Mission stopped")

    def pause_mission(self) -> None:
        """Pause current mission"""
        if self.mission_active:
            self.mission_paused = True
            self.publish_zero_velocity()
            self.get_logger().info("Mission paused")

    def resume_mission(self):
        """Resume paused mission"""
        if hasattr(self, "mission_paused") and self.mission_paused:
            self.mission_paused = False
            self.get_logger().info("Mission resumed")

    def execution_step(self):
        """Main execution loop"""
        if not self.mission_active or getattr(self, "mission_paused", False):
            return

        # Check for emergency conditions
        if self.emergency_coordinator.emergency_mode:
            self._handle_emergency_mode()
            return

        # Execute mission logic
        self.navigate_to_current_waypoint()

    def navigate_to_current_waypoint(self):
        """Navigate to current waypoint"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            self.mission_completed = True
            self.mission_active = False
            self.get_logger().info("Mission completed")
            return

        waypoint = self.waypoints[self.current_waypoint_index]

        if self.is_at_waypoint(waypoint):
            self.current_waypoint_index += 1
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}")
            if self.current_waypoint_index >= len(self.waypoints):
                self.mission_completed = True
                self.mission_active = False
                self.publish_status("mission_completed")
                self.get_logger().info("Mission completed")
            return

        # Calculate and publish velocity command
        self.calculate_velocity_to_waypoint(waypoint)

    def is_at_waypoint(self, waypoint) -> bool:
        """Check if robot is at waypoint"""
        if not self.current_position:
            return False

        dx = waypoint["x"] - self.current_position.x
        dy = waypoint["y"] - self.current_position.y
        distance = math.sqrt(dx * dx + dy * dy)

        return distance < 0.5  # 0.5 meter tolerance

    def calculate_velocity_to_waypoint(self, waypoint):
        """Calculate velocity command to reach waypoint"""
        if not self.current_position or not self.current_pose:
            return

        # Calculate direction to waypoint
        dx = waypoint["x"] - self.current_position.x
        dy = waypoint["y"] - self.current_position.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 0.1:  # Very close, stop
            self.publish_zero_velocity()
            return

        # Calculate velocity with speed adjustments
        base_speed = 0.5
        speed_factor = self._get_effective_speed_factor()
        speed = min(base_speed * speed_factor, 1.0)  # Cap at 1.0 m/s

        # Calculate angle to waypoint
        target_angle = math.atan2(dy, dx)

        # Get current orientation
        orientation = self.current_pose.orientation
        current_angle = 2 * math.atan2(
            orientation.z, orientation.w
        )  # Convert quaternion to yaw

        # Calculate angular velocity for rotation
        angle_diff = target_angle - current_angle
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        angular_speed = angle_diff * 2.0  # P controller for rotation
        angular_speed = max(-1.0, min(1.0, angular_speed))  # Cap angular speed

        # Publish velocity command
        self.publish_velocity_commands(speed, angular_speed)

    def _get_effective_speed_factor(self) -> float:
        """Get effective speed factor from all health considerations"""
        health_status = self.health_monitor.get_overall_health_status()
        return health_status.get("effective_speed_factor", 1.0)

    def _apply_speed_adjustment(self, speed_factor: float):
        """Apply speed adjustment factor"""
        self.get_logger().info(f"Applying speed adjustment: {speed_factor:.2f}")

    def _initiate_return_to_base(self):
        """Initiate return to base protocol"""
        self.get_logger().error("CRITICAL: Initiating return to base protocol")
        # Simplified return logic - could be more sophisticated
        self.stop_mission()

    def _handle_thermal_emergency(self, emergency_record):
        """Handle thermal emergency"""
        # Monitor emergency event (critical - always recorded)
        record_emergency(
            "mission_executor",
            {
                "emergency_type": "thermal_overload",
                "action": "emergency_shutdown",
                "emergency_record": emergency_record
            }
        )

        self.get_logger().error("THERMAL EMERGENCY: Initiating emergency shutdown")
        self.stop_mission()
        self.publish_zero_velocity()

    def _handle_battery_emergency(self, emergency_record):
        """Handle battery emergency"""
        # Monitor emergency event (critical - always recorded)
        record_emergency(
            "mission_executor",
            {
                "emergency_type": "battery_critical",
                "action": "return_to_base",
                "emergency_record": emergency_record
            }
        )

        self.get_logger().error("BATTERY EMERGENCY: Initiating return to base")
        self._initiate_return_to_base()

    def _handle_emergency_mode(self):
        """Handle ongoing emergency conditions"""
        # Emergency mode - stop all autonomous operation
        self.publish_zero_velocity()

    def publish_velocity_commands(self, linear_vel: float, angular_vel: float):
        """Publish velocity commands"""
        self.reusable_twist.linear.x = linear_vel
        self.reusable_twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(self.reusable_twist)

    def publish_zero_velocity(self):
        """Publish zero velocity command"""
        self.reusable_twist.linear.x = 0.0
        self.reusable_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.reusable_twist)

    def publish_status(self, status: str):
        """Publish mission status"""
        status_data = {
            "status": status,
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": len(self.waypoints),
            "mission_active": self.mission_active,
            "emergency_mode": self.emergency_coordinator.emergency_mode,
        }
        self.reusable_status.data = json.dumps(status_data)
        self.status_pub.publish(self.reusable_status)

    def publish_progress(self, progress: float):
        """Publish mission progress"""
        self.reusable_progress.data = progress
        self.progress_pub.publish(self.reusable_progress)

    def health_check_callback(self, request, response) -> None:
        """
        Health check callback for container orchestration.

        Returns health status including component checks and emergency state.
        """
        try:
            # Basic health checks
            checks_passed = True
            issues = []

            # Check ROS2 node health
            if not rclpy.ok():
                checks_passed = False
                issues.append("ROS2 context not healthy")

            # Check emergency state
            if self.emergency_coordinator.emergency_mode:
                checks_passed = False
                issues.append("System in emergency mode")

            # Check mission state
            if self.mission_failed:
                checks_passed = False
                issues.append("Mission failed")

            # Check data processor health (recent data)
            current_time = time.time()
            if hasattr(self, "last_data_time"):
                time_since_last_data = current_time - self.last_data_time
                if time_since_last_data > 5.0:  # 5 seconds timeout
                    checks_passed = False
                    issues.append(".2f")

            # Check health monitor status
            health_status = self.health_monitor.get_health_status()
            if health_status.get("emergency_mode", False):
                checks_passed = False
                issues.append("Health monitor in emergency mode")

            # Prepare response
            if checks_passed:
                # Include monitoring statistics in health check
                monitor_stats = self.monitor.get_monitoring_stats()
                response.success = True
                response.message = f"System healthy | Monitoring: {monitor_stats['events_processed']} events, {monitor_stats['buffer_size']}/{monitor_stats['buffer_capacity']} buffered"
                logger.info("Health check passed", monitoring_stats=monitor_stats)
            else:
                response.success = False
                response.message = f"Health check failed: {', '.join(issues)}"
                logger.warning(
                    "Health check failed",
                    issues=issues,
                    correlation_id=str(uuid.uuid4()),
                )

            return response

        except Exception as e:
            response.success = False
            response.message = f"Health check error: {str(e)}"
            logger.error(
                "Health check exception", error=str(e), correlation_id=str(uuid.uuid4())
            )
            return response

    def load_config(self):
        """Load configuration from YAML file"""
        env = os.environ.get("ROVER_ENV", "production")
        config_file = f"{env}.yaml"

        possible_paths = [
            os.path.join(os.path.dirname(__file__), "..", "config", config_file),
            os.path.join(os.path.dirname(__file__), "config", config_file),
            f"config/{config_file}",
        ]

        for config_path in possible_paths:
            try:
                with open(config_path, "r") as f:
                    config = yaml.safe_load(f)
                    self.get_logger().info(
                        f"Loaded {env} configuration from {config_path}"
                    )
                    return config
            except FileNotFoundError:
                continue

        self.get_logger().warn(
            f"Configuration file {config_file} not found, using defaults"
        )
        return {}


def main():
    """Main entry point"""
    # Validate configuration before starting
    logger.info("Starting Mission Executor - validating configuration")
    try:
        validate_startup_configuration()
        logger.info("Configuration validation passed")
    except SystemExit as e:
        # Configuration validation failed and already logged/exited
        return

    rclpy.init()
    try:
        node = MissionExecutor()
        logger.info("Mission Executor started successfully")
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Mission Executor shutdown requested")
    except Exception as e:
        logger.critical(
            "Mission Executor crashed", error=str(e), correlation_id=str(uuid.uuid4())
        )
        raise
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
