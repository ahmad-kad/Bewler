#!/usr/bin/env python3
"""
WebSocket Sensor Bridge Node - URC 2026 Mars Rover

Converts WebSocket sensor data streams to ROS2 topics.

ARCHITECTURE DESIGN:
- WebSocket client receives JSON sensor data using ROS2 timers (no threading)
- Data is parsed and converted to standard ROS2 sensor messages
- Publishers use appropriate QoS settings for real-time sensor data
- Comprehensive error handling and data validation
- Designed for easy migration to direct ROS2 transport

MIGRATION PATH:
1. Replace WebSocket client with ROS2 subscription
2. Remove JSON parsing layer
3. Keep ROS2 message publishing (no changes needed)
4. Update QoS settings as needed

FUTURE ROS2 DIRECT TRANSPORT:
- Deploy ROS2 nodes on sensor devices
- Use DDS for direct communication
- Remove WebSocket/JSON overhead
- Enable 1000Hz+ sensor rates
"""

import json
import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, Optional

import rclpy
from autonomy.utilities import safe_execute, NodeLogger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.timer import Timer

# ROS2 sensor messages
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, Temperature
from std_msgs.msg import String

logger = logging.getLogger(__name__)


class ConnectionState(Enum):
    """WebSocket connection states."""

    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"
    FAILED = "failed"


@dataclass
class SensorConfig:
    """Configuration for a sensor data stream."""

    enabled: bool = True
    topic_name: str = ""
    qos_profile: QoSProfile = None
    message_converter: Callable = None
    update_rate: float = 10.0  # Hz


@dataclass
class ConnectionMetrics:
    """Connection and performance metrics."""

    messages_received: int = 0
    messages_processed: int = 0
    messages_failed: int = 0
    connection_attempts: int = 0
    connection_failures: int = 0
    last_message_timestamp: float = 0.0
    avg_processing_time: float = 0.0
    uptime_seconds: float = 0.0


class WebSocketSensorBridgeNode(Node):
    """
    WebSocket-to-ROS2 Sensor Bridge Node.

    Receives sensor data via WebSocket and publishes as ROS2 topics.
    Uses ROS2 timers instead of threading for reliability.
    Includes comprehensive error handling, data validation, and monitoring.
    """

    def __init__(self):
        super().__init__("websocket_sensor_bridge")

        # Initialize utilities
        self.logger = NodeLogger(self, "sensor_bridge")

        # Declare parameters with validation
        self._declare_parameters()

        # Get validated parameters
        self.websocket_url = self.get_parameter("websocket_url").value
        self.base_reconnect_interval = self.get_parameter("reconnect_interval").value
        self.max_reconnect_attempts = self.get_parameter("max_reconnect_attempts").value
        self.connection_timeout = self.get_parameter("connection_timeout").value
        self.health_check_interval = self.get_parameter("health_check_interval").value
        self.message_timeout = self.get_parameter("message_timeout").value
        self.enable_graceful_degradation = self.get_parameter(
            "enable_graceful_degradation"
        ).value

        # Connection state and metrics
        self.connection_state = ConnectionState.DISCONNECTED
        self.current_reconnect_interval = self.base_reconnect_interval
        self.reconnect_attempts = 0
        self.last_message_time = 0.0
        self.start_time = time.time()

        # Metrics tracking
        self.metrics = ConnectionMetrics()

        # Publishers dictionary (sensor_name -> publisher)
        self.publishers: Dict[str, Any] = {}

        # Callback groups for thread safety
        self.connection_group = MutuallyExclusiveCallbackGroup()
        self.processing_group = ReentrantCallbackGroup()
        self.monitoring_group = ReentrantCallbackGroup()

        # Sensor configurations with validation
        try:
            self.sensor_configs = self._create_sensor_configs()
        except Exception as e:
            self.get_logger().error(f"Failed to create sensor configs: {e}")
            raise

        # Setup publishers for enabled sensors
        try:
            self._setup_publishers()
        except Exception as e:
            self.get_logger().error(f"Failed to setup publishers: {e}")
            raise

        # Setup monitoring publishers
        try:
            self._setup_monitoring_publishers()
        except Exception as e:
            self.get_logger().error(f"Failed to setup monitoring publishers: {e}")
            raise

        # WebSocket connection management (using ROS2 timers)
        self.websocket_connection = None
        self.connection_timer: Optional[Timer] = None
        self.reconnect_timer: Optional[Timer] = None

        # Start initial connection attempt
        self._schedule_connection_attempt()

        # Health monitoring and metrics timers
        self.health_timer = self.create_timer(
            self.health_check_interval,
            self._check_connection_health,
            callback_group=self.monitoring_group,
        )

        self.metrics_timer = self.create_timer(
            5.0,  # Every 5 seconds
            self._publish_metrics,
            callback_group=self.monitoring_group,
        )

        self.get_logger().info(
            f"WebSocket Sensor Bridge initialized - URL: {self.websocket_url}, "
            f"Graceful degradation: {self.enable_graceful_degradation}"
        )

    def _declare_parameters(self):
        """Declare and validate all node parameters."""
        try:
            self.declare_parameter("websocket_url", "ws://localhost:8080")
            self.declare_parameter("reconnect_interval", 3.0)
            self.declare_parameter("max_reconnect_attempts", 10)
            self.declare_parameter("connection_timeout", 5.0)
            self.declare_parameter("health_check_interval", 1.0)
            self.declare_parameter("message_timeout", 10.0)
            self.declare_parameter("enable_graceful_degradation", True)
            self.declare_parameter("max_reconnect_interval", 60.0)
            self.declare_parameter("reconnect_backoff_multiplier", 1.5)
        except Exception as e:
            self.get_logger().error(f"Failed to declare parameters: {e}")
            raise

    def _create_sensor_configs(self) -> Dict[str, SensorConfig]:
        """Create sensor configurations with appropriate QoS settings."""

        # High-frequency sensor QoS (IMU, wheel odometry) - BEST_EFFORT for low latency
        qos_sensor_high_freq = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,  # Increased depth for high-frequency data
            durability=DurabilityPolicy.VOLATILE,
        )

        # Standard sensor QoS (GPS, temperature) - RELIABLE for occasional data
        qos_sensor_standard = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Critical sensor QoS (battery) - RELIABLE with persistence
        qos_sensor_critical = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Last value persists
            history=HistoryPolicy.KEEP_LAST,
            depth=5,  # Keep more history for diagnostics
        )

        return {
            "imu": SensorConfig(
                enabled=True,
                topic_name="/imu/data",
                qos_profile=qos_sensor_high_freq,
                message_converter=self._convert_imu_data,
                update_rate=100.0,  # IMU typically 100Hz
            ),
            "gps": SensorConfig(
                enabled=True,
                topic_name="/gps/fix",
                qos_profile=qos_sensor_standard,
                message_converter=self._convert_gps_data,
                update_rate=10.0,  # GPS typically 1-10Hz
            ),
            "battery": SensorConfig(
                enabled=True,
                topic_name="/battery/status",
                qos_profile=qos_sensor_critical,
                message_converter=self._convert_battery_data,
                update_rate=1.0,  # Battery status 1Hz
            ),
            "wheel_odom": SensorConfig(
                enabled=True,
                topic_name="/wheel/odom",
                qos_profile=qos_sensor_high_freq,
                message_converter=self._convert_wheel_odom,
                update_rate=50.0,  # Wheel odometry 50Hz
            ),
            "temperature": SensorConfig(
                enabled=True,
                topic_name="/temperature/data",
                qos_profile=qos_sensor_standard,
                message_converter=self._convert_temperature,
                update_rate=5.0,  # Temperature 5Hz
            ),
        }

    def _setup_monitoring_publishers(self):
        """Setup publishers for monitoring and diagnostics."""
        try:
            # Connection status publisher
            self.status_publisher = self.create_publisher(
                String,
                "/sensor_bridge/status",
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
                callback_group=self.monitoring_group,
            )

            # Diagnostics publisher
            self.diagnostics_publisher = self.create_publisher(
                DiagnosticArray,
                "/diagnostics",
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                ),
                callback_group=self.monitoring_group,
            )

            # Metrics publisher
            self.metrics_publisher = self.create_publisher(
                String,
                "/sensor_bridge/metrics",
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5,
                ),
                callback_group=self.monitoring_group,
            )

        except Exception as e:
            self.get_logger().error(f"Failed to create monitoring publishers: {e}")
            raise

    def _setup_publishers(self):
        """Setup ROS2 publishers for enabled sensors."""
        for sensor_name, config in self.sensor_configs.items():
            if not config.enabled:
                continue

            try:
                # Create publisher based on sensor type
                if sensor_name == "imu":
                    publisher = self.create_publisher(
                        Imu, config.topic_name, config.qos_profile
                    )
                elif sensor_name == "gps":
                    publisher = self.create_publisher(
                        NavSatFix, config.topic_name, config.qos_profile
                    )
                elif sensor_name == "battery":
                    publisher = self.create_publisher(
                        BatteryState, config.topic_name, config.qos_profile
                    )
                elif sensor_name == "wheel_odom":
                    publisher = self.create_publisher(
                        Odometry, config.topic_name, config.qos_profile
                    )
                elif sensor_name == "temperature":
                    publisher = self.create_publisher(
                        Temperature, config.topic_name, config.qos_profile
                    )
                else:
                    self.get_logger().warn(f"Unknown sensor type: {sensor_name}")
                    continue

                self.publishers[sensor_name] = publisher
                self.get_logger().info(
                    f"Setup publisher for {sensor_name} on topic {config.topic_name}"
                )

            except Exception as e:
                self.get_logger().error(
                    f"Failed to create publisher for {sensor_name}: {e}"
                )

    def _schedule_connection_attempt(self):
        """Schedule a WebSocket connection attempt using ROS2 timer."""
        try:
            if self.connection_timer:
                self.connection_timer.cancel()

            self.connection_timer = self.create_timer(
                0.1,  # Immediate connection attempt
                self._attempt_connection,
                callback_group=self.connection_group,
                oneshot=True,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to schedule connection attempt: {e}")

    def _schedule_reconnection(self):
        """Schedule reconnection with exponential backoff."""
        try:
            if self.reconnect_timer:
                self.reconnect_timer.cancel()

            # Exponential backoff with max limit
            max_reconnect_interval = self.get_parameter("max_reconnect_interval").value
            backoff_multiplier = self.get_parameter(
                "reconnect_backoff_multiplier"
            ).value

            self.current_reconnect_interval = min(
                self.current_reconnect_interval * backoff_multiplier,
                max_reconnect_interval,
            )

            self.reconnect_timer = self.create_timer(
                self.current_reconnect_interval,
                self._attempt_connection,
                callback_group=self.connection_group,
                oneshot=True,
            )

            self.get_logger().info(
                f"Scheduled reconnection in {self.current_reconnect_interval:.1f}s "
                f"(attempt {self.reconnect_attempts + 1}/{self.max_reconnect_attempts})"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to schedule reconnection: {e}")

    def _attempt_connection(self):
        """Attempt WebSocket connection using ROS2 timer."""
        try:
            self.metrics.connection_attempts += 1
            self.connection_state = ConnectionState.CONNECTING

            # Note: In a real implementation, we would use a WebSocket library
            # that supports non-blocking operation with ROS2. For now, we'll
            # simulate connection and message handling.

            # This is a placeholder - real implementation would need:
            # 1. Non-blocking WebSocket library (like websocket-client)
            # 2. Integration with ROS2 executor
            # 3. Proper async handling

            # Simulate connection success/failure for now
            import random

            if random.random() > 0.3:  # 70% success rate for testing
                self._on_connection_success()
            else:
                self._on_connection_failure("Simulated connection failure")

        except Exception as e:
            self.get_logger().error(f"Connection attempt failed: {e}")
            self._on_connection_failure(str(e))

    def _on_connection_success(self):
        """Handle successful WebSocket connection."""
        try:
            self.connection_state = ConnectionState.CONNECTED
            self.reconnect_attempts = 0
            self.current_reconnect_interval = self.base_reconnect_interval
            self.last_message_time = time.time()

            self.get_logger().info(f"Connected to WebSocket: {self.websocket_url}")

            # Reset exponential backoff
            if self.reconnect_timer:
                self.reconnect_timer.cancel()
                self.reconnect_timer = None

            # Publish connection status
            self._publish_connection_status()

        except Exception as e:
            self.get_logger().error(f"Error handling connection success: {e}")

    def _on_connection_failure(self, error_msg: str):
        """Handle WebSocket connection failure."""
        try:
            self.connection_state = ConnectionState.DISCONNECTED
            self.metrics.connection_failures += 1

            self.get_logger().error(f"WebSocket connection failed: {error_msg}")

            # Check if we should attempt reconnection
            if self.reconnect_attempts < self.max_reconnect_attempts:
                self.reconnect_attempts += 1
                self.connection_state = ConnectionState.RECONNECTING
                self._schedule_reconnection()
            else:
                self.connection_state = ConnectionState.FAILED
                self.get_logger().error(
                    f"Max reconnection attempts ({self.max_reconnect_attempts}) reached"
                )

            # Publish connection status
            self._publish_connection_status()

        except Exception as e:
            self.get_logger().error(f"Error handling connection failure: {e}")

    def _handle_websocket_message(self, message: str) -> None:
        """Process incoming WebSocket message with type-safe error handling."""
        processing_start = time.time()

        try:
            self.metrics.messages_received += 1

            # Validate input using safe_execute pattern
            if not isinstance(message, str) or not message.strip():
                self.logger.warn("Received empty or invalid message", message_length=len(message) if message else 0)
                self.metrics.messages_failed += 1
                return

            # Parse JSON with safe_execute
            result, error = safe_execute(json.loads, message)
            if error:
                self.logger.error(
                    "Invalid JSON message received",
                    error=error,
                    message_preview=message[:100] if len(message) > 100 else message
                )
                self.metrics.messages_failed += 1
                return

            data = result

            # Validate message structure
            validation_result = self._validate_message_structure(data)
            if not validation_result["valid"]:
                self.get_logger().error(
                    f'Invalid message structure: {validation_result["error"]}'
                )
                self.metrics.messages_failed += 1
                return

            timestamp = data["timestamp"]
            sensors = data["sensors"]

            # Process each sensor in the message
            processed_sensors = 0
            failed_sensors = 0

            for sensor_name, sensor_data in sensors.items():
                try:
                    success = self._process_sensor_data(
                        sensor_name, sensor_data, timestamp
                    )
                    if success:
                        processed_sensors += 1
                    else:
                        failed_sensors += 1
                except Exception as e:
                    self.get_logger().error(
                        f"Error processing sensor {sensor_name}: {e}"
                    )
                    failed_sensors += 1

            # Update metrics
            self.metrics.messages_processed += processed_sensors
            self.metrics.messages_failed += failed_sensors
            self.metrics.last_message_timestamp = time.time()

            # Update processing time average
            processing_time = time.time() - processing_start
            if self.metrics.avg_processing_time == 0:
                self.metrics.avg_processing_time = processing_time
            else:
                self.metrics.avg_processing_time = (
                    self.metrics.avg_processing_time + processing_time
                ) / 2.0

            if processed_sensors > 0:
                self.get_logger().debug(
                    f"Processed {processed_sensors} sensors, {failed_sensors} failed "
                    f"({processing_time:.3f}s)"
                )

        except Exception as e:
            self.get_logger().error(f"Unexpected error in message handling: {e}")
            self.metrics.messages_failed += 1

    def _validate_message_structure(self, data: Any) -> Dict[str, Any]:
        """Validate WebSocket message structure and data types."""
        try:
            if not isinstance(data, dict):
                return {"valid": False, "error": "Message must be a JSON object"}

            # Check required fields
            required_fields = ["timestamp", "sensors"]
            for field in required_fields:
                if field not in data:
                    return {"valid": False, "error": f"Missing required field: {field}"}

            # Validate timestamp
            timestamp = data["timestamp"]
            if not isinstance(timestamp, (int, float)):
                return {"valid": False, "error": "Timestamp must be a number"}

            current_time = time.time()
            if abs(timestamp - current_time) > 3600:  # 1 hour tolerance
                self.get_logger().warn(
                    f"Timestamp drift detected: {timestamp} vs {current_time} "
                    f"(diff: {abs(timestamp - current_time):.1f}s)"
                )

            # Validate sensors field
            sensors = data["sensors"]
            if not isinstance(sensors, dict):
                return {"valid": False, "error": "Sensors field must be an object"}

            if not sensors:
                return {"valid": False, "error": "Sensors object cannot be empty"}

            # Validate sensor names
            valid_sensor_names = set(self.sensor_configs.keys())
            for sensor_name in sensors.keys():
                if sensor_name not in valid_sensor_names:
                    self.get_logger().warn(f"Unknown sensor in message: {sensor_name}")

            return {"valid": True}

        except Exception as e:
            return {"valid": False, "error": f"Validation error: {str(e)}"}

    def _process_sensor_data(
        self, sensor_name: str, sensor_data: Any, timestamp: float
    ) -> bool:
        """Process individual sensor data with validation and error handling."""
        try:
            # Check if sensor is enabled and configured
            if sensor_name not in self.sensor_configs:
                self.get_logger().warn(f"Unknown sensor: {sensor_name}")
                return False

            config = self.sensor_configs[sensor_name]
            if not config.enabled:
                return True  # Not an error, just skipped

            if sensor_name not in self.publishers:
                self.get_logger().warn(f"No publisher for sensor: {sensor_name}")
                return False

            # Validate sensor data
            validation_result = self._validate_sensor_data(sensor_name, sensor_data)
            if not validation_result["valid"]:
                self.get_logger().error(
                    f'Invalid data for sensor {sensor_name}: {validation_result["error"]}'
                )
                return False

            # Convert to ROS2 message
            ros_message = config.message_converter(sensor_data, timestamp)
            if ros_message is None:
                self.get_logger().error(
                    f"Failed to convert data for sensor {sensor_name}"
                )
                return False

            # Publish message
            try:
                self.publishers[sensor_name].publish(ros_message)
                return True
            except Exception as e:
                self.get_logger().error(f"Failed to publish {sensor_name}: {e}")
                return False

        except Exception as e:
            self.get_logger().error(f"Unexpected error processing {sensor_name}: {e}")
            return False

    def _validate_sensor_data(
        self, sensor_name: str, sensor_data: Any
    ) -> Dict[str, Any]:
        """Validate sensor-specific data with range and type checking."""
        try:
            if not isinstance(sensor_data, dict):
                return {
                    "valid": False,
                    "error": f"Sensor data must be an object, got {type(sensor_data)}",
                }

            if sensor_name == "imu":
                return self._validate_imu_data(sensor_data)
            elif sensor_name == "gps":
                return self._validate_gps_data(sensor_data)
            elif sensor_name == "battery":
                return self._validate_battery_data(sensor_data)
            elif sensor_name == "wheel_odom":
                return self._validate_wheel_odom_data(sensor_data)
            elif sensor_name == "temperature":
                return self._validate_temperature_data(sensor_data)
            else:
                return {"valid": False, "error": f"Unknown sensor type: {sensor_name}"}

        except Exception as e:
            return {"valid": False, "error": f"Validation error: {str(e)}"}

    def _validate_imu_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate IMU sensor data."""
        required_fields = ["accel", "gyro"]
        for field in required_fields:
            if field not in data:
                return {"valid": False, "error": f"Missing IMU field: {field}"}

        # Validate accelerometer data
        accel = data["accel"]
        if not isinstance(accel, dict) or not all(k in accel for k in ["x", "y", "z"]):
            return {"valid": False, "error": "Invalid accelerometer data structure"}

        for axis in ["x", "y", "z"]:
            if not isinstance(accel[axis], (int, float)):
                return {
                    "valid": False,
                    "error": f"Accelerometer {axis} must be numeric",
                }
            if abs(accel[axis]) > 100:  # Reasonable range check
                self.get_logger().warn(
                    f"Unusual accelerometer value: {axis}={accel[axis]}"
                )

        # Validate gyroscope data
        gyro = data["gyro"]
        if not isinstance(gyro, dict) or not all(k in gyro for k in ["x", "y", "z"]):
            return {"valid": False, "error": "Invalid gyroscope data structure"}

        for axis in ["x", "y", "z"]:
            if not isinstance(gyro[axis], (int, float)):
                return {"valid": False, "error": f"Gyroscope {axis} must be numeric"}

        return {"valid": True}

    def _validate_gps_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate GPS sensor data."""
        required_fields = ["lat", "lon", "altitude"]
        for field in required_fields:
            if field not in data:
                return {"valid": False, "error": f"Missing GPS field: {field}"}

        # Validate latitude (-90 to 90)
        lat = data["lat"]
        if not isinstance(lat, (int, float)) or not -90 <= lat <= 90:
            return {"valid": False, "error": f"Invalid latitude: {lat}"}

        # Validate longitude (-180 to 180)
        lon = data["lon"]
        if not isinstance(lon, (int, float)) or not -180 <= lon <= 180:
            return {"valid": False, "error": f"Invalid longitude: {lon}"}

        # Validate altitude (reasonable range)
        alt = data["altitude"]
        if not isinstance(alt, (int, float)) or alt < -1000 or alt > 10000:
            return {"valid": False, "error": f"Invalid altitude: {alt}"}

        return {"valid": True}

    def _validate_battery_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate battery sensor data."""
        required_fields = ["voltage", "percentage"]
        for field in required_fields:
            if field not in data:
                return {"valid": False, "error": f"Missing battery field: {field}"}

        # Validate voltage (reasonable range for robotics)
        voltage = data["voltage"]
        if not isinstance(voltage, (int, float)) or not 0 <= voltage <= 50:
            return {"valid": False, "error": f"Invalid voltage: {voltage}V"}

        # Validate percentage (0-100)
        percentage = data["percentage"]
        if not isinstance(percentage, (int, float)) or not 0 <= percentage <= 100:
            return {"valid": False, "error": f"Invalid percentage: {percentage}%"}

        # Validate current if present
        if "current" in data:
            current = data["current"]
            if not isinstance(current, (int, float)) or abs(current) > 100:
                return {"valid": False, "error": f"Invalid current: {current}A"}

        return {"valid": True}

    def _validate_wheel_odom_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate wheel odometry sensor data."""
        required_fields = ["x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz"]
        for field in required_fields:
            if field not in data:
                return {"valid": False, "error": f"Missing odometry field: {field}"}

        # Position validation
        for axis in ["x", "y", "z"]:
            if not isinstance(data[axis], (int, float)):
                return {"valid": False, "error": f"Position {axis} must be numeric"}

        # Quaternion validation (basic)
        for component in ["qx", "qy", "qz", "qw"]:
            if not isinstance(data[component], (int, float)):
                return {
                    "valid": False,
                    "error": f"Quaternion {component} must be numeric",
                }

        # Velocity validation
        for vel in ["vx", "vy", "vz"]:
            if not isinstance(data[vel], (int, float)):
                return {"valid": False, "error": f"Velocity {vel} must be numeric"}

        return {"valid": True}

    def _validate_temperature_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate temperature sensor data."""
        if "temperature" not in data:
            return {"valid": False, "error": "Missing temperature field"}

        temp = data["temperature"]
        if not isinstance(temp, (int, float)):
            return {
                "valid": False,
                "error": f"Temperature must be numeric, got {type(temp)}",
            }

        # Reasonable temperature range for robotics (-50°C to 150°C)
        if not -50 <= temp <= 150:
            return {"valid": False, "error": f"Temperature out of range: {temp}°C"}

        return {"valid": True}

    def _check_connection_health(self):
        """Monitor connection health and handle graceful degradation."""
        try:
            current_time = time.time()
            self.metrics.uptime_seconds = current_time - self.start_time

            # Check connection state
            if self.connection_state == ConnectionState.CONNECTED:
                # Check if we've received messages recently
                time_since_last_message = current_time - self.last_message_time

                if time_since_last_message > self.message_timeout:
                    self.get_logger().warn(
                        f"No messages received for {time_since_last_message:.1f}s "
                        f"(timeout: {self.message_timeout}s)"
                    )

                    if self.enable_graceful_degradation:
                        self._enter_graceful_degradation_mode()
                    else:
                        # Force reconnection
                        self.connection_state = ConnectionState.DISCONNECTED
                        self._schedule_reconnection()
                else:
                    # Connection is healthy, reset any degradation
                    self._exit_graceful_degradation_mode()

            # Publish health diagnostics
            self._publish_diagnostics()

        except Exception as e:
            self.get_logger().error(f"Error in health check: {e}")

    def _enter_graceful_degradation_mode(self):
        """Enter graceful degradation mode when connection issues detected."""
        try:
            self.get_logger().info("Entering graceful degradation mode")

            # Could implement fallback behaviors here:
            # - Use last known good values
            # - Reduce publishing frequency
            # - Publish diagnostic warnings
            # - Maintain minimal system operation

            # For now, just log the state
            status_msg = String()
            status_msg.data = json.dumps(
                {
                    "state": "graceful_degradation",
                    "reason": "message_timeout",
                    "timestamp": time.time(),
                    "last_message_age": time.time() - self.last_message_time,
                }
            )
            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error entering graceful degradation: {e}")

    def _exit_graceful_degradation_mode(self):
        """Exit graceful degradation mode when connection recovers."""
        try:
            # Reset to normal operation
            status_msg = String()
            status_msg.data = json.dumps(
                {"state": "normal_operation", "timestamp": time.time()}
            )
            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error exiting graceful degradation: {e}")

    def _publish_connection_status(self):
        """Publish current connection status."""
        try:
            status_msg = String()
            status_msg.data = json.dumps(
                {
                    "connection_state": self.connection_state.value,
                    "reconnect_attempts": self.reconnect_attempts,
                    "last_message_time": self.last_message_time,
                    "uptime_seconds": time.time() - self.start_time,
                    "timestamp": time.time(),
                }
            )
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing connection status: {e}")

    def _publish_metrics(self):
        """Publish performance metrics."""
        try:
            metrics_msg = String()
            metrics_msg.data = json.dumps(
                {
                    "timestamp": time.time(),
                    "uptime_seconds": self.metrics.uptime_seconds,
                    "messages_received": self.metrics.messages_received,
                    "messages_processed": self.metrics.messages_processed,
                    "messages_failed": self.metrics.messages_failed,
                    "connection_attempts": self.metrics.connection_attempts,
                    "connection_failures": self.metrics.connection_failures,
                    "avg_processing_time": self.metrics.avg_processing_time,
                    "last_message_timestamp": self.metrics.last_message_timestamp,
                    "connection_state": self.connection_state.value,
                }
            )
            self.metrics_publisher.publish(metrics_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing metrics: {e}")

    def _publish_diagnostics(self):
        """Publish diagnostic information for monitoring systems."""
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()

            # Connection diagnostic
            conn_status = DiagnosticStatus()
            conn_status.name = "WebSocket Sensor Bridge/Connection"
            conn_status.hardware_id = "websocket_bridge"

            if self.connection_state == ConnectionState.CONNECTED:
                conn_status.level = DiagnosticStatus.OK
                conn_status.message = "Connected"
            elif self.connection_state == ConnectionState.CONNECTING:
                conn_status.level = DiagnosticStatus.WARN
                conn_status.message = "Connecting"
            elif self.connection_state == ConnectionState.RECONNECTING:
                conn_status.level = DiagnosticStatus.WARN
                conn_status.message = "Reconnecting"
            else:
                conn_status.level = DiagnosticStatus.ERROR
                conn_status.message = "Disconnected"

            # Add diagnostic values
            conn_status.values.extend(
                [
                    KeyValue(key="connection_state", value=self.connection_state.value),
                    KeyValue(
                        key="reconnect_attempts", value=str(self.reconnect_attempts)
                    ),
                    KeyValue(
                        key="messages_received",
                        value=str(self.metrics.messages_received),
                    ),
                    KeyValue(
                        key="messages_failed", value=str(self.metrics.messages_failed)
                    ),
                    KeyValue(
                        key="uptime_seconds", value=f"{self.metrics.uptime_seconds:.1f}"
                    ),
                    KeyValue(
                        key="avg_processing_time",
                        value=f"{self.metrics.avg_processing_time:.3f}",
                    ),
                ]
            )

            # Performance diagnostic
            perf_status = DiagnosticStatus()
            perf_status.name = "WebSocket Sensor Bridge/Performance"
            perf_status.hardware_id = "websocket_bridge"

            # Calculate performance metrics
            success_rate = 0.0
            if self.metrics.messages_received > 0:
                success_rate = (
                    self.metrics.messages_processed / self.metrics.messages_received
                ) * 100

            if success_rate > 95:
                perf_status.level = DiagnosticStatus.OK
                perf_status.message = "Performance OK"
            elif success_rate > 80:
                perf_status.level = DiagnosticStatus.WARN
                perf_status.message = "Performance Degraded"
            else:
                perf_status.level = DiagnosticStatus.ERROR
                perf_status.message = "Performance Critical"

            perf_status.values.extend(
                [
                    KeyValue(key="success_rate_percent", value=f"{success_rate:.1f}"),
                    KeyValue(
                        key="processing_time_avg_ms",
                        value=f"{self.metrics.avg_processing_time * 1000:.1f}",
                    ),
                    KeyValue(
                        key="messages_per_second",
                        value=f"{self.metrics.messages_processed / max(self.metrics.uptime_seconds, 1):.1f}",
                    ),
                ]
            )

            diag_array.status = [conn_status, perf_status]
            self.diagnostics_publisher.publish(diag_array)

        except Exception as e:
            self.get_logger().error(f"Error publishing diagnostics: {e}")

    def destroy_node(self):
        """Clean shutdown with comprehensive resource cleanup."""
        try:
            self.get_logger().info("Initiating WebSocket Sensor Bridge shutdown...")

            # Cancel all timers
            timers_to_cancel = [
                self.health_timer,
                self.metrics_timer,
                self.connection_timer,
                self.reconnect_timer,
            ]

            for timer in timers_to_cancel:
                if timer:
                    try:
                        timer.cancel()
                        timer.destroy()
                    except Exception as e:
                        self.get_logger().warn(f"Error canceling timer: {e}")

            # Close WebSocket connection if it exists
            if hasattr(self, "websocket_connection") and self.websocket_connection:
                try:
                    # In real implementation, close WebSocket connection
                    self.websocket_connection = None
                except Exception as e:
                    self.get_logger().warn(f"Error closing WebSocket connection: {e}")

            # Clear publishers and subscribers
            try:
                self.publishers.clear()
                self.get_logger().debug("Cleared publishers")
            except Exception as e:
                self.get_logger().warn(f"Error clearing publishers: {e}")

            # Reset connection state
            self.connection_state = ConnectionState.DISCONNECTED

            # Final status update
            try:
                final_status = String()
                final_status.data = json.dumps(
                    {
                        "state": "shutdown",
                        "timestamp": time.time(),
                        "uptime_seconds": time.time() - self.start_time,
                        "final_metrics": {
                            "messages_processed": self.metrics.messages_processed,
                            "messages_failed": self.metrics.messages_failed,
                            "connection_attempts": self.metrics.connection_attempts,
                        },
                    }
                )
                # Note: Can't publish here as publishers are destroyed
            except Exception as e:
                self.get_logger().warn(f"Error creating final status: {e}")

            self.get_logger().info("WebSocket Sensor Bridge shutdown complete")

        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
        finally:
            # Always call parent destroy_node
            super().destroy_node()

    # ===== SENSOR DATA CONVERTERS =====
    # These convert JSON sensor data to ROS2 messages
    # When migrating to direct ROS2, keep these methods but change input source

    def _convert_imu_data(
        self, data: Dict[str, Any], timestamp: float
    ) -> Optional[Imu]:
        """Convert IMU JSON data to ROS2 Imu message with comprehensive error handling."""
        try:
            # Validate input data structure
            if not isinstance(data, dict):
                self.get_logger().error("IMU data must be a dictionary")
                return None

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            # Linear acceleration (m/s²) - with validation
            if "accel" in data and isinstance(data["accel"], dict):
                accel = data["accel"]
                try:
                    msg.linear_acceleration.x = float(accel.get("x", 0.0))
                    msg.linear_acceleration.y = float(accel.get("y", 0.0))
                    msg.linear_acceleration.z = float(accel.get("z", 0.0))

                    # Sanity check values (reasonable for robotics)
                    for axis, value in [
                        ("x", msg.linear_acceleration.x),
                        ("y", msg.linear_acceleration.y),
                        ("z", msg.linear_acceleration.z),
                    ]:
                        if abs(value) > 100:  # More than 100 m/s² is suspicious
                            self.get_logger().warn(
                                f"Unusual IMU acceleration {axis}: {value} m/s²"
                            )

                except (ValueError, TypeError) as e:
                    self.get_logger().error(f"Invalid accelerometer data: {e}")
                    return None
            else:
                self.get_logger().warn("No valid accelerometer data in IMU message")

            # Angular velocity (rad/s) - with validation
            if "gyro" in data and isinstance(data["gyro"], dict):
                gyro = data["gyro"]
                try:
                    msg.angular_velocity.x = float(gyro.get("x", 0.0))
                    msg.angular_velocity.y = float(gyro.get("y", 0.0))
                    msg.angular_velocity.z = float(gyro.get("z", 0.0))

                    # Sanity check values
                    for axis, value in [
                        ("x", msg.angular_velocity.x),
                        ("y", msg.angular_velocity.y),
                        ("z", msg.angular_velocity.z),
                    ]:
                        if abs(value) > 50:  # More than 50 rad/s is suspicious
                            self.get_logger().warn(
                                f"Unusual IMU angular velocity {axis}: {value} rad/s"
                            )

                except (ValueError, TypeError) as e:
                    self.get_logger().error(f"Invalid gyroscope data: {e}")
                    return None
            else:
                self.get_logger().warn("No valid gyroscope data in IMU message")

            # Orientation (quaternion) - optional
            if "orientation" in data and isinstance(data["orientation"], dict):
                orientation = data["orientation"]
                try:
                    msg.orientation.x = float(orientation.get("x", 0.0))
                    msg.orientation.y = float(orientation.get("y", 0.0))
                    msg.orientation.z = float(orientation.get("z", 0.0))
                    msg.orientation.w = float(orientation.get("w", 1.0))

                    # Basic quaternion normalization check
                    norm = (
                        msg.orientation.x**2
                        + msg.orientation.y**2
                        + msg.orientation.z**2
                        + msg.orientation.w**2
                    ) ** 0.5
                    if abs(norm - 1.0) > 0.1:  # Allow some tolerance
                        self.get_logger().warn(
                            f"Quaternion not normalized: norm={norm:.3f}"
                        )

                except (ValueError, TypeError) as e:
                    self.get_logger().error(f"Invalid orientation data: {e}")
                    # Don't return None - orientation is optional

            # Covariances - with safe defaults
            try:
                accel_cov = data.get("accel_covariance", [0.01, 0.01, 0.01])
                gyro_cov = data.get("gyro_covariance", [0.01, 0.01, 0.01])
                orientation_cov = data.get(
                    "orientation_covariance", [0.01, 0.01, 0.01, 0.01]
                )

                # Validate covariance dimensions
                if (
                    len(accel_cov) == 3
                    and len(gyro_cov) == 3
                    and len(orientation_cov) == 4
                ):
                    msg.linear_acceleration_covariance = (
                        accel_cov + [0.0] * 6 + [0.0] * 3
                    )
                    msg.angular_velocity_covariance = (
                        [0.0] * 3 + gyro_cov + [0.0] * 3 + [0.0] * 3
                    )
                    msg.orientation_covariance = orientation_cov + [0.0] * 5 + [0.0] * 6
                else:
                    self.get_logger().warn(
                        "Invalid covariance dimensions, using defaults"
                    )
                    # Use default covariances
                    msg.linear_acceleration_covariance = [
                        0.01,
                        0.0,
                        0.0,
                        0.0,
                        0.01,
                        0.0,
                        0.0,
                        0.0,
                        0.01,
                    ]
                    msg.angular_velocity_covariance = [
                        0.01,
                        0.0,
                        0.0,
                        0.0,
                        0.01,
                        0.0,
                        0.0,
                        0.0,
                        0.01,
                    ]
                    msg.orientation_covariance = [
                        0.01,
                        0.0,
                        0.0,
                        0.0,
                        0.01,
                        0.0,
                        0.0,
                        0.0,
                        0.01,
                    ]

            except Exception as e:
                self.get_logger().warn(
                    f"Error setting covariances, using defaults: {e}"
                )
                # Use safe defaults
                msg.linear_acceleration_covariance = [0.01] * 9
                msg.angular_velocity_covariance = [0.01] * 9
                msg.orientation_covariance = [0.01] * 9

            return msg

        except Exception as e:
            self.get_logger().error(f"Unexpected error converting IMU data: {e}")
            return None

    def _convert_gps_data(
        self, data: Dict[str, Any], timestamp: float
    ) -> Optional[NavSatFix]:
        """Convert GPS JSON data to ROS2 NavSatFix message with comprehensive error handling."""
        try:
            # Validate input data structure
            if not isinstance(data, dict):
                self.get_logger().error("GPS data must be a dictionary")
                return None

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"

            # Latitude validation (-90 to 90 degrees)
            try:
                latitude = float(data.get("lat", 0.0))
                if not -90 <= latitude <= 90:
                    self.get_logger().error(
                        f"Invalid GPS latitude: {latitude} (must be -90 to 90)"
                    )
                    return None
                msg.latitude = latitude
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Invalid GPS latitude value: {e}")
                return None

            # Longitude validation (-180 to 180 degrees)
            try:
                longitude = float(data.get("lon", 0.0))
                if not -180 <= longitude <= 180:
                    self.get_logger().error(
                        f"Invalid GPS longitude: {longitude} (must be -180 to 180)"
                    )
                    return None
                msg.longitude = longitude
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Invalid GPS longitude value: {e}")
                return None

            # Altitude validation (reasonable range)
            try:
                altitude = float(data.get("altitude", 0.0))
                if altitude < -1000 or altitude > 10000:  # -1km to 10km range
                    self.get_logger().warn(f"Unusual GPS altitude: {altitude}m")
                    # Don't fail on altitude, just warn
                msg.altitude = altitude
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Invalid GPS altitude value: {e}")
                return None

            # GPS status validation
            try:
                status = int(data.get("status", 0))
                if not 0 <= status <= 2:  # Valid NavSatStatus values
                    self.get_logger().warn(
                        f"Invalid GPS status: {status}, using STATUS_NO_FIX"
                    )
                    status = -1  # STATUS_NO_FIX
                msg.status.status = status

                service = int(data.get("service", 1))
                if service < 0:
                    self.get_logger().warn(
                        f"Invalid GPS service: {service}, using SERVICE_NONE"
                    )
                    service = 0  # SERVICE_NONE
                msg.status.service = service

            except (ValueError, TypeError) as e:
                self.get_logger().warn(
                    f"Invalid GPS status/service values: {e}, using defaults"
                )
                msg.status.status = -1  # STATUS_NO_FIX
                msg.status.service = 0  # SERVICE_NONE

            # Position covariance validation
            try:
                covariance = data.get(
                    "position_covariance", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
                )
                if isinstance(covariance, list) and len(covariance) == 9:
                    # Validate that diagonal elements are positive
                    if covariance[0] <= 0 or covariance[4] <= 0 or covariance[8] <= 0:
                        self.get_logger().warn(
                            "GPS covariance diagonal elements must be positive, using defaults"
                        )
                        covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
                    msg.position_covariance = covariance
                    msg.position_covariance_type = (
                        NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    )
                else:
                    self.get_logger().warn(
                        "Invalid GPS covariance format, using defaults"
                    )
                    msg.position_covariance = [
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                        2.0,
                    ]
                    msg.position_covariance_type = (
                        NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    )

            except Exception as e:
                self.get_logger().warn(
                    f"Error setting GPS covariance: {e}, using defaults"
                )
                msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            return msg

        except Exception as e:
            self.get_logger().error(f"Unexpected error converting GPS data: {e}")
            return None

    def _convert_battery_data(
        self, data: Dict[str, Any], timestamp: float
    ) -> Optional[BatteryState]:
        """Convert battery JSON data to ROS2 BatteryState message with comprehensive error handling."""
        try:
            # Validate input data structure
            if not isinstance(data, dict):
                self.get_logger().error("Battery data must be a dictionary")
                return None

            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "battery"

            # Voltage validation (reasonable range for robotics: 0-50V)
            try:
                voltage = float(data.get("voltage", 0.0))
                if voltage < 0 or voltage > 50:
                    self.get_logger().error(
                        f"Invalid battery voltage: {voltage}V (must be 0-50V)"
                    )
                    return None
                msg.voltage = voltage
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Invalid battery voltage value: {e}")
                return None

            # Percentage validation (0-100%)
            try:
                percentage = float(data.get("percentage", 0.0))
                if not 0 <= percentage <= 100:
                    self.get_logger().error(
                        f"Invalid battery percentage: {percentage}% (must be 0-100%)"
                    )
                    return None
                msg.percentage = percentage
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Invalid battery percentage value: {e}")
                return None

            # Current validation (reasonable range: -100A to 100A)
            try:
                current = float(data.get("current", float("nan")))
                if (
                    not (current != current) and abs(current) > 100
                ):  # Check if not NaN and within range
                    self.get_logger().warn(f"Unusual battery current: {current}A")
                    # Don't fail, just warn
                msg.current = current
            except (ValueError, TypeError) as e:
                self.get_logger().warn(
                    f"Invalid battery current value: {e}, setting to NaN"
                )
                msg.current = float("nan")

            # Charge validation (0 to capacity)
            try:
                charge = float(data.get("charge", float("nan")))
                if not (charge != charge):  # If not NaN
                    if charge < 0:
                        self.get_logger().warn(
                            f"Negative battery charge: {charge}Ah, setting to 0"
                        )
                        charge = 0.0
                    # Could validate against capacity here if available
                msg.charge = charge
            except (ValueError, TypeError) as e:
                self.get_logger().warn(
                    f"Invalid battery charge value: {e}, setting to NaN"
                )
                msg.charge = float("nan")

            # Capacity validation
            try:
                capacity = float(data.get("capacity", float("nan")))
                if (
                    not (capacity != capacity) and capacity <= 0
                ):  # If not NaN and negative/zero
                    self.get_logger().warn(
                        f"Invalid battery capacity: {capacity}Ah, setting to NaN"
                    )
                    capacity = float("nan")
                msg.capacity = capacity

                design_capacity = float(data.get("design_capacity", float("nan")))
                if not (design_capacity != design_capacity) and design_capacity <= 0:
                    self.get_logger().warn(
                        f"Invalid battery design capacity: {design_capacity}Ah, setting to NaN"
                    )
                    design_capacity = float("nan")
                msg.design_capacity = design_capacity

            except (ValueError, TypeError) as e:
                self.get_logger().warn(
                    f"Invalid battery capacity values: {e}, setting to NaN"
                )
                msg.capacity = float("nan")
                msg.design_capacity = float("nan")

            # Power supply status validation
            try:
                status = int(
                    data.get("status", BatteryState.POWER_SUPPLY_STATUS_UNKNOWN)
                )
                if not 0 <= status <= 4:  # Valid POWER_SUPPLY_STATUS_* constants
                    self.get_logger().warn(
                        f"Invalid power supply status: {status}, using UNKNOWN"
                    )
                    status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                msg.power_supply_status = status

                health = int(
                    data.get("health", BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN)
                )
                if not 0 <= health <= 8:  # Valid POWER_SUPPLY_HEALTH_* constants
                    self.get_logger().warn(
                        f"Invalid power supply health: {health}, using UNKNOWN"
                    )
                    health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
                msg.power_supply_health = health

                technology = int(
                    data.get("technology", BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
                )
                if (
                    not 0 <= technology <= 5
                ):  # Valid POWER_SUPPLY_TECHNOLOGY_* constants
                    self.get_logger().warn(
                        f"Invalid power supply technology: {technology}, using UNKNOWN"
                    )
                    technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
                msg.power_supply_technology = technology

            except (ValueError, TypeError) as e:
                self.get_logger().warn(
                    f"Invalid power supply status values: {e}, using UNKNOWN"
                )
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
                msg.power_supply_technology = (
                    BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
                )

            # Cell data validation
            try:
                cell_voltage = data.get("cell_voltages", [])
                if isinstance(cell_voltage, list):
                    # Validate each cell voltage
                    validated_voltages = []
                    for i, voltage in enumerate(cell_voltage):
                        try:
                            v = float(voltage)
                            if v < 0 or v > 10:  # Reasonable cell voltage range
                                self.get_logger().warn(
                                    f"Invalid cell {i} voltage: {v}V"
                                )
                                continue
                            validated_voltages.append(v)
                        except (ValueError, TypeError):
                            self.get_logger().warn(f"Invalid cell {i} voltage value")
                            continue
                    msg.cell_voltage = validated_voltages
                else:
                    self.get_logger().warn("Cell voltages must be an array")
                    msg.cell_voltage = []

                cell_temperature = data.get("cell_temperatures", [])
                if isinstance(cell_temperature, list):
                    validated_temps = []
                    for i, temp in enumerate(cell_temperature):
                        try:
                            t = float(temp)
                            if t < -50 or t > 150:  # Reasonable temperature range
                                self.get_logger().warn(
                                    f"Invalid cell {i} temperature: {t}°C"
                                )
                                continue
                            validated_temps.append(t)
                        except (ValueError, TypeError):
                            self.get_logger().warn(
                                f"Invalid cell {i} temperature value"
                            )
                            continue
                    msg.cell_temperature = validated_temps
                else:
                    self.get_logger().warn("Cell temperatures must be an array")
                    msg.cell_temperature = []

                # Overall temperature
                try:
                    temperature = float(data.get("temperature", float("nan")))
                    if not (temperature != temperature) and (
                        temperature < -50 or temperature > 150
                    ):
                        self.get_logger().warn(
                            f"Invalid battery temperature: {temperature}°C, setting to NaN"
                        )
                        temperature = float("nan")
                    msg.temperature = temperature
                except (ValueError, TypeError) as e:
                    self.get_logger().warn(
                        f"Invalid battery temperature: {e}, setting to NaN"
                    )
                    msg.temperature = float("nan")

            except Exception as e:
                self.get_logger().warn(
                    f"Error processing cell data: {e}, using empty arrays"
                )
                msg.cell_voltage = []
                msg.cell_temperature = []
                msg.temperature = float("nan")

            return msg

        except Exception as e:
            self.get_logger().error(f"Unexpected error converting battery data: {e}")
            return None

    def _convert_wheel_odom(
        self, data: Dict[str, Any], timestamp: float
    ) -> Optional[Odometry]:
        """Convert wheel odometry JSON data to ROS2 Odometry message with comprehensive error handling."""
        try:
            # Validate input data structure
            if not isinstance(data, dict):
                self.get_logger().error("Wheel odometry data must be a dictionary")
                return None

            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"

            # Position validation
            try:
                msg.pose.pose.position.x = float(data.get("x", 0.0))
                msg.pose.pose.position.y = float(data.get("y", 0.0))
                msg.pose.pose.position.z = float(data.get("z", 0.0))
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Invalid odometry position values: {e}")
                return None

            # Orientation quaternion validation
            try:
                qx = float(data.get("qx", 0.0))
                qy = float(data.get("qy", 0.0))
                qz = float(data.get("qz", 0.0))
                qw = float(data.get("qw", 1.0))

                # Basic quaternion normalization check
                norm = (qx**2 + qy**2 + qz**2 + qw**2) ** 0.5
                if abs(norm - 1.0) > 0.1:  # Allow some tolerance
                    self.get_logger().warn(
                        f"Odometry quaternion not normalized: norm={norm:.3f}"
                    )
                    # Don't fail, just warn

                msg.pose.pose.orientation.x = qx
                msg.pose.pose.orientation.y = qy
                msg.pose.pose.orientation.z = qz
                msg.pose.pose.orientation.w = qw

            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Invalid odometry orientation values: {e}")
                return None

            # Velocity validation
            try:
                msg.twist.twist.linear.x = float(data.get("vx", 0.0))
                msg.twist.twist.linear.y = float(data.get("vy", 0.0))
                msg.twist.twist.linear.z = float(data.get("vz", 0.0))
                msg.twist.twist.angular.x = float(data.get("wx", 0.0))
                msg.twist.twist.angular.y = float(data.get("wy", 0.0))
                msg.twist.twist.angular.z = float(data.get("wz", 0.0))

                # Sanity checks for velocities
                if abs(msg.twist.twist.linear.x) > 10:  # More than 10 m/s is suspicious
                    self.get_logger().warn(
                        f"Unusual linear velocity: {msg.twist.twist.linear.x} m/s"
                    )
                if (
                    abs(msg.twist.twist.angular.z) > 5
                ):  # More than 5 rad/s is suspicious
                    self.get_logger().warn(
                        f"Unusual angular velocity: {msg.twist.twist.angular.z} rad/s"
                    )

            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Invalid odometry velocity values: {e}")
                return None

            # Covariance validation
            try:
                pose_covariance = data.get("pose_covariance", [0.1] * 36)
                twist_covariance = data.get("twist_covariance", [0.1] * 36)

                if isinstance(pose_covariance, list) and len(pose_covariance) == 36:
                    msg.pose.covariance = pose_covariance
                else:
                    self.get_logger().warn(
                        "Invalid pose covariance format, using defaults"
                    )
                    msg.pose.covariance = [0.1] * 36

                if isinstance(twist_covariance, list) and len(twist_covariance) == 36:
                    msg.twist.covariance = twist_covariance
                else:
                    self.get_logger().warn(
                        "Invalid twist covariance format, using defaults"
                    )
                    msg.twist.covariance = [0.1] * 36

            except Exception as e:
                self.get_logger().warn(
                    f"Error setting covariances: {e}, using defaults"
                )
                msg.pose.covariance = [0.1] * 36
                msg.twist.covariance = [0.1] * 36

            return msg

        except Exception as e:
            self.get_logger().error(
                f"Unexpected error converting wheel odometry data: {e}"
            )
            return None

    def _convert_temperature(
        self, data: Dict[str, Any], timestamp: float
    ) -> Optional[Temperature]:
        """Convert temperature JSON data to ROS2 Temperature message with comprehensive error handling."""
        try:
            # Validate input data structure
            if not isinstance(data, dict):
                self.get_logger().error("Temperature data must be a dictionary")
                return None

            # Check for required temperature field
            if "temperature" not in data:
                self.get_logger().error(
                    'Temperature data missing required "temperature" field'
                )
                return None

            msg = Temperature()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "temperature_sensor"

            # Temperature validation (reasonable range: -50°C to 150°C)
            try:
                temperature = float(data["temperature"])
                if temperature < -50 or temperature > 150:
                    self.get_logger().error(
                        f"Invalid temperature: {temperature}°C (must be -50°C to 150°C)"
                    )
                    return None
                msg.temperature = temperature
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"Invalid temperature value: {e}")
                return None

            # Variance validation (must be positive)
            try:
                variance = float(data.get("variance", 0.1))
                if variance <= 0:
                    self.get_logger().warn(
                        f"Invalid variance: {variance}, must be positive, using 0.1"
                    )
                    variance = 0.1
                msg.variance = variance
            except (ValueError, TypeError) as e:
                self.get_logger().warn(
                    f"Invalid variance value: {e}, using default 0.1"
                )
                msg.variance = 0.1

            return msg

        except Exception as e:
            self.get_logger().error(
                f"Unexpected error converting temperature data: {e}"
            )
            return None

    def destroy_node(self):
        """Clean shutdown with comprehensive resource cleanup."""
        try:
            self.get_logger().info("Initiating WebSocket Sensor Bridge shutdown...")

            # Cancel all timers
            timers_to_cancel = [
                self.health_timer,
                self.metrics_timer,
                self.connection_timer,
                self.reconnect_timer,
            ]

            for timer in timers_to_cancel:
                if timer:
                    try:
                        timer.cancel()
                        timer.destroy()
                    except Exception as e:
                        self.get_logger().warn(f"Error canceling timer: {e}")

            # Close WebSocket connection if it exists
            if hasattr(self, "websocket_connection") and self.websocket_connection:
                try:
                    # In real implementation, close WebSocket connection
                    self.websocket_connection = None
                except Exception as e:
                    self.get_logger().warn(f"Error closing WebSocket connection: {e}")

            # Clear publishers and subscribers
            try:
                self.publishers.clear()
                self.get_logger().debug("Cleared publishers")
            except Exception as e:
                self.get_logger().warn(f"Error clearing publishers: {e}")

            # Reset connection state
            self.connection_state = ConnectionState.DISCONNECTED

            # Final status update
            try:
                final_status = String()
                final_status.data = json.dumps(
                    {
                        "state": "shutdown",
                        "timestamp": time.time(),
                        "uptime_seconds": time.time() - self.start_time,
                        "final_metrics": {
                            "messages_processed": self.metrics.messages_processed,
                            "messages_failed": self.metrics.messages_failed,
                            "connection_attempts": self.metrics.connection_attempts,
                        },
                    }
                )
                # Note: Can't publish here as publishers are destroyed
            except Exception as e:
                self.get_logger().warn(f"Error creating final status: {e}")

            self.get_logger().info("WebSocket Sensor Bridge shutdown complete")

        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
        finally:
            # Always call parent destroy_node
            super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = WebSocketSensorBridgeNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)

        node.get_logger().info("WebSocket Sensor Bridge starting...")
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
