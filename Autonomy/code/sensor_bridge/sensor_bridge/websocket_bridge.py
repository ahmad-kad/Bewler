#!/usr/bin/env python3
"""
WebSocket Sensor Bridge Node - URC 2026 Mars Rover

Converts WebSocket sensor data streams to ROS2 topics.

ARCHITECTURE DESIGN:
- WebSocket client receives JSON sensor data
- Data is parsed and converted to standard ROS2 sensor messages
- Publishers use appropriate QoS settings for real-time sensor data
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
import asyncio
import websockets
import threading
import time
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass
import logging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

# ROS2 sensor messages
from sensor_msgs.msg import Imu, NavSatFix, BatteryState, Temperature, Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Float64, Bool, String

logger = logging.getLogger(__name__)


@dataclass
class SensorConfig:
    """Configuration for a sensor data stream."""
    enabled: bool = True
    topic_name: str = ""
    qos_profile: QoSProfile = None
    message_converter: Callable = None
    update_rate: float = 10.0  # Hz


class WebSocketSensorBridgeNode(Node):
    """
    WebSocket-to-ROS2 Sensor Bridge Node.

    Receives sensor data via WebSocket and publishes as ROS2 topics.
    Designed for easy migration to direct ROS2 transport.
    """

    def __init__(self):
        super().__init__('websocket_sensor_bridge')

        # Declare parameters
        self.declare_parameter('websocket_url', 'ws://localhost:8080')
        self.declare_parameter('reconnect_interval', 3.0)
        self.declare_parameter('max_reconnect_attempts', 10)
        self.declare_parameter('connection_timeout', 5.0)

        # Get parameters
        self.websocket_url = self.get_parameter('websocket_url').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').value
        self.connection_timeout = self.get_parameter('connection_timeout').value

        # Connection state
        self.websocket_connected = False
        self.reconnect_attempts = 0
        self.last_message_time = 0.0

        # Publishers dictionary (sensor_name -> publisher)
        self.publishers: Dict[str, Any] = {}

        # Sensor configurations
        self.sensor_configs = self._create_sensor_configs()

        # Setup publishers for enabled sensors
        self._setup_publishers()

        # WebSocket thread
        self.websocket_thread: Optional[threading.Thread] = None
        self.shutdown_event = threading.Event()

        # Start WebSocket connection
        self._start_websocket_client()

        # Health monitoring timer
        self.health_timer = self.create_timer(1.0, self._check_connection_health)

        self.get_logger().info(
            f'WebSocket Sensor Bridge initialized - URL: {self.websocket_url}'
        )

    def _create_sensor_configs(self) -> Dict[str, SensorConfig]:
        """Create sensor configurations with appropriate QoS settings."""

        # High-frequency sensor QoS (IMU, wheel odometry)
        qos_sensor_high_freq = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,  # Keep last 10 messages
            durability=DurabilityPolicy.VOLATILE
        )

        # Standard sensor QoS (GPS, battery)
        qos_sensor_standard = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # Critical sensor QoS (safety-related)
        qos_sensor_critical = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # Last value persists
        )

        return {
            'imu': SensorConfig(
                enabled=True,
                topic_name='/imu/data',
                qos_profile=qos_sensor_high_freq,
                message_converter=self._convert_imu_data,
                update_rate=100.0  # IMU typically 100Hz
            ),
            'gps': SensorConfig(
                enabled=True,
                topic_name='/gps/fix',
                qos_profile=qos_sensor_standard,
                message_converter=self._convert_gps_data,
                update_rate=10.0  # GPS typically 1-10Hz
            ),
            'battery': SensorConfig(
                enabled=True,
                topic_name='/battery/status',
                qos_profile=qos_sensor_critical,
                message_converter=self._convert_battery_data,
                update_rate=1.0  # Battery status 1Hz
            ),
            'wheel_odom': SensorConfig(
                enabled=True,
                topic_name='/wheel/odom',
                qos_profile=qos_sensor_high_freq,
                message_converter=self._convert_wheel_odom,
                update_rate=50.0  # Wheel odometry 50Hz
            ),
            'temperature': SensorConfig(
                enabled=True,
                topic_name='/temperature/data',
                qos_profile=qos_sensor_standard,
                message_converter=self._convert_temperature,
                update_rate=5.0  # Temperature 5Hz
            )
        }

    def _setup_publishers(self):
        """Setup ROS2 publishers for enabled sensors."""
        for sensor_name, config in self.sensor_configs.items():
            if not config.enabled:
                continue

            try:
                # Create publisher based on sensor type
                if sensor_name == 'imu':
                    publisher = self.create_publisher(Imu, config.topic_name, config.qos_profile)
                elif sensor_name == 'gps':
                    publisher = self.create_publisher(NavSatFix, config.topic_name, config.qos_profile)
                elif sensor_name == 'battery':
                    publisher = self.create_publisher(BatteryState, config.topic_name, config.qos_profile)
                elif sensor_name == 'wheel_odom':
                    publisher = self.create_publisher(Odometry, config.topic_name, config.qos_profile)
                elif sensor_name == 'temperature':
                    publisher = self.create_publisher(Temperature, config.topic_name, config.qos_profile)
                else:
                    self.get_logger().warn(f'Unknown sensor type: {sensor_name}')
                    continue

                self.publishers[sensor_name] = publisher
                self.get_logger().info(f'Setup publisher for {sensor_name} on topic {config.topic_name}')

            except Exception as e:
                self.get_logger().error(f'Failed to create publisher for {sensor_name}: {e}')

    def _start_websocket_client(self):
        """Start WebSocket client in separate thread."""
        if self.websocket_thread and self.websocket_thread.is_alive():
            return

        self.websocket_thread = threading.Thread(
            target=self._websocket_client_loop,
            daemon=True
        )
        self.websocket_thread.start()

    def _websocket_client_loop(self):
        """WebSocket client connection loop with reconnection."""
        while not self.shutdown_event.is_set():
            try:
                # Create event loop for this thread
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

                # Attempt connection
                loop.run_until_complete(self._connect_websocket())

            except Exception as e:
                self.get_logger().error(f'WebSocket connection failed: {e}')
                self.websocket_connected = False

                if self.reconnect_attempts < self.max_reconnect_attempts:
                    self.reconnect_attempts += 1
                    self.get_logger().info(
                        f'Retrying connection in {self.reconnect_interval}s '
                        f'({self.reconnect_attempts}/{self.max_reconnect_attempts})'
                    )
                    time.sleep(self.reconnect_interval)
                else:
                    self.get_logger().error('Max reconnection attempts reached')
                    break

    async def _connect_websocket(self):
        """Establish WebSocket connection and handle messages."""
        try:
            async with websockets.connect(
                self.websocket_url,
                extra_headers={'User-Agent': 'URC2026-SensorBridge/1.0'}
            ) as websocket:
                self.websocket_connected = True
                self.reconnect_attempts = 0
                self.get_logger().info(f'Connected to WebSocket: {self.websocket_url}')

                # Send connection acknowledgment
                await websocket.send(json.dumps({
                    'type': 'connection_ack',
                    'node': 'sensor_bridge',
                    'timestamp': time.time()
                }))

                async for message in websocket:
                    try:
                        self._handle_websocket_message(message)
                        self.last_message_time = time.time()
                    except Exception as e:
                        self.get_logger().error(f'Error handling message: {e}')

        except websockets.exceptions.ConnectionClosed:
            self.get_logger().warn('WebSocket connection closed')
        except Exception as e:
            self.get_logger().error(f'WebSocket error: {e}')
        finally:
            self.websocket_connected = False

    def _handle_websocket_message(self, message: str):
        """Process incoming WebSocket message."""
        try:
            data = json.loads(message)

            # Validate message format
            if not isinstance(data, dict) or 'timestamp' not in data or 'sensors' not in data:
                self.get_logger().warn(f'Invalid message format: {message[:100]}...')
                return

            timestamp = data['timestamp']
            sensors = data['sensors']

            # Process each sensor in the message
            for sensor_name, sensor_data in sensors.items():
                if sensor_name in self.publishers and sensor_name in self.sensor_configs:
                    config = self.sensor_configs[sensor_name]
                    try:
                        # Convert and publish
                        ros_message = config.message_converter(sensor_data, timestamp)
                        if ros_message:
                            self.publishers[sensor_name].publish(ros_message)
                    except Exception as e:
                        self.get_logger().error(f'Error publishing {sensor_name}: {e}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON message: {e}')

    def _check_connection_health(self):
        """Monitor connection health and trigger reconnection if needed."""
        current_time = time.time()

        # Check if we've received messages recently
        if self.websocket_connected and (current_time - self.last_message_time) > 10.0:
            self.get_logger().warn('No messages received for 10 seconds, connection may be stale')
            # Could trigger reconnection here if needed

        # Publish connection status
        status_msg = String()
        status_msg.data = json.dumps({
            'connected': self.websocket_connected,
            'last_message': self.last_message_time,
            'reconnect_attempts': self.reconnect_attempts,
            'active_sensors': list(self.publishers.keys())
        })

        # Publish status on a status topic (could be added to publishers dict)
        # self.status_publisher.publish(status_msg)

    # ===== SENSOR DATA CONVERTERS =====
    # These convert JSON sensor data to ROS2 messages
    # When migrating to direct ROS2, keep these methods but change input source

    def _convert_imu_data(self, data: Dict[str, Any], timestamp: float) -> Optional[Imu]:
        """Convert IMU JSON data to ROS2 Imu message."""
        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # Linear acceleration (m/s²)
            if 'accel' in data:
                msg.linear_acceleration.x = data['accel'].get('x', 0.0)
                msg.linear_acceleration.y = data['accel'].get('y', 0.0)
                msg.linear_acceleration.z = data['accel'].get('z', 0.0)

            # Angular velocity (rad/s)
            if 'gyro' in data:
                msg.angular_velocity.x = data['gyro'].get('x', 0.0)
                msg.angular_velocity.y = data['gyro'].get('y', 0.0)
                msg.angular_velocity.z = data['gyro'].get('z', 0.0)

            # Orientation (quaternion) - if available
            if 'orientation' in data:
                msg.orientation.x = data['orientation'].get('x', 0.0)
                msg.orientation.y = data['orientation'].get('y', 0.0)
                msg.orientation.z = data['orientation'].get('z', 0.0)
                msg.orientation.w = data['orientation'].get('w', 1.0)

            # Covariances (simplified diagonal)
            accel_cov = data.get('accel_covariance', [0.01, 0.01, 0.01])
            gyro_cov = data.get('gyro_covariance', [0.01, 0.01, 0.01])
            orientation_cov = data.get('orientation_covariance', [0.01, 0.01, 0.01, 0.01])

            msg.linear_acceleration_covariance = accel_cov + [0.0] * 6 + [0.0] * 3
            msg.angular_velocity_covariance = [0.0] * 3 + gyro_cov + [0.0] * 3 + [0.0] * 3
            msg.orientation_covariance = orientation_cov + [0.0] * 5 + [0.0] * 6

            return msg

        except Exception as e:
            self.get_logger().error(f'Error converting IMU data: {e}')
            return None

    def _convert_gps_data(self, data: Dict[str, Any], timestamp: float) -> Optional[NavSatFix]:
        """Convert GPS JSON data to ROS2 NavSatFix message."""
        try:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'

            msg.latitude = data.get('lat', 0.0)
            msg.longitude = data.get('lon', 0.0)
            msg.altitude = data.get('altitude', 0.0)

            # GPS status
            msg.status.status = data.get('status', 0)  # STATUS_FIX
            msg.status.service = data.get('service', 1)  # SERVICE_GPS

            # Position covariance
            covariance = data.get('position_covariance', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0])
            msg.position_covariance = covariance
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            return msg

        except Exception as e:
            self.get_logger().error(f'Error converting GPS data: {e}')
            return None

    def _convert_battery_data(self, data: Dict[str, Any], timestamp: float) -> Optional[BatteryState]:
        """Convert battery JSON data to ROS2 BatteryState message."""
        try:
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'battery'

            msg.voltage = data.get('voltage', 0.0)
            msg.current = data.get('current', 0.0)
            msg.charge = data.get('charge', float('nan'))
            msg.capacity = data.get('capacity', float('nan'))
            msg.design_capacity = data.get('design_capacity', float('nan'))
            msg.percentage = data.get('percentage', 0.0)

            # Power supply status
            msg.power_supply_status = data.get('status', BatteryState.POWER_SUPPLY_STATUS_UNKNOWN)
            msg.power_supply_health = data.get('health', BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN)
            msg.power_supply_technology = data.get('technology', BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN)

            # Cell voltages and temperatures
            msg.cell_voltage = data.get('cell_voltages', [])
            msg.cell_temperature = data.get('cell_temperatures', [])
            msg.temperature = data.get('temperature', float('nan'))

            return msg

        except Exception as e:
            self.get_logger().error(f'Error converting battery data: {e}')
            return None

    def _convert_wheel_odom(self, data: Dict[str, Any], timestamp: float) -> Optional[Odometry]:
        """Convert wheel odometry JSON data to ROS2 Odometry message."""
        try:
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'

            # Position
            msg.pose.pose.position.x = data.get('x', 0.0)
            msg.pose.pose.position.y = data.get('y', 0.0)
            msg.pose.pose.position.z = data.get('z', 0.0)

            # Orientation
            msg.pose.pose.orientation.x = data.get('qx', 0.0)
            msg.pose.pose.orientation.y = data.get('qy', 0.0)
            msg.pose.pose.orientation.z = data.get('qz', 0.0)
            msg.pose.pose.orientation.w = data.get('qw', 1.0)

            # Velocity
            msg.twist.twist.linear.x = data.get('vx', 0.0)
            msg.twist.twist.linear.y = data.get('vy', 0.0)
            msg.twist.twist.linear.z = data.get('vz', 0.0)
            msg.twist.twist.angular.x = data.get('wx', 0.0)
            msg.twist.twist.angular.y = data.get('wy', 0.0)
            msg.twist.twist.angular.z = data.get('wz', 0.0)

            # Covariances (simplified)
            msg.pose.covariance = data.get('pose_covariance', [0.1] * 36)
            msg.twist.covariance = data.get('twist_covariance', [0.1] * 36)

            return msg

        except Exception as e:
            self.get_logger().error(f'Error converting wheel odometry data: {e}')
            return None

    def _convert_temperature(self, data: Dict[str, Any], timestamp: float) -> Optional[Temperature]:
        """Convert temperature JSON data to ROS2 Temperature message."""
        try:
            msg = Temperature()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'temperature_sensor'

            msg.temperature = data.get('temperature', 0.0)
            msg.variance = data.get('variance', 0.1)

            return msg

        except Exception as e:
            self.get_logger().error(f'Error converting temperature data: {e}')
            return None

    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down WebSocket Sensor Bridge...')

        # Signal shutdown
        self.shutdown_event.set()

        # Close WebSocket connection
        if self.websocket_thread and self.websocket_thread.is_alive():
            self.websocket_thread.join(timeout=2.0)

        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = WebSocketSensorBridgeNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)

        node.get_logger().info('WebSocket Sensor Bridge starting...')
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
