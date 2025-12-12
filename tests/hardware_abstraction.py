#!/usr/bin/env python3
"""
Hardware Abstraction Layer - Unified interface for mock/real hardware

Provides consistent APIs for:
- Motor control (velocity commands, odometry)
- Sensor data (GPS, IMU, cameras, encoders)
- Emergency systems (estop, recovery)
- Power management (battery monitoring)

Supports both mock implementations (for development) and real hardware drivers.

Author: URC 2026 Autonomy Team
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from autonomy_interfaces.msg import VisionDetection

# ROS2 messages
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, CameraInfo, Image, Imu, NavSatFix, Temperature
from std_msgs.msg import Bool, Float32


class SensorInterface(ABC):
    """Abstract interface for sensor data access"""

    @abstractmethod
    def get_current_position(self) -> Optional[Tuple[float, float, float]]:
        """Get current position (x, y, heading)"""
        pass

    @abstractmethod
    def get_gps_position(self) -> Optional[NavSatFix]:
        """Get current GPS position"""
        pass

    @abstractmethod
    def get_imu_data(self) -> Optional[Imu]:
        """Get current IMU data"""
        pass

    @abstractmethod
    def get_odometry(self) -> Optional[Odometry]:
        """Get current odometry data"""
        pass

    @abstractmethod
    def get_vision_detections(self) -> List[VisionDetection]:
        """Get recent vision detections"""
        pass

    @abstractmethod
    def get_aruco_tag_pose(self, tag_id: int) -> Optional[PoseStamped]:
        """Get pose of specific ArUco tag"""
        pass

    @abstractmethod
    def get_battery_status(self) -> Optional[BatteryState]:
        """Get battery status"""
        pass

    @abstractmethod
    def get_temperature_data(self) -> Optional[Temperature]:
        """Get temperature sensor data"""
        pass


class ActuatorInterface(ABC):
    """Abstract interface for actuator control"""

    @abstractmethod
    def send_velocity_command(self, linear_x: float, angular_z: float):
        """Send velocity command to motors"""
        pass

    @abstractmethod
    def emergency_stop(self):
        """Immediate emergency stop"""
        pass

    @abstractmethod
    def navigate_to_position(self, position: Dict[str, float]) -> bool:
        """Navigate to a specific position"""
        pass

    @abstractmethod
    def set_led_status(self, status: str):
        """Set LED status indicator"""
        pass


class HardwareInterface:
    """Combined sensor and actuator interface"""

    def __init__(self, node: Node, use_mock: bool = True):
        self.node = node
        self.use_mock = use_mock

        if use_mock:
            self.sensors = MockSensorInterface(node)
            self.actuators = MockActuatorInterface(node)
        else:
            self.sensors = RealSensorInterface(node)
            self.actuators = RealActuatorInterface(node)

        self.node.get_logger().info(f'Hardware interface initialized (mock={use_mock})')


class MockSensorInterface(SensorInterface):
    """Mock sensor interface for development and testing"""

    def __init__(self, node: Node):
        self.node = node
        self.current_position = (0.0, 0.0, 0.0)  # x, y, heading
        self.vision_detections = []
        self.last_gps_update = 0.0

        # Subscribe to simulated sensor data
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.odom_sub = node.create_subscription(
            Odometry, '/odom', self._odom_callback, qos_sensor)
        self.gps_sub = node.create_subscription(
            NavSatFix, '/gps/fix', self._gps_callback, qos_sensor)
        self.imu_sub = node.create_subscription(
            Imu, '/imu/data', self._imu_callback, qos_sensor)
        self.vision_sub = node.create_subscription(
            VisionDetection, '/vision/detections', self._vision_callback, qos_sensor)

        # Stored sensor data
        self.latest_odometry: Optional[Odometry] = None
        self.latest_gps: Optional[NavSatFix] = None
        self.latest_imu: Optional[Imu] = None
        self.latest_battery: Optional[BatteryState] = None
        self.latest_temperature: Optional[Temperature] = None

    def _odom_callback(self, msg: Odometry):
        """Update odometry data"""
        self.latest_odometry = msg
        # Update position estimate
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._quaternion_to_heading(msg.pose.pose.orientation)
        )

    def _gps_callback(self, msg: NavSatFix):
        """Update GPS data"""
        self.latest_gps = msg
        self.last_gps_update = self.node.get_clock().now().nanoseconds / 1e9

    def _imu_callback(self, msg: Imu):
        """Update IMU data"""
        self.latest_imu = msg

    def _vision_callback(self, msg: VisionDetection):
        """Update vision detections"""
        self.vision_detections.append(msg)
        # Keep only recent detections
        if len(self.vision_detections) > 20:
            self.vision_detections = self.vision_detections[-20:]

    def get_current_position(self) -> Optional[Tuple[float, float, float]]:
        """Get current position from odometry"""
        return self.current_position if self.latest_odometry else None

    def get_gps_position(self) -> Optional[NavSatFix]:
        """Get latest GPS position"""
        return self.latest_gps

    def get_imu_data(self) -> Optional[Imu]:
        """Get latest IMU data"""
        return self.latest_imu

    def get_odometry(self) -> Optional[Odometry]:
        """Get latest odometry data"""
        return self.latest_odometry

    def get_vision_detections(self) -> List[VisionDetection]:
        """Get recent vision detections"""
        return self.vision_detections[-10:]  # Last 10 detections

    def get_aruco_tag_pose(self, tag_id: int) -> Optional[PoseStamped]:
        """Get pose of specific ArUco tag from vision data"""
        # Look for ArUco tag in recent detections
        for detection in reversed(self.vision_detections):
            if hasattr(detection, 'tag_id') and detection.tag_id == tag_id:
                pose = PoseStamped()
                pose.header = detection.header
                # Convert detection to pose (simplified)
                pose.pose.position.x = detection.position.x
                pose.pose.position.y = detection.position.y
                pose.pose.position.z = detection.position.z
                return pose
        return None

    def get_battery_status(self) -> Optional[BatteryState]:
        """Get mock battery status"""
        # Return mock battery data
        battery = BatteryState()
        battery.header.stamp = self.node.get_clock().now().to_msg()
        battery.voltage = 12.3
        battery.current = 2.1
        battery.percentage = 85.0
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        return battery

    def get_temperature_data(self) -> Optional[Temperature]:
        """Get mock temperature data"""
        temp = Temperature()
        temp.header.stamp = self.node.get_clock().now().to_msg()
        temp.temperature = 35.0
        temp.variance = 0.1
        return temp

    def _quaternion_to_heading(self, orientation) -> float:
        """Convert quaternion to heading angle"""
        import math

        # Simplified conversion for yaw
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)


class MockActuatorInterface(ActuatorInterface):
    """Mock actuator interface for development and testing"""

    def __init__(self, node: Node):
        self.node = node

        # Publishers for mock hardware control
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', qos_reliable)

        # Mock LED control (would publish to LED topic)
        self.led_pub = node.create_publisher(String, '/led/status', qos_reliable)

    def send_velocity_command(self, linear_x: float, angular_z: float):
        """Send velocity command to mock motors"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        self.cmd_vel_pub.publish(twist)
        self.node.get_logger().debug('.2f')

    def emergency_stop(self):
        """Mock emergency stop"""
        self.node.get_logger().warn('MOCK EMERGENCY STOP ACTIVATED')
        self.send_velocity_command(0.0, 0.0)
        self.set_led_status('EMERGENCY_RED_BLINK')

    def navigate_to_position(self, position: Dict[str, float]) -> bool:
        """Mock navigation to position"""
        self.node.get_logger().info(f'Mock navigation to position: {position}')

        # In a real implementation, this would use the navigation stack
        # For mock, just simulate success after delay
        import time
        time.sleep(1.0)  # Simulate navigation time

        return True  # Mock success

    def set_led_status(self, status: str):
        """Set mock LED status"""
        msg = String()
        msg.data = status
        self.led_pub.publish(msg)
        self.node.get_logger().debug(f'Mock LED status: {status}')


class RealSensorInterface(SensorInterface):
    """Real sensor interface for actual hardware"""

    def __init__(self, node: Node):
        self.node = node
        self.node.get_logger().info('Real sensor interface initialized')

        # Initialize real sensor drivers here
        # - CAN bus for drive/IMU data
        # - Network clients for camera data
        # - GPIO interfaces for encoders
        # - Serial interfaces for additional sensors

        # TODO: Implement real sensor drivers
        raise NotImplementedError("Real sensor interface not yet implemented")

    # Implement all abstract methods with real hardware...


class RealActuatorInterface(ActuatorInterface):
    """Real actuator interface for actual hardware"""

    def __init__(self, node: Node):
        self.node = node
        self.node.get_logger().info('Real actuator interface initialized')

        # Initialize real actuator drivers here
        # - Motor controllers (PWM/speed controllers)
        # - CAN bus interfaces
        # - Emergency stop circuits
        # - LED controllers

        # TODO: Implement real actuator drivers
        raise NotImplementedError("Real actuator interface not yet implemented")

    # Implement all abstract methods with real hardware...


# Utility functions for hardware configuration
def create_hardware_interface(node: Node, config: Dict[str, Any]) -> HardwareInterface:
    """
    Factory function to create appropriate hardware interface

    Args:
        node: ROS2 node
        config: Hardware configuration dictionary

    Returns:
        Configured HardwareInterface instance
    """
    use_mock = config.get('use_mock', True)

    interface = HardwareInterface(node, use_mock)

    # Configure sensor sources based on config
    sensor_sources = config.get('sensor_sources', {})

    if 'websocket' in sensor_sources:
        # Configure WebSocket sensor bridge
        interface.websocket_url = sensor_sources['websocket'].get('url', 'ws://localhost:8080')

    if 'can_bus' in sensor_sources:
        # Configure CAN bus sensors
        interface.can_interface = sensor_sources['can_bus'].get('interface', 'can0')

    if 'network_cameras' in sensor_sources:
        # Configure network camera clients
        interface.camera_endpoints = sensor_sources['network_cameras'].get('endpoints', [])

    return interface


def validate_hardware_config(config: Dict[str, Any]) -> List[str]:
    """
    Validate hardware configuration

    Args:
        config: Hardware configuration to validate

    Returns:
        List of validation error messages (empty if valid)
    """
    errors = []

    if not isinstance(config, dict):
        errors.append("Hardware config must be a dictionary")
        return errors

    # Check required fields
    if 'use_mock' not in config:
        errors.append("Missing 'use_mock' field in hardware config")

    # Validate sensor sources
    sensor_sources = config.get('sensor_sources', {})
    if not isinstance(sensor_sources, dict):
        errors.append("'sensor_sources' must be a dictionary")
    else:
        # Validate WebSocket config
        if 'websocket' in sensor_sources:
            ws_config = sensor_sources['websocket']
            if 'url' not in ws_config:
                errors.append("WebSocket config missing 'url' field")

        # Validate CAN bus config
        if 'can_bus' in sensor_sources:
            can_config = sensor_sources['can_bus']
            if 'interface' not in can_config:
                errors.append("CAN bus config missing 'interface' field")

        # Validate network cameras config
        if 'network_cameras' in sensor_sources:
            cam_config = sensor_sources['network_cameras']
            if 'endpoints' not in cam_config:
                errors.append("Network cameras config missing 'endpoints' field")

    return errors
