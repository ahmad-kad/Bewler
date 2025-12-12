#!/usr/bin/env python3
"""
Sensor Integration Node - Basic sensor interfaces for navigation

This module provides placeholder implementations for sensor integration that will
be replaced with actual hardware interfaces later. It provides:

- GPS data publishing (simulated)
- IMU data publishing (simulated)
- Camera data publishing (simulated)
- Wheel odometry publishing (simulated)

All sensors publish at appropriate frequencies for navigation algorithms.
"""

import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix
from std_msgs.msg import Header


class SensorIntegrationNode(Node):
    """
    Basic sensor integration node providing placeholder sensor data.

    This node simulates sensor inputs for development and testing.
    Replace with actual hardware interfaces when available.
    """

    def __init__(self):
        super().__init__("sensor_integration_node")

        # Publishers for sensor data
        self.gps_publisher = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.imu_publisher = self.create_publisher(Imu, "imu/data", 10)
        self.image_publisher = self.create_publisher(Image, "camera/image_raw", 10)
        self.odom_publisher = self.create_publisher(Odometry, "wheel/odom", 10)

        # Publishers for navigation inputs
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Simulation state
        self.sim_time = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # GPS simulation (start at URC 2026-ish coordinates)
        self.base_latitude = 37.7749
        self.base_longitude = -122.4194
        self.gps_noise_std = 1.0  # meters

        # Timers for different sensor update rates
        self.gps_timer = self.create_timer(1.0, self.publish_gps_data)  # 1 Hz
        self.imu_timer = self.create_timer(0.1, self.publish_imu_data)  # 10 Hz
        self.camera_timer = self.create_timer(0.033, self.publish_camera_data)  # ~30 Hz
        self.odom_timer = self.create_timer(0.05, self.publish_odometry)  # 20 Hz

        # Subscribe to velocity commands to update simulation
        self.cmd_vel_subscription = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        self.get_logger().info("Sensor integration node initialized with simulated sensors")

    def cmd_vel_callback(self, msg: Twist):
        """Update robot velocity from navigation commands"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def publish_gps_data(self):
        """Publish simulated GPS data"""
        # Update robot position based on velocity
        dt = 1.0  # GPS update rate
        self.robot_x += self.linear_velocity * math.cos(self.robot_yaw) * dt
        self.robot_y += self.linear_velocity * math.sin(self.robot_yaw) * dt
        self.robot_yaw += self.angular_velocity * dt

        # Convert to GPS coordinates (rough approximation)
        # 1 degree lat/lon ≈ 111 km, so small displacements in meters
        lat_offset = self.robot_y / 111000.0  # Convert meters to degrees latitude
        lon_offset = self.robot_x / (111000.0 * math.cos(math.radians(self.base_latitude)))

        # Add noise
        lat_noise = random.gauss(0, self.gps_noise_std / 111000.0)
        lon_noise = random.gauss(
            0,
            self.gps_noise_std / (111000.0 * math.cos(math.radians(self.base_latitude))),
        )

        gps_msg = NavSatFix()
        gps_msg.header = self.create_header("gps")
        gps_msg.latitude = self.base_latitude + lat_offset + lat_noise
        gps_msg.longitude = self.base_longitude + lon_offset + lon_noise
        gps_msg.altitude = 10.0  # Assume we're at 10m elevation

        # GPS status (simulated as fixed)
        gps_msg.status.status = 0  # STATUS_FIX
        gps_msg.status.service = 1  # SERVICE_GPS

        # Position covariance (diagonal matrix)
        gps_msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.gps_publisher.publish(gps_msg)

    def publish_imu_data(self):
        """Publish simulated IMU data"""
        imu_msg = Imu()
        imu_msg.header = self.create_header("imu")

        # Orientation (simulated)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = math.sin(self.robot_yaw / 2)
        imu_msg.orientation.w = math.cos(self.robot_yaw / 2)

        # Angular velocity (from our simulation + noise)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = self.angular_velocity + random.gauss(0, 0.01)

        # Linear acceleration (simulated gravity + movement)
        imu_msg.linear_acceleration.x = random.gauss(0, 0.1)
        imu_msg.linear_acceleration.y = random.gauss(0, 0.1)
        imu_msg.linear_acceleration.z = 9.81 + random.gauss(0, 0.1)  # Gravity

        # Covariances (simplified diagonal)
        imu_msg.orientation_covariance = [
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
        imu_msg.angular_velocity_covariance = [
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
        imu_msg.linear_acceleration_covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.1,
        ]

        self.imu_publisher.publish(imu_msg)

    def publish_camera_data(self):
        """Publish simulated camera data"""
        # Create a simple simulated image (grayscale gradient)
        width, height = 640, 480

        # Generate simple pattern data
        image_data = []
        for y in range(height):
            for x in range(width):
                # Simple gradient pattern
                intensity = int(255 * (x / width) * (y / height))
                image_data.extend([intensity, intensity, intensity])  # RGB

        image_msg = Image()
        image_msg.header = self.create_header("camera")
        image_msg.height = height
        image_msg.width = width
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = False
        image_msg.step = width * 3
        image_msg.data = image_data

        self.image_publisher.publish(image_msg)

    def publish_odometry(self):
        """Publish simulated wheel odometry"""
        odom_msg = Odometry()
        odom_msg.header = self.create_header("odom")

        # Position
        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.robot_yaw / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.robot_yaw / 2)

        # Velocity
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity

        # Covariances (simplified)
        odom_msg.pose.covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
        ]

        odom_msg.twist.covariance = odom_msg.pose.covariance.copy()

        self.odom_publisher.publish(odom_msg)

    def create_header(self, frame_id: str) -> Header:
        """Create a standard ROS2 header"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        sensor_node = SensorIntegrationNode()
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
