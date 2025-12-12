#!/usr/bin/env python3
"""
Mock Rover Simulation - Simulates complete rover environment without Gazebo

Provides realistic ROS2 topics and data flow for testing:
- Odometry data (moving robot)
- IMU data (accelerometer/gyroscope)
- GPS data (simulated position)
- SLAM pose data (localization)
- Sensor fusion and realistic timing

This allows testing of your complete mission system without requiring
physical hardware or complex simulation setup.
"""

import math
import os
import random

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Header


class MockRoverSimulation(Node):
    """
    Mock simulation providing realistic rover sensor data

    Simulates a rover moving in a simple pattern while publishing
    all the sensor data your mission system expects.
    """

    def __init__(self):
        super().__init__('mock_rover_simulation')

        # Load configuration
        self.config = self.load_config()

        # Simulation parameters (from config)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.linear_velocity = 0.3  # m/s
        self.angular_velocity = 0.1  # rad/s

        # GPS origin (San Francisco for realism)
        self.gps_origin_lat = 37.7749
        self.gps_origin_lon = -122.4194
        self.gps_origin_alt = 10.0

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.slam_pub = self.create_publisher(PoseStamped, '/slam/pose', 10)  # SLAM pose

        # Simulation timer (10 Hz for production efficiency)
        self.sim_timer = self.create_timer(0.1, self.update_simulation)

        # Control input (can be overridden by mission system)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.current_cmd_vel = Twist()

        self.get_logger().info('Mock Rover Simulation started')
        self.get_logger().info('Robot starting at (0, 0), facing positive X')
        self.get_logger().info('Publishing odometry, IMU, GPS, and SLAM data')

    def load_config(self):
        """Load configuration from YAML file based on environment"""
        env = os.environ.get('ROVER_ENV', 'production')  # Default to production
        config_file = f'{env}.yaml'

        # Try multiple possible config locations
        possible_paths = [
            os.path.join(os.path.dirname(__file__), '..', 'config', config_file),
            os.path.join(os.path.dirname(__file__), 'config', config_file),
            f'config/{config_file}'
        ]

        for config_path in possible_paths:
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    self.get_logger().info(f'Loaded {env} configuration from {config_path}')
                    return config
            except FileNotFoundError:
                continue

        self.get_logger().warn(f'Config file {config_file} not found in any expected location, using defaults')
        return {}

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands from mission system"""
        self.current_cmd_vel = msg

        # Override simulation velocities with commanded ones
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_simulation(self):
        """Update robot state and publish sensor data"""
        # Update robot pose based on velocity commands
        dt = 0.05  # 20 Hz

        # Apply velocity commands
        if abs(self.current_cmd_vel.linear.x) > 0.01:
            self.linear_velocity = self.current_cmd_vel.linear.x
        if abs(self.current_cmd_vel.angular.z) > 0.01:
            self.angular_velocity = self.current_cmd_vel.angular.z

        # Update pose
        self.robot_x += self.linear_velocity * math.cos(self.robot_yaw) * dt
        self.robot_y += self.linear_velocity * math.sin(self.robot_yaw) * dt
        self.robot_yaw += self.angular_velocity * dt

        # Keep yaw in reasonable range
        while self.robot_yaw > math.pi:
            self.robot_yaw -= 2 * math.pi
        while self.robot_yaw < -math.pi:
            self.robot_yaw += 2 * math.pi

        # Publish odometry
        self.publish_odometry()

        # Publish IMU data
        self.publish_imu()

        # Publish GPS data
        self.publish_gps()

        # Publish SLAM pose (with some localization noise)
        self.publish_slam_pose()

    def publish_odometry(self):
        """Publish realistic odometry data"""
        odom = Odometry()
        odom.header = self.create_header('odom')
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.robot_yaw / 2.0)

        # Add small pose covariance (realistic odometry uncertainty)
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.001
        ]

        # Velocity
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        # Velocity covariance
        odom.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        self.odom_pub.publish(odom)

    def publish_imu(self):
        """Publish realistic IMU data"""
        imu = Imu()
        imu.header = self.create_header('imu_link')
        imu.header.frame_id = 'imu_link'

        # Accelerometer (gravity + motion)
        gravity = 9.81
        motion_accel_x = -self.linear_velocity * self.angular_velocity  # Centripetal acceleration
        motion_accel_y = 0.0

        imu.linear_acceleration.x = motion_accel_x + random.gauss(0, 0.1)
        imu.linear_acceleration.y = motion_accel_y + random.gauss(0, 0.1)
        imu.linear_acceleration.z = gravity + random.gauss(0, 0.05)

        # Gyroscope
        imu.angular_velocity.x = random.gauss(0, 0.01)  # Small noise
        imu.angular_velocity.y = random.gauss(0, 0.01)
        imu.angular_velocity.z = self.angular_velocity + random.gauss(0, 0.02)

        # Orientation (from odometry, with IMU drift simulation)
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = math.sin(self.robot_yaw / 2.0) + random.gauss(0, 0.01)
        imu.orientation.w = math.cos(self.robot_yaw / 2.0) + random.gauss(0, 0.01)

        # Covariances (realistic IMU specs)
        imu.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        imu.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]

        # Orientation covariance (3x3 matrix = 9 values)
        imu.orientation_covariance = [
            0.001, 0.0, 0.0,  # Roll variance, roll-pitch covariance, roll-yaw covariance
            0.0, 0.001, 0.0,  # Pitch-roll covariance, pitch variance, pitch-yaw covariance
            0.0, 0.0, 0.001   # Yaw-roll covariance, yaw-pitch covariance, yaw variance
        ]

        self.imu_pub.publish(imu)

    def publish_gps(self):
        """Publish realistic GPS data"""
        gps = NavSatFix()
        gps.header = self.create_header('gps_link')
        gps.header.frame_id = 'gps_link'

        # Convert local coordinates to GPS
        # Approximate conversion: 1 degree ≈ 111,320 meters
        lat_offset = self.robot_y / 111320.0  # Convert meters to degrees latitude
        lon_offset = self.robot_x / (111320.0 * math.cos(math.radians(self.gps_origin_lat)))

        gps.latitude = self.gps_origin_lat + lat_offset + random.gauss(0, 0.00001)  # GPS noise
        gps.longitude = self.gps_origin_lon + lon_offset + random.gauss(0, 0.00001)
        gps.altitude = self.gps_origin_alt + random.gauss(0, 0.5)  # Altitude variation

        # GPS status (fix acquired)
        gps.status.status = 0  # STATUS_FIX
        gps.status.service = 1  # SERVICE_GPS

        # Position covariance (GPS accuracy)
        gps.position_covariance = [
            1.0, 0.0, 0.0,  # East position error
            0.0, 1.0, 0.0,  # North position error
            0.0, 0.0, 4.0   # Altitude error
        ]
        gps.position_covariance_type = 1  # COVARIANCE_TYPE_APPROXIMATED

        self.gps_pub.publish(gps)

    def publish_slam_pose(self):
        """Publish SLAM pose with localization characteristics"""
        pose = PoseStamped()
        pose.header = self.create_header('map')
        pose.header.frame_id = 'map'

        # SLAM pose (similar to odometry but with different characteristics)
        # Add small localization drift and corrections
        slam_drift_x = random.gauss(0, 0.05)  # SLAM drift
        slam_drift_y = random.gauss(0, 0.05)

        pose.pose.position.x = self.robot_x + slam_drift_x
        pose.pose.position.y = self.robot_y + slam_drift_y
        pose.pose.position.z = 0.0

        # Orientation (SLAM typically has better heading accuracy)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.robot_yaw / 2.0)

        self.slam_pub.publish(pose)

    def create_header(self, frame_id: str) -> Header:
        """Create a ROS2 header with current timestamp"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header


def main():
    """Main entry point"""
    rclpy.init()
    try:
        simulation = MockRoverSimulation()
        rclpy.spin(simulation)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
