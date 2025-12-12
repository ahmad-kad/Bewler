#!/usr/bin/env python3
"""
Mock Motor Controller - Simulates rover movement for development and testing

This node subscribes to /cmd_vel commands and publishes simulated odometry,
allowing development of navigation and mission logic without real hardware.

Author: URC 2026 Autonomy Team
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class MockMotorController(Node):
    """
    Mock motor controller that simulates rover movement.

    Subscribes to:
    - /cmd_vel (geometry_msgs/Twist): Velocity commands

    Publishes:
    - /odom (nav_msgs/Odometry): Simulated odometry
    - /imu/data (sensor_msgs/Imu): Simulated IMU data
    """

    def __init__(self):
        super().__init__('mock_motor_controller')

        # Declare parameters
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('wheel_separation', 0.5)  # meters
        self.declare_parameter('wheel_radius', 0.15)  # meters
        self.declare_parameter('max_linear_velocity', 2.0)  # m/s
        self.declare_parameter('max_angular_velocity', 1.5)  # rad/s
        self.declare_parameter('position_noise_std', 0.01)  # meters
        self.declare_parameter('orientation_noise_std', 0.01)  # radians

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.pos_noise_std = self.get_parameter('position_noise_std').value
        self.orient_noise_std = self.get_parameter('orientation_noise_std').value

        # Control state
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

        # Simulated rover state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        # Simulated IMU state
        self.imu_ax = 0.0
        self.imu_ay = 0.0
        self.imu_az = 9.81  # Gravity
        self.imu_gx = 0.0
        self.imu_gy = 0.0
        self.imu_gz = 0.0

        # Setup QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_reliable)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_sensor)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_reliable)

        # Control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.update_rate, self.control_loop)

        # Odometry sequence counter
        self.odom_seq = 0

        self.get_logger().info('Mock Motor Controller initialized')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(
            f'Max velocities: linear={self.max_linear_vel} m/s, angular={self.max_angular_vel} rad/s')

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands from navigation stack"""
        # Clamp velocities to safe limits
        linear_x = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

        self.target_linear_vel = linear_x
        self.target_angular_vel = angular_z

        self.get_logger().debug('.2f')

    def control_loop(self):
        """Main control loop - simulate physics and publish sensor data"""
        dt = 1.0 / self.update_rate

        # Simple velocity ramping (acceleration limit)
        accel_limit = 1.0  # m/s²
        angular_accel_limit = 2.0  # rad/s²

        # Linear velocity control
        linear_error = self.target_linear_vel - self.current_linear_vel
        linear_accel = max(-accel_limit, min(accel_limit, linear_error / dt))
        self.current_linear_vel += linear_accel * dt

        # Angular velocity control
        angular_error = self.target_angular_vel - self.current_angular_vel
        angular_accel = max(-angular_accel_limit, min(angular_accel_limit, angular_error / dt))
        self.current_angular_vel += angular_accel * dt

        # Update simulated rover position (kinematic model)
        self.update_kinematics(dt)

        # Update simulated IMU data
        self.update_simulated_imu()

        # Publish odometry
        self.publish_odometry()

        # Publish IMU data
        self.publish_imu()

    def update_kinematics(self, dt: float):
        """Update rover position using kinematic model"""
        # Add some noise to simulate real sensors
        import random
        pos_noise = random.gauss(0, self.pos_noise_std)
        orient_noise = random.gauss(0, self.orient_noise_std)

        # Update position
        self.x += (self.current_linear_vel * math.cos(self.theta) * dt) + pos_noise
        self.y += (self.current_linear_vel * math.sin(self.theta) * dt) + pos_noise
        self.theta += (self.current_angular_vel * dt) + orient_noise

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Update velocities
        self.vx = self.current_linear_vel * math.cos(self.theta)
        self.vy = self.current_linear_vel * math.sin(self.theta)
        self.vtheta = self.current_angular_vel

    def update_simulated_imu(self):
        """Update simulated IMU readings based on motion"""
        # Simulate accelerometer readings (including gravity and acceleration)
        ax_world = self.current_linear_vel * self.current_angular_vel  # Centripetal acceleration
        ay_world = 0.0
        az_world = 9.81  # Gravity

        # Rotate to body frame (simple approximation)
        self.imu_ax = ax_world * math.cos(self.theta) - ay_world * math.sin(self.theta)
        self.imu_ay = ax_world * math.sin(self.theta) + ay_world * math.cos(self.theta)
        self.imu_az = az_world

        # Simulate gyroscope readings
        self.imu_gx = 0.0  # Roll rate (assume flat ground)
        self.imu_gy = 0.0  # Pitch rate
        self.imu_gz = self.current_angular_vel  # Yaw rate

        # Add some noise
        import random
        noise_std = 0.01
        self.imu_ax += random.gauss(0, noise_std)
        self.imu_ay += random.gauss(0, noise_std)
        self.imu_az += random.gauss(0, noise_std)
        self.imu_gx += random.gauss(0, noise_std * 10)
        self.imu_gy += random.gauss(0, noise_std * 10)
        self.imu_gz += random.gauss(0, noise_std * 10)

    def publish_odometry(self):
        """Publish simulated odometry data"""
        odom = Odometry()

        # Header
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        self.odom_seq += 1
        odom.header.seq = self.odom_seq

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Pose covariance (diagonal matrix)
        odom.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        # Twist
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vtheta

        # Twist covariance
        odom.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.01, 0.0]

        self.odom_pub.publish(odom)

    def publish_imu(self):
        """Publish simulated IMU data"""
        imu = Imu()

        # Header
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        # Orientation (from odometry)
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = math.sin(self.theta / 2.0)
        imu.orientation.w = math.cos(self.theta / 2.0)
        imu.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        # Angular velocity
        imu.angular_velocity.x = self.imu_gx
        imu.angular_velocity.y = self.imu_gy
        imu.angular_velocity.z = self.imu_gz
        imu.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        # Linear acceleration
        imu.linear_acceleration.x = self.imu_ax
        imu.linear_acceleration.y = self.imu_ay
        imu.linear_acceleration.z = self.imu_az
        imu.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        self.imu_pub.publish(imu)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        controller = MockMotorController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
