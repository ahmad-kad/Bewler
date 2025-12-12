#!/usr/bin/env python3
"""
Test Teleoperation Integration with Autonomy
Publishes mock teleoperation data to test autonomy integration
"""

import math
import random
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Float32MultiArray


class MockTeleoperationPublisher(Node):
    """Mock teleoperation data publisher for testing autonomy integration"""

    def __init__(self):
        super().__init__('mock_teleoperation_publisher')

        # Publishers matching teleoperation topics
        self.joint_pub = self.create_publisher(
            JointState, '/teleoperation/joint_states', 10)
        self.chassis_vel_pub = self.create_publisher(
            TwistStamped, '/teleoperation/chassis_velocity', 10)
        self.temp_pub = self.create_publisher(
            Float32MultiArray, '/teleoperation/motor_temperatures', 10)
        self.status_pub = self.create_publisher(
            BatteryState, '/teleoperation/system_status', 10)

        # Publish at 10Hz
        self.timer = self.create_timer(0.1, self.publish_mock_data)

        # Mock data state
        self.time_offset = time.time()
        self.temperature_trend = 25.0  # Starting temperature
        self.battery_level = 85.0

        self.get_logger().info("Mock Teleoperation Publisher started")
        self.get_logger().info("Publishing to:")
        self.get_logger().info("  - /teleoperation/joint_states")
        self.get_logger().info("  - /teleoperation/chassis_velocity")
        self.get_logger().info("  - /teleoperation/motor_temperatures")
        self.get_logger().info("  - /teleoperation/system_status")

    def publish_mock_data(self):
        """Publish realistic mock teleoperation data"""
        current_time = self.get_clock().now()

        # 1. Joint States (motor positions and velocities)
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.header.frame_id = 'base_link'
        joint_state.name = ['swerve_front_left', 'swerve_front_right',
                            'swerve_rear_left', 'swerve_rear_right']

        # Realistic swerve module positions (simulate slow rotation)
        t = time.time() - self.time_offset
        joint_state.position = [
            math.sin(t * 0.5) * 0.5,      # FL
            math.sin(t * 0.5 + 1.57) * 0.5,  # FR
            math.sin(t * 0.5 + 3.14) * 0.5,  # RL
            math.sin(t * 0.5 + 4.71) * 0.5  # RR
        ]

        # Realistic velocities (some movement)
        joint_state.velocity = [
            math.cos(t * 0.5) * 2.0 + random.uniform(-0.1, 0.1),
            math.cos(t * 0.5 + 1.57) * 2.0 + random.uniform(-0.1, 0.1),
            math.cos(t * 0.5 + 3.14) * 2.0 + random.uniform(-0.1, 0.1),
            math.cos(t * 0.5 + 4.71) * 2.0 + random.uniform(-0.1, 0.1)
        ]

        self.joint_pub.publish(joint_state)

        # 2. Chassis Velocity (actual measured velocity)
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = current_time.to_msg()
        twist_stamped.header.frame_id = 'odom'

        # Simulate some movement with noise
        base_speed = 0.5 + math.sin(t * 0.3) * 0.3
        twist_stamped.twist.linear.x = base_speed + random.uniform(-0.05, 0.05)
        twist_stamped.twist.linear.y = math.sin(t * 0.2) * 0.1 + random.uniform(-0.02, 0.02)
        twist_stamped.twist.angular.z = math.cos(t * 0.4) * 0.2 + random.uniform(-0.05, 0.05)

        self.chassis_vel_pub.publish(twist_stamped)

        # 3. Motor Temperatures (gradually increasing)
        self.temperature_trend += random.uniform(-0.1, 0.2)  # Gradual warming
        self.temperature_trend = max(20.0, min(80.0, self.temperature_trend))  # Bounds

        temp_array = Float32MultiArray()
        temp_array.data = [
            self.temperature_trend + random.uniform(-2, 2),
            self.temperature_trend + random.uniform(-2, 2),
            self.temperature_trend + random.uniform(-2, 2),
            self.temperature_trend + random.uniform(-2, 2)
        ]

        self.temp_pub.publish(temp_array)

        # 4. System Status (battery gradually discharging)
        self.battery_level -= random.uniform(0, 0.01)  # Slow discharge
        self.battery_level = max(5.0, self.battery_level)  # Don't go below 5%

        battery_state = BatteryState()
        battery_state.header.stamp = current_time.to_msg()
        battery_state.voltage = 20.0 + (self.battery_level / 100.0) * 6.0  # 20-26V range
        battery_state.current = -5.0 + random.uniform(-1, 1)  # Negative = discharging
        battery_state.percentage = self.battery_level

        self.status_pub.publish(battery_state)

        # Log status every 10 seconds
        if int(t) % 10 == 0 and int(t * 10) % 10 == 0:
            self.get_logger().info(".1f"
                                   ".1f")


def main():
    """Main test function"""
    print("🧪 Teleoperation Integration Test")
    print("Starting mock teleoperation data publisher...")
    print("This will publish realistic test data to autonomy system")
    print("Press Ctrl+C to stop")
    print()

    rclpy.init()
    publisher = MockTeleoperationPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print("\n🛑 Stopping mock publisher...")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()
        print("✅ Mock publisher stopped")


if __name__ == '__main__':
    main()
