#!/usr/bin/env python3
"""
Test Mission Executor - Verify simplified mission execution works

Tests the core functionality:
- Command routing through topics
- Waypoint navigation logic
- Status and progress publishing
"""

import json
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String


class MissionExecutorTester(Node):
    """Test node for mission executor functionality"""

    def __init__(self):
        super().__init__('mission_executor_tester')

        # Publishers for testing
        self.cmd_pub = self.create_publisher(String, '/mission/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Subscribers for monitoring
        self.status_sub = self.create_subscription(
            String, '/mission/status', self.status_callback, 10)
        self.progress_sub = self.create_subscription(
            Float32, '/mission/progress', self.progress_callback, 10)

        # Test data storage
        self.received_status = None
        self.received_progress = None
        self.status_count = 0
        self.progress_count = 0

        # Simulate robot position
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Start odometry publishing
        self.odom_timer = self.create_timer(0.1, self.publish_odometry)

        self.get_logger().info('Mission Executor Tester ready')

    def status_callback(self, msg):
        """Monitor mission status"""
        try:
            self.received_status = json.loads(msg.data)
            self.status_count += 1
            self.get_logger().info(f'Status: {self.received_status}')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid status JSON')

    def progress_callback(self, msg):
        """Monitor mission progress"""
        self.received_progress = msg.data
        self.progress_count += 1
        self.get_logger().info(f'Progress: {self.received_progress:.1f}%')

    def publish_odometry(self):
        """Publish simulated odometry"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.orientation.w = 1.0  # Facing positive x
        self.odom_pub.publish(odom)

    def send_command(self, command_data):
        """Send command to mission executor"""
        msg = String()
        msg.data = json.dumps(command_data)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command_data}')

    def simulate_movement(self, target_x, target_y, speed=0.5):
        """Simulate robot movement toward target"""
        while abs(self.robot_x - target_x) > 0.1 or abs(self.robot_y - target_y) > 0.1:
            dx = target_x - self.robot_x
            dy = target_y - self.robot_y
            dist = (dx**2 + dy**2)**0.5

            if dist < 0.1:
                break

            # Move toward target
            self.robot_x += (dx / dist) * speed * 0.1
            self.robot_y += (dy / dist) * speed * 0.1
            time.sleep(0.1)

        # Snap to exact position
        self.robot_x = target_x
        self.robot_y = target_y

    def run_test(self):
        """Run comprehensive test"""
        self.get_logger().info('Starting Mission Executor Test')

        # Wait for system to stabilize
        time.sleep(1.0)

        # Test 1: Start waypoint mission
        waypoints = [
            {'x': 5.0, 'y': 0.0},
            {'x': 5.0, 'y': 5.0},
            {'x': 0.0, 'y': 5.0}
        ]

        self.get_logger().info('Test 1: Starting waypoint mission')
        self.send_command({
            'command': 'start_waypoint_mission',
            'waypoints': waypoints
        })

        # Wait for mission to start
        time.sleep(1.0)

        # Simulate reaching first waypoint
        self.get_logger().info('Simulating movement to first waypoint')
        self.simulate_movement(5.0, 0.0)

        # Wait for waypoint detection
        time.sleep(2.0)

        # Simulate reaching second waypoint
        self.get_logger().info('Simulating movement to second waypoint')
        self.simulate_movement(5.0, 5.0)

        # Wait for waypoint detection
        time.sleep(2.0)

        # Simulate reaching third waypoint
        self.get_logger().info('Simulating movement to third waypoint')
        self.simulate_movement(0.0, 5.0)

        # Wait for mission completion
        time.sleep(3.0)

        # Test 2: Stop mission
        self.get_logger().info('Test 2: Stopping mission')
        self.send_command({'command': 'stop_mission'})
        time.sleep(1.0)

        # Print test results
        self.print_test_results()

    def print_test_results(self):
        """Print test results"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('MISSION EXECUTOR TEST RESULTS')
        self.get_logger().info('=' * 50)

        success = True

        # Check status messages received
        if self.status_count > 0:
            self.get_logger().info(f'✅ Status messages received: {self.status_count}')
        else:
            self.get_logger().error('❌ No status messages received')
            success = False

        # Check progress messages received
        if self.progress_count > 0:
            self.get_logger().info(f'✅ Progress messages received: {self.progress_count}')
        else:
            self.get_logger().error('❌ No progress messages received')
            success = False

        # Check mission completion - look for completion in status history
        mission_completed = any(status.get('status') == 'completed' for status in [self.received_status] if status)
        if mission_completed:
            self.get_logger().info('✅ Mission completed successfully')
        else:
            self.get_logger().error('❌ Mission did not complete')
            success = False

        # Check final progress
        if self.received_progress and self.received_progress >= 99.0:
            self.get_logger().info('✅ Final progress 100%')
        else:
            self.get_logger().error(f'❌ Final progress not 100%: {self.received_progress}')
            success = False

        if success:
            self.get_logger().info('🎉 ALL TESTS PASSED!')
        else:
            self.get_logger().error('❌ SOME TESTS FAILED')

        self.get_logger().info('=' * 50)


def main():
    """Main test function"""
    rclpy.init()

    # Create test node
    tester = MissionExecutorTester()

    # Let system stabilize
    time.sleep(2.0)

    # Run test in separate thread
    test_thread = threading.Thread(target=tester.run_test)
    test_thread.start()

    # Spin ROS2
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass

    # Wait for test to complete
    test_thread.join(timeout=30.0)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
