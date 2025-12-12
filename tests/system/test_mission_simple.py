#!/usr/bin/env python3
"""
Simple Mission Test - Direct test of mission execution
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleMissionTester(Node):
    """Simple tester for mission execution"""

    def __init__(self):
        super().__init__('simple_mission_tester')

        # Publisher for mission commands
        self.cmd_pub = self.create_publisher(String, '/mission/commands', 10)

        # Subscriber for mission status
        self.status_sub = self.create_subscription(
            String, '/mission/status', self.status_callback, 10)

        self.mission_status = None
        self.get_logger().info('🧪 Simple Mission Tester ready')

    def status_callback(self, msg):
        """Handle mission status updates"""
        try:
            self.mission_status = json.loads(msg.data)
            self.get_logger().info(f'📋 Mission status: {self.mission_status}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse status: {e}')

    def send_command(self, command, params=None):
        """Send a mission command"""
        cmd_data = {'command': command}
        if params:
            cmd_data.update(params)

        msg = String()
        msg.data = json.dumps(cmd_data)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'📤 Sent: {cmd_data}')

    def test_mission(self):
        """Run the mission test"""
        self.get_logger().info('🧪 Testing mission execution...')

        # Wait for system to be ready
        time.sleep(2)

        # Send mission command
        waypoints = [{'x': 2.0, 'y': 0.0}]
        self.send_command('start_waypoint_mission', {'waypoints': waypoints})

        # Wait for mission to execute
        start_time = time.time()
        while time.time() - start_time < 15 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # Check if mission completed
            if self.mission_status and self.mission_status.get('status') == 'completed':
                self.get_logger().info('✅ Mission completed successfully!')
                return True

        # Send stop command
        self.send_command('stop_mission')

        if self.mission_status and self.mission_status.get('status') == 'completed':
            self.get_logger().info('✅ Mission completed!')
            return True
        else:
            self.get_logger().error('❌ Mission did not complete')
            return False


def main():
    rclpy.init()

    try:
        tester = SimpleMissionTester()
        success = tester.test_mission()

        if success:
            print("✅ MISSION TEST PASSED")
        else:
            print("❌ MISSION TEST FAILED")

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
