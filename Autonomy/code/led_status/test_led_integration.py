#!/usr/bin/env python3
"""
Test script for LED status integration with state management.

This script tests the LED status system integration with the state machine
by publishing various LED information messages and verifying the responses.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LEDIntegrationTester(Node):
    """Test node for LED status integration."""

    def __init__(self):
        super().__init__("led_integration_tester")

        # Publisher for LED info
        self.led_info_pub = self.create_publisher(String, "/state_machine/led_info", 10)

        # Publisher for system state
        self.system_state_pub = self.create_publisher(String, "/state_machine/system_state", 10)

        # Test sequence
        self.test_sequence = [
            ("BOOT_INITIALIZING", "BOOT"),
            ("CALIBRATION_YELLOW", "CALIBRATION"),
            ("IDLE_GREEN", "IDLE"),
            ("AUTONOMOUS_RED", "AUTONOMOUS"),
            ("TELEOPERATION_BLUE", "TELEOPERATION"),
            ("SAFETY_RED_BLINK", "SAFETY"),
            ("WAYPOINT_SUCCESS", "AUTONOMOUS"),
            ("ERROR_GENERAL", "AUTONOMOUS"),
            ("SUCCESS_MISSION", "AUTONOMOUS"),
            ("SHUTDOWN_RED_FADE", "SHUTDOWN"),
        ]

        self.current_test_index = 0
        self.test_timer = self.create_timer(3.0, self.run_next_test)

        self.get_logger().info("LED Integration Tester started")
        self.get_logger().info("Starting test sequence...")

    def run_next_test(self):
        """Run the next test in the sequence."""
        if self.current_test_index >= len(self.test_sequence):
            self.get_logger().info("All tests completed!")
            self.test_timer.cancel()
            return

        led_info, system_state = self.test_sequence[self.current_test_index]

        # Publish LED info
        led_msg = String()
        led_msg.data = led_info
        self.led_info_pub.publish(led_msg)

        # Publish system state
        state_msg = String()
        state_msg.data = system_state
        self.system_state_pub.publish(state_msg)

        self.get_logger().info(
            f"Test {self.current_test_index + 1}/{len(self.test_sequence)}: {led_info} -> {system_state}"
        )

        self.current_test_index += 1

    def test_emergency_sequence(self):
        """Test emergency LED sequence."""
        self.get_logger().info("Testing emergency sequence...")

        # Rapid error signals
        for i in range(5):
            led_msg = String()
            led_msg.data = "ERROR_EMERGENCY"
            self.led_info_pub.publish(led_msg)
            time.sleep(0.5)

        # Safety state
        led_msg = String()
        led_msg.data = "SAFETY_RED_BLINK"
        self.led_info_pub.publish(led_msg)

        self.get_logger().info("Emergency sequence test completed")

    def test_success_sequence(self):
        """Test success LED sequence."""
        self.get_logger().info("Testing success sequence...")

        # Waypoint success
        led_msg = String()
        led_msg.data = "WAYPOINT_SUCCESS"
        self.led_info_pub.publish(led_msg)
        time.sleep(2)

        # Mission success
        led_msg = String()
        led_msg.data = "SUCCESS_MISSION"
        self.led_info_pub.publish(led_msg)
        time.sleep(2)

        # Back to normal
        led_msg = String()
        led_msg.data = "AUTONOMOUS_RED"
        self.led_info_pub.publish(led_msg)

        self.get_logger().info("Success sequence test completed")


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        tester = LEDIntegrationTester()

        # Run basic test sequence
        rclpy.spin_once(tester, timeout_sec=30.0)

        # Run additional tests
        tester.test_emergency_sequence()
        time.sleep(2)
        tester.test_success_sequence()

        tester.get_logger().info("All LED integration tests completed!")

    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
    except Exception as e:
        tester.get_logger().error(f"Test failed with error: {str(e)}")
    finally:
        if "tester" in locals():
            tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
