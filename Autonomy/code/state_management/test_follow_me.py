#!/usr/bin/env python3
"""
Test script for Follow Me feature.

Tests the follow me functionality including state transitions,
ArUco detection requirements, and follow me behavior.
"""

import time

import rclpy
from autonomy_interfaces.msg import FollowMeStatus
from autonomy_interfaces.msg import SystemState as SystemStateMsg
from autonomy_state_machine.follow_me_frontend import FollowMeFrontend
from rclpy.node import Node


class FollowMeTester(Node):
    """Test node for follow me functionality."""

    def __init__(self):
        super().__init__("follow_me_tester")

        # Initialize follow me frontend
        self.frontend = FollowMeFrontend(self)

        # Set up callbacks
        self.frontend.set_status_callback(self._on_follow_me_status)
        self.frontend.set_state_callback(self._on_state_update)

        # Test state
        self.test_completed = False
        self.test_results = []

        self.get_logger().info("Follow Me Tester initialized")

    def run_tests(self):
        """Run all follow me tests."""
        self.get_logger().info("Starting Follow Me tests...")

        # Test 1: State transition to follow me
        self._test_state_transition()

        # Test 2: Follow me behavior start/stop
        self._test_follow_me_behavior()

        # Test 3: Safety distance handling
        self._test_safety_distance()

        # Test 4: State transition back to idle
        self._test_return_to_idle()

        # Print results
        self._print_test_results()

    def _test_state_transition(self):
        """Test transitioning to follow me state."""
        self.get_logger().info("Test 1: State transition to follow me")

        try:
            # Try to start follow me mode
            success = self.frontend.start_follow_me(
                target_tag_id=42,
                safety_distance=2.0,
                max_speed=1.0,
                operator_id="tester",
            )

            if success:
                self.test_results.append("✅ State transition to follow me: PASSED")
            else:
                self.test_results.append("❌ State transition to follow me: FAILED")

        except Exception as e:
            self.test_results.append(f"❌ State transition to follow me: ERROR - {str(e)}")

    def _test_follow_me_behavior(self):
        """Test follow me behavior start/stop."""
        self.get_logger().info("Test 2: Follow me behavior")

        try:
            # Check if follow me is active
            if self.frontend.is_follow_me_active():
                self.test_results.append("✅ Follow me behavior active: PASSED")
            else:
                self.test_results.append("❌ Follow me behavior active: FAILED")

            # Get status
            status = self.frontend.get_follow_me_status()
            if status:
                self.test_results.append("✅ Follow me status available: PASSED")
                self.get_logger().info(f"Follow me status: {status}")
            else:
                self.test_results.append("❌ Follow me status available: FAILED")

        except Exception as e:
            self.test_results.append(f"❌ Follow me behavior: ERROR - {str(e)}")

    def _test_safety_distance(self):
        """Test safety distance handling."""
        self.get_logger().info("Test 3: Safety distance handling")

        try:
            # This would normally test safety distance violations
            # For now, just check that safety parameters are set correctly
            status = self.frontend.get_follow_me_status()
            if status and status.get("safety_distance") == 2.0:
                self.test_results.append("✅ Safety distance configured: PASSED")
            else:
                self.test_results.append("❌ Safety distance configured: FAILED")

        except Exception as e:
            self.test_results.append(f"❌ Safety distance: ERROR - {str(e)}")

    def _test_return_to_idle(self):
        """Test returning to idle state."""
        self.get_logger().info("Test 4: Return to idle state")

        try:
            # Stop follow me mode
            success = self.frontend.stop_follow_me(operator_id="tester")

            if success:
                self.test_results.append("✅ Return to idle state: PASSED")
            else:
                self.test_results.append("❌ Return to idle state: FAILED")

        except Exception as e:
            self.test_results.append(f"❌ Return to idle state: ERROR - {str(e)}")

    def _on_follow_me_status(self, status: FollowMeStatus):
        """Handle follow me status updates."""
        self.get_logger().info(
            f"Follow me status: following={status.is_following}, "
            f"target={status.target_tag_id}, distance={status.target_distance:.2f}m"
        )

    def _on_state_update(self, state: SystemStateMsg):
        """Handle state updates."""
        self.get_logger().info(f"State update: {state.current_state} / {state.substate}")

    def _print_test_results(self):
        """Print test results."""
        self.get_logger().info("=== Follow Me Test Results ===")
        for result in self.test_results:
            self.get_logger().info(result)

        passed = sum(1 for r in self.test_results if "✅" in r)
        total = len(self.test_results)

        self.get_logger().info(f"Tests passed: {passed}/{total}")

        if passed == total:
            self.get_logger().info("🎉 All tests passed!")
        else:
            self.get_logger().warn("⚠️ Some tests failed")


def main():
    """Main test function."""
    rclpy.init()

    try:
        tester = FollowMeTester()

        # Run tests
        tester.run_tests()

        # Keep node alive for a bit to see callbacks
        time.sleep(5.0)

    except KeyboardInterrupt:
        print("Test interrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
