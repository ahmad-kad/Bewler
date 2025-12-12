#!/usr/bin/env python3
"""
Test script for Mission-Specific ArUco Detection.

Tests the enhanced ArUco detection functionality for autonomous typing
and USB connection missions with auto-alignment capabilities.
"""

import time

import rclpy
from autonomy_interfaces.srv import DetectMissionAruco
from autonomy_state_machine.mission_aruco_detector import MissionArUcoDetector
from rclpy.node import Node


class MissionArUcoTester(Node):
    """Test node for mission-specific ArUco detection."""

    def __init__(self):
        super().__init__("mission_aruco_tester")

        # Initialize mission ArUco detector
        self.detector = MissionArUcoDetector(self)

        # Test state
        self.test_completed = False
        self.test_results = []

        self.get_logger().info("Mission ArUco Tester initialized")

    def run_tests(self):
        """Run all mission ArUco detection tests."""
        self.get_logger().info("Starting Mission ArUco Detection tests...")

        # Test 1: Autonomous Typing Mission
        self._test_autonomous_typing()

        # Test 2: USB Connection Mission
        self._test_usb_connection()

        # Test 3: Panel Operations Mission
        self._test_panel_operations()

        # Test 4: Alignment Quality Assessment
        self._test_alignment_quality()

        # Print results
        self._print_test_results()

    def _test_autonomous_typing(self):
        """Test autonomous typing mission detection."""
        self.get_logger().info("Test 1: Autonomous Typing Mission")

        try:
            # Test with 4 corner tags (keyboard layout)
            result = self.detector.detect_mission_tags(
                mission_type="AUTONOMOUS_TYPING",
                required_tag_ids=[1, 2, 3, 4],  # 4 corner tags
                optional_tag_ids=[5, 6],  # Additional tags for better alignment
                target_depth=0.3,  # 30cm from keyboard
                timeout=5.0,
            )

            if result["success"]:
                self.test_results.append("✅ Autonomous typing detection: PASSED")
                self.get_logger().info(f"Alignment quality: {result['alignment_quality']:.2f}")
                self.get_logger().info(f"Mission ready: {result['mission_ready']}")
            else:
                self.test_results.append("❌ Autonomous typing detection: FAILED")
                self.get_logger().error(f"Error: {result['message']}")

        except Exception as e:
            self.test_results.append(f"❌ Autonomous typing detection: ERROR - {str(e)}")

    def _test_usb_connection(self):
        """Test USB connection mission detection."""
        self.get_logger().info("Test 2: USB Connection Mission")

        try:
            # Test with 2 tags (USB port alignment)
            result = self.detector.detect_mission_tags(
                mission_type="USB_CONNECTION",
                required_tag_ids=[10, 11],  # 2 tags for USB port
                optional_tag_ids=[12],  # Additional tag for better alignment
                target_depth=0.2,  # 20cm from USB port
                timeout=3.0,
            )

            if result["success"]:
                self.test_results.append("✅ USB connection detection: PASSED")
                self.get_logger().info(f"Alignment quality: {result['alignment_quality']:.2f}")
                self.get_logger().info(f"Mission ready: {result['mission_ready']}")
            else:
                self.test_results.append("❌ USB connection detection: FAILED")
                self.get_logger().error(f"Error: {result['message']}")

        except Exception as e:
            self.test_results.append(f"❌ USB connection detection: ERROR - {str(e)}")

    def _test_panel_operations(self):
        """Test panel operations mission detection."""
        self.get_logger().info("Test 3: Panel Operations Mission")

        try:
            # Test with 4 corner tags (panel layout)
            result = self.detector.detect_mission_tags(
                mission_type="PANEL_OPERATIONS",
                required_tag_ids=[20, 21, 22, 23],  # 4 corner tags
                optional_tag_ids=[24, 25],  # Additional tags
                target_depth=0.4,  # 40cm from panel
                timeout=4.0,
            )

            if result["success"]:
                self.test_results.append("✅ Panel operations detection: PASSED")
                self.get_logger().info(f"Alignment quality: {result['alignment_quality']:.2f}")
                self.get_logger().info(f"Mission ready: {result['mission_ready']}")
            else:
                self.test_results.append("❌ Panel operations detection: FAILED")
                self.get_logger().error(f"Error: {result['message']}")

        except Exception as e:
            self.test_results.append(f"❌ Panel operations detection: ERROR - {str(e)}")

    def _test_alignment_quality(self):
        """Test alignment quality assessment."""
        self.get_logger().info("Test 4: Alignment Quality Assessment")

        try:
            # Test with various tag configurations
            test_cases = [
                {
                    "name": "Perfect alignment",
                    "tags": [1, 2, 3, 4],
                    "expected_quality": 0.9,
                },
                {
                    "name": "Good alignment",
                    "tags": [1, 2, 3],  # Missing one tag
                    "expected_quality": 0.7,
                },
                {
                    "name": "Poor alignment",
                    "tags": [1, 2],  # Missing two tags
                    "expected_quality": 0.5,
                },
            ]

            for case in test_cases:
                result = self.detector.detect_mission_tags(
                    mission_type="AUTONOMOUS_TYPING",
                    required_tag_ids=case["tags"],
                    target_depth=0.3,
                    timeout=2.0,
                )

                if result["success"]:
                    quality = result["alignment_quality"]
                    expected = case["expected_quality"]
                    if quality >= expected * 0.8:  # Allow 20% tolerance
                        self.test_results.append(f"✅ {case['name']}: PASSED (quality: {quality:.2f})")
                    else:
                        self.test_results.append(f"❌ {case['name']}: FAILED (quality: {quality:.2f} < {expected:.2f})")
                else:
                    self.test_results.append(f"❌ {case['name']}: FAILED - {result['message']}")

        except Exception as e:
            self.test_results.append(f"❌ Alignment quality assessment: ERROR - {str(e)}")

    def _print_test_results(self):
        """Print test results."""
        self.get_logger().info("=== Mission ArUco Detection Test Results ===")
        for result in self.test_results:
            self.get_logger().info(result)

        passed = sum(1 for r in self.test_results if "✅" in r)
        total = len(self.test_results)

        self.get_logger().info(f"Tests passed: {passed}/{total}")

        if passed == total:
            self.get_logger().info("🎉 All tests passed!")
        else:
            self.get_logger().warn("⚠️ Some tests failed")

    def test_service_calls(self):
        """Test direct service calls."""
        self.get_logger().info("Testing direct service calls...")

        # Create service client
        client = self.create_client(DetectMissionAruco, "/mission_aruco/detect")

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service not available")
            return

        # Test autonomous typing service call
        request = DetectMissionAruco.Request()
        request.mission_type = "AUTONOMOUS_TYPING"
        request.required_tag_ids = [1, 2, 3, 4]
        request.optional_tag_ids = [5, 6]
        request.detection_timeout = 3.0
        request.target_depth = 0.3
        request.max_detection_distance = 5.0
        request.require_all_tags = False
        request.min_alignment_quality = 0.7

        try:
            future = client.call_async(request)
            self.get_executor().spin_until_future_complete(future, timeout_sec=10.0)

            if future.done():
                response = future.result()
                self.get_logger().info(f"Service call result: {response.success}")
                self.get_logger().info(f"Message: {response.message}")
                self.get_logger().info(f"Alignment quality: {response.alignment_quality}")
            else:
                self.get_logger().error("Service call timed out")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main():
    """Main test function."""
    rclpy.init()

    try:
        tester = MissionArUcoTester()

        # Run tests
        tester.run_tests()

        # Test service calls
        tester.test_service_calls()

        # Keep node alive for a bit
        time.sleep(2.0)

    except KeyboardInterrupt:
        print("Test interrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
