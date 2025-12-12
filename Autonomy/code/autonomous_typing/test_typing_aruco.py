#!/usr/bin/env python3
"""
Test script for Typing ArUco Detection.

Tests the ArUco detection functionality for autonomous typing missions
using any ArUco tag IDs detected in the scene.
"""

import time

import rclpy
from autonomy_autonomous_typing.aruco_detection import TypingArUcoDetector
from autonomy_interfaces.srv import DetectAruco
from rclpy.node import Node


class TypingArUcoTester(Node):
    """Test node for typing ArUco detection."""

    def __init__(self):
        super().__init__("typing_aruco_tester")

        # Initialize typing ArUco detector
        self.detector = TypingArUcoDetector(self)

        # Test state
        self.test_completed = False
        self.test_results = []

        self.get_logger().info("Typing ArUco Tester initialized")

    def run_tests(self):
        """Run all typing ArUco detection tests."""
        self.get_logger().info("Starting Typing ArUco Detection tests...")

        # Test 1: Basic ArUco detection
        self._test_basic_detection()

        # Test 2: Minimum tag requirement
        self._test_minimum_tags()

        # Test 3: Alignment quality assessment
        self._test_alignment_quality()

        # Test 4: Keyboard size estimation
        self._test_keyboard_estimation()

        # Test 5: Service integration
        self._test_service_integration()

        # Print results
        self._print_test_results()

    def _test_basic_detection(self):
        """Test basic ArUco detection for typing."""
        self.get_logger().info("Test 1: Basic ArUco Detection")

        try:
            # Test with any tags detected
            result = self.detector.detect_typing_tags(target_depth=0.3, timeout=5.0, max_detection_distance=3.0)

            if result["success"]:
                self.test_results.append("✅ Basic ArUco detection: PASSED")
                self.get_logger().info(f"Detected {len(result['detected_tag_ids'])} tags")
                self.get_logger().info(f"Alignment quality: {result['alignment_quality']:.2f}")
            else:
                self.test_results.append("❌ Basic ArUco detection: FAILED")
                self.get_logger().error(f"Error: {result['message']}")

        except Exception as e:
            self.test_results.append(f"❌ Basic ArUco detection: ERROR - {str(e)}")

    def _test_minimum_tags(self):
        """Test minimum tag requirement."""
        self.get_logger().info("Test 2: Minimum Tag Requirement")

        try:
            # Test with minimum tag requirement
            result = self.detector.detect_typing_tags_with_minimum(min_tags=3, target_depth=0.3, timeout=3.0)

            if result["success"]:
                detected_count = len(result["detected_tag_ids"])
                if detected_count >= 3:
                    self.test_results.append("✅ Minimum tag requirement: PASSED")
                    self.get_logger().info(f"Detected {detected_count} tags (>= 3)")
                else:
                    self.test_results.append("❌ Minimum tag requirement: FAILED")
                    self.get_logger().error(f"Only {detected_count} tags detected")
            else:
                self.test_results.append("❌ Minimum tag requirement: FAILED")
                self.get_logger().error(f"Error: {result['message']}")

        except Exception as e:
            self.test_results.append(f"❌ Minimum tag requirement: ERROR - {str(e)}")

    def _test_alignment_quality(self):
        """Test alignment quality assessment."""
        self.get_logger().info("Test 3: Alignment Quality Assessment")

        try:
            # Test alignment quality
            result = self.detector.detect_typing_tags(target_depth=0.3, timeout=3.0)

            if result["success"]:
                quality = result["alignment_quality"]
                mission_ready = result["mission_ready"]

                if quality >= 0.6:  # Minimum quality threshold
                    self.test_results.append(f"✅ Alignment quality: PASSED (quality: {quality:.2f})")
                else:
                    self.test_results.append(f"❌ Alignment quality: FAILED (quality: {quality:.2f})")

                if mission_ready:
                    self.test_results.append("✅ Mission readiness: PASSED")
                else:
                    self.test_results.append("❌ Mission readiness: FAILED")

            else:
                self.test_results.append("❌ Alignment quality: FAILED - Detection failed")

        except Exception as e:
            self.test_results.append(f"❌ Alignment quality: ERROR - {str(e)}")

    def _test_keyboard_estimation(self):
        """Test keyboard size estimation."""
        self.get_logger().info("Test 4: Keyboard Size Estimation")

        try:
            # Test keyboard estimation
            result = self.detector.detect_typing_tags(target_depth=0.3, timeout=3.0)

            if result["success"]:
                keyboard_info = result.get("keyboard_info", {})
                if keyboard_info:
                    size = keyboard_info.get("estimated_keyboard_size", 0)
                    aspect_ratio = keyboard_info.get("aspect_ratio", 0)

                    self.test_results.append(
                        f"✅ Keyboard estimation: PASSED (size: {size:.2f}m, ratio: {aspect_ratio:.2f})"
                    )
                    self.get_logger().info(f"Keyboard size: {size:.2f}m")
                    self.get_logger().info(f"Aspect ratio: {aspect_ratio:.2f}")
                else:
                    self.test_results.append("❌ Keyboard estimation: FAILED - No keyboard info")
            else:
                self.test_results.append("❌ Keyboard estimation: FAILED - Detection failed")

        except Exception as e:
            self.test_results.append(f"❌ Keyboard estimation: ERROR - {str(e)}")

    def _test_service_integration(self):
        """Test service integration."""
        self.get_logger().info("Test 5: Service Integration")

        try:
            # Test direct service call
            client = self.create_client(DetectAruco, "/aruco_detection/detect")

            if not client.wait_for_service(timeout_sec=5.0):
                self.test_results.append("❌ Service integration: FAILED - Service not available")
                return

            # Create request
            request = DetectAruco.Request()
            request.target_tag_ids = []  # Detect any tags
            request.detection_timeout = 3.0
            request.require_distance_estimate = True
            request.max_detection_distance = 3.0
            request.calculate_alignment = True
            request.target_depth = 0.3
            request.mission_type = "AUTONOMOUS_TYPING"

            # Call service
            future = client.call_async(request)
            self.get_executor().spin_until_future_complete(future, timeout_sec=10.0)

            if future.done():
                response = future.result()
                if response.success:
                    self.test_results.append("✅ Service integration: PASSED")
                    self.get_logger().info(f"Service call successful: {len(response.detected_tag_ids)} tags")
                else:
                    self.test_results.append("❌ Service integration: FAILED - Service call failed")
            else:
                self.test_results.append("❌ Service integration: FAILED - Service call timed out")

        except Exception as e:
            self.test_results.append(f"❌ Service integration: ERROR - {str(e)}")

    def _print_test_results(self):
        """Print test results."""
        self.get_logger().info("=== Typing ArUco Detection Test Results ===")
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
        tester = TypingArUcoTester()

        # Run tests
        tester.run_tests()

        # Keep node alive for a bit
        time.sleep(2.0)

    except KeyboardInterrupt:
        print("Test interrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
