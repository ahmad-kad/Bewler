#!/usr/bin/env python3
"""
Unit tests for Camera Check and Calibration functionality.

Tests the camera validation, calibration, and ROS 2 publishing modules.
"""

import unittest
from unittest.mock import patch, MagicMock
import subprocess
import tempfile
import json
import os

# Add the parent directory to the path for imports
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

# Import the functions from camera validation, calibration and system utilities
from camera.validation import validate_camera_setup
from camera.calibration import (
    calibrate_and_publish,
    publish_calibration_to_ros,
    CalibrationPublisher,
)
from utils.system import (
    run_command,
    run_system_command,
    check_system_setup,
    list_camera_devices,
)
from utils.camera import save_calibration_file, load_calibration_file


class TestCameraCheck(unittest.TestCase):
    """Test cases for camera check functions."""

    def test_run_command_success(self):
        """Test successful command execution."""
        result = run_system_command("echo 'test'")

        self.assertTrue(result[0])  # success should be True
        self.assertIn("test", result[1])  # output should contain 'test'

    def test_run_command_failure(self):
        """Test command execution failure."""
        result = run_command("false", "failing command")

        self.assertFalse(result[0])  # success should be False
        self.assertIsInstance(result[1], str)  # error should be string

    def test_run_command_timeout(self):
        """Test command timeout handling."""
        result = run_command("sleep 10", "slow command")

        self.assertFalse(result[0])  # should fail due to timeout
        self.assertIsInstance(result[1], str)

    @patch("subprocess.run")
    def test_check_system_setup_mocked(self, mock_run):
        """Test system setup check with mocked subprocess."""
        # Mock successful responses for all checks
        mock_run.side_effect = [
            MagicMock(returncode=0, stdout="video0\nvideo1\n"),  # Video devices
            MagicMock(returncode=0, stdout="user video\n"),  # User in video group
            MagicMock(returncode=0, stdout=""),  # v4l2-utils available
            MagicMock(returncode=0, stdout="1\n"),  # Camera enabled
        ]

        results = check_system_setup()

        # Verify all checks passed
        self.assertTrue(results["video_devices"])
        self.assertTrue(results["user_in_video_group"])
        self.assertTrue(results["v4l2_utils"])
        self.assertTrue(results["camera_enabled"])

    @patch("subprocess.run")
    def test_check_system_setup_partial_failure(self, mock_run):
        """Test system setup with some failures."""
        mock_run.side_effect = [
            MagicMock(returncode=0, stdout=""),  # No video devices
            MagicMock(returncode=0, stdout="user video\n"),  # User in video group
            MagicMock(returncode=1, stdout=""),  # v4l2-utils not available
            MagicMock(returncode=0, stdout="0\n"),  # Camera not enabled
        ]

        results = check_system_setup()

        # Verify results
        self.assertFalse(results["video_devices"])
        self.assertTrue(results["user_in_video_group"])
        self.assertFalse(results["v4l2_utils"])
        self.assertFalse(results["camera_enabled"])

    @patch("builtins.print")
    @patch("pathlib.Path.glob")
    def test_list_camera_devices_found(self, mock_glob, _mock_print):
        """Test listing camera devices when devices are found."""
        mock_video0 = MagicMock()
        mock_video0.__str__ = MagicMock(returncode="video0")
        mock_glob.return_value = [mock_video0]

        list_camera_devices()

        # Verify glob was called correctly
        mock_glob.assert_called_once_with("video*")

    @patch("builtins.print")
    @patch("pathlib.Path.glob")
    def test_list_camera_devices_none(self, mock_glob, _mock_print):
        """Test listing camera devices when no devices are found."""
        mock_glob.return_value = []

        list_camera_devices()

        mock_glob.assert_called_once_with("video*")

    @patch("subprocess.run")
    @patch("builtins.print")
    @patch("pathlib.Path.glob")
    def test_list_camera_devices_with_v4l2_info(self, mock_glob, _mock_print, mock_run):
        """Test listing devices with v4l2-ctl info."""
        mock_device = MagicMock()
        mock_device.__str__ = MagicMock(returncode="/dev/video0")
        mock_glob.return_value = [mock_device]

        # Mock successful v4l2-ctl response
        mock_run.return_value = MagicMock(
            returncode=0, stdout="Driver info:\n\tDriver name: uvcvideo\n"
        )

        list_camera_devices()

        # Verify v4l2-ctl was called
        mock_run.assert_called()

    def test_run_command_output_types(self):
        """Test that run_command returns correct types."""
        success, output = run_command("echo 'hello'", "test")

        self.assertIsInstance(success, bool)
        self.assertIsInstance(output, str)

    def test_run_command_with_complex_command(self):
        """Test run_command with shell operators."""
        success, output = run_command("echo 'test' && true", "complex command")

        self.assertTrue(success)
        self.assertIn("test", output)

    @patch("subprocess.run")
    def test_check_system_setup_subprocess_errors(self, mock_run):
        """Test system setup check when subprocess raises exceptions."""
        mock_run.side_effect = subprocess.TimeoutExpired("cmd", 10)

        # Should not crash, should handle the exception
        results = check_system_setup()

        # Results should still be returned (with False values)
        self.assertIsInstance(results, dict)
        self.assertIn("video_devices", results)

    def test_empty_command_handling(self):
        """Test handling of empty commands."""
        success, output = run_command("", "empty command")

        # Empty command should fail
        self.assertFalse(success)
        self.assertIsInstance(output, str)

    # ROS 2 Calibration Publishing Tests
    @patch('camera.calibration.ROS2_AVAILABLE', False)
    def test_publish_calibration_to_ros_no_ros2(self):
        """Test ROS publishing when ROS 2 is not available."""
        calib_data = {
            'camera_matrix': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'distortion_coefficients': [0.0, 0.0, 0.0, 0.0, 0.0],
            'image_width': 640,
            'image_height': 480,
        }

        success = publish_calibration_to_ros(calib_data, 'test_camera')
        self.assertFalse(success)

    @patch('camera.calibration.rclpy')
    @patch('camera.calibration.ROS2_AVAILABLE', True)
    def test_publish_calibration_to_ros_success(self, mock_rclpy):
        """Test successful ROS calibration publishing."""
        # Mock ROS 2 components
        mock_node = MagicMock()
        mock_rclpy.init.return_value = None
        mock_rclpy.ok.return_value = True
        mock_rclpy.shutdown.return_value = None

        with patch('camera.calibration.CalibrationPublisher') as mock_publisher_class:
            mock_publisher = MagicMock()
            mock_publisher.publish_calibration.return_value = True
            mock_publisher_class.return_value = mock_publisher

            calib_data = {
                'camera_matrix': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                'distortion_coefficients': [0.0, 0.0, 0.0, 0.0, 0.0],
                'image_width': 640,
                'image_height': 480,
            }

            success = publish_calibration_to_ros(calib_data, 'test_camera')
            self.assertTrue(success)

            # Verify ROS 2 lifecycle
            mock_rclpy.init.assert_called_once()
            mock_rclpy.spin_once.assert_called_once()
            mock_rclpy.shutdown.assert_called_once()

    def test_calibration_publisher_creation(self):
        """Test CalibrationPublisher node creation."""
        with patch('camera.calibration.ROS2_AVAILABLE', False):
            # Should not crash when ROS 2 not available
            try:
                publisher = CalibrationPublisher()
                # If ROS 2 not available, this should be handled gracefully
            except Exception as e:
                self.fail(f"CalibrationPublisher creation failed: {e}")

    def test_calibrate_and_publish_integration(self):
        """Test the integrated calibrate and publish function."""
        with patch('camera.calibration.calibrate_camera') as mock_calibrate:
            with patch('camera.calibration.publish_calibration_to_ros') as mock_publish:
                # Mock successful calibration
                mock_calibrate.return_value = True
                mock_publish.return_value = True

                # Test with publish_to_ros=True
                success = calibrate_and_publish(
                    camera_index=0,
                    duration=5,
                    publish_to_ros=True,
                    camera_name='test_camera'
                )

                self.assertTrue(success)
                mock_calibrate.assert_called_once()
                mock_publish.assert_called_once()

    def test_calibrate_and_publish_without_ros(self):
        """Test calibrate function without ROS publishing."""
        with patch('camera.calibration.calibrate_camera') as mock_calibrate:
            mock_calibrate.return_value = True

            # Test with publish_to_ros=False
            success = calibrate_and_publish(
                camera_index=0,
                duration=5,
                publish_to_ros=False,
                camera_name='test_camera'
            )

            self.assertTrue(success)
            mock_calibrate.assert_called_once()

    def test_calibration_file_operations(self):
        """Test saving and loading calibration files."""
        with tempfile.TemporaryDirectory() as temp_dir:
            calib_data = {
                'camera_matrix': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                'distortion_coefficients': [0.0, 0.0, 0.0, 0.0, 0.0],
                'image_width': 640,
                'image_height': 480,
                'camera_index': 0,
                'sensor': 'test_sensor',
            }

            camera_name = 'test_camera'

            # Test saving
            filepath = save_calibration_file(calib_data, camera_name, temp_dir)
            self.assertTrue(os.path.exists(filepath))

            # Test loading
            loaded_data = load_calibration_file(camera_name, temp_dir)
            self.assertEqual(loaded_data['camera_index'], 0)
            self.assertEqual(loaded_data['sensor'], 'test_sensor')
            self.assertEqual(loaded_data['image_width'], 640)

    def test_calibration_file_not_found(self):
        """Test loading non-existent calibration file."""
        with tempfile.TemporaryDirectory() as temp_dir:
            with self.assertRaises(FileNotFoundError):
                load_calibration_file('nonexistent_camera', temp_dir)

    @patch('camera.calibration.rclpy')
    @patch('camera.calibration.ROS2_AVAILABLE', True)
    def test_publish_calibration_error_handling(self, mock_rclpy):
        """Test error handling in ROS calibration publishing."""
        mock_rclpy.init.side_effect = Exception("ROS init failed")

        calib_data = {
            'camera_matrix': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'distortion_coefficients': [0.0, 0.0, 0.0, 0.0, 0.0],
            'image_width': 640,
            'image_height': 480,
        }

        success = publish_calibration_to_ros(calib_data, 'test_camera')
        self.assertFalse(success)


if __name__ == "__main__":
    unittest.main()
