#!/usr/bin/env python3
"""
Unit tests for Camera Check functionality.

Tests the camera_check.py module.
"""

import unittest
from unittest.mock import patch, MagicMock
import subprocess

# Add the parent directory to the path for imports
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

# Import the functions from camera validation and system utilities
from camera.validation import validate_camera_setup
from utils.system import (
    run_command,
    run_system_command,
    check_system_setup,
    list_camera_devices,
)


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


if __name__ == "__main__":
    unittest.main()
