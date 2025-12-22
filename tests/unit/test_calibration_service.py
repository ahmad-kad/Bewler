#!/usr/bin/env python3
"""
Unit tests for Calibration Service ROS 2 Node.

Tests the calibration service that provides intrinsics to distributed nodes.
"""

import unittest
from unittest.mock import patch, MagicMock
import tempfile
import json
import os

# Add the parent directory to the path for imports
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

# Import the calibration service
from camera.calibration_service import CalibrationService


class TestCalibrationService(unittest.TestCase):
    """Test cases for calibration service ROS 2 node."""

    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        """Clean up test fixtures."""
        import shutil
        shutil.rmtree(self.temp_dir)

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', False)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', False)
    def test_service_creation_missing_dependencies(self, mock_rclpy):
        """Test service creation when dependencies are missing."""
        with self.assertRaises(RuntimeError):
            CalibrationService()

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', True)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', False)
    def test_service_creation_fallback_interface(self, mock_rclpy):
        """Test service creation with fallback to standard interface."""
        # Mock ROS node
        mock_node = MagicMock()
        mock_rclpy.node.Node.return_value = mock_node

        service = CalibrationService()

        # Verify fallback Trigger service was created
        self.assertIsNotNone(service.service)
        mock_node.create_service.assert_called()

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', True)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', True)
    def test_service_creation_custom_interface(self, mock_rclpy):
        """Test service creation with custom interface."""
        # Mock ROS node
        mock_node = MagicMock()
        mock_rclpy.node.Node.return_value = mock_node

        service = CalibrationService()

        # Verify custom service was created
        self.assertIsNotNone(service.service)
        mock_node.create_service.assert_called()

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', True)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', True)
    def test_load_calibration_data_success(self, mock_rclpy):
        """Test successful calibration data loading."""
        # Mock ROS node
        mock_node = MagicMock()
        mock_rclpy.node.Node.return_value = mock_node

        service = CalibrationService()

        # Create test calibration file
        calib_file = os.path.join(self.temp_dir, 'test_camera.json')
        calib_data = {
            'camera_matrix': [1000.0, 0.0, 320.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 1.0],
            'distortion_coefficients': [0.1, -0.2, 0.0, 0.0, 0.0],
            'image_width': 640,
            'image_height': 480,
            'sensor': 'imx219',
        }

        with open(calib_file, 'w') as f:
            json.dump(calib_data, f)

        # Set calibration directory
        service.calibration_dir = self.temp_dir

        # Test loading
        loaded_data = service.load_calibration_data('test_camera')

        # Verify data was loaded correctly
        self.assertIsNotNone(loaded_data)
        self.assertEqual(loaded_data['sensor'], 'imx219')
        self.assertEqual(loaded_data['image_width'], 640)

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', True)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', True)
    def test_load_calibration_data_not_found(self, mock_rclpy):
        """Test calibration data loading when file doesn't exist."""
        # Mock ROS node
        mock_node = MagicMock()
        mock_rclpy.node.Node.return_value = mock_node

        service = CalibrationService()
        service.calibration_dir = self.temp_dir

        # Test loading non-existent file
        loaded_data = service.load_calibration_data('nonexistent_camera')

        # Should return None
        self.assertIsNone(loaded_data)

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', True)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', True)
    def test_calib_data_to_msg_conversion(self, mock_rclpy):
        """Test conversion of calibration data to CameraInfo message."""
        from sensor_msgs.msg import CameraInfo

        # Mock ROS node
        mock_node = MagicMock()
        mock_rclpy.node.Node.return_value = mock_node

        service = CalibrationService()

        # Test calibration data
        calib_data = {
            'camera_matrix': [1000.0, 0.0, 320.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 1.0],
            'distortion_coefficients': [0.1, -0.2, 0.0, 0.0, 0.0],
            'image_width': 640,
            'image_height': 480,
            'rectification_matrix': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'projection_matrix': [1000.0, 0.0, 320.0, 0.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        }

        # Convert to message
        msg = service.calib_data_to_msg(calib_data, 'test_camera')

        # Verify message contents
        self.assertEqual(msg.width, 640)
        self.assertEqual(msg.height, 480)
        self.assertEqual(msg.distortion_model, 'plumb_bob')
        self.assertEqual(msg.d, [0.1, -0.2, 0.0, 0.0, 0.0])
        self.assertEqual(msg.k, [1000.0, 0.0, 320.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 1.0])
        self.assertEqual(msg.r, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(msg.p, [1000.0, 0.0, 320.0, 0.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0])
        self.assertEqual(msg.header.frame_id, 'imx219_test_camera_link')

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', True)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', True)
    def test_get_calibration_callback_success(self, mock_rclpy):
        """Test successful calibration service callback."""
        # Mock ROS node and service
        mock_node = MagicMock()
        mock_rclpy.node.Node.return_value = mock_node

        service = CalibrationService()
        service.calibration_dir = self.temp_dir

        # Create test calibration file
        calib_file = os.path.join(self.temp_dir, 'test_camera.json')
        calib_data = {
            'camera_matrix': [1000.0, 0.0, 320.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 1.0],
            'distortion_coefficients': [0.1, -0.2, 0.0, 0.0, 0.0],
            'image_width': 640,
            'image_height': 480,
        }

        with open(calib_file, 'w') as f:
            json.dump(calib_data, f)

        # Mock request
        mock_request = MagicMock()
        mock_request.camera_name = 'test_camera'

        mock_response = MagicMock()

        # Call callback
        response = service.get_calibration_callback(mock_request, mock_response)

        # Verify response
        self.assertTrue(response.success)
        self.assertEqual(response.error_message, "")
        self.assertIsNotNone(response.camera_info)

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', True)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', True)
    def test_get_calibration_callback_not_found(self, mock_rclpy):
        """Test calibration service callback when calibration not found."""
        # Mock ROS node and service
        mock_node = MagicMock()
        mock_rclpy.node.Node.return_value = mock_node

        service = CalibrationService()
        service.calibration_dir = self.temp_dir

        # Mock request for non-existent camera
        mock_request = MagicMock()
        mock_request.camera_name = 'nonexistent_camera'

        mock_response = MagicMock()

        # Call callback
        response = service.get_calibration_callback(mock_request, mock_response)

        # Verify response
        self.assertFalse(response.success)
        self.assertIn('not found', response.error_message)

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', True)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', False)
    def test_get_calibration_trigger_callback(self, mock_rclpy):
        """Test calibration service with Trigger interface fallback."""
        # Mock ROS node and service
        mock_node = MagicMock()
        mock_rclpy.node.Node.return_value = mock_node

        service = CalibrationService()
        service.calibration_dir = self.temp_dir

        # Create test calibration file
        calib_file = os.path.join(self.temp_dir, 'camera.json')  # Default name
        calib_data = {
            'camera_matrix': [1000.0, 0.0, 320.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 1.0],
            'distortion_coefficients': [0.1, -0.2, 0.0, 0.0, 0.0],
            'image_width': 640,
            'image_height': 480,
        }

        with open(calib_file, 'w') as f:
            json.dump(calib_data, f)

        # Mock request and response
        mock_request = MagicMock()
        mock_response = MagicMock()

        # Call trigger callback
        response = service.get_calibration_trigger_callback(mock_request, mock_response)

        # Verify response
        self.assertTrue(response.success)
        self.assertIn('published', response.message)

        # Verify message was published
        service.calib_pub.publish.assert_called_once()

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', True)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', True)
    def test_list_available_calibrations(self, mock_rclpy):
        """Test listing available calibrations."""
        # Mock ROS node
        mock_node = MagicMock()
        mock_rclpy.node.Node.return_value = mock_node

        service = CalibrationService()
        service.calibration_dir = self.temp_dir

        # Create test calibration files
        cameras = ['camera1', 'camera2', 'camera3']
        for camera in cameras:
            calib_file = os.path.join(self.temp_dir, f'{camera}.json')
            with open(calib_file, 'w') as f:
                json.dump({'test': 'data'}, f)

        # List calibrations
        available = service.list_available_calibrations()

        # Verify all cameras are listed
        self.assertEqual(len(available), 3)
        for camera in cameras:
            self.assertIn(camera, available)

    @patch('camera.calibration_service.rclpy')
    @patch('camera.calibration_service.STD_SERVICES_AVAILABLE', True)
    @patch('camera.calibration_service.CUSTOM_INTERFACE_AVAILABLE', True)
    def test_publish_all_calibrations(self, mock_rclpy):
        """Test publishing all available calibrations."""
        # Mock ROS node
        mock_node = MagicMock()
        mock_rclpy.node.Node.return_value = mock_node

        service = CalibrationService()
        service.calibration_dir = self.temp_dir

        # Create test calibration files
        cameras = ['camera1', 'camera2']
        for camera in cameras:
            calib_file = os.path.join(self.temp_dir, f'{camera}.json')
            calib_data = {
                'camera_matrix': [1000.0, 0.0, 320.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 1.0],
                'distortion_coefficients': [0.0, 0.0, 0.0, 0.0, 0.0],
                'image_width': 640,
                'image_height': 480,
            }
            with open(calib_file, 'w') as f:
                json.dump(calib_data, f)

        # Publish all calibrations
        service.publish_all_calibrations()

        # Verify messages were published
        self.assertEqual(service.calib_pub.publish.call_count, 2)


if __name__ == "__main__":
    unittest.main()
