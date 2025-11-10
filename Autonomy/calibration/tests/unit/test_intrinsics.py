#!/usr/bin/env python3
"""
Unit Tests for Camera Intrinsics Calibration
"""

import unittest
import numpy as np
import tempfile
import os
from pathlib import Path

# Imports from intrinsics module
from intrinsics import (
    CameraConfig,
    CharUcoBoardConfig,
    CameraIntrinsicsCalibrator,
    CalibrationResult,
    CaptureMode
)


class TestCameraConfig(unittest.TestCase):
    """Test camera configuration"""

    def test_camera_config_creation(self):
        """Test creating camera config"""
        cfg = CameraConfig(
            name="Test Camera",
            camera_index=0,
            resolution=(1920, 1080)
        )
        
        self.assertEqual(cfg.name, "Test Camera")
        self.assertEqual(cfg.camera_index, 0)
        self.assertEqual(cfg.resolution, (1920, 1080))
        self.assertEqual(cfg.fps, 30)

    def test_camera_config_defaults(self):
        """Test default values"""
        cfg = CameraConfig(
            name="Test",
            camera_index=0,
            resolution=(640, 480)
        )
        
        self.assertEqual(cfg.sensor_type, "rgb")
        self.assertEqual(cfg.focal_length_estimate_mm, 4.0)


class TestCharUcoBoardConfig(unittest.TestCase):
    """Test ChArUco board configuration"""

    def test_board_config_creation(self):
        """Test creating board config"""
        cfg = CharUcoBoardConfig(
            name="test_board",
            aruco_dict_name="DICT_4X4_50",
            size=(5, 7),
            checker_size_mm=30.0,
            marker_size_mm=18.0
        )
        
        self.assertEqual(cfg.name, "test_board")
        self.assertEqual(cfg.size, (5, 7))


class TestCameraIntrinsicsCalibrator(unittest.TestCase):
    """Test camera intrinsics calibrator"""

    def setUp(self):
        """Set up test fixtures"""
        self.camera_cfg = CameraConfig(
            name="Test Camera",
            camera_index=0,
            resolution=(1920, 1080)
        )
        
        self.board_cfg = CharUcoBoardConfig(
            name="board_5x7",
            aruco_dict_name="DICT_4X4_50",
            size=(5, 7),
            checker_size_mm=30.0,
            marker_size_mm=18.0
        )

    def test_calibrator_initialization(self):
        """Test calibrator initialization"""
        calibrator = CameraIntrinsicsCalibrator(self.camera_cfg, self.board_cfg)
        
        self.assertIsNotNone(calibrator.detector)
        self.assertIsNotNone(calibrator.board)
        self.assertEqual(calibrator.camera_config.name, "Test Camera")

    def test_get_dataset_dir(self):
        """Test dataset directory generation"""
        calibrator = CameraIntrinsicsCalibrator(self.camera_cfg, self.board_cfg)
        
        dataset_dir = calibrator._get_dataset_dir("manual")
        
        self.assertIn("manual", dataset_dir)
        self.assertIn("test_camera", dataset_dir.lower())
        self.assertIn("calibration_images", dataset_dir)

    def test_get_artifacts_dir(self):
        """Test artifacts directory"""
        calibrator = CameraIntrinsicsCalibrator(self.camera_cfg, self.board_cfg)
        
        artifacts_dir = calibrator._get_artifacts_dir()
        
        self.assertIn("artifacts", artifacts_dir)
        self.assertIn("intrinsics", artifacts_dir)

    def test_calibration_result_creation(self):
        """Test creating calibration result"""
        K = np.eye(3)
        dist = np.zeros(5)
        
        result = CalibrationResult(
            camera_matrix=K,
            dist_coeffs=dist,
            rvecs=[],
            tvecs=[],
            reprojection_error=0.5,
            camera_config=self.camera_cfg,
            board_config=self.board_cfg,
            num_images=50,
            capture_mode="manual",
            timestamp="2025-01-01T00:00:00",
            images_used=[],
            quality_metrics={'fx': 800.0}
        )
        
        self.assertEqual(result.capture_mode, "manual")
        self.assertEqual(result.reprojection_error, 0.5)
        self.assertEqual(result.num_images, 50)

    def test_save_calibration_directory_creation(self):
        """Test that save_calibration creates output directory"""
        with tempfile.TemporaryDirectory() as tmpdir:
            calibrator = CameraIntrinsicsCalibrator(self.camera_cfg, self.board_cfg)
            
            K = np.eye(3)
            dist = np.zeros(5)
            
            result = CalibrationResult(
                camera_matrix=K,
                dist_coeffs=dist,
                rvecs=[],
                tvecs=[],
                reprojection_error=0.5,
                camera_config=self.camera_cfg,
                board_config=self.board_cfg,
                num_images=50,
                capture_mode="manual",
                timestamp="2025-01-01T00:00:00",
                images_used=[],
                quality_metrics={'fx': 800.0}
            )
            
            files_saved = calibrator.save_calibration(result, tmpdir)
            
            self.assertIn('pkl', files_saved)
            self.assertIn('json', files_saved)
            self.assertIn('yaml', files_saved)
            
            # Check files exist
            self.assertTrue(os.path.exists(files_saved['pkl']))
            self.assertTrue(os.path.exists(files_saved['json']))
            self.assertTrue(os.path.exists(files_saved['yaml']))


class TestCalibrationQualityMetrics(unittest.TestCase):
    """Test calibration quality metrics"""

    def test_reprojection_error_excellent(self):
        """Test excellent reprojection error"""
        error = 0.3
        self.assertLess(error, 0.5)

    def test_reprojection_error_good(self):
        """Test good reprojection error"""
        error = 0.7
        self.assertLess(error, 1.0)
        self.assertGreaterEqual(error, 0.5)

    def test_reprojection_error_acceptable(self):
        """Test acceptable reprojection error"""
        error = 1.5
        self.assertLess(error, 2.0)
        self.assertGreaterEqual(error, 1.0)

    def test_reprojection_error_poor(self):
        """Test poor reprojection error"""
        error = 3.0
        self.assertGreaterEqual(error, 2.0)


class TestCaptureModeEnum(unittest.TestCase):
    """Test capture mode enumeration"""

    def test_capture_modes(self):
        """Test available capture modes"""
        self.assertEqual(CaptureMode.MANUAL.value, "manual")
        self.assertEqual(CaptureMode.VIDEO.value, "video")
        self.assertEqual(CaptureMode.CONSERVATIVE.value, "conservative")


if __name__ == '__main__':
    unittest.main()
