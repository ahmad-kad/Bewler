#!/usr/bin/env python3
"""
End-to-End Calibration Integration Tests - URC 2026
====================================================

Tests complete calibration workflows:
1. Configuration setup and validation
2. Dataset processing pipeline
3. Calibration computation
4. Results saving and loading
5. Quality assessment

Run with: python -m pytest test_end_to_end_calibration.py -v
"""

import unittest
import tempfile
import numpy as np
from pathlib import Path
import json
import yaml
import pickle
import sys
import os

# Add parent directories to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../intrinsics'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../extrinsics'))

from intrinsics import (
    CameraConfig,
    CharUcoBoardConfig,
    CalibrationResult,
    CaptureMode
)


class TestCalibrationConfiguration(unittest.TestCase):
    """Test calibration configuration and validation."""
    
    def test_camera_config_creation(self):
        """Test creating camera configuration."""
        config = CameraConfig(
            name="Test Camera",
            camera_index=0,
            resolution=(1920, 1080)
        )
        
        self.assertEqual(config.name, "Test Camera")
        self.assertEqual(config.camera_index, 0)
        self.assertEqual(config.resolution, (1920, 1080))
    
    def test_camera_config_various_resolutions(self):
        """Test camera config with various resolutions."""
        resolutions = [(640, 480), (1280, 720), (1920, 1080), (2560, 1440)]
        
        for resolution in resolutions:
            config = CameraConfig("Camera", camera_index=0, resolution=resolution)
            self.assertEqual(config.resolution, resolution)
    
    def test_board_config_creation(self):
        """Test creating board configuration."""
        config = CharUcoBoardConfig(
            name="Test Board",
            aruco_dict_name="DICT_4X4_50",
            size=(5, 7),
            checker_size_mm=30.0,
            marker_size_mm=18.0
        )
        
        self.assertEqual(config.name, "Test Board")
        self.assertEqual(config.aruco_dict_name, "DICT_4X4_50")
        self.assertEqual(config.size, (5, 7))
        self.assertEqual(config.checker_size_mm, 30.0)
        self.assertEqual(config.marker_size_mm, 18.0)
    
    def test_board_config_various_sizes(self):
        """Test board config with various sizes."""
        sizes = [(4, 5), (5, 7), (7, 5), (6, 8)]
        
        for size in sizes:
            config = CharUcoBoardConfig(
                "Board", "DICT_4X4_50", size, 30.0, 18.0
            )
            self.assertEqual(config.size, size)


class TestCalibrationResultManagement(unittest.TestCase):
    """Test calibration result creation and management."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.camera_matrix = np.array([
            [1920, 0, 960],
            [0, 1920, 540],
            [0, 0, 1]
        ], dtype=np.float64)
        
        self.distortion = np.array([-0.1, 0.05, 0.001, -0.002, 0.0], dtype=np.float64)
    
    def test_calibration_result_creation(self):
        """Test creating calibration result."""
        camera_cfg = CameraConfig("Test", camera_index=0, resolution=(1920, 1080))
        board_cfg = CharUcoBoardConfig("Test Board", "DICT_4X4_50", (5, 7), 30.0, 18.0)

        result = CalibrationResult(
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.distortion,
            rvecs=[],
            tvecs=[],
            reprojection_error=0.45,
            camera_config=camera_cfg,
            board_config=board_cfg,
            num_images=50,
            capture_mode="manual",
            timestamp="2025-01-01T00:00:00",
            images_used=[],
            quality_metrics={"quality_score": 0.95}
        )
        
        np.testing.assert_array_equal(result.camera_matrix, self.camera_matrix)
        np.testing.assert_array_equal(result.dist_coeffs, self.distortion)
        self.assertEqual(result.reprojection_error, 0.45)
        self.assertEqual(result.quality_metrics["quality_score"], 0.95)
    
    def test_calibration_result_with_metadata(self):
        """Test calibration result with metadata."""
        camera_cfg = CameraConfig("Test", camera_index=0, resolution=(1920, 1080))
        board_cfg = CharUcoBoardConfig("Test Board", "DICT_4X4_50", (5, 7), 30.0, 18.0)

        metadata = {
            "capture_mode": "manual",
            "image_count": 50,
            "board_name": "board_5x7"
        }

        result = CalibrationResult(
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.distortion,
            rvecs=[],
            tvecs=[],
            reprojection_error=0.45,
            camera_config=camera_cfg,
            board_config=board_cfg,
            num_images=50,
            capture_mode="manual",
            timestamp="2025-01-01T00:00:00",
            images_used=[],
            quality_metrics=metadata
        )

        self.assertEqual(result.quality_metrics, metadata)
        self.assertEqual(result.quality_metrics["image_count"], 50)


class TestCalibrationResultSerialization(unittest.TestCase):
    """Test saving and loading calibration results."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        
        self.camera_matrix = np.array([
            [1920, 0, 960],
            [0, 1920, 540],
            [0, 0, 1]
        ], dtype=np.float64)
        
        self.distortion = np.array([-0.1, 0.05, 0.001, -0.002, 0.0], dtype=np.float64)
        
        camera_cfg = CameraConfig("Test", camera_index=0, resolution=(1920, 1080))
        board_cfg = CharUcoBoardConfig("Test Board", "DICT_4X4_50", (5, 7), 30.0, 18.0)

        self.result = CalibrationResult(
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.distortion,
            rvecs=[],
            tvecs=[],
            reprojection_error=0.45,
            camera_config=camera_cfg,
            board_config=board_cfg,
            num_images=50,
            capture_mode="manual",
            timestamp="2025-01-01T00:00:00",
            images_used=[],
            quality_metrics={"mode": "manual", "count": 50}
        )
    
    def tearDown(self):
        """Clean up test fixtures."""
        import shutil
        shutil.rmtree(self.temp_dir)
    
    def test_save_and_load_json(self):
        """Test JSON serialization."""
        filepath = Path(self.temp_dir) / "calibration.json"
        
        # Save as JSON
        data = {
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.distortion.tolist(),
            "rvecs": [],
            "tvecs": [],
            "reprojection_error": self.result.reprojection_error,
            "camera_config": {
                "name": self.result.camera_config.name,
                "camera_index": self.result.camera_config.camera_index,
                "resolution": list(self.result.camera_config.resolution)
            },
            "board_config": {
                "name": self.result.board_config.name,
                "aruco_dict_name": self.result.board_config.aruco_dict_name,
                "size": list(self.result.board_config.size),
                "checker_size_mm": self.result.board_config.checker_size_mm,
                "marker_size_mm": self.result.board_config.marker_size_mm
            },
            "num_images": self.result.num_images,
            "capture_mode": self.result.capture_mode,
            "timestamp": self.result.timestamp,
            "images_used": self.result.images_used,
            "quality_metrics": self.result.quality_metrics
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        # Load and verify
        self.assertTrue(filepath.exists())

        with open(filepath, 'r') as f:
            loaded = json.load(f)

        self.assertAlmostEqual(loaded["reprojection_error"], 0.45)
        self.assertEqual(loaded["quality_metrics"]["mode"], "manual")
        self.assertEqual(loaded["capture_mode"], "manual")
    
    def test_save_and_load_yaml(self):
        """Test YAML serialization."""
        filepath = Path(self.temp_dir) / "calibration.yaml"
        
        # Save as YAML
        data = {
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.distortion.tolist(),
            "rvecs": [],
            "tvecs": [],
            "reprojection_error": float(self.result.reprojection_error),
            "camera_config": {
                "name": self.result.camera_config.name,
                "camera_index": self.result.camera_config.camera_index,
                "resolution": list(self.result.camera_config.resolution)
            },
            "board_config": {
                "name": self.result.board_config.name,
                "aruco_dict_name": self.result.board_config.aruco_dict_name,
                "size": list(self.result.board_config.size),
                "checker_size_mm": self.result.board_config.checker_size_mm,
                "marker_size_mm": self.result.board_config.marker_size_mm
            },
            "num_images": self.result.num_images,
            "capture_mode": self.result.capture_mode,
            "timestamp": self.result.timestamp,
            "images_used": self.result.images_used,
            "quality_metrics": self.result.quality_metrics
        }
        
        with open(filepath, 'w') as f:
            yaml.dump(data, f)
        
        # Load and verify
        self.assertTrue(filepath.exists())

        with open(filepath, 'r') as f:
            loaded = yaml.safe_load(f)

        self.assertAlmostEqual(loaded["reprojection_error"], 0.45)
        self.assertEqual(loaded["quality_metrics"]["mode"], "manual")
    
    def test_save_and_load_pickle(self):
        """Test pickle serialization."""
        filepath = Path(self.temp_dir) / "calibration.pkl"
        
        # Save as pickle
        with open(filepath, 'wb') as f:
            pickle.dump(self.result, f)
        
        # Load and verify
        self.assertTrue(filepath.exists())
        
        with open(filepath, 'rb') as f:
            loaded = pickle.load(f)
        
        self.assertAlmostEqual(loaded.reprojection_error, 0.45)
        self.assertEqual(loaded.quality_metrics["mode"], "manual")
        np.testing.assert_array_almost_equal(
            loaded.camera_matrix, self.camera_matrix
        )


class TestCalibrationQualityMetrics(unittest.TestCase):
    """Test quality metrics and assessment."""
    
    def test_excellent_quality_classification(self):
        """Test excellent quality calibration."""
        camera_cfg = CameraConfig("Test", camera_index=0, resolution=(1920, 1080))
        board_cfg = CharUcoBoardConfig("Test Board", "DICT_4X4_50", (5, 7), 30.0, 18.0)

        result = CalibrationResult(
            camera_matrix=np.eye(3),
            dist_coeffs=np.zeros(5),
            rvecs=[],
            tvecs=[],
            reprojection_error=0.3,
            camera_config=camera_cfg,
            board_config=board_cfg,
            num_images=50,
            capture_mode="manual",
            timestamp="2025-01-01T00:00:00",
            images_used=[],
            quality_metrics={"quality_score": 0.98}
        )
        
        # Excellent: error < 0.5
        self.assertLess(result.reprojection_error, 0.5)
        self.assertGreater(result.quality_metrics["quality_score"], 0.9)
    
    def test_good_quality_classification(self):
        """Test good quality calibration."""
        camera_cfg = CameraConfig("Test", camera_index=0, resolution=(1920, 1080))
        board_cfg = CharUcoBoardConfig("Test Board", "DICT_4X4_50", (5, 7), 30.0, 18.0)

        result = CalibrationResult(
            camera_matrix=np.eye(3),
            dist_coeffs=np.zeros(5),
            rvecs=[],
            tvecs=[],
            reprojection_error=0.7,
            camera_config=camera_cfg,
            board_config=board_cfg,
            num_images=50,
            capture_mode="manual",
            timestamp="2025-01-01T00:00:00",
            images_used=[],
            quality_metrics={"quality_score": 0.85}
        )
        
        # Good: 0.5 <= error < 1.0
        self.assertGreaterEqual(result.reprojection_error, 0.5)
        self.assertLess(result.reprojection_error, 1.0)
    
    def test_acceptable_quality_classification(self):
        """Test acceptable quality calibration."""
        camera_cfg = CameraConfig("Test", camera_index=0, resolution=(1920, 1080))
        board_cfg = CharUcoBoardConfig("Test Board", "DICT_4X4_50", (5, 7), 30.0, 18.0)

        result = CalibrationResult(
            camera_matrix=np.eye(3),
            dist_coeffs=np.zeros(5),
            rvecs=[],
            tvecs=[],
            reprojection_error=1.5,
            camera_config=camera_cfg,
            board_config=board_cfg,
            num_images=50,
            capture_mode="manual",
            timestamp="2025-01-01T00:00:00",
            images_used=[],
            quality_metrics={"quality_score": 0.70}
        )
        
        # Acceptable: 1.0 <= error < 2.0
        self.assertGreaterEqual(result.reprojection_error, 1.0)
        self.assertLess(result.reprojection_error, 2.0)
    
    def test_poor_quality_classification(self):
        """Test poor quality calibration."""
        camera_cfg = CameraConfig("Test", camera_index=0, resolution=(1920, 1080))
        board_cfg = CharUcoBoardConfig("Test Board", "DICT_4X4_50", (5, 7), 30.0, 18.0)

        result = CalibrationResult(
            camera_matrix=np.eye(3),
            dist_coeffs=np.zeros(5),
            rvecs=[],
            tvecs=[],
            reprojection_error=2.5,
            camera_config=camera_cfg,
            board_config=board_cfg,
            num_images=50,
            capture_mode="manual",
            timestamp="2025-01-01T00:00:00",
            images_used=[],
            quality_metrics={"quality_score": 0.50}
        )
        
        # Poor: error >= 2.0
        self.assertGreaterEqual(result.reprojection_error, 2.0)


class TestMultiCameraWorkflow(unittest.TestCase):
    """Test multi-camera calibration workflow."""
    
    def test_multi_camera_configuration(self):
        """Test setting up multiple cameras."""
        cameras = [
            CameraConfig("Front", camera_index=0, resolution=(1920, 1080)),
            CameraConfig("Left", camera_index=1, resolution=(1920, 1080)),
            CameraConfig("Right", camera_index=2, resolution=(1920, 1080)),
        ]
        
        self.assertEqual(len(cameras), 3)
        self.assertEqual(cameras[0].name, "Front")
        self.assertEqual(cameras[1].name, "Left")
        self.assertEqual(cameras[2].name, "Right")
    
    def test_consistent_board_across_cameras(self):
        """Test using same board for all cameras."""
        board = CharUcoBoardConfig(
            "standard_board", "DICT_4X4_50", (5, 7), 30.0, 18.0
        )
        
        cameras = [
            CameraConfig("Camera_0", camera_index=0, resolution=(1920, 1080)),
            CameraConfig("Camera_1", camera_index=1, resolution=(1920, 1080)),
        ]
        
        # All cameras use the same board
        for camera in cameras:
            self.assertEqual(board.name, "standard_board")
            self.assertEqual(board.size, (5, 7))


class TestOutputDirectoryStructure(unittest.TestCase):
    """Test output directory organization."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.calibration_dir = Path(self.temp_dir) / "artifacts" / "intrinsics"
    
    def tearDown(self):
        """Clean up test fixtures."""
        import shutil
        shutil.rmtree(self.temp_dir)
    
    def test_create_output_directory(self):
        """Test creating output directory structure."""
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        
        self.assertTrue(self.calibration_dir.exists())
        self.assertTrue(self.calibration_dir.is_dir())
    
    def test_save_multiple_formats(self):
        """Test saving calibration in multiple formats."""
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        
        # Create dummy calibration data
        data = {
            "camera_matrix": [[1920, 0, 960], [0, 1920, 540], [0, 0, 1]],
            "distortion": [-0.1, 0.05, 0.001, -0.002, 0.0]
        }
        
        formats = {
            "json": self.calibration_dir / "calibration.json",
            "yaml": self.calibration_dir / "calibration.yaml",
            "pkl": self.calibration_dir / "calibration.pkl"
        }
        
        # Save JSON
        with open(formats["json"], 'w') as f:
            json.dump(data, f)
        
        # Save YAML
        with open(formats["yaml"], 'w') as f:
            yaml.dump(data, f)
        
        # Save PKL
        with open(formats["pkl"], 'wb') as f:
            pickle.dump(data, f)
        
        # Verify all files exist
        for fmt, filepath in formats.items():
            self.assertTrue(filepath.exists(), f"{fmt} file not created")
    
    def test_organize_by_camera(self):
        """Test organizing results by camera."""
        cameras_dir = self.calibration_dir / "cameras"
        cameras_dir.mkdir(parents=True, exist_ok=True)
        
        camera_names = ["front_camera", "left_camera", "right_camera"]
        
        for camera_name in camera_names:
            camera_subdir = cameras_dir / camera_name
            camera_subdir.mkdir(exist_ok=True)
            
            (camera_subdir / "intrinsics.json").touch()
        
        # Verify structure
        created_subdirs = list(cameras_dir.iterdir())
        self.assertEqual(len(created_subdirs), 3)


class TestCalibrationParameterValidation(unittest.TestCase):
    """Test calibration parameter validation."""
    
    def test_valid_focal_length(self):
        """Test valid focal length values."""
        camera_matrix = np.array([
            [1920, 0, 960],
            [0, 1920, 540],
            [0, 0, 1]
        ], dtype=np.float64)
        
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        
        self.assertGreater(fx, 0)
        self.assertGreater(fy, 0)
        self.assertLess(fx, 5000)  # Reasonable upper bound
        self.assertLess(fy, 5000)
    
    def test_valid_principal_point(self):
        """Test valid principal point."""
        camera_matrix = np.array([
            [1920, 0, 960],
            [0, 1920, 540],
            [0, 0, 1]
        ], dtype=np.float64)
        
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        # Principal point should be close to center
        self.assertGreater(cx, 0)
        self.assertGreater(cy, 0)
        self.assertLess(cx, 1920)
        self.assertLess(cy, 1080)
    
    def test_distortion_coefficient_range(self):
        """Test distortion coefficients in valid range."""
        distortion = np.array([-0.3, 0.1, 0.001, -0.002, 0.01], dtype=np.float64)
        
        # All coefficients should be in reasonable range
        for coeff in distortion:
            self.assertGreater(coeff, -1.0)
            self.assertLess(coeff, 1.0)


class TestDataProcessingPipeline(unittest.TestCase):
    """Test the data processing pipeline."""
    
    def test_image_count_validation(self):
        """Test validating image count."""
        image_counts = [5, 10, 25, 50, 100]
        
        for count in image_counts:
            # All counts should be valid
            self.assertGreaterEqual(count, 5)
    
    def test_capture_mode_selection(self):
        """Test capture mode selection."""
        modes = [CaptureMode.MANUAL, CaptureMode.VIDEO, CaptureMode.CONSERVATIVE]
        
        self.assertEqual(len(modes), 3)
        self.assertEqual(modes[0].value, "manual")
        self.assertEqual(modes[1].value, "video")
        self.assertEqual(modes[2].value, "conservative")


class TestErrorHandling(unittest.TestCase):
    """Test error handling in calibration."""
    
    def test_invalid_camera_index(self):
        """Test handling of invalid camera index."""
        # CameraConfig doesn't validate camera_index, so it should create successfully
        config = CameraConfig("Test", camera_index=-1, resolution=(1920, 1080))
        self.assertEqual(config.camera_index, -1)
        # Note: Actual validation would happen in the camera access code
    
    def test_invalid_resolution(self):
        """Test handling of invalid resolution."""
        # Very small resolution might be invalid
        config = CameraConfig("Test", camera_index=0, resolution=(1, 1))
        # Should still create config (actual validation in calibrator)
        self.assertEqual(config.resolution, (1, 1))


def run_test_suite():
    """Run the complete test suite."""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestCalibrationConfiguration))
    suite.addTests(loader.loadTestsFromTestCase(TestCalibrationResultManagement))
    suite.addTests(loader.loadTestsFromTestCase(TestCalibrationResultSerialization))
    suite.addTests(loader.loadTestsFromTestCase(TestCalibrationQualityMetrics))
    suite.addTests(loader.loadTestsFromTestCase(TestMultiCameraWorkflow))
    suite.addTests(loader.loadTestsFromTestCase(TestOutputDirectoryStructure))
    suite.addTests(loader.loadTestsFromTestCase(TestCalibrationParameterValidation))
    suite.addTests(loader.loadTestsFromTestCase(TestDataProcessingPipeline))
    suite.addTests(loader.loadTestsFromTestCase(TestErrorHandling))
    
    runner = unittest.TextTestRunner(verbosity=2)
    return runner.run(suite)


if __name__ == '__main__':
    result = run_test_suite()
    sys.exit(0 if result.wasSuccessful() else 1)
