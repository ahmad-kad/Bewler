#!/usr/bin/env python3
"""
Integration Tests for RGB-D SLAM System

Tests:
1. Depth processor filtering pipeline
2. GPS fusion Extended Kalman Filter
3. End-to-end SLAM system
4. Fallback mechanisms
5. Performance metrics
"""

import sys
import unittest

import numpy as np
from depth_processor import DepthProcessor
from gps_fusion_node import ExtendedKalmanFilter

sys.path.insert(0, "/Users/ahmadkaddoura/robotics2025/Autonomy/code/slam/autonomy_slam")


class TestDepthProcessing(unittest.TestCase):
    """Test depth processing for desert environments."""

    def setUp(self):
        """Set up test fixtures."""
        # Create synthetic depth images
        self.height, self.width = 480, 640
        self.valid_depth_range = (100, 5000)  # mm

    def test_range_filtering(self):
        """Test range-based depth filtering."""
        # Create depth image with outliers
        depth = np.full((self.height, self.width), 1000, dtype=np.uint16)
        depth[100:150, 100:150] = 50  # Out of range (too close)
        depth[200:250, 200:250] = 6000  # Out of range (too far)

        processor = DepthProcessor()
        filtered = processor._range_filter(depth)

        # Out-of-range values should be zero
        self.assertEqual(filtered[125, 125], 0)
        self.assertEqual(filtered[225, 225], 0)
        # In-range values should be preserved
        self.assertEqual(filtered[0, 0], 1000)

    def test_dust_detection(self):
        """Test dust particle detection and removal."""
        # Create depth map with isolated dust (high variance regions)
        depth = np.full((self.height, self.width), 1000, dtype=np.uint16)

        # Add isolated dust speckles
        depth[100, 100] = 500  # Isolated point
        depth[200, 200] = 1500  # Another isolated point

        processor = DepthProcessor()
        cleaned = processor._detect_and_remove_dust(depth)

        # Isolated high-variance points should be removed
        self.assertLess(cleaned[100, 100], depth[100, 100])
        self.assertLess(cleaned[200, 200], depth[200, 200])

    def test_temporal_smoothing(self):
        """Test temporal filtering across frames."""
        processor = DepthProcessor()
        processor.temporal_window = 3

        # Create three different depth frames
        frame1 = np.full((self.height, self.width), 1000, dtype=np.uint16)
        frame2 = np.full((self.height, self.width), 1100, dtype=np.uint16)
        frame3 = np.full((self.height, self.width), 900, dtype=np.uint16)

        # Apply temporal smoothing
        smooth1 = processor._temporal_smooth(frame1)
        smooth2 = processor._temporal_smooth(frame2)
        smooth3 = processor._temporal_smooth(frame3)

        # Smoothed should be closer to median than raw extremes
        self.assertGreater(smooth1[0, 0], 900)
        self.assertLess(smooth3[0, 0], 1100)


class TestGPSFusion(unittest.TestCase):
    """Test GPS-SLAM fusion via Extended Kalman Filter."""

    def setUp(self):
        """Initialize EKF for testing."""
        self.ekf = ExtendedKalmanFilter()

    def test_ekf_initialization(self):
        """Test EKF state and covariance initialization."""
        self.assertTrue(np.allclose(self.ekf.state, np.zeros(6)))
        self.assertEqual(self.ekf.P.shape, (6, 6))
        # High initial uncertainty
        self.assertGreater(np.trace(self.ekf.P), 0.5)

    def test_ekf_predict(self):
        """Test EKF prediction step (motion model)."""
        # Set initial state: position at (1, 2, 0), velocity (1, 0)
        self.ekf.state = np.array([1.0, 2.0, 0.0, 0.0, 1.0, 0.0])
        dt = 0.1

        # Predict forward
        self.ekf.predict(dt)

        # Position should advance by velocity * dt
        predicted_x = 1.0 + 1.0 * dt
        self.assertAlmostEqual(self.ekf.state[0], predicted_x, places=5)
        self.assertAlmostEqual(self.ekf.state[1], 2.0, places=5)

    def test_slam_measurement_update(self):
        """Test SLAM measurement update."""
        # Set reference GPS point
        self.ekf.gps_reference = (0.0, 0.0)

        # Initial state
        initial_state = self.ekf.state.copy()

        # Receive SLAM pose estimate (position and yaw)
        slam_position = np.array([1.0, 2.0, 0.5])
        slam_yaw = 0.1
        slam_covariance = np.eye(6) * 0.1

        # Update with SLAM
        self.ekf.update_slam(slam_position, slam_yaw, slam_covariance)

        # State should move toward SLAM measurement
        self.assertNotEqual(self.ekf.state[0], initial_state[0])
        self.assertNotEqual(self.ekf.state[1], initial_state[1])

    def test_gps_measurement_update(self):
        """Test GPS measurement update."""
        # Set reference GPS point
        ref_lat, ref_lon = 0.0, 0.0
        self.ekf.gps_reference = (ref_lat, ref_lon)

        # First GPS fix (reference)
        self.ekf.update_gps(ref_lat, ref_lon)

        # Second GPS fix (offset)
        offset_lat = 0.00001  # ~1 meter north
        offset_lon = 0.00001  # ~1 meter east
        self.ekf.update_gps(ref_lat + offset_lat, ref_lon + offset_lon)

        # State should move toward GPS measurement
        # (approximately 1 meter in both directions)
        self.assertGreater(self.ekf.state[0], 0.5)  # x moved east
        self.assertGreater(self.ekf.state[1], 0.5)  # y moved north

    def test_gps_to_local_conversion(self):
        """Test GPS to local coordinate conversion."""
        ref_lat, ref_lon = 0.0, 0.0
        self.ekf.gps_reference = (ref_lat, ref_lon)

        # Test at same location (should be origin)
        local = self.ekf._gps_to_local(ref_lat, ref_lon)
        self.assertAlmostEqual(local[0], 0.0, places=5)
        self.assertAlmostEqual(local[1], 0.0, places=5)

        # Test offset (approximately 1111 meters per degree at equator)
        offset_lat = 0.01  # ~1111 meters north
        offset_lon = 0.01  # ~1111 meters east
        local = self.ekf._gps_to_local(ref_lat + offset_lat, ref_lon + offset_lon)

        self.assertGreater(local[0], 1000)  # East offset
        self.assertGreater(local[1], 1000)  # North offset

    def test_angle_normalization(self):
        """Test angle normalization to [-pi, pi]."""
        # Test various angles
        self.assertAlmostEqual(ExtendedKalmanFilter._normalize_angle(4 * np.pi), 0.0, places=5)
        self.assertAlmostEqual(ExtendedKalmanFilter._normalize_angle(3 * np.pi), -np.pi, places=5)
        self.assertAlmostEqual(ExtendedKalmanFilter._normalize_angle(-3 * np.pi), np.pi, places=5)

    def test_fusion_modes(self):
        """Test graceful degradation between fusion modes."""
        # Scenario 1: Good SLAM confidence, GPS available
        self.ekf.state[0] = 1.0  # Has pose
        slam_position = np.array([1.0, 2.0, 0.0])

        # Should fuse both
        self.ekf.gps_reference = (0.0, 0.0)
        self.ekf.update_slam(slam_position, 0.0)  # SLAM update
        self.ekf.update_gps(0.00001, 0.00001)  # GPS update

        # Both should have influenced state
        self.assertNotEqual(self.ekf.state[0], 1.0)

    def test_covariance_growth(self):
        """Test that covariance grows with prediction, shrinks with measurement."""
        initial_trace = np.trace(self.ekf.P)

        # Predict (covariance should grow)
        self.ekf.predict(0.1)
        predicted_trace = np.trace(self.ekf.P)
        self.assertGreater(predicted_trace, initial_trace)

        # Measure SLAM (covariance should shrink)
        slam_position = np.array([0.0, 0.0, 0.0])
        self.ekf.update_slam(slam_position, 0.0)
        updated_trace = np.trace(self.ekf.P)
        self.assertLess(updated_trace, predicted_trace)


class TestSystemIntegration(unittest.TestCase):
    """Test end-to-end system integration."""

    def test_fusion_chain(self):
        """Test complete fusion chain: SLAM → EKF → GPS → Pose."""
        ekf = ExtendedKalmanFilter()
        ekf.gps_reference = (0.0, 0.0)

        # Simulate SLAM producing poses
        for i in range(5):
            slam_pos = np.array([float(i), float(i) * 0.5, 0.0])
            slam_yaw = float(i) * 0.05

            # Predict motion
            ekf.predict(0.1)

            # Update with SLAM
            ekf.update_slam(slam_pos, slam_yaw)

        # Final state should reflect progression
        final_state = ekf.get_state()
        self.assertGreater(final_state.x, 0.0)
        self.assertGreater(final_state.y, 0.0)

    def test_performance_metrics(self):
        """Test that system stays within performance targets."""
        # Memory: EKF state should be small
        ekf = ExtendedKalmanFilter()
        import sys

        ekf_size = sys.getsizeof(ekf.state) + sys.getsizeof(ekf.P)
        self.assertLess(ekf_size, 10000)  # < 10KB for core EKF

        # Update rate: should be < 1ms per update
        import time

        slam_pos = np.array([1.0, 2.0, 0.5])

        start = time.time()
        for _ in range(100):
            ekf.predict(0.01)
            ekf.update_slam(slam_pos, 0.0)
        elapsed = (time.time() - start) / 100.0

        self.assertLess(elapsed, 0.001)  # < 1ms per update


class TestErrorHandling(unittest.TestCase):
    """Test error handling and robustness."""

    def test_invalid_depth(self):
        """Test handling of invalid depth values."""
        processor = DepthProcessor()

        # Depth with NaN and Inf values
        depth = np.full((100, 100), 1000.0, dtype=np.float32)
        depth[10:20, 10:20] = np.nan
        depth[30:40, 30:40] = np.inf

        # Should not crash, should filter invalid values
        filtered = processor._range_filter(depth.astype(np.uint16))
        self.assertEqual(filtered.shape, depth.shape)

    def test_gps_without_reference(self):
        """Test GPS update before reference is set."""
        ekf = ExtendedKalmanFilter()

        # GPS update without reference should set reference
        ekf.update_gps(0.0, 0.0)
        self.assertIsNotNone(ekf.gps_reference)
        self.assertEqual(ekf.gps_reference, (0.0, 0.0))

    def test_matrix_inversion_robustness(self):
        """Test that EKF handles near-singular matrices."""
        ekf = ExtendedKalmanFilter()

        # Create covariance near singular but not quite
        ekf.P = np.eye(6) * 1e-10

        # Should not crash when inverting
        try:
            slam_pos = np.array([1.0, 2.0, 0.0])
            ekf.update_slam(slam_pos, 0.0)
            # If we get here, we handled it gracefully
            self.assertTrue(True)
        except np.linalg.LinAlgError:
            self.fail("EKF should handle near-singular matrices gracefully")


if __name__ == "__main__":
    unittest.main()
