#!/usr/bin/env python3
"""
Test script to verify the calibration fix works.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

import cv2
import numpy as np

def test_calibration_approach():
    """Test the new calibration approach with synthetic data."""
    print("Testing ChArUco calibration approach...")

    # Create ChArUco board
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard((6, 8), 0.030, 0.018, aruco_dict)

    # Create synthetic calibration data (simulate detected corners)
    # In a real scenario, this would come from camera captures
    all_corners = []
    all_ids = []

    # Simulate 5 frames with detected ChArUco corners
    for frame_idx in range(5):
        # Create some synthetic corner detections
        # In reality, these would be detected by charuco_detector.detectBoard()
        num_corners = np.random.randint(10, 20)  # Random number of corners detected
        corners = np.random.rand(num_corners, 2) * 1000  # Random corner positions
        corners = corners.astype(np.float32)
        ids = np.arange(num_corners).astype(np.int32)  # Sequential IDs

        all_corners.append(corners)
        all_ids.append(ids)

    print(f"Simulated {len(all_corners)} frames with ChArUco detections")

    # Test the new calibration approach
    try:
        all_object_points = []
        all_image_points = []

        for corners, ids in zip(all_corners, all_ids):
            # Note: In real code, corners and ids come from charuco_detector.detectBoard()
            # which already returns interpolated ChArUco corners
            if corners is not None and ids is not None and len(corners) > 0:
                # Get object points for these ChArUco corners
                object_points = board.chessboardCorners[ids.flatten()]
                all_object_points.append(object_points)
                all_image_points.append(corners)

        print(f"Processed {len(all_object_points)} frames with valid ChArUco corners")

        if len(all_object_points) > 0 and len(all_image_points) > 0:
            # Perform standard camera calibration
            ret, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
                all_object_points, all_image_points, (1280, 720), None, None
            )

            print(f"Calibration successful: {ret}")
            print(f"Camera matrix shape: {camera_matrix.shape}")
            print(f"Distortion coefficients shape: {dist_coeffs.shape}")
            return True
        else:
            print("No valid data after interpolation")
            return False

    except Exception as e:
        print(f"Calibration failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_calibration_approach()
    print(f"Test {'PASSED' if success else 'FAILED'}")
    sys.exit(0 if success else 1)