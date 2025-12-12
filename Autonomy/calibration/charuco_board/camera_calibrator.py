#!/usr/bin/env python3
"""
Camera Calibration Processor - URC 2026

Processes sets of calibration images to compute camera intrinsic parameters.
Supports multiple camera types and provides comprehensive validation.

Usage:
    python camera_calibrator.py --images /path/to/calibration_images --pattern 8x6 --square-size 0.025 --output camera_calibration.yaml

Author: URC 2026 Autonomy Team
"""

import argparse
import glob
import json
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml


class CameraCalibrator:
    """Camera calibration processor for intrinsic parameter estimation using ChArUco boards."""

    def __init__(self):
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # ArUco dictionary for ChArUco boards
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        # ChArUco board detector
        self.charuco_params = cv2.aruco.CharucoParameters()
        self.detector = cv2.aruco.CharucoDetector(self.aruco_dict, self.charuco_params, self.aruco_params)

    def detect_charuco_corners(self, image_path: str, board_squares: Tuple[int, int],
                               show_corners: bool = False) -> Optional[Tuple[np.ndarray, np.ndarray, int]]:
        """
        Detect ChArUco board corners and IDs in an image.

        Args:
            image_path: Path to calibration image
            board_squares: (squares_x, squares_y) of ChArUco board
            show_corners: Whether to display detection results

        Returns:
            Tuple of (charuco_corners, charuco_ids, marker_corners) if detected, None otherwise
        """
        # Read image
        img = cv2.imread(image_path)
        if img is None:
            print(f"Warning: Could not read image {image_path}")
            return None

        # Detect ChArUco board
        charuco_corners, charuco_ids, marker_corners, marker_ids = self.detector.detectBoard(img)

        if charuco_corners is not None and len(charuco_corners) > 0:
            print(f"  ✓ Detected {len(charuco_corners)} ChArUco corners")

            if show_corners:
                # Draw detected corners and markers
                display_img = img.copy()
                cv2.aruco.drawDetectedCornersCharuco(display_img, charuco_corners, charuco_ids)
                if marker_corners is not None:
                    cv2.aruco.drawDetectedMarkers(display_img, marker_corners, marker_ids)

                cv2.imshow('ChArUco Detection', display_img)
                cv2.waitKey(500)
                cv2.destroyAllWindows()

            return charuco_corners, charuco_ids, marker_corners
        else:
            print(f"  ✗ No ChArUco board detected in {image_path}")
            return None

    def calibrate_camera_charuco(self, image_files: List[str], board_squares: Tuple[int, int],
                                square_size: float, marker_size: float) -> Dict:
        """
        Perform camera calibration using ChArUco boards.

        Args:
            image_files: List of calibration image file paths
            board_squares: (squares_x, squares_y) of ChArUco board
            square_size: Size of chessboard squares in meters
            marker_size: Size of ArUco markers in meters

        Returns:
            Dictionary containing calibration results
        """
        # Create ChArUco board object
        board = cv2.aruco.CharucoBoard(board_squares, square_size, marker_size, self.aruco_dict)

        # Arrays to store detected corners and IDs
        all_corners = []
        all_ids = []
        image_size = None

        print(f"Processing {len(image_files)} calibration images with ChArUco board...")

        valid_images = 0
        for image_file in image_files:
            print(f"Processing: {os.path.basename(image_file)}")

            # Detect ChArUco corners
            detection_result = self.detect_charuco_corners(image_file, board_squares)

            if detection_result is not None:
                charuco_corners, charuco_ids, marker_corners = detection_result

                # Store detected corners and IDs
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)

                # Get image size from first valid image
                if image_size is None:
                    img = cv2.imread(image_file)
                    image_size = (img.shape[1], img.shape[0])  # (width, height)

                valid_images += 1
            else:
                print(f"  ✗ No ChArUco board detected")

        print(f"\nCalibration Summary:")
        print(f"  Total images: {len(image_files)}")
        print(f"  Valid images: {valid_images}")
        print(f"  Success rate: {valid_images/len(image_files)*100:.1f}%")

        if valid_images < 8:
            raise ValueError(f"Insufficient valid images for calibration. Need at least 8, got {valid_images}")

        # Perform camera calibration using ChArUco
        print("\nRunning ChArUco camera calibration...")
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            all_corners, all_ids, board, image_size, None, None
        )

        # Calculate reprojection error
        total_error = 0
        total_points = 0

        for i in range(len(all_corners)):
            # Project ChArUco corners back to image
            projected_corners, _ = cv2.aruco.interpolateCornersCharuco(
                all_corners[i], all_ids[i], None, board, camera_matrix, dist_coeffs
            )

            if projected_corners is not None:
                # Calculate reprojection error
                error = cv2.norm(all_corners[i], projected_corners, cv2.NORM_L2) / len(all_corners[i])
                total_error += error
                total_points += 1

        mean_error = total_error / total_points if total_points > 0 else float('inf')

        print(f"Calibration Results:")
        print(f"  RMS error: {mean_error:.6f}")
        print(f"  Reprojection error: {mean_error:.4f} pixels")

        # Assess calibration quality
        quality_rating = self.assess_calibration_quality(mean_error, valid_images, board_squares)

        # Prepare results
        results = {
            'calibration_success': ret > 0,
            'camera_matrix': camera_matrix.tolist(),
            'distortion_coefficients': dist_coeffs.tolist(),
            'image_width': image_size[0],
            'image_height': image_size[1],
            'board_type': 'charuco',
            'board_squares': list(board_squares),
            'square_size': square_size,
            'marker_size': marker_size,
            'aruco_dict': '4X4_50',
            'total_images': len(image_files),
            'valid_images': valid_images,
            'reprojection_error': float(mean_error),
            'quality_rating': quality_rating,
            'calibration_date': str(np.datetime64('now')),
            'opencv_version': cv2.__version__,
            'calibration_method': 'charuco'
        }

        return results

    def assess_calibration_quality(self, reprojection_error: float, num_images: int,
                                 pattern_size: Tuple[int, int]) -> str:
        """Assess the quality of calibration results."""
        # Reprojection error assessment
        if reprojection_error < 0.3:
            error_rating = "excellent"
        elif reprojection_error < 0.5:
            error_rating = "good"
        elif reprojection_error < 1.0:
            error_rating = "acceptable"
        else:
            error_rating = "poor"

        # Image count assessment
        expected_images = max(20, pattern_size[0] * pattern_size[1])
        if num_images >= expected_images:
            count_rating = "excellent"
        elif num_images >= expected_images * 0.75:
            count_rating = "good"
        elif num_images >= expected_images * 0.5:
            count_rating = "acceptable"
        else:
            count_rating = "poor"

        # Overall rating
        ratings = [error_rating, count_rating]
        if "poor" in ratings:
            return "poor"
        elif "acceptable" in ratings:
            return "acceptable"
        elif "good" in ratings:
            return "good"
        else:
            return "excellent"

    def save_calibration_yaml(self, results: Dict, output_path: str):
        """Save calibration results to YAML file."""
        # Convert to ROS2 camera_info format
        camera_info = {
            'image_width': results['image_width'],
            'image_height': results['image_height'],
            'camera_name': 'calibrated_camera',
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [float(x) for row in results['camera_matrix'] for x in row]
            },
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1,
                'cols': 5,
                'data': [float(x) for x in results['distortion_coefficients'][0]]
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': [float(x) for row in results['camera_matrix'] for x in row] + [0.0]
            }
        }

        # Add metadata
        camera_info.update({
            'calibration_date': results['calibration_date'],
            'calibration_method': 'opencv_chessboard',
            'pattern_size': results['pattern_size'],
            'square_size': results['square_size'],
            'total_images': results['total_images'],
            'valid_images': results['valid_images'],
            'reprojection_error': results['reprojection_error'],
            'quality_rating': results['quality_rating'],
            'opencv_version': results['opencv_version']
        })

        # Save to YAML
        with open(output_path, 'w') as f:
            yaml.dump(camera_info, f, default_flow_style=False)

        print(f"Calibration saved to: {output_path}")

    def save_calibration_json(self, results: Dict, output_path: str):
        """Save calibration results to JSON file."""
        with open(output_path, 'w') as f:
            json.dump(results, f, indent=2, default=str)

        print(f"Calibration JSON saved to: {output_path}")

    def validate_calibration(self, results: Dict, test_image_path: Optional[str] = None) -> Dict:
        """Validate calibration by undistorting a test image."""
        validation_results = {
            'undistortion_test': False,
            'focal_length_check': False,
            'distortion_check': False
        }

        # Check focal lengths are reasonable
        fx, fy = results['camera_matrix'][0][0], results['camera_matrix'][1][1]
        cx, cy = results['camera_matrix'][0][2], results['camera_matrix'][1][2]

        # Focal length should be positive and reasonable for camera
        if 100 < fx < 2000 and 100 < fy < 2000:
            validation_results['focal_length_check'] = True
            print("✓ Focal lengths are reasonable")
        else:
            print("✗ Focal lengths seem incorrect")

        # Principal point should be near image center
        img_width, img_height = results['image_width'], results['image_height']
        cx_expected, cy_expected = img_width / 2, img_height / 2

        if abs(cx - cx_expected) < img_width * 0.1 and abs(cy - cy_expected) < img_height * 0.1:
            validation_results['principal_point_check'] = True
            print("✓ Principal point is near image center")
        else:
            print("✗ Principal point seems incorrect")

        # Test undistortion if test image provided
        if test_image_path and os.path.exists(test_image_path):
            try:
                img = cv2.imread(test_image_path)
                if img is not None:
                    camera_matrix = np.array(results['camera_matrix'])
                    dist_coeffs = np.array(results['distortion_coefficients'])

                    # Undistort image
                    undistorted = cv2.undistort(img, camera_matrix, dist_coeffs)

                    # Save undistorted image for inspection
                    output_dir = os.path.dirname(test_image_path)
                    base_name = os.path.splitext(os.path.basename(test_image_path))[0]
                    undistorted_path = os.path.join(output_dir, f"{base_name}_undistorted.jpg")
                    cv2.imwrite(undistorted_path, undistorted)

                    validation_results['undistortion_test'] = True
                    print(f"✓ Undistortion test passed - saved to {undistorted_path}")
                else:
                    print("✗ Could not load test image")
            except Exception as e:
                print(f"✗ Undistortion test failed: {e}")
        else:
            print("⚠ No test image provided for undistortion validation")

        return validation_results

def main():
    parser = argparse.ArgumentParser(description='Camera calibration from ChArUco board images')
    parser.add_argument('--images', required=True,
                       help='Path to directory containing calibration images or glob pattern')
    parser.add_argument('--squares', default='5x7',
                       help='ChArUco board squares pattern (e.g., 5x7 for 5 columns, 7 rows of squares)')
    parser.add_argument('--square-size', type=float, default=0.025,
                       help='Chessboard square size in meters (default: 0.025)')
    parser.add_argument('--marker-size', type=float, default=0.020,
                       help='ArUco marker size in meters (default: 0.020)')
    parser.add_argument('--output', default='camera_calibration.yaml',
                       help='Output calibration file (default: camera_calibration.yaml)')
    parser.add_argument('--format', choices=['yaml', 'json', 'both'], default='yaml',
                       help='Output format (default: yaml)')
    parser.add_argument('--test-image',
                       help='Path to test image for validation')
    parser.add_argument('--show-corners', action='store_true',
                       help='Display detected corners during processing')

    args = parser.parse_args()

    try:
        # Parse board squares
        squares_x, squares_y = map(int, args.squares.split('x'))

        # Find image files
        if os.path.isdir(args.images):
            # Directory of images
            image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff']
            image_files = []
            for ext in image_extensions:
                image_files.extend(glob.glob(os.path.join(args.images, ext)))
        else:
            # Glob pattern
            image_files = glob.glob(args.images)

        image_files.sort()  # Sort for consistent processing

        if not image_files:
            raise ValueError(f"No image files found in {args.images}")

        print(f"Found {len(image_files)} image files")
        print(f"ChArUco Board: {squares_x}x{squares_y} squares")
        print(f"Square size: {args.square_size}m, Marker size: {args.marker_size}m")

        # Create calibrator
        calibrator = CameraCalibrator()

        # Perform ChArUco calibration
        results = calibrator.calibrate_camera_charuco(
            image_files=image_files,
            board_squares=(squares_x, squares_y),
            square_size=args.square_size,
            marker_size=args.marker_size
        )

        # Validate results
        if args.test_image:
            validation = calibrator.validate_calibration(results, args.test_image)
            results['validation'] = validation

        # Save results
        if args.format in ['yaml', 'both']:
            yaml_path = args.output if args.output.endswith('.yaml') else args.output + '.yaml'
            calibrator.save_calibration_yaml(results, yaml_path)

        if args.format in ['json', 'both']:
            json_path = args.output.replace('.yaml', '.json') if args.output.endswith('.yaml') else args.output + '.json'
            calibrator.save_calibration_json(results, json_path)

        # Print summary
        print("\nCalibration Summary:")
        print(f"  Quality Rating: {results['quality_rating'].upper()}")
        print(f"  Reprojection Error: {results['reprojection_error']:.4f} pixels")
        print(f"  Valid Images: {results['valid_images']}/{results['total_images']}")
        print(f"  Focal Length: {results['camera_matrix'][0][0]:.1f}, {results['camera_matrix'][1][1]:.1f} pixels")
        print(f"  Board Type: {results['board_type'].upper()}")
        print(f"  ArUco Dictionary: {results['aruco_dict']}")

        # Recommendations
        if results['quality_rating'] in ['poor', 'acceptable']:
            print("\n⚠️  Recommendations for better calibration:")
            print("  - Capture more images (aim for 20-30 with ChArUco)")
            print("  - Ensure even lighting and no glare on markers")
            print("  - Vary camera angles and distances more widely")
            print("  - Keep camera steady during capture")
            print("  - Ensure ChArUco board is flat and visible")

        if results['reprojection_error'] > 1.0:
            print("\n❌ High reprojection error detected!")
            print("  - Recalibrate with better image set")
            print("  - Check square/marker size measurements")
            print("  - Ensure ChArUco board is not damaged")
            print("  - Try different lighting conditions")

        print("\n✅ ChArUco calibration advantages:")
        print("  • Automatic marker identification")
        print("  • Better robustness to motion blur")
        print("  • Superior pose estimation accuracy")
        print("  • Works at greater distances")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0

if __name__ == '__main__':
    exit(main())
