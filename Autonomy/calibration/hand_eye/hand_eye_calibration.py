#!/usr/bin/env python3
"""
Hand-Eye Calibration for Robotics

Calculates the transformation between a robot's end-effector ("hand") and a camera ("eye").
Supports both Eye-on-Hand and Eye-to-Hand calibration setups.

Usage:
    python hand_eye_calibration.py --setup eye_on_hand --data my_data.npz
    python hand_eye_calibration.py --generate-sample  # Test with sample data
"""

import argparse
import glob
import os
from typing import List, Optional, Tuple

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def create_pose_matrix(R, t):
    """Converts a rotation matrix and translation vector into a 4x4 pose matrix."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T


class ChArUcoDetector:
    """ChArUco board detector for hand-eye calibration."""

    def __init__(self, squares_x: int = 10, squares_y: int = 7, square_size: float = 0.020,
                 marker_size: float = 0.016):
        """
        Initialize ChArUco detector.

        Args:
            squares_x: Number of squares in X direction
            squares_y: Number of squares in Y direction
            square_size: Size of each square in meters
            marker_size: Size of each ArUco marker in meters
        """
        self.squares_x = squares_x
        self.squares_y = squares_y
        self.square_size = square_size
        self.marker_size = marker_size

        # Initialize ArUco and ChArUco components
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.charuco_params = cv2.aruco.CharucoParameters()
        self.detector = cv2.aruco.CharucoDetector(self.aruco_dict, self.charuco_params, self.aruco_params)

        # Create ChArUco board
        self.board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y), square_size, marker_size, self.aruco_dict
        )

    def detect_board_pose(self, image_path: str, camera_matrix: np.ndarray,
                         dist_coeffs: np.ndarray) -> Optional[np.ndarray]:
        """
        Detect ChArUco board in image and estimate its pose.

        Args:
            image_path: Path to image file
            camera_matrix: Camera intrinsic matrix (3x3)
            dist_coeffs: Distortion coefficients

        Returns:
            4x4 transformation matrix from camera to board, or None if detection failed
        """
        # Read image
        img = cv2.imread(image_path)
        if img is None:
            print(f"Could not load image: {image_path}")
            return None

        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect ChArUco board
        charuco_corners, charuco_ids, marker_corners, marker_ids = self.detector.detectBoard(gray)

        if charuco_corners is None or len(charuco_corners) < 4:
            print(f"No valid ChArUco board detected in {os.path.basename(image_path)}")
            return None

        # Estimate board pose
        valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, self.board, camera_matrix, dist_coeffs, None, None
        )

        if not valid:
            print(f"Could not estimate board pose in {os.path.basename(image_path)}")
            return None

        # Convert to transformation matrix
        R, _ = cv2.Rodrigues(rvec)
        T_camera_to_board = create_pose_matrix(R, tvec)

        return T_camera_to_board


def generate_sample_data(setup_type='eye_on_hand', num_poses=20):
    """
    Generates realistic sample data for testing the calibration script.

    This simulates the data collection process. In a real scenario, you would
    replace this with your own data loading function.
    """
    # Ground truth transformations (These are what we are trying to find)
    # Transformation from robot hand to camera
    R_hand_to_camera_true = Rotation.from_euler('xyz', [np.pi, 0, np.pi/2]).as_matrix()
    t_hand_to_camera_true = np.array([0.1, 0.05, 0.05])  # 10cm forward, 5cm left, 5cm up
    T_hand_to_camera_true = create_pose_matrix(R_hand_to_camera_true, t_hand_to_camera_true)

    # Transformation from robot base to camera
    R_base_to_camera_true = Rotation.from_euler('xyz', [np.pi, 0, 0]).as_matrix()
    t_base_to_camera_true = np.array([0.5, 0, 0.5])  # 50cm right, 50cm up
    T_base_to_camera_true = create_pose_matrix(R_base_to_camera_true, t_base_to_camera_true)

    # Transformation of the static target relative to the world/base
    R_base_to_target_true = Rotation.from_euler('xyz', [0, 0, 0]).as_matrix()
    t_base_to_target_true = np.array([0.5, 0.2, 0])  # Target at a fixed world point
    T_base_to_target_true = create_pose_matrix(R_base_to_target_true, t_base_to_target_true)

    # Lists to store the collected poses
    robot_poses = []
    target_in_camera_poses = []

    for i in range(num_poses):
        # Generate random but reasonable robot hand poses
        rand_trans = np.random.rand(3) * 0.2 - 0.1  # Move +/- 10cm
        rand_rot = np.random.rand(3) * np.pi/6 - np.pi/12  # Rotate +/- 15 degrees

        t_base_to_hand = np.array([0.4 + rand_trans[0], 0 + rand_trans[1], 0.3 + rand_trans[2]])
        R_base_to_hand = Rotation.from_euler('xyz', rand_rot).as_matrix()
        T_base_to_hand = create_pose_matrix(R_base_to_hand, t_base_to_hand)

        if setup_type == 'eye_on_hand':
            # Camera is on hand, target is static
            # T_base_to_target = T_base_to_hand * T_hand_to_camera * T_camera_to_target
            # T_camera_to_target = inv(T_hand_to_camera) * inv(T_base_to_hand) * T_base_to_target
            T_camera_to_target = np.linalg.inv(T_hand_to_camera_true) @ np.linalg.inv(T_base_to_hand) @ T_base_to_target_true

        elif setup_type == 'eye_to_hand':
            # Camera is static, target is on hand (let's assume target is at hand origin for simplicity)
            # T_base_to_target = T_base_to_hand
            # T_base_to_target = T_base_to_camera * T_camera_to_target
            # T_camera_to_target = inv(T_base_to_camera) * T_base_to_hand
            T_camera_to_target = np.linalg.inv(T_base_to_camera_true) @ T_base_to_hand

        else:
            raise ValueError("Invalid setup type. Must be 'eye_on_hand' or 'eye_to_hand'")

        robot_poses.append(T_base_to_hand)
        target_in_camera_poses.append(T_camera_to_target)

    print(f"✅ Generated {num_poses} sample poses for '{setup_type}' setup.")
    return robot_poses, target_in_camera_poses


def load_calibration_data(data_file):
    """
    Load calibration data from an .npz file.

    Expected format:
    - robot_poses: list of 4x4 transformation matrices (T_base_to_hand)
    - target_poses: list of 4x4 transformation matrices (T_camera_to_target)

    Args:
        data_file: Path to .npz file containing calibration data

    Returns:
        robot_poses, target_poses: Lists of 4x4 numpy arrays
    """
    try:
        data = np.load(data_file)
        robot_poses = data['robot_poses']
        target_poses = data['target_poses']
        print(f"✅ Loaded {len(robot_poses)} pose pairs from '{data_file}'")
        return robot_poses, target_poses
    except Exception as e:
        print(f"❌ Error loading data from '{data_file}': {e}")
        return None, None


def perform_hand_eye_calibration(robot_poses, target_poses, setup_type='eye_on_hand', method=cv2.CALIB_HAND_EYE_TSAI):
    """
    Perform hand-eye calibration using OpenCV.

    Args:
        robot_poses: List of 4x4 transformation matrices (T_base_to_hand)
        target_poses: List of 4x4 transformation matrices (T_camera_to_target)
        setup_type: 'eye_on_hand' or 'eye_to_hand'
        method: OpenCV calibration method

    Returns:
        result_matrix: 4x4 transformation matrix (result of calibration)
    """
    # Prepare data for OpenCV's calibrateHandEye function
    # It requires separate lists of rotation and translation vectors.
    R_all_robots = []
    t_all_robots = []
    R_all_targets = []
    t_all_targets = []

    for T_base_to_hand, T_camera_to_target in zip(robot_poses, target_poses):
        # For Eye-on-Hand, OpenCV needs T_hand_to_base
        if setup_type == 'eye_on_hand':
            T_hand_to_base = np.linalg.inv(T_base_to_hand)
            R_all_robots.append(T_hand_to_base[:3, :3])
            t_all_robots.append(T_hand_to_base[:3, 3])

            R_all_targets.append(T_camera_to_target[:3, :3])
            t_all_targets.append(T_camera_to_target[:3, 3])

        # For Eye-to-Hand, OpenCV needs T_base_to_hand
        elif setup_type == 'eye_to_hand':
            R_all_robots.append(T_base_to_hand[:3, :3])
            t_all_robots.append(T_base_to_hand[:3, 3])

            # And it needs T_target_to_camera
            T_target_to_camera = np.linalg.inv(T_camera_to_target)
            R_all_targets.append(T_target_to_camera[:3, :3])
            t_all_targets.append(T_target_to_camera[:3, 3])

    # Perform calibration
    print(f"\n🔧 Performing hand-eye calibration using {setup_type} setup...")
    R, t = cv2.calibrateHandEye(
        R_gripper2base=R_all_robots,
        t_gripper2base=t_all_robots,
        R_target2cam=R_all_targets,
        t_target2cam=t_all_targets,
        method=method
    )

    if R is not None and t is not None:
        # Combine into a 4x4 homogeneous transformation matrix
        result_matrix = create_pose_matrix(R, t)
        return result_matrix
    else:
        return None


def collect_hand_eye_data(image_dir: str, robot_poses_file: str, camera_calibration_file: str,
                         board_squares: Tuple[int, int] = (10, 7), square_size: float = 0.020,
                         marker_size: float = 0.016) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    Collect hand-eye calibration data by detecting ChArUco board in images.

    Args:
        image_dir: Directory containing calibration images
        robot_poses_file: File containing robot poses (one pose per line, format TBD)
        camera_calibration_file: Camera calibration YAML file
        board_squares: (squares_x, squares_y) of ChArUco board
        square_size: Size of each square in meters
        marker_size: Size of each ArUco marker in meters

    Returns:
        Tuple of (robot_poses, board_poses_in_camera) lists
    """
    # Load camera calibration
    import yaml
    with open(camera_calibration_file, 'r') as f:
        calib_data = yaml.safe_load(f)

    camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
    dist_coeffs = np.array(calib_data['distortion_coefficients']['data'])

    # Initialize detector
    detector = ChArUcoDetector(board_squares[0], board_squares[1], square_size, marker_size)

    # Find all images
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
    image_files = []
    for ext in image_extensions:
        image_files.extend(glob.glob(os.path.join(image_dir, ext)))

    image_files.sort()  # Ensure consistent ordering

    # Load robot poses (simplified - assumes one pose per image)
    robot_poses = []
    with open(robot_poses_file, 'r') as f:
        for line in f:
            # Parse robot pose from file (format: tx ty tz qx qy qz qw)
            values = list(map(float, line.strip().split()))
            if len(values) == 7:  # Position + quaternion
                t = np.array(values[:3])
                q = values[3:]
                R = Rotation.from_quat(q).as_matrix()
                T = create_pose_matrix(R, t)
                robot_poses.append(T)

    if len(robot_poses) != len(image_files):
        print(f"Warning: Number of robot poses ({len(robot_poses)}) doesn't match number of images ({len(image_files)})")

    # Detect board poses
    board_poses_in_camera = []
    valid_pairs = 0

    for i, image_file in enumerate(image_files):
        print(f"Processing image {i+1}/{len(image_files)}: {os.path.basename(image_file)}")

        board_pose = detector.detect_board_pose(image_file, camera_matrix, dist_coeffs)

        if board_pose is not None and i < len(robot_poses):
            board_poses_in_camera.append(board_pose)
            valid_pairs += 1
            print("  ✓ Board detected")
        else:
            print("  ✗ Board not detected or no corresponding robot pose")

    print(f"\nData collection complete:")
    print(f"  Total images: {len(image_files)}")
    print(f"  Robot poses: {len(robot_poses)}")
    print(f"  Valid pairs: {valid_pairs}")

    return robot_poses[:valid_pairs], board_poses_in_camera


def save_calibration_result(result_matrix, setup_type, output_file='hand_eye_calibration_result.npz'):
    """
    Save the calibration result to a file.

    Args:
        result_matrix: 4x4 transformation matrix
        setup_type: 'eye_on_hand' or 'eye_to_hand'
        output_file: Output filename
    """
    try:
        np.savez(output_file, transformation=result_matrix, setup_type=setup_type)
        print(f"✅ Calibration result saved to '{output_file}'")
    except Exception as e:
        print(f"❌ Error saving result: {e}")


def print_calibration_result(result_matrix, setup_type):
    """
    Print the calibration result in a readable format.
    """
    if result_matrix is not None:
        print("\n✅ Calibration successful!")
        if setup_type == 'eye_on_hand':
            print("Result: Transformation from Hand to Camera (T_hand_to_camera)")
        else:  # eye_to_hand
            print("Result: Transformation from Base to Camera (T_base_to_camera)")

        np.set_printoptions(precision=4, suppress=True)
        print("\n4x4 Transformation Matrix:")
        print(result_matrix)

        # Also print rotation angles and translation
        R = result_matrix[:3, :3]
        t = result_matrix[:3, 3]

        # Convert rotation matrix to Euler angles
        rot = Rotation.from_matrix(R)
        euler_angles = rot.as_euler('xyz', degrees=True)

        print("\nRotation (Euler angles in degrees - XYZ order):")
        print(".2f")
        print("\nTranslation (in meters):")
        print(".4f")
    else:
        print("\n❌ Calibration failed. Check your data.")


def main():
    parser = argparse.ArgumentParser(description='Hand-Eye Calibration for Robotics')

    # Setup configuration
    parser.add_argument('--setup', choices=['eye_on_hand', 'eye_to_hand'],
                       default='eye_on_hand',
                       help='Calibration setup type (default: eye_on_hand)')

    # Data source
    parser.add_argument('--data', type=str,
                       help='Path to .npz file containing calibration data')
    parser.add_argument('--generate-sample', action='store_true',
                       help='Generate and use sample data for testing')

    # Data collection from images (new feature)
    parser.add_argument('--collect-data', action='store_true',
                       help='Collect calibration data from images and robot poses')
    parser.add_argument('--images', type=str,
                       help='Directory containing calibration images (used with --collect-data)')
    parser.add_argument('--robot-poses', type=str,
                       help='File containing robot poses (used with --collect-data)')
    parser.add_argument('--camera-calibration', type=str,
                       help='Camera calibration YAML file (used with --collect-data)')
    parser.add_argument('--board-squares', type=str, default='10x7',
                       help='ChArUco board squares (format: WxH, default: 10x7)')
    parser.add_argument('--square-size', type=float, default=0.020,
                       help='Square size in meters (default: 0.020)')
    parser.add_argument('--marker-size', type=float, default=0.016,
                       help='ArUco marker size in meters (default: 0.016)')

    # Calibration options
    parser.add_argument('--method', choices=['tsai', 'park', 'horaud', 'andreff', 'daniilidis'],
                       default='tsai',
                       help='Calibration method (default: tsai)')
    parser.add_argument('--num-poses', type=int, default=20,
                       help='Number of sample poses to generate (only used with --generate-sample)')

    # Output
    parser.add_argument('--output', type=str, default='hand_eye_calibration_result.npz',
                       help='Output file for calibration result (default: hand_eye_calibration_result.npz)')

    args = parser.parse_args()

    # Map method names to OpenCV constants
    method_map = {
        'tsai': cv2.CALIB_HAND_EYE_TSAI,
        'park': cv2.CALIB_HAND_EYE_PARK,
        'horaud': cv2.CALIB_HAND_EYE_HORAUD,
        'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS
    }

    # Load or generate data
    if args.collect_data:
        if not all([args.images, args.robot_poses, args.camera_calibration]):
            print("❌ Error: --collect-data requires --images, --robot-poses, and --camera-calibration")
            return

        print("📷 Collecting calibration data from images...")
        # Parse board squares
        squares_w, squares_h = map(int, args.board_squares.split('x'))
        robot_poses, target_poses = collect_hand_eye_data(
            args.images, args.robot_poses, args.camera_calibration,
            (squares_w, squares_h), args.square_size, args.marker_size
        )
    elif args.generate_sample:
        print("🎯 Generating sample calibration data...")
        robot_poses, target_poses = generate_sample_data(args.setup, args.num_poses)
    elif args.data:
        print(f"📂 Loading calibration data from '{args.data}'...")
        robot_poses, target_poses = load_calibration_data(args.data)
        if robot_poses is None:
            return
    else:
        print("❌ Error: Must specify one of:")
        print("  --collect-data (with --images, --robot-poses, --camera-calibration)")
        print("  --data <file>")
        print("  --generate-sample")
        print("Use --help for more information")
        return

    # Validate data
    if len(robot_poses) != len(target_poses):
        print(f"❌ Error: Robot poses ({len(robot_poses)}) and target poses ({len(target_poses)}) must have the same length")
        return

    if len(robot_poses) < 3:
        print(f"❌ Error: Need at least 3 pose pairs for calibration (you have {len(robot_poses)})")
        return

    print(f"📊 Using {len(robot_poses)} pose pairs for calibration")

    # Perform calibration
    result_matrix = perform_hand_eye_calibration(
        robot_poses, target_poses,
        setup_type=args.setup,
        method=method_map[args.method]
    )

    # Print and save results
    print_calibration_result(result_matrix, args.setup)

    if result_matrix is not None:
        save_calibration_result(result_matrix, args.setup, args.output)


if __name__ == '__main__':
    main()
