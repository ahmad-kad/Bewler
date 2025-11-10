#!/usr/bin/env python3
"""
Intrinsics Calibration CLI Tool - URC 2026
===========================================

Easy-to-use command-line interface for camera intrinsic calibration.
Designed for SSH usage on Raspberry Pi with clear feedback and progress.

Usage:
    python3 calibration_cli.py --camera 0 --mode manual
    python3 calibration_cli.py --camera 0 --mode video --count 30
    python3 calibration_cli.py --list-cameras
"""

import argparse
import sys
import os
from pathlib import Path
from datetime import datetime
import logging
from typing import Optional

from camera_intrinsics_calibrator import (
    CameraConfig,
    CharUcoBoardConfig,
    CameraIntrinsicsCalibrator,
    CaptureMode
)


# ============================================================================
# CONFIGURATION
# ============================================================================

# Standard boards (can be extended)
STANDARD_BOARDS = {
    "board_5x7": CharUcoBoardConfig(
        name="board_5x7",
        aruco_dict_name="DICT_4X4_50",
        size=(5, 7),
        checker_size_mm=30.0,
        marker_size_mm=18.0
    ),
    "board_7x5": CharUcoBoardConfig(
        name="board_7x5",
        aruco_dict_name="DICT_4X4_50",
        size=(7, 5),
        checker_size_mm=25.0,
        marker_size_mm=15.0
    ),
    "board_small": CharUcoBoardConfig(
        name="board_small",
        aruco_dict_name="DICT_4X4_50",
        size=(4, 5),
        checker_size_mm=20.0,
        marker_size_mm=12.0
    ),
}

# Default camera configurations
DEFAULT_CAMERAS = {
    "default": (640, 480),
    "hd": (1280, 720),
    "fullhd": (1920, 1080),
    "2k": (2560, 1440),
}

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s: %(message)s'
)
logger = logging.getLogger(__name__)


# ============================================================================
# CLI UTILITY FUNCTIONS
# ============================================================================

def print_header(text: str):
    """Print formatted header."""
    print(f"\n{'='*70}")
    print(f"  {text}")
    print(f"{'='*70}\n")


def print_section(text: str):
    """Print formatted section."""
    print(f"\n{text}")
    print(f"{'-'*len(text)}")


def print_success(text: str):
    """Print success message."""
    print(f"✓ {text}")


def print_info(text: str):
    """Print info message."""
    print(f"ℹ {text}")


def print_warning(text: str):
    """Print warning message."""
    print(f"⚠ {text}")


def print_error(text: str):
    """Print error message."""
    print(f"✗ {text}", file=sys.stderr)


def detect_available_cameras(max_cameras: int = 5) -> dict:
    """Detect available cameras on the system."""
    import cv2
    
    available = {}
    for i in range(max_cameras):
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    h, w = frame.shape[:2]
                    available[i] = f"Camera {i}: {w}x{h}"
                    cap.release()
        except Exception:
            pass
    
    return available


# ============================================================================
# COMMAND HANDLERS
# ============================================================================

def cmd_list_cameras(args):
    """List available cameras."""
    print_header("AVAILABLE CAMERAS")
    
    available = detect_available_cameras()
    
    if not available:
        print_error("No cameras detected!")
        print_info("Check USB connections or permissions.")
        return False
    
    for camera_id, info in available.items():
        print(f"  {info}")
    
    return True


def cmd_calibrate(args):
    """Run calibration."""
    print_header("INTRINSICS CALIBRATION")
    
    # Validate camera
    camera_id = args.camera
    available = detect_available_cameras()
    
    if camera_id not in available:
        print_error(f"Camera {camera_id} not available!")
        print_info("Available cameras:")
        for cid, info in available.items():
            print(f"  {info}")
        return False
    
    print_info(f"Using {available[camera_id]}")
    
    # Parse resolution
    if args.resolution:
        try:
            w, h = map(int, args.resolution.split('x'))
            resolution = (w, h)
        except ValueError:
            print_error(f"Invalid resolution: {args.resolution}")
            print_info("Use format: WIDTHxHEIGHT (e.g., 1920x1080)")
            return False
    else:
        # Use default based on preset
        if args.preset in DEFAULT_CAMERAS:
            resolution = DEFAULT_CAMERAS[args.preset]
        else:
            resolution = (1920, 1080)
    
    print_info(f"Resolution: {resolution[0]}x{resolution[1]}")
    
    # Parse board
    if args.board not in STANDARD_BOARDS:
        print_error(f"Unknown board: {args.board}")
        print_info(f"Available boards: {', '.join(STANDARD_BOARDS.keys())}")
        return False
    
    board = STANDARD_BOARDS[args.board]
    print_info(f"Board: {board.name} ({board.size[0]}x{board.size[1]})")
    
    # Parse capture mode
    try:
        capture_mode = CaptureMode[args.mode.upper()]
    except KeyError:
        print_error(f"Unknown mode: {args.mode}")
        print_info(f"Available modes: {', '.join([m.value for m in CaptureMode])}")
        return False
    
    print_info(f"Mode: {capture_mode.value}")
    
    # Create calibrator
    print_section("INITIALIZING CALIBRATOR")
    
    try:
        camera_name = f"Camera_{camera_id}"
        camera_config = CameraConfig(
            name=camera_name,
            camera_index=camera_id,
            resolution=resolution
        )
        calibrator = CameraIntrinsicsCalibrator(camera_config, board)
        print_success("Calibrator initialized")
    except Exception as e:
        print_error(f"Failed to initialize calibrator: {e}")
        return False
    
    # Capture images based on mode
    print_section(f"CAPTURING IMAGES ({capture_mode.value.upper()})")
    
    try:
        target_images = args.count
        print_info(f"Target: {target_images} images")
        
        if capture_mode == CaptureMode.MANUAL:
            print_info("\nManual Mode - Full control over each capture")
            images = calibrator.capture_manual(target_images=target_images)
        
        elif capture_mode == CaptureMode.VIDEO:
            print_info("\nVideo Mode - Continuous extraction (faster)")
            images = calibrator.capture_video(target_images=target_images)
        
        elif capture_mode == CaptureMode.CONSERVATIVE:
            print_info("\nConservative Mode - Video + quality checks")
            images = calibrator.capture_conservative(target_images=target_images)
        
        if not images:
            print_error("No images captured!")
            return False
        
        print_success(f"Captured {len(images)} images")
    
    except KeyboardInterrupt:
        print_warning("\nCapture interrupted by user")
        return False
    except Exception as e:
        print_error(f"Capture failed: {e}")
        return False
    
    # Process dataset
    print_section("PROCESSING DATASET")
    
    try:
        print_info("Detecting CharUco corners...")
        corners, ids, rejected = calibrator.process_dataset(images)
        print_success(f"Found {len(corners)} valid frames with corners")
        
        if len(corners) < 5:
            print_error("Not enough valid frames for calibration!")
            print_info("Need at least 5 frames with detected corners")
            return False
    
    except Exception as e:
        print_error(f"Processing failed: {e}")
        return False
    
    # Calibrate
    print_section("CALIBRATING CAMERA")
    
    try:
        print_info("Computing camera matrix and distortion coefficients...")
        result = calibrator.calibrate(corners, ids, images, capture_mode.value)
        print_success("Calibration complete!")
        
        # Display results
        print_section("CALIBRATION RESULTS")
        print(f"\nReprojection Error: {result.reprojection_error:.4f} px")
        print(f"Focal Length (fx): {result.camera_matrix[0, 0]:.2f}")
        print(f"Focal Length (fy): {result.camera_matrix[1, 1]:.2f}")
        print(f"Principal Point (cx): {result.camera_matrix[0, 2]:.2f}")
        print(f"Principal Point (cy): {result.camera_matrix[1, 2]:.2f}")
        print(f"\nDistortion Coefficients: {result.distortion.flatten()}")
        print(f"Image Quality Score: {result.quality_score:.2%}")
        
        # Quality assessment
        if result.reprojection_error < 0.5:
            quality_level = "EXCELLENT"
        elif result.reprojection_error < 1.0:
            quality_level = "GOOD"
        elif result.reprojection_error < 2.0:
            quality_level = "ACCEPTABLE"
        else:
            quality_level = "POOR"
        
        print(f"\nQuality Assessment: {quality_level}")
    
    except Exception as e:
        print_error(f"Calibration failed: {e}")
        return False
    
    # Save results
    print_section("SAVING CALIBRATION")
    
    try:
        files = calibrator.save_calibration(result)
        print_success(f"Calibration saved ({len(files)} formats)")
        
        for fmt, filepath in files.items():
            print_info(f"  {fmt.upper()}: {filepath}")
    
    except Exception as e:
        print_error(f"Save failed: {e}")
        return False
    
    print_section("CALIBRATION COMPLETE")
    print_success("Ready to use!")
    print_info(f"Load calibration in your application and enjoy accurate vision!")
    
    return True


def cmd_test(args):
    """Test calibration system."""
    print_header("CALIBRATION SYSTEM TEST")
    
    tests_passed = 0
    tests_failed = 0
    
    # Test 1: Import modules
    print_section("Test 1: Module Imports")
    try:
        from camera_intrinsics_calibrator import (
            CameraConfig, CharUcoBoardConfig, CameraIntrinsicsCalibrator
        )
        print_success("All modules imported successfully")
        tests_passed += 1
    except Exception as e:
        print_error(f"Import failed: {e}")
        tests_failed += 1
    
    # Test 2: Camera detection
    print_section("Test 2: Camera Detection")
    try:
        available = detect_available_cameras()
        if available:
            print_success(f"Found {len(available)} camera(s)")
            for cid, info in available.items():
                print_info(f"  {info}")
            tests_passed += 1
        else:
            print_warning("No cameras detected")
            tests_passed += 1  # Not a failure
    except Exception as e:
        print_error(f"Camera detection failed: {e}")
        tests_failed += 1
    
    # Test 3: Configuration validation
    print_section("Test 3: Configuration Validation")
    try:
        camera_config = CameraConfig("Test", camera_index=0, resolution=(1920, 1080))
        board_config = CharUcoBoardConfig(
            "test_board", "DICT_4X4_50", (5, 7), 30.0, 18.0
        )
        print_success("Configurations created successfully")
        tests_passed += 1
    except Exception as e:
        print_error(f"Configuration failed: {e}")
        tests_failed += 1
    
    # Test 4: Output directory creation
    print_section("Test 4: Output Directory Setup")
    try:
        output_dir = Path("/tmp/urc_calibration_test")
        output_dir.mkdir(parents=True, exist_ok=True)
        if output_dir.exists():
            print_success(f"Output directory created: {output_dir}")
            tests_passed += 1
        else:
            print_error("Failed to create output directory")
            tests_failed += 1
    except Exception as e:
        print_error(f"Directory setup failed: {e}")
        tests_failed += 1
    
    # Summary
    print_section("TEST SUMMARY")
    print(f"Passed: {tests_passed}")
    print(f"Failed: {tests_failed}")
    
    if tests_failed == 0:
        print_success("All tests passed! System is ready to use.")
        return True
    else:
        print_warning(f"{tests_failed} test(s) failed. Check errors above.")
        return False


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Camera Intrinsics Calibration CLI - URC 2026",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # List available cameras
  python3 calibration_cli.py --list-cameras
  
  # Calibrate camera 0 in manual mode (detailed control)
  python3 calibration_cli.py --camera 0 --mode manual --count 50
  
  # Calibrate camera 0 in video mode (faster)
  python3 calibration_cli.py --camera 0 --mode video --count 30
  
  # Calibrate with specific resolution
  python3 calibration_cli.py --camera 0 --resolution 1920x1080 --mode manual
  
  # Test system
  python3 calibration_cli.py --test
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Commands')
    
    # Calibrate command
    calibrate_parser = subparsers.add_parser('calibrate', help='Run calibration')
    calibrate_parser.add_argument('--camera', type=int, default=0, help='Camera index')
    calibrate_parser.add_argument('--mode', default='manual', 
                                 choices=['manual', 'video', 'conservative'],
                                 help='Capture mode')
    calibrate_parser.add_argument('--count', type=int, default=50, 
                                 help='Target number of images')
    calibrate_parser.add_argument('--board', default='board_5x7',
                                 help='Board type')
    calibrate_parser.add_argument('--preset', default='fullhd',
                                 choices=['default', 'hd', 'fullhd', '2k'],
                                 help='Resolution preset')
    calibrate_parser.add_argument('--resolution', help='Custom resolution (WIDTHxHEIGHT)')
    
    # List cameras command
    list_parser = subparsers.add_parser('list', help='List available cameras')
    
    # Test command
    test_parser = subparsers.add_parser('test', help='Test system')
    
    # Handle legacy command structure (backward compatibility)
    if len(sys.argv) > 1 and sys.argv[1] == '--list-cameras':
        args = argparse.Namespace(command='list')
        return cmd_list_cameras(args)
    elif len(sys.argv) > 1 and sys.argv[1] == '--test':
        args = argparse.Namespace(command='test')
        return cmd_test(args)
    elif len(sys.argv) > 1 and sys.argv[1] == '--camera':
        # Build new-style args from old format
        parser_old = argparse.ArgumentParser()
        parser_old.add_argument('--camera', type=int, default=0)
        parser_old.add_argument('--mode', default='manual')
        parser_old.add_argument('--count', type=int, default=50)
        parser_old.add_argument('--board', default='board_5x7')
        parser_old.add_argument('--preset', default='fullhd')
        parser_old.add_argument('--resolution', default=None)
        
        args = parser_old.parse_args()
        args.command = 'calibrate'
        return cmd_calibrate(args)
    
    args = parser.parse_args()
    
    if args.command == 'calibrate':
        return cmd_calibrate(args)
    elif args.command == 'list':
        return cmd_list_cameras(args)
    elif args.command == 'test':
        return cmd_test(args)
    else:
        parser.print_help()
        return False


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)

