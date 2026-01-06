#!/usr/bin/env python3
"""
Simple RPi Camera Calibration - URC 2026

Calibrate cameras with visual feedback. For one-time setup and occasional recalibration.

Usage:
    python quick_calibration.py --camera 0
    python quick_calibration.py --batch 5
"""

import argparse
import json
import time
from typing import Optional

# Initialize globals
OPENCV_AVAILABLE = False
GUI_AVAILABLE = False

# Import cv2 and check GUI support
try:
    import cv2
    import numpy as np

    OPENCV_AVAILABLE = True
    # Check if GUI support is available (headless builds don't have highgui)
    GUI_AVAILABLE = False
    try:
        # First check if highgui module is available
        if hasattr(cv2, 'namedWindow') and hasattr(cv2, 'destroyWindow'):
            # Try to create and destroy a test window
            cv2.namedWindow("__gui_test__", cv2.WINDOW_NORMAL)
            cv2.destroyWindow("__gui_test__")
            GUI_AVAILABLE = True
    except (cv2.error, AttributeError, Exception):
        # opencv-python-headless doesn't support GUI, or other GUI issues
        GUI_AVAILABLE = False
except ImportError:
    cv2 = None  # type: ignore
    OPENCV_AVAILABLE = False
    GUI_AVAILABLE = False

# Import shared utilities
import sys
import os

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from utils.camera import (
    detect_camera_sensor,
    generate_sensor_based_camera_id,
    save_calibration_file,
    list_calibrated_cameras,
    rename_calibration_file,
)

# Import charuco calibration library for better stability
from charuco.calibrator import CameraCalibrator


def calibrate_camera(
    camera_index: int = 0,
    duration: int = 30,
    auto_capture: bool = True,
    camera_name: Optional[str] = None,
    calibration_dir: str = "calibrations",
    force_gui: bool = False,
) -> bool:
    """Calibrate single camera with visual feedback."""

    print(f"Calibrating Camera {camera_index}")
    print("=" * 40)

    # Get actual device path for sensor detection
    from utils.camera import get_camera_device_path
    device_path = get_camera_device_path(camera_index)
    
    # Detect sensor before opening camera
    sensor = detect_camera_sensor()
    if sensor and sensor != "unknown_sensor":
        print(f"Detected sensor: {sensor.upper()}")
    else:
        print("Could not detect camera sensor")

    # Generate camera name if not provided
    if camera_name is None:
        camera_name = generate_sensor_based_camera_id(calibration_dir)
        print(f"Auto-assigned camera ID: {camera_name}")

    print(f"Calibrating Camera {camera_index} ({camera_name})")
    print("=" * 50)

    # Open camera using robust helper - optimize for smooth streaming
    from utils.camera import setup_camera_capture
    cap = setup_camera_capture(camera_index, width=1280, height=720, fps=30)
    
    if cap is None:
        print(f" Cannot open camera {camera_index}")
        return False

    # Get actual resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Resolution: {width}x{height}")

    # Setup detection using charuco library configuration for better stability
    calibrator = CameraCalibrator()

    # User specified 6x8 board (48 markers, IDs 0-47)
    squares_x, squares_y = 6, 8  # 48 markers (IDs 0-47)

    # Create detectors for marker detection and charuco detection
    board_squares = (squares_x, squares_y)
    board = cv2.aruco.CharucoBoard(board_squares, 0.030, 0.018, calibrator.aruco_dict)
    aruco_detector = cv2.aruco.ArucoDetector(calibrator.aruco_dict, calibrator.aruco_params)
    charuco_detector = cv2.aruco.CharucoDetector(board, calibrator.charuco_params, calibrator.aruco_params)

    print(f"Using {squares_x}x{squares_y} ChArUco board for calibration (max {squares_x * squares_y} markers)")
    print("  Detected marker IDs in range suggest this board size")

    # Storage for calibration data
    all_corners = []
    all_ids = []
    captured = 0

    # Determine GUI mode
    use_gui = GUI_AVAILABLE or force_gui
    if force_gui and not GUI_AVAILABLE:
        print("   Warning: GUI forced but GUI support not detected - may not work")

    print(f" Move ChArUco board around for {duration}s")
    print("   Auto-capture: ON" if auto_capture else "   Press SPACE to capture")
    if use_gui:
        print("   GUI preview: ENABLED")
        print("   Press Q to quit\n")
    else:
        print("   Running in headless mode (no preview window)")
        print("   Status updates will be printed to console")
        print("   Press Ctrl+C to quit")
        print("   Note: Install 'opencv-python' (not headless) for GUI preview\n")

    start_time = time.time()
    frame_count = 0
    last_capture_time = 0  # Track time of last successful capture
    capture_delay = 1.5  # Minimum seconds between captures
    frame_skip_counter = 0  # For frame rate control

    try:
        while time.time() - start_time < duration and captured < 15:
            # Frame rate control - process every 2nd frame for smoother performance
            frame_skip_counter += 1
            if frame_skip_counter % 2 != 0:
                time.sleep(0.03)  # Brief pause to maintain timing
                continue

            # Try to read a frame with retry logic for V4L2 stability
            ret = False
            frame = None
            for retry in range(3):
                ret, frame = cap.read()
                if ret and frame is not None:
                    break
                time.sleep(0.1)

            if not ret:
                break

            frame_count += 1
            marker_count = 0
            status = " Searching..."
            color = (0, 0, 255)  # Red

            # Detect markers using ArUco detector
            corners, ids, _ = aruco_detector.detectMarkers(frame)
            marker_count = len(ids) if ids is not None else 0

            if ids is not None:
                # Always attempt ChArUco board detection when we have markers
                try:
                    charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(
                        frame
                    )
                    charuco_corner_count = len(charuco_corners) if charuco_corners is not None else 0
                except Exception as e:
                    charuco_corners = None
                    charuco_ids = None
                    charuco_corner_count = 0

                if charuco_corners is not None and len(charuco_corners) >= 4:
                    # Check timing restriction between captures
                    time_since_last_capture = time.time() - last_capture_time
                    if time_since_last_capture < capture_delay:
                        status = f" {captured + 1}/15 Wait {capture_delay - time_since_last_capture:.1f}s"
                        color = (0, 255, 255)  # Yellow
                        should_capture = False
                        capture_reason = "timing_restriction"
                    else:
                        status = f" {captured + 1}/15 Ready!"
                        color = (0, 255, 0)  # Green

                        # Auto-capture or manual
                        should_capture = False
                        capture_reason = "none"

                        if auto_capture:
                            should_capture = True
                            capture_reason = "auto_capture_enabled"
                        elif GUI_AVAILABLE:
                            key = cv2.waitKey(1) & 0xFF
                            if key == ord(" "):
                                should_capture = True
                                capture_reason = "space_pressed"

                    if should_capture:
                        all_corners.append(charuco_corners)
                        all_ids.append(charuco_ids)
                        captured += 1
                        last_capture_time = time.time()  # Record capture time
                        print(f"    Captured frame {captured}/15")

                    # Draw markers
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                elif charuco_corners is not None and len(charuco_corners) > 0:
                    status = f" {len(charuco_corners)} corners (need 4+ for capture)"
                    color = (0, 165, 255)  # Orange
                else:
                    status = f" {marker_count} markers (need board pattern)"
                    color = (0, 165, 255)  # Orange
            else:
                pass

            # Visual feedback overlay - use separate display frame to prevent artifacts
            display_frame = frame.copy()
            cv2.rectangle(display_frame, (10, 10), (400, 120), (0, 0, 0), -1)
            cv2.addWeighted(display_frame, 0.7, frame, 0.3, 0, display_frame)

            # Status
            cv2.putText(
                display_frame, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2
            )

            # Progress bar
            progress = min(captured / 15, 1.0)
            bar_width = int(progress * 360)
            cv2.rectangle(display_frame, (20, 50), (20 + bar_width, 60), color, -1)
            cv2.rectangle(display_frame, (20, 50), (380, 60), (64, 64, 64), 2)

            # Stats
            cv2.putText(
                display_frame,
                f"Frames: {captured}/15",
                (20, 85),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            time_left = int(duration - (time.time() - start_time))
            cv2.putText(
                display_frame,
                f"Time: {time_left}s",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

            if use_gui:
                try:
                    cv2.imshow(f"Camera {camera_index}", display_frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                except cv2.error as e:
                    # GUI not actually available despite check
                    print(f"  Warning: GUI display failed ({e}), switching to headless mode")
                    use_gui = False
            else:
                # Headless mode: print status periodically
                if frame_count % 30 == 0:  # Print every 30 frames (~1 second at 30fps)
                    print(f"  Status: {status.strip()} | Captured: {captured}/15 | Time: {time_left}s")

    finally:
        cap.release()
        if use_gui:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass  # Ignore errors if windows already closed

    if captured < 5:
        print(f" Only {captured} frames (need 5+)")
        return False

    # Calibrate using proper ChArUco calibration
    print(" Computing calibration...")
    try:
        if len(all_corners) == 0 or len(all_ids) == 0:
            print(" No valid calibration data")
            return False

        # Validate and filter calibration data
        valid_corners = []
        valid_ids = []

        for i, (corners, ids) in enumerate(zip(all_corners, all_ids)):
            # Ensure corners and ids are valid numpy arrays
            if corners is not None and ids is not None and len(corners) >= 6:
                # Ensure correct shapes for calibrateCameraCharucoExtended
                # Corners should be (N, 1, 2) and IDs should be (N, 1)
                if len(corners.shape) == 2:
                    corners = corners.reshape(-1, 1, 2)
                if len(ids.shape) == 1:
                    ids = ids.reshape(-1, 1)

                valid_corners.append(corners.astype(np.float32))
                valid_ids.append(ids.astype(np.int32))
            else:
                if corners is not None:
                    print(f"  Warning: Skipping frame {i+1} with only {len(corners)} corners (need 6+)")

        if len(valid_corners) < 5:
            print(f" Not enough valid frames for calibration (need 5+, got {len(valid_corners)})")
            return False

        print(f"  Using {len(valid_corners)} valid frames out of {len(all_corners)} captured")

        # Use the dedicated ChArUco calibration function with error estimation
        # This is the proper implementation for ChArUco boards
        try:
            ret, camera_matrix, dist_coeffs, rvecs, tvecs, _, _, per_view_errors = cv2.aruco.calibrateCameraCharucoExtended(
                valid_corners, valid_ids, board, (width, height), None, None
            )
        except cv2.error as e:
            print(f" OpenCV calibration error: {e}")
            return False

        if not ret:
            print(" Calibration failed")
            return False

        # Calculate average reprojection error
        reprojection_error = np.mean(per_view_errors) if len(per_view_errors) > 0 else float("inf")

        # Prepare calibration data
        total_points = sum(len(c) for c in valid_corners)
        calib_data = {
            "camera_index": camera_index,
            "sensor": sensor,
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": dist_coeffs.tolist(),
            "image_width": width,
            "image_height": height,
            "frames_used": len(valid_corners),
            "frames_captured": captured,
            "total_points_used": total_points,
            "reprojection_error": float(reprojection_error),
            "opencv_version": cv2.__version__,
            "calibration_method": "charuco_extended_proper",
        }

        # Save calibration file
        save_calibration_file(calib_data, camera_name, calibration_dir)

        print(" Calibration complete!")
        print(f"    Camera: {camera_name}")
        print(f"    Quality: {captured} frames used")
        print(".4f")
        print(f"    Directory: {calibration_dir}")
        return True

    except Exception as e:
        print(f" Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def batch_calibrate(
    num_cameras: int = 5, calibration_dir: str = "calibrations"
) -> bool:
    """Calibrate multiple cameras in sequence."""
    print(f" Batch calibrating {num_cameras} cameras")
    print("=" * 50)

    results = []
    for i in range(num_cameras):
        print(f"\n{'='*50}")
        print(f"Camera {i+1}/{num_cameras} (Device {i})")
        print(f"{'='*50}")

        success = calibrate_camera(
            camera_index=i, duration=25, calibration_dir=calibration_dir
        )
        results.append((i, success))

        if success:
            print(f"Camera {i}: SUCCESS")
        else:
            print(f"Camera {i}: FAILED")

        if i < num_cameras - 1:
            input("\n Connect next camera and press ENTER...")

    # Summary
    print(f"\n{'='*50}")
    print(" CALIBRATION SUMMARY")
    print(f"{'='*50}")

    successful = sum(1 for _, success in results if success)
    print(f"Total cameras: {num_cameras}")
    print(f"Successful: {successful}")
    print(f"Failed: {num_cameras - successful}")

    if successful > 0:
        print(f"\nCalibrations saved to: {calibration_dir}/")
        cameras = list_calibrated_cameras(calibration_dir)
        print(f"Calibrated cameras: {', '.join(cameras)}")

    return successful == num_cameras


def main() -> None:
    parser = argparse.ArgumentParser(
        description="RPi Camera Calibration with Sensor-Based IDs",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python quick_calibration.py                    # Calibrate camera 0 with auto ID
  python quick_calibration.py --camera 1        # Calibrate camera 1
  python quick_calibration.py --name front_left # Custom camera name
  python quick_calibration.py --batch 3         # Calibrate 3 cameras in sequence
  python quick_calibration.py --list-cameras    # List all calibrated cameras
  python quick_calibration.py --rename-camera imx219_1 front_left
  python quick_calibration.py --calibration-dir my_cals  # Custom output directory

Sensor-based automatic IDs:
  - IMX219 cameras: imx219_1, imx219_2, imx219_3...
  - IMX477 cameras: imx477_1, imx477_2, imx477_3...
  - Unknown sensors: camera_1, camera_2, camera_3...

Files saved as: calibrations/{camera_name}.json
        """,
    )

    parser.add_argument(
        "--camera", type=int, default=0, help="Camera device index (default: 0)"
    )
    parser.add_argument(
        "--name", "-n", help="Custom camera name (default: auto-generated)"
    )
    parser.add_argument(
        "--batch", type=int, help="Batch calibrate N cameras in sequence"
    )
    parser.add_argument("--manual", action="store_true", help="Manual capture mode")
    parser.add_argument(
        "--calibration-dir",
        "-d",
        default="calibrations",
        help="Calibration output directory (default: calibrations)",
    )
    parser.add_argument(
        "--list-cameras", action="store_true", help="List all calibrated cameras"
    )
    parser.add_argument(
        "--rename-camera",
        nargs=2,
        metavar=("OLD_NAME", "NEW_NAME"),
        help="Rename a camera calibration file",
    )
    parser.add_argument(
        "--force-gui",
        action="store_true",
        help="Force GUI mode even if GUI support detection fails",
    )

    args = parser.parse_args()

    # Handle different command modes
    if args.list_cameras:
        cameras = list_calibrated_cameras(args.calibration_dir)
        if cameras:
            print(f"📹 Calibrated Cameras ({len(cameras)} total):")
            print("=" * 40)
            for cam in cameras:
                print(f"  {cam}")
        else:
            print("No cameras calibrated yet.")
        return

    if args.rename_camera:
        old_name, new_name = args.rename_camera
        rename_calibration_file(old_name, new_name, args.calibration_dir)
        return

    # Calibration modes
    if args.batch:
        success = batch_calibrate(args.batch, args.calibration_dir)
    else:
        auto_capture = not args.manual
        success = calibrate_camera(
            camera_index=args.camera,
            auto_capture=auto_capture,
            camera_name=args.name,
            calibration_dir=args.calibration_dir,
            force_gui=args.force_gui,
        )

    exit(0 if success else 1)


if __name__ == "__main__":
    main()
