#!/usr/bin/env python3
"""
Simple RPi Camera Calibration - URC 2026

Calibrate cameras with visual feedback. For one-time setup and occasional recalibration.

Usage:
    python quick_calibration.py --camera 0
    python quick_calibration.py --batch 5
"""

import argparse
import time
from typing import Optional

try:
    import cv2
    import numpy as np

    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None  # type: ignore
    OPENCV_AVAILABLE = False

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


def calibrate_camera(
    camera_index: int = 0,
    duration: int = 30,
    auto_capture: bool = True,
    camera_name: Optional[str] = None,
    calibration_dir: str = "calibrations",
) -> bool:
    """Calibrate single camera with visual feedback."""

    print(f"Calibrating Camera {camera_index}")
    print("=" * 40)

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

    # Open camera
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(" Cannot open camera")
        return False

    # Configure camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Resolution: {width}x{height}")

    # Setup detection
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard((7, 5), 0.030, 0.018, aruco_dict)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    charuco_detector = cv2.aruco.CharucoDetector(board)

    # Storage for calibration data
    all_corners = []
    all_ids = []
    captured = 0

    print(f" Move ChArUco board around for {duration}s")
    print("   Auto-capture: ON" if auto_capture else "   Press SPACE to capture")
    print("   Press Q to quit\n")

    start_time = time.time()
    frame_count = 0

    try:
        while time.time() - start_time < duration and captured < 15:
            ret, frame = cap.read()
            if not ret:
                break

            frame_count += 1
            marker_count = 0
            status = " Searching..."
            color = (0, 0, 255)  # Red

            # Detect markers
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                marker_count = len(ids)

                # Check for good board detection
                if marker_count >= 4:
                    charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(
                        frame
                    )
                    if charuco_corners is not None and len(charuco_corners) >= 6:
                        status = f" {captured + 1}/15 Ready!"
                        color = (0, 255, 0)  # Green

                        # Auto-capture or manual
                        if auto_capture or cv2.waitKey(1) == ord(" "):
                            all_corners.append(charuco_corners)
                            all_ids.append(charuco_ids)
                            captured += 1
                            print(f"    Captured frame {captured}/15")

                        # Draw markers
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    else:
                        status = f" {marker_count} markers (need better angle)"
                        color = (0, 165, 255)  # Orange
                else:
                    status = f" {marker_count} markers (need 4+)"
                    color = (0, 165, 255)  # Orange

            # Visual feedback overlay
            overlay = frame.copy()
            cv2.rectangle(overlay, (10, 10), (400, 120), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

            # Status
            cv2.putText(
                frame, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2
            )

            # Progress bar
            progress = min(captured / 15, 1.0)
            bar_width = int(progress * 360)
            cv2.rectangle(frame, (20, 50), (20 + bar_width, 60), color, -1)
            cv2.rectangle(frame, (20, 50), (380, 60), (64, 64, 64), 2)

            # Stats
            cv2.putText(
                frame,
                f"Frames: {captured}/15",
                (20, 85),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            time_left = int(duration - (time.time() - start_time))
            cv2.putText(
                frame,
                f"Time: {time_left}s",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

            cv2.imshow(f"Camera {camera_index}", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

    if captured < 5:
        print(f" Only {captured} frames (need 5+)")
        return False

    # Calibrate
    print(" Computing calibration...")
    try:
        camera_matrix = np.zeros((3, 3))
        dist_coeffs = np.zeros((5, 1))

        ret, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
            all_corners, all_ids, board, (width, height), camera_matrix, dist_coeffs
        )

        if not ret:
            print(" Calibration failed")
            return False

        # Prepare calibration data
        calib_data = {
            "camera_index": camera_index,
            "sensor": sensor,
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": dist_coeffs.flatten().tolist(),
            "image_width": width,
            "image_height": height,
            "frames_used": captured,
            "reprojection_error": 0.0,
            "opencv_version": cv2.__version__,
            "calibration_method": "charuco",
        }

        # Save calibration file
        save_calibration_file(calib_data, camera_name, calibration_dir)

        print(" Calibration complete!")
        print(f"    Camera: {camera_name}")
        print(f"    Quality: {captured} frames used")
        print(f"    Directory: {calibration_dir}")
        return True

    except Exception as e:
        print(f" Error: {e}")
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
        )

    exit(0 if success else 1)


if __name__ == "__main__":
    main()
