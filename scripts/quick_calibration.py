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
    try:
        # Try to create and destroy a test window
        cv2.namedWindow("__gui_test__", cv2.WINDOW_NORMAL)
        cv2.destroyWindow("__gui_test__")
        GUI_AVAILABLE = True
    except (cv2.error, AttributeError):
        # opencv-python-headless doesn't support GUI
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

    # Open camera using robust helper
    from utils.camera import setup_camera_capture
    cap = setup_camera_capture(camera_index, width=1280, height=720, fps=30)
    
    if cap is None:
        print(f" Cannot open camera {camera_index}")
        return False

    # Get actual resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Resolution: {width}x{height}")

    # Setup detection
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # Try different board sizes based on detected marker IDs
    # User specified 6x8 board (48 markers, IDs 0-47)
    squares_x, squares_y = 6, 8  # 48 markers (IDs 0-47)
    board = cv2.aruco.CharucoBoard((squares_x, squares_y), 0.030, 0.018, aruco_dict)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    charuco_detector = cv2.aruco.CharucoDetector(board)

    print(f"Using {squares_x}x{squares_y} ChArUco board for calibration (max {squares_x * squares_y} markers)")
    print("  Detected marker IDs in range suggest this board size")

    # Storage for calibration data
    all_corners = []
    all_ids = []
    captured = 0

    print(f" Move ChArUco board around for {duration}s")
    print("   Auto-capture: ON" if auto_capture else "   Press SPACE to capture")
    if GUI_AVAILABLE:
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

    try:
        while time.time() - start_time < duration and captured < 15:
            # Try to read a frame with retry logic for V4L2 stability
            ret = False
            frame = None
            for retry in range(3):
                ret, frame = cap.read()
                if ret and frame is not None:
                    break
                time.sleep(0.1)

            if not ret:
                # #region agent log - Frame read failure
                log_entry = {
                    "timestamp": int(time.time() * 1000),
                    "location": "quick_calibration.py:frame_read",
                    "message": "Frame read failed after retries",
                    "data": {"camera_index": camera_index, "frame_count": frame_count, "elapsed": time.time() - start_time},
                    "sessionId": "debug-session",
                    "runId": "calibration_debug",
                    "hypothesisId": "frame_read_fail"
                }
                try:
                    with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                        f.write(json.dumps(log_entry) + "\n")
                except: pass
                # #endregion
                break

            frame_count += 1
            marker_count = 0
            status = " Searching..."
            color = (0, 0, 255)  # Red

            # #region agent log - Frame processing start
            log_entry = {
                "timestamp": int(time.time() * 1000),
                "location": "quick_calibration.py:frame_process",
                "message": "Processing frame",
                "data": {"camera_index": camera_index, "frame_count": frame_count, "frame_shape": list(frame.shape) if frame is not None else None},
                "sessionId": "debug-session",
                "runId": "calibration_debug",
                "hypothesisId": "frame_processing"
            }
            try:
                with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                    f.write(json.dumps(log_entry) + "\n")
            except: pass
            # #endregion

            # Detect markers
            corners, ids, _ = detector.detectMarkers(frame)
            marker_count = len(ids) if ids is not None else 0

            # #region agent log - Marker detection results
            log_entry = {
                "timestamp": int(time.time() * 1000),
                "location": "quick_calibration.py:marker_detect",
                "message": "Marker detection results",
                "data": {"camera_index": camera_index, "frame_count": frame_count, "marker_count": marker_count, "ids_found": ids.tolist() if ids is not None else None},
                "sessionId": "debug-session",
                "runId": "calibration_debug",
                "hypothesisId": "marker_detection"
            }
            try:
                with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                    f.write(json.dumps(log_entry) + "\n")
            except: pass
            # #endregion

            if ids is not None:
                # Always attempt ChArUco board detection when we have markers
                try:
                    charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(
                        frame
                    )
                    charuco_corner_count = len(charuco_corners) if charuco_corners is not None else 0

                    # #region agent log - ChArUco detection results
                    log_entry = {
                        "timestamp": int(time.time() * 1000),
                        "location": "quick_calibration.py:charuco_detect",
                        "message": "ChArUco detection results",
                        "data": {"camera_index": camera_index, "frame_count": frame_count, "charuco_corners": charuco_corner_count, "charuco_ids_count": len(charuco_ids) if charuco_ids is not None else 0, "board_size": f"{squares_x}x{squares_y}"},
                        "sessionId": "debug-session",
                        "runId": "calibration_debug",
                        "hypothesisId": "charuco_detection"
                    }
                    try:
                        with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                            f.write(json.dumps(log_entry) + "\n")
                    except: pass
                    # #endregion
                except Exception as e:
                    # Log ChArUco detection errors
                    log_entry = {
                        "timestamp": int(time.time() * 1000),
                        "location": "quick_calibration.py:charuco_error",
                        "message": "ChArUco detection error",
                        "data": {"camera_index": camera_index, "frame_count": frame_count, "error": str(e)},
                        "sessionId": "debug-session",
                        "runId": "calibration_debug",
                        "hypothesisId": "charuco_error"
                    }
                    try:
                        with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                            f.write(json.dumps(log_entry) + "\n")
                    except: pass
                    # #endregion
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

                    # #region agent log - Capture decision
                    log_entry = {
                        "timestamp": int(time.time() * 1000),
                        "location": "quick_calibration.py:capture_decision",
                        "message": "Capture decision made",
                        "data": {"camera_index": camera_index, "frame_count": frame_count, "should_capture": should_capture, "reason": capture_reason, "current_captured": captured},
                        "sessionId": "debug-session",
                        "runId": "calibration_debug",
                        "hypothesisId": "capture_logic"
                    }
                    try:
                        with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                            f.write(json.dumps(log_entry) + "\n")
                    except: pass
                    # #endregion

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
                # #region agent log - No markers found
                log_entry = {
                    "timestamp": int(time.time() * 1000),
                    "location": "quick_calibration.py:no_markers",
                    "message": "No markers detected in frame",
                    "data": {"camera_index": camera_index, "frame_count": frame_count},
                    "sessionId": "debug-session",
                    "runId": "calibration_debug",
                    "hypothesisId": "no_markers_found"
                }
                try:
                    with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                        f.write(json.dumps(log_entry) + "\n")
                except: pass
                # #endregion

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

            if GUI_AVAILABLE:
                try:
                    cv2.imshow(f"Camera {camera_index}", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                except cv2.error:
                    # GUI not actually available despite check
                    # GUI_AVAILABLE is already False from module initialization
                    print("  Warning: GUI not available, switching to headless mode")
            else:
                # Headless mode: print status periodically
                if frame_count % 30 == 0:  # Print every 30 frames (~1 second at 30fps)
                    print(f"  Status: {status.strip()} | Captured: {captured}/15 | Time: {time_left}s")

            # #region agent log - Frame processing complete
            log_entry = {
                "timestamp": int(time.time() * 1000),
                "location": "quick_calibration.py:frame_complete",
                "message": "Frame processing complete",
                "data": {"camera_index": camera_index, "frame_count": frame_count, "final_status": status.strip(), "total_captured": captured, "time_left": time_left},
                "sessionId": "debug-session",
                "runId": "calibration_debug",
                "hypothesisId": "frame_completion"
            }
            try:
                with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                    f.write(json.dumps(log_entry) + "\n")
            except: pass
            # #endregion

    finally:
        cap.release()
        if GUI_AVAILABLE:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass  # Ignore errors if windows already closed

    if captured < 5:
        print(f" Only {captured} frames (need 5+)")
        return False

    # Calibrate
    print(" Computing calibration...")
    try:
        if len(all_corners) == 0 or len(all_ids) == 0:
            print(" No valid calibration data")
            return False

        # Prepare data for standard camera calibration
        obj_points = []  # 3D points in world coordinates
        img_points = []  # 2D points in image coordinates

        for corners, ids in zip(all_corners, all_ids):
            if corners is not None and ids is not None and len(corners) > 0:
                # Get 3D object points for these ChArUco corners
                board_corners = board.getChessboardCorners()
                # Filter to only the detected corner IDs
                detected_obj_pts = board_corners[ids.flatten()]
                obj_points.append(detected_obj_pts.astype(np.float32))
                img_points.append(corners.astype(np.float32))

        if len(obj_points) == 0 or len(img_points) == 0:
            print(" No valid calibration data after processing")
            return False

        # Calibrate camera using standard calibration
        # Initialize camera matrix with reasonable defaults
        focal_length = width * 0.8  # Rough estimate based on field of view
        camera_matrix = np.array([
            [focal_length, 0, width/2],
            [0, focal_length, height/2],
            [0, 0, 1]
        ], dtype=np.float32)
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        ret, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
            obj_points, img_points, (width, height), camera_matrix, dist_coeffs,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS
        )

        if not ret:
            print(" Calibration failed")
            return False

        # Calculate reprojection error
        total_error = 0
        for i, (obj_pts, img_pts) in enumerate(zip(obj_points, img_points)):
            img_pts_projected, _ = cv2.projectPoints(obj_pts, np.zeros((3, 1)), np.zeros((3, 1)), camera_matrix, dist_coeffs)
            error = cv2.norm(img_pts, img_pts_projected, cv2.NORM_L2) / len(img_pts_projected)
            total_error += error

        reprojection_error = total_error / len(obj_points)

        # Prepare calibration data
        calib_data = {
            "camera_index": camera_index,
            "sensor": sensor,
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": dist_coeffs.flatten().tolist(),
            "image_width": width,
            "image_height": height,
            "frames_used": captured,
            "reprojection_error": reprojection_error,
            "opencv_version": cv2.__version__,
            "calibration_method": "charuco_manual",
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
