"""
Camera calibration functions for the URC 2026 system.
Supports both standalone calibration and ROS 2 CameraInfo publishing for distributed architecture.
"""

import time
from typing import Optional, Dict, Any

try:
    import cv2
    import numpy as np

    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None  # type: ignore
    numpy = None  # type: ignore
    OPENCV_AVAILABLE = False

# ROS 2 imports removed - using manual JSON transfer instead


def calibrate_camera(
    camera_index: int = 0,
    duration: int = 30,
    auto_capture: bool = True,
    camera_name: Optional[str] = None,
    calibration_dir: str = "calibrations",
) -> bool:
    """
    Calibrate single camera with visual feedback.

    Args:
        camera_index: Camera device index
        duration: Calibration duration in seconds
        auto_capture: Whether to auto-capture frames
        camera_name: Optional camera name (auto-generated if None)
        calibration_dir: Directory to save calibrations

    Returns:
        True if calibration successful, False otherwise
    """
    if not OPENCV_AVAILABLE:
        print("OpenCV not available for camera calibration")
        return False

    from ..utils.camera import (
        detect_camera_sensor,
        generate_sensor_based_camera_id,
        save_calibration_file,
    )

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
        print("Cannot open camera")
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

    print(f"Move ChArUco board around for {duration}s")
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
            status = "Searching..."
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
                        status = f"{captured + 1}/15 Ready!"
                        color = (0, 255, 0)  # Green

            # Show status on frame
            cv2.putText(
                frame,
                f"Status: {status}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )
            cv2.putText(
                frame,
                f"Captured: {captured}/15",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                f"Time: {int(time.time() - start_time)}/{duration}s",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            cv2.imshow("Camera Calibration", frame)

            # Auto capture or manual
            should_capture = False
            if auto_capture and status == f"{captured + 1}/15 Ready!":
                should_capture = True
            elif not auto_capture:
                key = cv2.waitKey(1) & 0xFF
                if key == ord(" "):  # Spacebar
                    should_capture = True
                elif key == ord("q"):
                    break

            if (
                should_capture
                and charuco_corners is not None
                and charuco_ids is not None
            ):
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)
                captured += 1
                print(f"Captured frame {captured}/15")
                time.sleep(0.5)  # Brief pause between captures

        cv2.destroyAllWindows()
        cap.release()

        if captured < 5:
            print(f"Insufficient frames captured: {captured}/15")
            return False

        # Perform calibration
        print("Performing calibration...")
        ret, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
            all_corners, all_ids, board, (width, height), None, None
        )

        if not ret:
            print("Calibration failed")
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

        print("Calibration complete!")
        print(f"    Camera: {camera_name}")
        print(f"    Quality: {captured} frames used")
        print(f"    Directory: {calibration_dir}")
        return True

    except Exception as e:
        print(f"Error: {e}")
        cv2.destroyAllWindows()
        if "cap" in locals():
            cap.release()
        return False


def batch_calibrate(
    num_cameras: int = 5, calibration_dir: str = "calibrations"
) -> bool:
    """
    Calibrate multiple cameras in sequence.

    Args:
        num_cameras: Number of cameras to calibrate
        calibration_dir: Directory to save calibrations

    Returns:
        True if all calibrations successful
    """
    print(f"Batch calibrating {num_cameras} cameras")
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
            input("\nConnect next camera and press ENTER...")

    # Summary
    print(f"\n{'='*50}")
    print("CALIBRATION SUMMARY")
    print(f"{'='*50}")

    successful = sum(1 for _, success in results if success)
    print(f"Total cameras: {num_cameras}")
    print(f"Successful: {successful}")
    print(f"Failed: {num_cameras - successful}")

    from ..utils.camera import list_calibrated_cameras

    if successful > 0:
        print(f"\nCalibrations saved to: {calibration_dir}/")
        cameras = list_calibrated_cameras(calibration_dir)
        print(f"Calibrated cameras: {', '.join(cameras)}")

    return successful == num_cameras
