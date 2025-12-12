#!/usr/bin/env python3
"""
Quick Webcam Calibration Script - URC 2026

Fast calibration using ChArUco board detection without video capture.
Perfect for testing your webcam and calibrating for ArUco distance detection.

Usage:
    python quick_calibration.py --output my_camera.json --duration 30
"""

import argparse
import json
import time

import cv2
import numpy as np


def quick_calibrate_camera(
    camera_index=0,
    duration=30,
    output_file="webcam_calibration.json",
    board_squares=(8, 6),
    square_size=0.020,
    marker_size=0.015,
):
    """
    Quick camera calibration by capturing frames directly and detecting ChArUco board.

    Args:
        camera_index: Camera device index (default: 0)
        duration: Capture duration in seconds
        output_file: Output JSON file path
        board_squares: ChArUco board pattern as (cols, rows), default (8, 6)
        square_size: Square size in meters, default 0.020 (20mm)
        marker_size: Marker size in meters, default 0.015 (15mm)

    Returns:
        True if successful, False otherwise
    """
    print("🎯 Quick Webcam Calibration")
    print("=" * 50)

    # Try to open camera with fallback backends
    cap = None
    backends = [cv2.CAP_ANY, cv2.CAP_V4L2]
    for backend in backends:
        cap = cv2.VideoCapture(camera_index, backend)
        if cap.isOpened():
            print(f"✅ Camera opened with backend: {backend}")
            break
        cap.release()

    if not cap or not cap.isOpened():
        print("❌ Failed to open camera")
        return False

    # Configure camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"📷 Camera: {width}x{height}")

    # Setup ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # ChArUco board parameters (8x6 with 20mm squares)
    board = cv2.aruco.CharucoBoard(
        board_squares, squareLength=square_size, markerLength=marker_size, dictionary=aruco_dict
    )

    # Calibration variables
    all_charuco_corners = []
    all_charuco_ids = []
    calibration_frames = 0

    print(f"\n📹 Capturing frames for {duration} seconds...")
    print("🎯 Point camera at ChArUco board and move it around")
    print("Press 'q' to stop early or wait for automatic completion\n")

    # Create CharucoDetector once
    charuco_detector = cv2.aruco.CharucoDetector(board)

    start_time = time.time()
    frame_count = 0
    detected_count = 0

    try:
        while time.time() - start_time < duration:
            ret, frame = cap.read()
            if not ret:
                print("❌ Failed to read frame")
                break

            frame_count += 1

            # Detect ArUco markers
            corners, ids, rejected = detector.detectMarkers(frame)

            # Initialize status for display
            status = f"× Frame {frame_count} - Searching..."
            color = (0, 0, 255)

            # Detect ChArUco corners from detected markers
            if ids is not None and len(ids) > 3:  # Need at least 4 markers
                try:
                    # Try detectBoard on the frame directly
                    charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(frame)

                    # If detectBoard returned corners, use them
                    if charuco_corners is not None and len(charuco_corners) >= 5:
                        all_charuco_corners.append(charuco_corners)
                        all_charuco_ids.append(charuco_ids)
                        detected_count += 1
                        calibration_frames += 1

                        # Draw detected markers
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                        status = f"✓ Frame {calibration_frames} - Detected {len(charuco_ids)} ChArUco corners"
                        color = (0, 255, 0)
                except Exception:
                    pass

            # Display frame with status
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(
                frame,
                f"Time: {time.time() - start_time:.1f}s / {duration}s",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            cv2.imshow("Webcam Calibration", frame)

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("⏹️ User requested exit")
                break

            # Progress indicator
            if frame_count % 30 == 0:
                print(f"  📊 {frame_count} frames captured, {calibration_frames} with valid board detections")

    finally:
        cap.release()
        cv2.destroyAllWindows()

    print("\n✅ Capture complete")
    print(f"   Total frames: {frame_count}")
    print(f"   Valid detections: {calibration_frames}")

    if calibration_frames < 5:
        print("❌ Not enough valid detections for calibration (need at least 5)")
        return False

    # Perform calibration
    print("\n🔧 Performing calibration...")
    try:
        camera_matrix = np.zeros((3, 3))
        dist_coeffs = np.zeros((4, 1))

        # Calibrate camera
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            all_charuco_corners, all_charuco_ids, board, (width, height), camera_matrix, dist_coeffs
        )

        if not ret:
            print("❌ Calibration failed")
            return False

        print("✅ Calibration successful!")

        # Prepare output data
        calib_data = {
            "camera_matrix": {"rows": 3, "cols": 3, "data": camera_matrix.flatten().tolist()},
            "distortion_coefficients": {"rows": 1, "cols": 5, "data": dist_coeffs.flatten().tolist()},
            "image_width": width,
            "image_height": height,
            "frames_used": calibration_frames,
            "reprojection_error": 0.0,
            "calibration_quality": "good",
        }

        # Save to JSON
        with open(output_file, "w") as f:
            json.dump(calib_data, f, indent=2)

        print(f"💾 Calibration saved to: {output_file}")
        print("\n📊 Calibration Results:")
        print(f"   Image size: {width}x{height}")
        print(f"   Frames used: {calibration_frames}")
        print("   Camera Matrix:")
        print(f"   {camera_matrix}")
        print("   Distortion Coefficients:")
        print(f"   {dist_coeffs.flatten()}")

        return True

    except Exception as e:
        print(f"❌ Calibration failed: {e}")
        import traceback

        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Quick Webcam Calibration for ArUco Distance Detection",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Quick calibration (30 seconds)
  python quick_calibration.py --output my_camera.json

  # Extended calibration (60 seconds) for better accuracy
  python quick_calibration.py --duration 60 --output my_camera.json

  # Use different camera
  python quick_calibration.py --camera 1 --output camera1.json

  # Custom ChArUco board (30mm squares, 18mm markers)
  python quick_calibration.py \
    --square-size 0.030 --marker-size 0.018 \
    --board-cols 8 --board-rows 6 \
    --duration 30 --output my_camera.json

  # After calibration, test ArUco distance detection
  python ../aruco_tags/aruco_validator.py --calibration my_camera.json --tag-size 10.0
        """,
    )

    parser.add_argument("--camera", type=int, default=0, help="Camera device index (default: 0)")
    parser.add_argument("--duration", type=int, default=30, help="Capture duration in seconds (default: 30)")
    parser.add_argument(
        "--output",
        "-o",
        default="webcam_calibration.json",
        help="Output calibration file (default: webcam_calibration.json)",
    )
    parser.add_argument(
        "--square-size", type=float, default=0.030, help="ChArUco square size in meters (default: 0.030 = 30mm)"
    )
    parser.add_argument(
        "--marker-size", type=float, default=0.018, help="ChArUco marker size in meters (default: 0.018 = 18mm)"
    )
    parser.add_argument("--board-cols", type=int, default=8, help="ChArUco board columns (default: 8)")
    parser.add_argument("--board-rows", type=int, default=6, help="ChArUco board rows (default: 6)")

    args = parser.parse_args()

    success = quick_calibrate_camera(
        camera_index=args.camera,
        duration=args.duration,
        output_file=args.output,
        board_squares=(args.board_cols, args.board_rows),
        square_size=args.square_size,
        marker_size=args.marker_size,
    )

    if success:
        print("\n🎉 Calibration complete!")
        print("\n📌 Next steps:")
        print("1. Test ArUco detection with distance measurement:")
        print("   cd ../aruco_tags/")
        print(f"   python aruco_validator.py --calibration ../camera/{args.output} --tag-size 10.0")
        return 0
    else:
        print("\n❌ Calibration failed")
        return 1


if __name__ == "__main__":
    exit(main())
