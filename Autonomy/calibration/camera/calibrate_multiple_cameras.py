#!/usr/bin/env python3
"""
Multi-Camera Calibration System - URC 2026

Sequentially calibrate multiple cameras and save individual calibrations.
Each camera gets its own calibration file.

Usage:
    python calibrate_multiple_cameras.py --num-cameras 5 --output-dir ./calibrations
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


def calibrate_single_camera(
    camera_index, duration=30, cols=7, rows=5, square_size=0.030, marker_size=0.018, output_file="camera_calib.json"
):
    """Calibrate a single camera."""

    print(f"\n{'='*70}")
    print(f"📷 CALIBRATING CAMERA {camera_index}")
    print(f"{'='*70}")
    print(f"Output: {output_file}")
    print(f"Board: {cols}×{rows} ({cols*rows} markers)")
    print(f"Square: {square_size*1000:.0f}mm, Marker: {marker_size*1000:.0f}mm\n")

    # Setup
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    board = cv2.aruco.CharucoBoard(
        (cols, rows), squareLength=square_size, markerLength=marker_size, dictionary=aruco_dict
    )

    # Open camera
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print(f"❌ Cannot open camera {camera_index}")
        return False

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ Camera opened: {width}x{height}\n")

    # Collect markers
    print(f"📹 Capturing for {duration} seconds...")
    print("Move board around to collect good frames (various angles)\n")

    all_corners = []
    all_ids = []
    frame_count = 0
    collected_frames = 0
    start_time = time.time()

    try:
        while time.time() - start_time < duration:
            ret, frame = cap.read()
            if not ret:
                break

            frame_count += 1

            # Detect markers
            corners, ids, rejected = detector.detectMarkers(frame)

            # If we found markers, try to extract ChArUco corners
            if ids is not None and len(ids) > 4:
                try:
                    # Try interpolating ChArUco corners
                    try:
                        charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, frame, board)
                    except BaseException:
                        # Fallback: use raw marker corners
                        charuco_corners = corners
                        charuco_ids = ids

                    # Check if we got valid corners
                    if charuco_corners is not None and charuco_ids is not None:
                        n_corners = len(charuco_ids) if isinstance(charuco_ids, np.ndarray) else 0
                        if n_corners > 5:
                            all_corners.append(charuco_corners)
                            all_ids.append(charuco_ids)
                            collected_frames += 1

                            print(
                                f"  ✅ Frame {frame_count}: Collected {n_corners} corners (total: {collected_frames})"
                            )

                            if collected_frames >= 15:
                                print("\n✅ Collected enough frames!")
                                break

                    # Draw for visualization
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                except Exception:
                    if frame_count % 30 == 0:
                        print(f"  ⏳ Frame {frame_count}: {len(ids) if ids is not None else 0} markers detected...")
            else:
                if frame_count % 30 == 0:
                    print(f"  ⏳ Frame {frame_count}: Searching for board...")

            cv2.imshow(f"Camera {camera_index} - Calibration", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("\n⏹️  User interrupted")
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

    print("\n📊 Results:")
    print(f"   Frames processed: {frame_count}")
    print(f"   Frames collected: {collected_frames}")

    if collected_frames < 5:
        print(f"❌ Need at least 5 frames, got {collected_frames}")
        return False

    # Calibrate
    print("\n🔧 Calibrating...")
    try:
        camera_matrix = np.zeros((3, 3))
        dist_coeffs = np.zeros((5, 1))

        # Convert marker corners to 3D object points
        object_points_list = []
        image_points_list = []

        for frame_idx, (corners_frame, ids_frame) in enumerate(zip(all_corners, all_ids)):
            if ids_frame is None or len(ids_frame) == 0:
                continue

            frame_obj_pts = []
            frame_img_pts = []

            for i, marker_id in enumerate(ids_frame.flatten()):
                # 3D coordinates for ArUco marker corners
                marker_obj_pt = np.array(
                    [
                        [-marker_size / 2, -marker_size / 2, 0],
                        [marker_size / 2, -marker_size / 2, 0],
                        [marker_size / 2, marker_size / 2, 0],
                        [-marker_size / 2, marker_size / 2, 0],
                    ],
                    dtype=np.float32,
                )

                frame_obj_pts.append(marker_obj_pt)
                frame_img_pts.append(corners_frame[i].reshape(4, 2).astype(np.float32))

            if len(frame_obj_pts) > 0:
                object_points_list.append(np.vstack(frame_obj_pts))
                image_points_list.append(np.vstack(frame_img_pts))

        if len(object_points_list) < 3:
            print(f"❌ Not enough frames with valid markers ({len(object_points_list)})")
            return False

        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            object_points_list, image_points_list, (width, height), camera_matrix, dist_coeffs
        )

        if not ret:
            print("❌ Calibration failed")
            return False

        print("✅ Calibration successful!")

        # Save
        calib_data = {
            "camera_index": camera_index,
            "camera_matrix": {"rows": 3, "cols": 3, "data": camera_matrix.flatten().tolist()},
            "distortion_coefficients": {"rows": 1, "cols": 5, "data": dist_coeffs.flatten().tolist()},
            "image_width": width,
            "image_height": height,
            "frames_used": collected_frames,
            "board_size": f"{cols}x{rows}",
            "calibration_quality": "good",
        }

        with open(output_file, "w") as f:
            json.dump(calib_data, f, indent=2)

        print(f"💾 Calibration saved to: {output_file}")
        print(f"\nCamera Matrix:\n{camera_matrix}")
        print(f"\nDistortion Coefficients:\n{dist_coeffs.flatten()}")

        return True

    except Exception as e:
        print(f"❌ Calibration error: {e}")
        import traceback

        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Multi-camera calibration system",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Calibrate 5 cameras
  python calibrate_multiple_cameras.py --num-cameras 5 --output-dir ./camera_calibrations

  # Calibrate 3 cameras with custom board size
  python calibrate_multiple_cameras.py --num-cameras 3 --cols 8 --rows 6 --duration 40

  # Calibrate 5 cameras with custom square and marker sizes
  python calibrate_multiple_cameras.py --num-cameras 5 --square-size 0.025 --marker-size 0.015
        """,
    )

    parser.add_argument("--num-cameras", type=int, default=5, help="Number of cameras to calibrate (default: 5)")
    parser.add_argument("--cols", type=int, default=7, help="Board columns (default: 7)")
    parser.add_argument("--rows", type=int, default=5, help="Board rows (default: 5)")
    parser.add_argument(
        "--square-size", type=float, default=0.030, help="Square size in meters (default: 0.030 = 30mm)"
    )
    parser.add_argument(
        "--marker-size", type=float, default=0.018, help="Marker size in meters (default: 0.018 = 18mm)"
    )
    parser.add_argument("--duration", type=int, default=30, help="Capture duration per camera in seconds (default: 30)")
    parser.add_argument(
        "--output-dir",
        "-o",
        default="./camera_calibrations",
        help="Output directory for calibration files (default: ./camera_calibrations)",
    )
    parser.add_argument("--start-camera", type=int, default=0, help="Starting camera index (default: 0)")

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"📁 Output directory: {output_dir.absolute()}\n")

    # Calibrate each camera
    successful = 0
    failed = 0

    for i in range(args.num_cameras):
        camera_index = args.start_camera + i
        output_file = output_dir / f"camera_{camera_index}_calibration.json"

        print(f"\n{'='*70}")
        print(f"Camera {i+1}/{args.num_cameras} (Device Index: {camera_index})")
        print(f"{'='*70}")

        success = calibrate_single_camera(
            camera_index=camera_index,
            duration=args.duration,
            cols=args.cols,
            rows=args.rows,
            square_size=args.square_size,
            marker_size=args.marker_size,
            output_file=str(output_file),
        )

        if success:
            successful += 1
            print(f"\n✅ Camera {camera_index} calibrated successfully!")
        else:
            failed += 1
            print(f"\n❌ Camera {camera_index} calibration FAILED!")

        # Ask if user wants to continue
        if i < args.num_cameras - 1:
            print(f"\n⏳ Prepare camera {camera_index + 1} and press ENTER to continue...")
            try:
                input()
            except KeyboardInterrupt:
                print("\n⏹️  Calibration interrupted by user")
                break

    # Summary
    print(f"\n{'='*70}")
    print("📊 CALIBRATION SUMMARY")
    print(f"{'='*70}")
    print(f"Total cameras: {args.num_cameras}")
    print(f"✅ Successful: {successful}")
    print(f"❌ Failed: {failed}")
    print(f"📁 Calibrations saved to: {output_dir.absolute()}\n")

    # List all calibration files
    calib_files = sorted(output_dir.glob("camera_*_calibration.json"))
    if calib_files:
        print("📋 Calibration files:")
        for calib_file in calib_files:
            try:
                with open(calib_file) as f:
                    data = json.load(f)
                    cam_idx = data.get("camera_index", "?")
                    print(f"   - {calib_file.name} (Camera {cam_idx})")
            except BaseException:
                print(f"   - {calib_file.name}")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    exit(main())
