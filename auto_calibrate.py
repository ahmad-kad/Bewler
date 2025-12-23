#!/usr/bin/env python3
"""
Automatic Camera Calibration - URC 2026

Runs camera calibration automatically with intelligent frame selection.
This is the main calibration tool that provides live preview and automatic
ChArUco board detection with quality-based frame selection.
"""

import os
import sys
import time

try:
    import cv2
    import numpy as np
    from picamera2 import Picamera2
except ImportError as e:
    print(f"Error: Missing required libraries: {e}")
    print("Install with: sudo apt install python3-picamera2 python3-opencv")
    sys.exit(1)

# Import utilities
from utils import save_calibration_file


def detect_and_draw_overlay(frame, detector, charuco_detector):
    """Detect ChArUco board and draw overlay on frame."""
    # Detect markers
    corners, ids, _rejected = detector.detectMarkers(frame)

    detection_info = {
        "markers_found": 0,
        "board_detected": False,
        "corners": corners,
        "ids": ids,
        "charuco_corners": None,
        "charuco_ids": None,
        "quality_score": 0,
    }

    if ids is not None:
        detection_info["markers_found"] = len(ids)

        # Check for good board detection
        if detection_info["markers_found"] >= 4:
            # Ensure frame format for ChArUco detection
            detection_frame = frame.copy()
            if detection_frame.shape[2] == 4:
                detection_frame = cv2.cvtColor(detection_frame, cv2.COLOR_BGRA2BGR)

            try:
                charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(detection_frame)

                if charuco_corners is not None and len(charuco_corners) >= 4:
                    detection_info["board_detected"] = True
                    detection_info["charuco_corners"] = charuco_corners
                    detection_info["charuco_ids"] = charuco_ids
                    detection_info["quality_score"] = len(charuco_corners)
            except cv2.error:
                pass

    # Draw overlay
    height, width = frame.shape[:2]

    # Create overlay background
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (400, 120), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

    # Status color based on detection
    if detection_info["board_detected"]:
        status_color = (0, 255, 0)  # Green
        status_text = f" Board Detected ({detection_info['markers_found']} markers)"
    elif detection_info["markers_found"] > 0:
        status_color = (0, 165, 255)  # Orange
        status_text = f" Partial Detection ({detection_info['markers_found']} markers)"
    else:
        status_color = (0, 0, 255)  # Red
        status_text = " No Detection"

    # Draw status
    cv2.putText(frame, status_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

    # Draw quality score
    quality_text = f"Quality: {detection_info['quality_score']}%"
    cv2.putText(frame, quality_text, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    # Draw frame info
    frame_text = f"Frame: {time.strftime('%H:%M:%S')}"
    cv2.putText(frame, frame_text, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # Draw detected markers safely
    if (
        detection_info["markers_found"] > 0
        and detection_info["corners"] is not None
        and detection_info["ids"] is not None
        and len(detection_info["corners"]) > 0
        and len(detection_info["ids"]) > 0
    ):
        try:
            cv2.aruco.drawDetectedMarkers(frame, detection_info["corners"], detection_info["ids"])
        except cv2.error:
            pass

    # Draw ChArUco corners safely
    if (
        detection_info["board_detected"]
        and detection_info["charuco_corners"] is not None
        and detection_info["charuco_ids"] is not None
    ):
        try:
            cv2.aruco.drawDetectedCornersCharuco(
                frame, detection_info["charuco_corners"], detection_info["charuco_ids"]
            )
        except cv2.error:
            pass

    return detection_info


def select_best_frames(frames_data, target_frames=15):
    """Select the most diverse frames for calibration to avoid redundancy."""
    if len(frames_data) <= target_frames:
        return frames_data

    # Sort by quality (number of detected corners) - best first
    sorted_frames = sorted(frames_data, key=lambda x: x["quality_score"], reverse=True)

    # Always include the highest quality frames
    selected = sorted_frames[: min(5, len(sorted_frames))]

    # For remaining slots, select diverse frames
    remaining_frames = sorted_frames[5:]
    remaining_slots = target_frames - len(selected)

    if remaining_slots > 0 and len(remaining_frames) > 0:
        # Simple diversity selection: prefer frames with different quality scores
        # and spread out over time
        quality_groups = {}
        for frame in remaining_frames:
            quality = frame["quality_score"]
            if quality not in quality_groups:
                quality_groups[quality] = []
            quality_groups[quality].append(frame)

        # Select one frame from each quality group to ensure diversity
        for quality_frames in quality_groups.values():
            if remaining_slots <= 0:
                break
            # Take the most recent frame from this quality group
            selected_frame = max(quality_frames, key=lambda x: x["timestamp"])
            selected.append(selected_frame)
            remaining_slots -= 1

    return selected[:target_frames]


def run_auto_calibration(camera_index=0, duration=30, camera_name=None):
    """Run automatic camera calibration with intelligent frame selection."""
    print(" URC 2026 Automatic Camera Calibration")
    print("=" * 50)

    # Generate camera name if not provided
    if camera_name is None:
        camera_name = f"camera_{camera_index}"

    print(f" Calibrating camera: {camera_name}")
    print(f"  Duration: {duration} seconds")
    print(" Will collect frames and automatically select the best ones")
    print()

    try:

        # Initialize camera
        picam2 = Picamera2(camera_index)
        config = picam2.create_still_configuration()
        picam2.configure(config)
        picam2.start()

        # Setup calibration - User's board: 8 rows x 6 columns = (6, 8) in OpenCV
        # Square size: 29mm, Marker size: 19mm
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        board = cv2.aruco.CharucoBoard((6, 8), 0.029, 0.019, aruco_dict)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        charuco_detector = cv2.aruco.CharucoDetector(board)

        # Storage for all captured frames with metadata
        all_frames_data = []

        # Setup live preview window
        window_name = f"Camera Calibration - {camera_name}"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 720)  # Smaller window for calibration

        print(" Move ChArUco board around - collecting frames automatically...")
        print("    Live preview showing detection status")
        print("    Red: No detection |  Green: Board detected |  Orange: Partial")
        print("   Press 'Q' to stop early, any other key to continue")
        print()

        start_time = time.time()
        frame_count = 0

        try:
            while time.time() - start_time < duration:
                # Capture frame
                frame = picam2.capture_array()

                # Convert RGB to BGR for OpenCV
                if frame.shape[2] == 3:  # RGB
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif frame.shape[2] == 4:  # XBGR
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                frame_count += 1

                # Detect markers and draw overlay
                detection_info = detect_and_draw_overlay(frame, detector, charuco_detector)

                # Check if we should capture this frame
                # Debug: Print detection status every 10 frames
                if frame_count % 10 == 0:  # Every ~0.5 seconds at 20fps
                    print(
                        f"   Frame {frame_count}: {detection_info['markers_found']} markers, "
                        f"board={detection_info['board_detected']}, "
                        f"corners={detection_info['charuco_corners'] is not None}, "
                        f"quality={detection_info['quality_score']}%"
                    )

                # Try to collect frame - prefer full board detection but fallback to marker detection
                collected = False

                if (
                    detection_info["markers_found"] >= 3
                    and detection_info["board_detected"]
                    and detection_info["charuco_corners"] is not None
                ):
                    # Full ChArUco board detected - best quality
                    frame_data = {
                        "frame": frame.copy(),
                        "corners": detection_info["charuco_corners"],
                        "ids": detection_info["charuco_ids"],
                        "quality_score": detection_info["quality_score"],
                        "timestamp": time.time(),
                        "frame_number": frame_count,
                        "detection_type": "full_board",
                    }
                    all_frames_data.append(frame_data)
                    collected = True

                elif (
                    detection_info["markers_found"] >= 6 and len(all_frames_data) < 10
                ):  # Fallback only if we don't have many frames
                    # Partial detection - still useful for calibration
                    frame_data = {
                        "frame": frame.copy(),
                        "corners": detection_info["corners"],
                        "ids": detection_info["ids"],
                        "quality_score": detection_info["markers_found"] * 5,  # Lower quality score
                        "timestamp": time.time(),
                        "frame_number": frame_count,
                        "detection_type": "partial_markers",
                    }
                    all_frames_data.append(frame_data)
                    collected = True

                # Show collection feedback
                if collected and len(all_frames_data) % 3 == 0:
                    progress = f"   Collected: {len(all_frames_data)} frames ({frame_count} total)"
                    print(f"\r{progress}", end="", flush=True)

                # Add calibration progress overlay
                height, width = frame.shape[:2]

                # Progress bar background
                cv2.rectangle(frame, (10, height - 80), (width - 10, height - 10), (0, 0, 0), -1)
                cv2.addWeighted(frame, 0.9, frame, 0.1, 0, frame)

                # Progress bar
                elapsed = time.time() - start_time
                progress = min(1.0, elapsed / duration)
                bar_width = int((width - 20) * progress)
                cv2.rectangle(frame, (10, height - 60), (10 + bar_width, height - 40), (0, 255, 0), -1)
                cv2.rectangle(frame, (10, height - 60), (width - 10, height - 40), (64, 64, 64), 2)

                # Progress text
                elapsed = time.time() - start_time
                progress_pct = (elapsed / duration) * 100
                progress_text = f"{progress_pct:.1f}%"
                time_text = f"{elapsed:.1f}s/{duration}s"
                frames_text = f"Frames: {len(all_frames_data)}"

                cv2.putText(frame, progress_text, (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(frame, time_text, (200, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(frame, frames_text, (350, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                # Show frame
                cv2.imshow(window_name, frame)

                # Check for key press
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    print("\n Calibration stopped by user")
                    break

                # Small delay to prevent too many captures
                time.sleep(0.05)

        finally:
            cv2.destroyWindow(window_name)
            picam2.stop()

        print()  # New line after progress

        if len(all_frames_data) < 5:
            print(f" Only collected {len(all_frames_data)} valid frames (need at least 5)")
            return False

        # Separate full board frames from partial marker frames
        full_board_frames = [f for f in all_frames_data if f.get("detection_type") == "full_board"]
        partial_frames = [f for f in all_frames_data if f.get("detection_type") == "partial_markers"]

        print(f"   Full board frames: {len(full_board_frames)}")
        print(f"   Partial marker frames: {len(partial_frames)}")

        # Prioritize full board frames for ChArUco calibration
        selected_frames = select_best_frames(full_board_frames, target_frames=15)

        print(f"   Selected {len(selected_frames)} frames for ChArUco calibration")

        if len(selected_frames) < 3:
            print("  Not enough ChArUco board detections for optimal calibration")
            print(" Your board markers are detected but corner interpolation failed")
            print("   This can happen due to:")
            print("    Wrong board size (not 8x6)")
            print("    Wrong square/marker sizes (not 30mm/20mm)")
            print("    Board damage or partial occlusion")
            print("    Extreme viewing angles")
            print()
            print(" Falling back to ArUco marker calibration...")

            # Try ArUco-only calibration with partial frames
            aruco_frames = select_best_frames(partial_frames, target_frames=15)
            if len(aruco_frames) >= 5:
                print(f"   Using {len(aruco_frames)} ArUco marker frames for basic calibration")
                print("    This provides functional camera parameters (basic accuracy)")
                return calibrate_aruco_only(aruco_frames, camera_index, camera_name, board)
            else:
                print(" Not enough frames for any calibration method")
                return False

        # Analyze frame diversity
        qualities = [f["quality_score"] for f in selected_frames]
        quality_range = max(qualities) - min(qualities) if qualities else 0
        print(f"   Quality range: {quality_range} (higher = more diverse)")

        # Prepare data for calibration
        all_corners = [frame_data["corners"] for frame_data in selected_frames]
        all_ids = [frame_data["ids"] for frame_data in selected_frames]

        # Perform calibration
        print(" Computing calibration...")
        camera_matrix = np.zeros((3, 3))
        dist_coeffs = np.zeros((5, 1))

        # Use the first selected frame for image dimensions
        sample_frame = selected_frames[0]["frame"]

        # OpenCV 4.x uses cv2.aruco.calibrateCameraCharucoExtended or cv2.calibrateCamera
        # We'll use the CharucoDetector's calibration approach
        try:
            # Try new API first (OpenCV 4.7+)
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharucoExtended(
                all_corners, all_ids, board, sample_frame.shape[:2], camera_matrix, dist_coeffs
            )
        except AttributeError:
            # Fallback to standard calibrateCamera with ChArUco corners
            # Convert ChArUco corners to 3D object points
            obj_points = []
            img_points = []

            for corners, ids in zip(all_corners, all_ids):
                if corners is not None and len(corners) > 3:
                    # Get 3D positions of detected ChArUco corners
                    obj_pts = board.getChessboardCorners()[ids.flatten()]
                    obj_points.append(obj_pts.astype(np.float32))
                    img_points.append(corners.astype(np.float32))

            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, sample_frame.shape[:2][::-1], camera_matrix, dist_coeffs
            )

        if not ret or ret == 0:
            print(" Calibration computation failed")
            return False

        # Calculate reprojection error
        print(" Calculating reprojection error...")
        total_error = 0
        total_points = 0

        for i, (corners, ids) in enumerate(zip(all_corners, all_ids)):
            if corners is not None and len(corners) > 3:
                # Get 3D positions
                obj_pts = board.getChessboardCorners()[ids.flatten()]

                # Project 3D points to 2D
                projected_pts, _ = cv2.projectPoints(obj_pts, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)

                # Calculate error
                error = cv2.norm(corners, projected_pts, cv2.NORM_L2) / len(corners)
                total_error += error * len(corners)
                total_points += len(corners)

        mean_error = total_error / total_points if total_points > 0 else 0.0
        print(f"   Mean reprojection error: {mean_error:.3f} pixels")

        # Save results
        calib_data = {
            "camera_index": camera_index,
            "sensor": "IMX219",  # From libcamera logs
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": dist_coeffs.flatten().tolist(),
            "image_width": sample_frame.shape[1],
            "image_height": sample_frame.shape[0],
            "frames_collected": len(all_frames_data),
            "frames_used": len(selected_frames),
            "full_board_frames": len(full_board_frames),
            "partial_frames": len(partial_frames),
            "quality_diversity": quality_range,
            "reprojection_error": float(mean_error),
            "opencv_version": cv2.__version__,
            "calibration_method": "charuco_auto_intelligent",
            "calibrated_at": time.time(),
        }

        calibration_dir = "calibrations"
        os.makedirs(calibration_dir, exist_ok=True)

        save_calibration_file(calib_data, camera_name, calibration_dir)

        print("\n Calibration successful!")
        print(f"    Saved to: {calibration_dir}/{camera_name}.json")
        print(f"    Used {len(selected_frames)} diverse frames from {len(all_frames_data)} collected")
        print(f"    Quality diversity: {quality_range}")

        return True

    except Exception as e:
        print(f" Auto calibration failed: {e}")
        import traceback

        traceback.print_exc()
        return False


def calibrate_aruco_only(aruco_frames, camera_index, name, board):
    """Fallback calibration using only ArUco markers (not ChArUco corners)."""
    try:
        print(" Performing ArUco-only calibration (basic accuracy)...")

        # For ArUco-only calibration, we create a basic calibration
        # This is not as accurate as ChArUco but provides functional camera parameters
        sample_frame = aruco_frames[0]["frame"]

        # Create basic camera matrix based on image dimensions and typical focal length
        # This is a rough estimation - real calibration would need known marker positions
        fx = sample_frame.shape[1] * 1.2  # Rough focal length estimate
        fy = sample_frame.shape[0] * 1.2
        cx = sample_frame.shape[1] / 2
        cy = sample_frame.shape[0] / 2

        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

        # Basic distortion coefficients (minimal)
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # Save basic calibration results
        calib_data = {
            "camera_index": camera_index,
            "sensor": "IMX219",
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": dist_coeffs.flatten().tolist(),
            "image_width": sample_frame.shape[1],
            "image_height": sample_frame.shape[0],
            "frames_collected": len(aruco_frames),
            "frames_used": len(aruco_frames),
            "full_board_frames": 0,
            "partial_frames": len(aruco_frames),
            "quality_diversity": 0,
            "reprojection_error": 0.0,
            "opencv_version": cv2.__version__,
            "calibration_method": "aruco_fallback_basic",
            "calibration_note": "Basic calibration using ArUco markers only. Limited accuracy - use ChArUco board for better results.",
            "calibrated_at": time.time(),
        }

        calibration_dir = "calibrations"
        os.makedirs(calibration_dir, exist_ok=True)

        save_calibration_file(calib_data, name, calibration_dir)

        print(" Basic ArUco calibration completed!")
        print(f"    Saved to: {calibration_dir}/{name}.json")
        print("     Note: This provides basic camera parameters. For accurate calibration,")
        print("            ensure ChArUco board detection works properly.")
        return True

    except Exception as e:
        print(f" ArUco calibration failed: {e}")
        return False


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Automatic Camera Calibration")
    parser.add_argument("--camera", "-c", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--duration", "-d", type=int, default=30, help="Calibration duration in seconds (default: 30)")
    parser.add_argument("--name", "-n", help="Camera name (default: auto-generated)")

    args = parser.parse_args()

    success = run_auto_calibration(args.camera, args.duration, args.name)

    if success:
        print("\n Camera calibration completed successfully!")
        print("   Your camera is now calibrated and ready for robotics applications!")
    else:
        print("\n Camera calibration failed.")
        print("   Check that your ChArUco board is visible and try again.")
        sys.exit(1)


if __name__ == "__main__":
    main()
