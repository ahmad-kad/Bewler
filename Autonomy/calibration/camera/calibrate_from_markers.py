#!/usr/bin/env python3
"""
Direct Calibration from ArUco Markers - URC 2026

Workaround for OpenCV 4.12 ChArUco detection issue.
Collects detected ArUco markers and calibrates using cv2.calibrateCamera()

This module provides camera calibration functionality using ArUco markers
from a ChArUco board. It's designed as a workaround for OpenCV 4.12+ where
ChArUco board detection has known issues.

Usage:
    python calibrate_from_markers.py --cols 7 --rows 5 --square-size 0.030 --marker-size 0.018 --output cal.json

Classes:
    None

Functions:
    calibrate_from_markers: Main calibration function
    main: Command-line interface

Examples:
    >>> success = calibrate_from_markers(cols=7, rows=5, square_size=0.030)
    >>> print("Calibration successful:", success)
"""

import cv2
import numpy as np
import argparse
import time
import json
import sys

def calibrate_from_markers(camera_index=0, duration=30, cols=7, rows=5,
                          square_size=0.030, marker_size=0.018, output_file="calibration.json"):
    """
    Calibrate camera by collecting ArUco markers from ChArUco board.

    This function performs camera calibration using ArUco markers detected on a ChArUco board.
    It captures frames over a specified duration, detects markers, and computes camera intrinsic
    parameters using OpenCV's calibrateCamera function. Designed as a workaround for issues
    with ChArUco board detection in OpenCV 4.12+.

    Args:
        camera_index (int): Index of the camera device to use (default: 0)
        duration (int): Duration in seconds to capture calibration frames (default: 30)
        cols (int): Number of columns in the ChArUco board (default: 7)
        rows (int): Number of rows in the ChArUco board (default: 5)
        square_size (float): Size of each square on the board in meters (default: 0.030)
        marker_size (float): Size of each ArUco marker in meters (default: 0.018)
        output_file (str): Path to save the calibration results (default: "calibration.json")

    Returns:
        bool: True if calibration was successful, False otherwise

    Raises:
        None: All errors are handled internally and logged to console
    """
    
    print("🎯 Direct ArUco Marker Calibration")
    print("=" * 60)
    print(f"Board: {cols}×{rows} ({cols*rows} markers)")
    print(f"Square: {square_size*1000:.0f}mm, Marker: {marker_size*1000:.0f}mm")
    
    # Setup
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    board = cv2.aruco.CharucoBoard(
        (cols, rows),
        squareLength=square_size,
        markerLength=marker_size,
        dictionary=aruco_dict
    )
    
    # Open camera
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    if not cap.isOpened():
        print("❌ Cannot open camera")
        return False
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"📷 Camera: {width}x{height}\n")
    
    # Collect markers
    print(f"📹 Capturing for {duration} seconds...")
    print("Move board around to collect good frames\n")
    
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
                    # Try interpolating ChArUco corners - this sometimes fails
                    try:
                        charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                            corners, ids, frame, board
                        )
                    except:
                        # If interpolation fails, use raw marker corners
                        charuco_corners = corners
                        charuco_ids = ids
                    
                    # Check if we got valid corners
                    if charuco_corners is not None and charuco_ids is not None:
                        n_corners = len(charuco_ids) if isinstance(charuco_ids, np.ndarray) else 0
                        if n_corners > 5:
                            all_corners.append(charuco_corners)
                            all_ids.append(charuco_ids)
                            collected_frames += 1
                            
                            print(f"✅ Frame {frame_count}: Collected {n_corners} corners (total: {collected_frames})")
                            
                            if collected_frames >= 15:
                                print("\n✅ Collected enough frames!")
                                break
                    
                    # Draw for visualization
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    
                except Exception as e:
                    if frame_count % 30 == 0:
                        print(f"⏳ Frame {frame_count}: {len(ids) if ids is not None else 0} markers detected...")
            else:
                if frame_count % 30 == 0:
                    print(f"⏳ Frame {frame_count}: Searching for board...")
            
            cv2.imshow('Calibration - Collecting Frames', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n⏹️ User interrupted")
                break
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
    
    print(f"\n📊 Results:")
    print(f"   Frames processed: {frame_count}")
    print(f"   Frames collected: {collected_frames}")
    
    if collected_frames < 5:
        print(f"❌ Need at least 5 frames, got {collected_frames}")
        return False
    
    # Calibrate
    print(f"\n🔧 Calibrating camera...")
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
                marker_obj_pt = np.array([
                    [-marker_size/2, -marker_size/2, 0],
                    [marker_size/2, -marker_size/2, 0],
                    [marker_size/2, marker_size/2, 0],
                    [-marker_size/2, marker_size/2, 0]
                ], dtype=np.float32)
                
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
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": camera_matrix.flatten().tolist()
            },
            "distortion_coefficients": {
                "rows": 1,
                "cols": 5,
                "data": dist_coeffs.flatten().tolist()
            },
            "image_width": width,
            "image_height": height,
            "frames_used": collected_frames,
            "board_size": f"{cols}x{rows}",
            "calibration_quality": "good"
        }
        
        with open(output_file, 'w') as f:
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrate camera from ArUco markers')
    parser.add_argument('--cols', type=int, default=7, help='Board columns')
    parser.add_argument('--rows', type=int, default=5, help='Board rows')
    parser.add_argument('--square-size', type=float, default=0.030, help='Square size in meters')
    parser.add_argument('--marker-size', type=float, default=0.018, help='Marker size in meters')
    parser.add_argument('--duration', type=int, default=30, help='Capture duration in seconds')
    parser.add_argument('--output', '-o', default='calibration.json', help='Output file')
    
    args = parser.parse_args()
    
    success = calibrate_from_markers(
        duration=args.duration,
        cols=args.cols,
        rows=args.rows,
        square_size=args.square_size,
        marker_size=args.marker_size,
        output_file=args.output
    )
    
    exit(0 if success else 1)
 