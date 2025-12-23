#!/usr/bin/env python3
"""
Detection Test - URC 2026
Quick test to verify camera and ChArUco detection are working
"""

import cv2
import numpy as np
import time
import sys
import os

# Import shared configuration
from camera_config import CHARUCO_CONFIG

try:
    import picamera2

    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False


def test_camera_access(camera_index=0):
    """Test basic camera access."""
    print(" Testing camera access...")

    if PICAMERA_AVAILABLE:
        try:
            cameras = picamera2.Picamera2.global_camera_info()
            if camera_index < len(cameras):
                print(f" Libcamera: Found camera {camera_index}")
                cam_info = cameras[camera_index]
                print(f"   Model: {cam_info.get('Model', 'Unknown')}")
                print(f"   Resolution: {cam_info.get('Width', 0)}x{cam_info.get('Height', 0)}")
                return "libcamera", cam_info
        except Exception as e:
            print(f" Libcamera failed: {e}")

    # Fallback to OpenCV
    try:
        cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        if cap.isOpened():
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            cap.release()
            print(f" OpenCV: Camera {camera_index} accessible ({width}x{height})")
            return "opencv", {"width": width, "height": height}
    except Exception as e:
        print(f" OpenCV failed: {e}")

    print(" No camera accessible")
    return None, None


def test_charuco_detection(camera_method, camera_index=0, duration=5):
    """Test ChArUco board detection."""
    print(f"\n Testing ChArUco detection for {duration} seconds...")

    # Setup detection using shared configuration
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, CHARUCO_CONFIG["dictionary"]))
    board = cv2.aruco.CharucoBoard(
        CHARUCO_CONFIG["board_size"], CHARUCO_CONFIG["square_size"], CHARUCO_CONFIG["marker_size"], aruco_dict
    )
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    charuco_detector = cv2.aruco.CharucoDetector(board)

    # Initialize camera
    cap = None
    picam2 = None

    try:
        if camera_method == "libcamera" and PICAMERA_AVAILABLE:
            picam2 = picamera2.Picamera2(camera_index)
            config = picam2.create_preview_configuration()
            picam2.configure(config)
            picam2.start()
        elif camera_method == "opencv":
            cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise Exception("Cannot open camera")

        # Test detection
        detections = []
        start_time = time.time()

        print("   Move ChArUco board in front of camera...")
        print("   Testing detection...")

        frame_count = 0
        while time.time() - start_time < duration:
            # Capture frame
            if picam2:
                frame = picam2.capture_array()
                if frame.shape[2] == 3:  # RGB
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif cap:
                ret, frame = cap.read()
                if not ret:
                    continue
            else:
                break

            frame_count += 1

            # Detect markers
            corners, ids, _rejected = detector.detectMarkers(frame)

            detection_info = {"frame": frame_count, "markers": 0, "board_detected": False, "quality": 0}

            if ids is not None:
                detection_info["markers"] = len(ids)

                if len(ids) >= 4:
                    charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(frame)
                    if charuco_corners is not None and len(charuco_corners) >= 6:
                        detection_info["board_detected"] = True
                        detection_info["quality"] = min(100, len(charuco_corners) * 10)

            detections.append(detection_info)

            # Progress indicator
            elapsed = time.time() - start_time
            progress = min(100, int((elapsed / duration) * 100))
            print(f"\r   Progress: {progress}% ({len(detections)} frames tested)", end="", flush=True)

        print()  # New line

        # Analyze results
        total_frames = len(detections)
        marker_frames = sum(1 for d in detections if d["markers"] > 0)
        board_frames = sum(1 for d in detections if d["board_detected"])
        avg_quality = sum(d["quality"] for d in detections) / max(1, len(detections))

        print("\n Detection Results:")
        print(f"   Total frames tested: {total_frames}")
        print(f"   Frames with markers: {marker_frames} ({marker_frames/total_frames*100:.1f}%)")
        print(f"   Frames with board: {board_frames} ({board_frames/total_frames*100:.1f}%)")
        print(f"   Average quality: {avg_quality:.1f}%")

        # Recommendations
        print("\n Recommendations:")
        if board_frames == 0:
            print("    No board detections - check:")
            print("       ChArUco board visibility")
            print("       Lighting conditions")
            print("       Camera focus and angle")
            print("       Board distance from camera")
        elif board_frames / total_frames < 0.3:
            print("    Low detection rate - try:")
            print("       Better lighting")
            print("       Slower board movement")
            print("       Different viewing angles")
        elif avg_quality < 50:
            print("    Low quality detections - try:")
            print("       More stable board position")
            print("       Better focus")
            print("       Reduce motion blur")
        else:
            print("    Detection working well!")
            print("      Ready for calibration.")

        return board_frames > 0

    except Exception as e:
        print(f" Detection test failed: {e}")
        return False
    finally:
        if picam2:
            picam2.stop()
        if cap:
            cap.release()


def generate_test_pattern():
    """Generate a test ChArUco board image for reference."""
    print("\n Generating test ChArUco board image...")

    # User's board: 8 rows x 6 columns = (6, 8) in OpenCV
    # Square size: 29mm, Marker size: 19mm
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard((6, 8), 0.029, 0.019, aruco_dict)

    # Generate board image (scaled for 8 rows x 6 columns)
    # Aspect ratio: 6 columns x 8 rows = 6:8 = 900:1200
    board_image = cv2.aruco.CharucoBoard.generateImage(board, (900, 1200))

    # Save image
    output_path = "charuco_test_board_6x8.png"
    cv2.imwrite(output_path, board_image)

    print(f" Test board saved as: {output_path}")
    print("   Print this image for testing detection")
    print("   Board size: 8x6 squares, 30mm squares, 20mm markers")
    print("   When printing, ensure the square size matches 30mm for accurate calibration")


def main():
    """Main test function."""
    import argparse

    parser = argparse.ArgumentParser(description="Camera and Detection Test")
    parser.add_argument("--camera", "-c", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--duration", "-d", type=int, default=5, help="Test duration in seconds (default: 5)")
    parser.add_argument("--generate-board", action="store_true", help="Generate test ChArUco board image")

    args = parser.parse_args()

    print(" URC 2026 Camera & Detection Test")
    print("=" * 40)

    if args.generate_board:
        generate_test_pattern()
        return

    # Test camera access
    camera_method, camera_info = test_camera_access(args.camera)

    if not camera_method:
        print("\n Camera test failed - cannot proceed")
        print(" Check camera connection and try: python3 scripts/setup_cameras.py --diagnostics")
        return 1

    # Test detection
    detection_ok = test_charuco_detection(camera_method, args.camera, args.duration)

    # Summary
    print(f"\n Test Summary:")
    print(f"   Camera: {' Working' if camera_method else ' Failed'}")
    print(f"   Detection: {' Working' if detection_ok else ' Needs improvement'}")

    if camera_method and detection_ok:
        print(f"\n Ready for calibration!")
        print(f"   Run: python3 scripts/camera_preview.py --camera {args.camera}")
        print(f"   Or:  python3 scripts/camera_wizard.py")
    else:
        print(f"\n Issues detected - run full diagnostics:")
        print(f"   python3 scripts/setup_cameras.py --diagnostics")

    return 0 if (camera_method and detection_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
