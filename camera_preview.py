#!/usr/bin/env python3
"""
Camera Preview with Detection - URC 2026
Live video stream with real-time ChArUco board detection and visual feedback
"""

import cv2
import numpy as np
import time
import argparse
from pathlib import Path
import sys
import os

# Import shared configuration
from camera_config import CHARUCO_CONFIG

try:
    import picamera2

    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False


class CameraPreview:
    """Live camera preview with ChArUco detection."""

    def __init__(self, camera_index=0, method="auto", board_size=None, square_size=None, marker_size=None):
        self.camera_index = camera_index
        self.method = method
        self.cap = None
        self.picam2 = None

        # Use config values as defaults
        self.board_size = board_size or CHARUCO_CONFIG["board_size"]
        self.square_size = square_size or CHARUCO_CONFIG["square_size"]
        self.marker_size = marker_size or CHARUCO_CONFIG["marker_size"]

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, CHARUCO_CONFIG["dictionary"]))
        self.board = cv2.aruco.CharucoBoard(self.board_size, self.square_size, self.marker_size, self.aruco_dict)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
        self.charuco_detector = cv2.aruco.CharucoDetector(self.board)

        # Statistics
        self.frame_count = 0
        self.fps_start_time = time.time()
        self.fps_counter = 0
        self.fps = 0

        # Detection stats
        self.marker_detections = 0
        self.board_detections = 0
        self.last_detection_time = 0

    def initialize_camera(self):
        """Initialize camera based on method preference."""
        print(f" Initializing camera {self.camera_index}...")

        if self.method == "auto":
            # Try libcamera first (Raspberry Pi 5), fallback to OpenCV
            if PICAMERA_AVAILABLE:
                try:
                    camera_info = picamera2.Picamera2.global_camera_info()
                    if self.camera_index < len(camera_info):
                        self.method = "libcamera"
                        print(" Using libcamera (Raspberry Pi 5)")
                    else:
                        self.method = "opencv"
                        print(" Fallback to OpenCV V4L2")
                except:
                    self.method = "opencv"
                    print(" Using OpenCV V4L2")
            else:
                self.method = "opencv"
                print(" Using OpenCV V4L2")

        if self.method == "libcamera" and PICAMERA_AVAILABLE:
            try:
                self.picam2 = picamera2.Picamera2(self.camera_index)
                config = self.picam2.create_preview_configuration()
                self.picam2.configure(config)
                self.picam2.start()
                print(" Libcamera initialized successfully")
                return True
            except Exception as e:
                print(f" Libcamera failed: {e}")
                return False

        elif self.method == "opencv":
            try:
                self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
                if not self.cap.isOpened():
                    print(f" Cannot open camera {self.camera_index}")
                    return False

                # Set resolution
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

                width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(f" OpenCV camera initialized: {width}x{height}")
                return True
            except Exception as e:
                print(f" OpenCV failed: {e}")
                return False

        return False

    def capture_frame(self):
        """Capture a frame from the camera."""
        if self.method == "libcamera" and self.picam2:
            frame = self.picam2.capture_array()
            # Convert RGB to BGR for OpenCV
            if frame.shape[2] == 3:  # RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame
        elif self.method == "opencv" and self.cap:
            ret, frame = self.cap.read()
            return frame if ret else None
        return None

    def detect_charuco(self, frame):
        """Detect ChArUco board in frame."""
        # Detect markers
        corners, ids, _rejected = self.detector.detectMarkers(frame)

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

            # Try to detect ChArUco board
            if detection_info["markers_found"] >= 4:
                try:
                    # Ensure frame is in correct format for ChArUco detection
                    if frame.shape[2] == 4:  # XBGR format from libcamera
                        # Convert to BGR
                        detection_frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                    else:
                        detection_frame = frame

                    charuco_corners, charuco_ids, _rejected_corners, _rejected_ids = self.charuco_detector.detectBoard(
                        detection_frame
                    )

                    if charuco_corners is not None and len(charuco_corners) >= 6:
                        detection_info["board_detected"] = True
                        detection_info["charuco_corners"] = charuco_corners
                        detection_info["charuco_ids"] = charuco_ids
                        detection_info["quality_score"] = min(100, len(charuco_corners) * 10)
                except cv2.error as e:
                    print(f"Warning: ChArUco detection failed: {e}")
                    # Continue without ChArUco detection
                    pass

        return detection_info

    def draw_overlay(self, frame, detection_info):
        """Draw detection overlay on frame."""
        height, width = frame.shape[:2]

        # Create overlay background
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (400, 150), (0, 0, 0), -1)
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

        # Draw FPS
        fps_text = f"FPS: {self.fps:.1f}"
        cv2.putText(frame, fps_text, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Draw camera info
        method_text = f"Camera {self.camera_index} ({self.method})"
        cv2.putText(frame, method_text, (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Draw detected markers with safety checks
        if (
            detection_info["markers_found"] > 0
            and detection_info["corners"] is not None
            and detection_info["ids"] is not None
            and len(detection_info["corners"]) > 0
            and len(detection_info["ids"]) > 0
        ):
            try:
                cv2.aruco.drawDetectedMarkers(frame, detection_info["corners"], detection_info["ids"])
            except cv2.error as e:
                # Log the error but don't crash
                print(f"Warning: Failed to draw markers: {e}")
                # Draw a simple indicator instead
                cv2.putText(
                    frame, "MARKERS", (width - 100, height - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2
                )

        # Draw ChArUco corners if detected
        if (
            detection_info["board_detected"]
            and detection_info["charuco_corners"] is not None
            and detection_info["charuco_ids"] is not None
        ):
            try:
                cv2.aruco.drawDetectedCornersCharuco(
                    frame, detection_info["charuco_corners"], detection_info["charuco_ids"]
                )
            except cv2.error as e:
                # Log the error but don't crash
                print(f"Warning: Failed to draw ChArUco corners: {e}")
                # Draw a simple indicator instead
                cv2.putText(
                    frame, "CHARUCO", (width - 120, height - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
                )

        # Draw help text
        help_y = height - 120
        cv2.putText(frame, "Controls:", (10, help_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "  S - Save frame", (10, help_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(
            frame, "  C - Capture for calibration", (10, help_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1
        )
        cv2.putText(frame, "  Q - Quit", (10, help_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        return frame

    def update_fps(self):
        """Update FPS calculation."""
        self.fps_counter += 1
        current_time = time.time()

        if current_time - self.fps_start_time >= 1.0:
            self.fps = self.fps_counter / (current_time - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = current_time

    def save_frame(self, frame, detection_info, prefix="frame"):
        """Save current frame with metadata."""
        timestamp = time.strftime("%Y%m%d_%H%M%S")

        # Create output directory
        output_dir = Path("preview_frames")
        output_dir.mkdir(exist_ok=True)

        # Save image
        filename = f"{prefix}_{timestamp}_{detection_info['quality_score']}pct.png"
        filepath = output_dir / filename
        cv2.imwrite(str(filepath), frame)

        # Save metadata
        metadata = {
            "timestamp": timestamp,
            "camera_index": self.camera_index,
            "method": self.method,
            "markers_found": detection_info["markers_found"],
            "board_detected": detection_info["board_detected"],
            "quality_score": detection_info["quality_score"],
            "fps": self.fps,
        }

        metadata_file = filepath.with_suffix(".json")
        import json

        with open(metadata_file, "w") as f:
            json.dump(metadata, f, indent=2)

        print(f" Saved frame: {filepath}")
        return str(filepath)

    def run_preview(self, window_name="Camera Preview - URC 2026"):
        """Run the live preview loop."""
        print(" Starting camera preview...")
        print("Controls:")
        print("  S - Save current frame")
        print("  C - Capture frame for calibration")
        print("  Q - Quit")
        print()

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)

        saved_frames = []

        try:
            while True:
                # Capture frame
                frame = self.capture_frame()
                if frame is None:
                    print(" Failed to capture frame")
                    break

                self.frame_count += 1

                # Detect ChArUco board
                detection_info = self.detect_charuco(frame)

                # Update statistics
                if detection_info["markers_found"] > 0:
                    self.marker_detections += 1
                if detection_info["board_detected"]:
                    self.board_detections += 1
                    self.last_detection_time = time.time()

                # Draw overlay
                display_frame = self.draw_overlay(frame, detection_info)

                # Update FPS
                self.update_fps()

                # Show frame
                cv2.imshow(window_name, display_frame)

                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    break
                elif key == ord("s"):
                    filepath = self.save_frame(display_frame, detection_info, "preview")
                    print(f" Frame saved: {filepath}")
                elif key == ord("c"):
                    if detection_info["board_detected"]:
                        filepath = self.save_frame(frame, detection_info, "calibration")
                        saved_frames.append(filepath)
                        print(f" Calibration frame captured: {filepath}")
                        print(f"   Quality: {detection_info['quality_score']}%")
                    else:
                        print(" No board detected - move board into view")

                # Auto-save good frames occasionally
                if detection_info["board_detected"] and detection_info["quality_score"] >= 80:
                    if len(saved_frames) < 5:  # Keep max 5 auto-saved frames
                        filepath = self.save_frame(frame, detection_info, "auto")
                        saved_frames.append(filepath)

        finally:
            self.cleanup()

        # Summary
        duration = time.time() - self.fps_start_time
        print("Preview Summary:")
        print(f"   Duration: {duration:.1f} seconds")
        print(f"   Total frames: {self.frame_count}")
        print(f"   Average FPS: {self.fps:.1f}")
        print(f"   Marker detections: {self.marker_detections}")
        print(f"   Board detections: {self.board_detections}")
        print(f"   Frames saved: {len(saved_frames)}")

        if saved_frames:
            print(f"\n Saved frames in: preview_frames/")
            for frame in saved_frames[-3:]:  # Show last 3
                print(f"   {frame}")

    def cleanup(self):
        """Clean up camera resources."""
        if self.picam2:
            self.picam2.stop()
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Camera Preview with ChArUco Detection")
    parser.add_argument("--camera", "-c", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument(
        "--method",
        "-m",
        choices=["auto", "libcamera", "opencv"],
        default="auto",
        help="Camera access method (default: auto)",
    )
    parser.add_argument("--width", type=int, default=1280, help="Frame width (default: 1280)")
    parser.add_argument("--height", type=int, default=720, help="Frame height (default: 720)")
    parser.add_argument(
        "--board-size",
        type=int,
        nargs=2,
        default=CHARUCO_CONFIG["board_size"],
        help=f'ChArUco board size as columns rows (default: {CHARUCO_CONFIG["board_size"]})',
    )
    parser.add_argument(
        "--square-size",
        type=float,
        default=CHARUCO_CONFIG["square_size"],
        help=f'Size of checkerboard squares in meters (default: {CHARUCO_CONFIG["square_size"]})',
    )
    parser.add_argument(
        "--marker-size",
        type=float,
        default=CHARUCO_CONFIG["marker_size"],
        help=f'Size of ArUco markers in meters (default: {CHARUCO_CONFIG["marker_size"]})',
    )

    args = parser.parse_args()

    print(" URC 2026 Camera Preview with Detection")
    print("=" * 50)

    # Create preview instance
    preview = CameraPreview(args.camera, args.method, tuple(args.board_size), args.square_size, args.marker_size)

    # Initialize camera
    if not preview.initialize_camera():
        print(" Failed to initialize camera")
        return 1

    # Run preview
    try:
        preview.run_preview()
    except KeyboardInterrupt:
        print("\n Preview interrupted by user")
    except Exception as e:
        print(f"\n Preview error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
