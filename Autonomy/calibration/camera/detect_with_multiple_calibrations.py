#!/usr/bin/env python3
"""
Multi-Camera ArUco Detection System - URC 2026

Use individual calibrations for each camera to detect ArUco tags with distance.
Process multiple cameras in parallel (or sequentially).

Usage:
    python detect_with_multiple_calibrations.py --calibration-dir ./camera_calibrations --tag-size 18
"""

import argparse
import json
import logging
import threading
from pathlib import Path

import cv2
import numpy as np

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(message)s")
logger = logging.getLogger(__name__)


class MultiCameraDetector:
    """Detect ArUco tags using multiple calibrated cameras."""

    def __init__(self, calibration_dir, tag_size_mm=18):
        """Initialize multi-camera detector."""
        self.calibration_dir = Path(calibration_dir)
        self.tag_size_mm = tag_size_mm
        self.tag_size_m = tag_size_mm / 1000.0
        self.calibrations = {}
        self.load_calibrations()

    def load_calibrations(self):
        """Load all calibration files from directory."""
        calib_files = sorted(self.calibration_dir.glob("camera_*_calibration.json"))

        if not calib_files:
            logger.warning(f"⚠️  No calibration files found in {self.calibration_dir}")
            return

        for calib_file in calib_files:
            try:
                with open(calib_file) as f:
                    calib_data = json.load(f)

                camera_index = calib_data.get("camera_index", 0)
                camera_matrix = np.array(calib_data["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
                dist_coeffs = np.array(calib_data["distortion_coefficients"]["data"], dtype=np.float32).flatten()

                self.calibrations[camera_index] = {
                    "file": str(calib_file),
                    "camera_matrix": camera_matrix,
                    "dist_coeffs": dist_coeffs,
                }
                logger.info(f"✅ Loaded calibration for camera {camera_index}: {calib_file.name}")
            except Exception as e:
                logger.error(f"❌ Failed to load {calib_file}: {e}")

        logger.info(f"📊 Total calibrations loaded: {len(self.calibrations)}")

    def calculate_distance(self, corners, calibration):
        """Calculate distance for a tag given corners and calibration."""
        try:
            # 3D object points for marker
            obj_points = np.array(
                [
                    [-self.tag_size_m / 2, -self.tag_size_m / 2, 0],
                    [self.tag_size_m / 2, -self.tag_size_m / 2, 0],
                    [self.tag_size_m / 2, self.tag_size_m / 2, 0],
                    [-self.tag_size_m / 2, self.tag_size_m / 2, 0],
                ],
                dtype=np.float32,
            )

            # Solve PnP
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                corners.astype(np.float32),
                calibration["camera_matrix"],
                calibration["dist_coeffs"],
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )

            if success:
                distance_m = np.linalg.norm(tvec)
                return distance_m * 1000.0  # Return in mm
            return None
        except Exception as e:
            logger.debug(f"Distance calculation failed: {e}")
            return None

    def process_camera(self, camera_index, duration=None, display=True):
        """Process a single camera."""
        if camera_index not in self.calibrations:
            logger.error(f"❌ No calibration for camera {camera_index}")
            return False

        logger.info(f"📷 Starting detection on camera {camera_index}...")

        # Open camera
        cap = cv2.VideoCapture(camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not cap.isOpened():
            logger.error(f"❌ Cannot open camera {camera_index}")
            return False

        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info(f"✅ Camera {camera_index} opened: {width}x{height}")

        # Setup ArUco
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        calibration = self.calibrations[camera_index]

        frame_count = 0
        import time

        start_time = time.time()

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                frame_count += 1

                # Detect ArUco markers
                corners, ids, rejected = detector.detectMarkers(frame)

                # Draw detections and distances
                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                    for i, (corner, tag_id) in enumerate(zip(corners, ids)):
                        tag_id = int(tag_id)

                        # Calculate distance
                        distance_mm = self.calculate_distance(corner[0], calibration)

                        # Draw distance info
                        center_x = int(np.mean(corner[0][:, 0]))
                        center_y = int(np.mean(corner[0][:, 1]))

                        if distance_mm is not None:
                            info_text = f"ID:{tag_id} {distance_mm:.1f}mm"
                            color = (0, 255, 0)
                        else:
                            info_text = f"ID:{tag_id}"
                            color = (0, 0, 255)

                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(frame, info_text, (center_x - 50, center_y), font, 0.5, color, 2)

                # Draw status
                status_text = (
                    f"Camera {camera_index} | Frame {frame_count} | Detections: {len(ids) if ids is not None else 0}"
                )
                cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # Display
                if display:
                    cv2.imshow(f"Camera {camera_index}", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        break

                # Check duration
                if duration and (time.time() - start_time) >= duration:
                    break

        finally:
            cap.release()
            cv2.destroyAllWindows()
            logger.info(f"📊 Camera {camera_index}: Processed {frame_count} frames")

        return True

    def process_all_cameras(self, duration=None, display=True, threaded=False):
        """Process all cameras."""
        if not self.calibrations:
            logger.error("❌ No calibrations loaded")
            return False

        logger.info(f"🎬 Processing {len(self.calibrations)} cameras...")

        if threaded:
            # Process cameras in parallel threads
            threads = []
            for camera_index in sorted(self.calibrations.keys()):
                thread = threading.Thread(target=self.process_camera, args=(camera_index, duration, display))
                threads.append(thread)
                thread.start()

            for thread in threads:
                thread.join()
        else:
            # Process cameras sequentially
            for camera_index in sorted(self.calibrations.keys()):
                self.process_camera(camera_index, duration, display)

        logger.info("✅ Detection completed")
        return True


def main():
    parser = argparse.ArgumentParser(
        description="Multi-camera ArUco detection with calibrations",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Detect on all cameras with individual calibrations
  python detect_with_multiple_calibrations.py --calibration-dir ./camera_calibrations

  # Sequential detection (one at a time, useful for setup)
  python detect_with_multiple_calibrations.py --calibration-dir ./camera_calibrations --sequential

  # Process specific camera only
  python detect_with_multiple_calibrations.py --calibration-dir ./camera_calibrations --camera 0

  # Run for 30 seconds then exit
  python detect_with_multiple_calibrations.py --calibration-dir ./camera_calibrations --duration 30
        """,
    )

    parser.add_argument("--calibration-dir", "-c", required=True, help="Directory containing calibration files")
    parser.add_argument("--tag-size", type=int, default=18, help="Tag size in mm (default: 18)")
    parser.add_argument("--camera", type=int, help="Process specific camera index only")
    parser.add_argument("--sequential", action="store_true", help="Process cameras sequentially instead of in parallel")
    parser.add_argument("--duration", type=int, help="Run for N seconds then exit")
    parser.add_argument("--no-display", action="store_true", help="Run without display")

    args = parser.parse_args()

    # Create detector
    detector = MultiCameraDetector(args.calibration_dir, args.tag_size)

    if not detector.calibrations:
        logger.error("❌ No calibrations loaded. Exiting.")
        return 1

    # Process
    if args.camera is not None:
        # Process single camera
        logger.info(f"Processing camera {args.camera} only")
        success = detector.process_camera(args.camera, duration=args.duration, display=not args.no_display)
    else:
        # Process all cameras
        success = detector.process_all_cameras(
            duration=args.duration, display=not args.no_display, threaded=not args.sequential
        )

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
