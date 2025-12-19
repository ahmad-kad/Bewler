#!/usr/bin/env python3
"""
ArUco Tag Validation System for Raspberry Pi - URC 2026

Real-time ArUco tag detection and distance calculation for validation.
Provides live camera feed with overlay showing:
- Detected ArUco tags with IDs
- Distance measurements to camera
- Validation status indicators
- FPS and detection statistics

Usage:
    python aruco_validator.py \\
        --calibration ../camera/camera_calibration.json \\
        --tag-size 10.0

Author: URC 2026 Autonomy Team
"""

import argparse
import json
import logging
import os
import sys
import time
from typing import Optional, Tuple

import cv2
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class ArucoValidator:
    """Real-time ArUco tag validation system for Raspberry Pi."""

    def __init__(
        self,
        calibration_file: Optional[str] = None,
        tag_size_cm: float = 10.0,
        aruco_dict: str = "DICT_4X4_50",
        camera_index: int = 0,
    ):
        """
        Initialize the ArUco validator.

        Args:
            calibration_file: Path to camera calibration JSON file
            tag_size_cm: Physical size of ArUco tags in centimeters
            aruco_dict: ArUco dictionary name
            camera_index: Camera device index
        """
        self.calibration_file = calibration_file
        self.tag_size_cm = tag_size_cm
        self.camera_index = camera_index
        self.aruco_dict_name = aruco_dict

        # Camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_loaded = False

        # Detection parameters
        self.aruco_dict = None
        self.parameters = None
        self.detector = None

        # Performance tracking
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.detection_stats = {
            "frames_processed": 0,
            "tags_detected": 0,
            "avg_distance": 0.0,
            "last_detection_time": 0.0,
        }

        # Initialize components
        self._setup_aruco_detector()
        self._load_calibration()

    def _setup_aruco_detector(self):
        """Setup ArUco detector with optimized parameters."""
        try:
            dict_attr = getattr(cv2.aruco, self.aruco_dict_name)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_attr)
            logger.info(f" ArUco dictionary loaded: {self.aruco_dict_name}")
        except AttributeError:
            logger.error(f" Invalid ArUco dictionary: {self.aruco_dict_name}")
            sys.exit(1)

        # Optimize detection parameters for real-time performance
        self.parameters = cv2.aruco.DetectorParameters()
        # Type ignore: OpenCV types not fully understood by mypy
        self.parameters.adaptiveThreshWinSizeMin = 3  # type: ignore[attr-defined]
        self.parameters.adaptiveThreshWinSizeMax = 23  # type: ignore[attr-defined]
        self.parameters.adaptiveThreshWinSizeStep = 10  # type: ignore[attr-defined]
        self.parameters.minMarkerPerimeterRate = 0.03  # type: ignore[attr-defined]
        self.parameters.maxMarkerPerimeterRate = 4.0  # type: ignore[attr-defined]
        self.parameters.polygonalApproxAccuracyRate = 0.05  # type: ignore[attr-defined]
        self.parameters.minCornerDistanceRate = 0.05  # type: ignore[attr-defined]
        self.parameters.minDistanceToBorder = 3  # type: ignore[attr-defined]
        self.parameters.minMarkerDistanceRate = 0.05  # type: ignore[attr-defined]

        # Create detector (OpenCV 4.7+ API)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        if self.detector is None:
            raise RuntimeError("Failed to create ArucoDetector")

    def _load_calibration(self):
        """Load camera calibration parameters from JSON file."""
        if not self.calibration_file:
            logger.warning(
                "  No calibration file provided - "
                "distance calculation will be unavailable"
            )
            return

        try:
            with open(self.calibration_file, "r") as f:
                calib_data = json.load(f)

            # Extract camera matrix and distortion coefficients
            if "camera_matrix" in calib_data:
                camera_matrix_data = calib_data["camera_matrix"]
                is_dict_with_data = (
                    isinstance(camera_matrix_data, dict)
                    and "data" in camera_matrix_data
                )
                if is_dict_with_data:
                    matrix_data = camera_matrix_data["data"]
                    self.camera_matrix = np.array(
                        matrix_data, dtype=np.float32
                    ).reshape(3, 3)
                else:
                    self.camera_matrix = np.array(
                        camera_matrix_data, dtype=np.float32
                    ).reshape(3, 3)
            elif "K" in calib_data:  # Alternative key
                self.camera_matrix = np.array(calib_data["K"], dtype=np.float32)

            # Load distortion coefficients
            if "distortion_coefficients" in calib_data:
                dist_data = calib_data["distortion_coefficients"]
                if isinstance(dist_data, dict) and "data" in dist_data:
                    self.dist_coeffs = np.array(
                        dist_data["data"], dtype=np.float32
                    ).flatten()
                else:
                    self.dist_coeffs = np.array(dist_data, dtype=np.float32).flatten()
            elif "D" in calib_data:
                # Alternative key for distortion coefficients
                self.dist_coeffs = np.array(calib_data["D"], dtype=np.float32).flatten()

            # Check if both camera matrix and distortion coefficients are loaded
            has_matrix = self.camera_matrix is not None
            has_dist_coeffs = self.dist_coeffs is not None
            if has_matrix and has_dist_coeffs:
                self.calibration_loaded = True
                logger.info(f" Camera calibration loaded from {self.calibration_file}")
                # Type ignore: mypy doesn't understand the None check above
                logger.info(f"   Camera matrix shape: {self.camera_matrix.shape}")  # type: ignore[attr-defined]
                logger.info(f"   Distortion coeffs shape: {self.dist_coeffs.shape}")  # type: ignore[attr-defined]
            else:
                logger.warning(
                    "  Camera calibration data incomplete - "
                    "distance calculation unavailable"
                )

        except Exception as e:
            logger.error(
                f" Failed to load calibration file " f"{self.calibration_file}: {e}"
            )

    def calculate_distance_to_tag(self, corners: np.ndarray) -> Tuple[float, float]:
        """
        Calculate distance to ArUco tag using camera calibration.

        Args:
            corners: Detected tag corners (4x2 array)

        Returns:
            Tuple of (distance_cm, confidence_score)
        """
        if not self.calibration_loaded:
            return 0.0, 0.0

        try:
            # Convert tag size from cm to meters for OpenCV
            tag_size_m = self.tag_size_cm / 100.0

            # Define 3D object points (tag corners in world coordinates)
            obj_points = np.array(
                [
                    [-tag_size_m / 2, tag_size_m / 2, 0],  # Top-left
                    [tag_size_m / 2, tag_size_m / 2, 0],  # Top-right
                    [tag_size_m / 2, -tag_size_m / 2, 0],  # Bottom-right
                    [-tag_size_m / 2, -tag_size_m / 2, 0],  # Bottom-left
                ],
                dtype=np.float32,
            )

            # Solve PnP to get rotation and translation vectors
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                corners.astype(np.float32),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )

            if success:
                # Translation vector gives position in camera coordinates
                # Distance is the magnitude of the translation vector
                distance_m = np.linalg.norm(tvec)
                distance_cm = distance_m * 100.0

                # Calculate confidence based on reprojection error
                projected_points, _ = cv2.projectPoints(
                    obj_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
                )
                diff = corners - projected_points.squeeze()
                reprojection_error = np.mean(np.linalg.norm(diff, axis=1))

                # Confidence decreases with reprojection error
                # (lower error = higher confidence)
                confidence = max(0.0, min(1.0, 1.0 - (reprojection_error / 10.0)))

                return distance_cm, confidence
            else:
                return 0.0, 0.0

        except Exception as e:
            logger.warning(f"Distance calculation failed: {e}")
            return 0.0, 0.0

    def draw_detection_overlay(
        self, frame: np.ndarray, corners: np.ndarray, ids: np.ndarray
    ) -> np.ndarray:
        """
        Draw detection overlay on frame with tag info and distance.

        Args:
            frame: Input video frame
            corners: Detected tag corners
            ids: Detected tag IDs

        Returns:
            Frame with overlay drawn
        """
        if ids is None or len(ids) == 0:
            return frame

        # Draw ArUco detections
        cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))

        # Process each detected tag
        for i, (corner, tag_id) in enumerate(zip(corners, ids)):
            tag_id = int(tag_id)

            # Calculate center point for text placement
            center_x = int(np.mean(corner[0][:, 0]))
            center_y = int(np.mean(corner[0][:, 1]))

            # Calculate distance if calibration available
            distance_cm, confidence = self.calculate_distance_to_tag(corner[0])

            # Choose color based on confidence/distance
            if confidence > 0.8:
                color = (0, 255, 0)  # Green - good detection
                status = ""
            elif confidence > 0.5:
                color = (0, 255, 255)  # Yellow - moderate
                status = "~"
            else:
                color = (0, 0, 255)  # Red - poor
                status = ""

            # Draw tag info box
            info_text = f"ID:{tag_id}"
            if self.calibration_loaded and distance_cm > 0:
                info_text += f" {distance_cm:.1f}cm"

            # Get text size for background rectangle
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2
            text_size_result = cv2.getTextSize(info_text, font, font_scale, thickness)
            (text_width, text_height), baseline = text_size_result

            # Draw background rectangle
            bg_x1 = center_x - text_width // 2 - 5
            bg_y1 = center_y - text_height - 5
            bg_x2 = center_x + text_width // 2 + 5
            bg_y2 = center_y + 5

            cv2.rectangle(frame, (bg_x1, bg_y1), (bg_x2, bg_y2), (0, 0, 0), -1)
            cv2.rectangle(frame, (bg_x1, bg_y1), (bg_x2, bg_y2), color, 2)

            # Draw text
            text_x = center_x - text_width // 2
            text_y = center_y - 5
            cv2.putText(
                frame, info_text, (text_x, text_y), font, font_scale, color, thickness
            )

            # Draw status indicator
            status_pos = (center_x - 10, center_y - text_height - 15)
            cv2.putText(frame, status, status_pos, font, 0.8, color, 2)

            # Update statistics
            self.detection_stats["tags_detected"] += 1
            if distance_cm > 0:
                prev_avg = self.detection_stats["avg_distance"]
                prev_count = self.detection_stats["tags_detected"] - 1
                self.detection_stats["avg_distance"] = (
                    (prev_avg * prev_count) + distance_cm
                ) / self.detection_stats["tags_detected"]

        return frame

    def draw_status_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Draw status and performance overlay on frame."""
        height, width = frame.shape[:2]

        # Calculate FPS
        current_time = time.time()
        self.fps_counter += 1
        if current_time - self.fps_start_time >= 1.0:
            fps = self.fps_counter / (current_time - self.fps_start_time)
            self.detection_stats["fps"] = fps
            self.fps_counter = 0
            self.fps_start_time = current_time

        # Status bar background
        status_height = 120
        rect_start = (0, height - status_height)
        rect_end = (width, height)
        cv2.rectangle(frame, rect_start, rect_end, (0, 0, 0), -1)
        cv2.rectangle(frame, rect_start, rect_end, (255, 255, 255), 2)

        # Status text
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 1
        y_offset = height - status_height + 25

        # Title
        title_text = "ArUco Tag Validator - URC 2026"
        cv2.putText(
            frame,
            title_text,
            (10, y_offset),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
        )

        # Calibration status
        y_offset += 25
        if self.calibration_loaded:
            calib_status = "Calibrated"
            calib_color = (0, 255, 0)
        else:
            calib_status = "No Calibration"
            calib_color = (0, 0, 255)
        cv2.putText(
            frame,
            f"Camera: {calib_status}",
            (10, y_offset),
            font,
            font_scale,
            calib_color,
            thickness,
        )

        # Performance stats
        y_offset += 20
        fps_text = f"FPS: {self.detection_stats.get('fps', 0):.1f}"
        cv2.putText(
            frame, fps_text, (10, y_offset), font, font_scale, (255, 255, 0), thickness
        )

        y_offset += 20
        frames_text = f"Frames: {self.detection_stats['frames_processed']}"
        cv2.putText(
            frame,
            frames_text,
            (10, y_offset),
            font,
            font_scale,
            (255, 255, 0),
            thickness,
        )

        # Detection stats
        y_offset += 20
        detections_text = f"Detections: {self.detection_stats['tags_detected']}"
        cv2.putText(
            frame,
            detections_text,
            (10, y_offset),
            font,
            font_scale,
            (0, 255, 0),
            thickness,
        )

        has_calib_and_distance = (
            self.calibration_loaded and self.detection_stats["avg_distance"] > 0
        )
        if has_calib_and_distance:
            y_offset += 20
            avg_dist = self.detection_stats["avg_distance"]
            avg_dist_text = f"Avg Distance: {avg_dist:.1f}cm"
            cv2.putText(
                frame,
                avg_dist_text,
                (10, y_offset),
                font,
                font_scale,
                (0, 255, 255),
                thickness,
            )

        # Instructions
        y_offset += 25
        instructions = "Press 'q' to quit, 'c' to clear stats"
        cv2.putText(frame, instructions, (10, y_offset), font, 0.5, (200, 200, 200), 1)

        return frame

    def run_validation(
        self,
        display: bool = True,
        save_video: Optional[str] = None,
        test_duration: Optional[int] = None,
    ) -> bool:
        """
        Run real-time ArUco tag validation.

        Args:
            display: Whether to display video feed
            save_video: Optional path to save video output
            test_duration: Optional test duration in seconds (auto-exit after this time)

        Returns:
            True if successful, False otherwise
        """
        logger.info(" Starting ArUco tag validation...")
        logger.info(f"   Camera index: {self.camera_index}")
        logger.info(f"   Tag size: {self.tag_size_cm}cm")
        logger.info(f"   Dictionary: {self.aruco_dict_name}")
        calib_status = "Loaded" if self.calibration_loaded else "None"
        logger.info(f"   Calibration: {calib_status}")

        # Open camera with fallback backends
        cap = None
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        for backend in backends:
            cap = cv2.VideoCapture(self.camera_index, backend)
            if cap.isOpened():
                logger.info(f" Camera opened with backend: {backend}")
                break
            cap.release()

        if not cap or not cap.isOpened():
            logger.error(
                f" Failed to open camera {self.camera_index} " f"with any backend"
            )
            return False

        # Configure camera for better performance
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency

        # Get actual camera properties
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))

        logger.info(f" Camera opened: {width}x{height} @ {fps} FPS")

        # Setup video writer if requested
        video_writer = None
        if save_video:
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            video_writer = cv2.VideoWriter(save_video, fourcc, fps, (width, height))
            logger.info(f" Recording to: {save_video}")

        try:
            if test_duration:
                logger.info(
                    f" Starting detection loop... "
                    f"(Will run for {test_duration} seconds)"
                )
                start_time = time.time()
            else:
                logger.info(" Starting detection loop... (Press 'q' to quit)")

            while True:
                # Capture frame
                ret, frame = cap.read()
                if not ret:
                    logger.error(" Failed to read frame from camera")
                    break

                self.detection_stats["frames_processed"] += 1

                # Detect ArUco tags (OpenCV 4.7+ API)
                # Detector is guaranteed to be set in __init__ or RuntimeError is raised
                if self.detector is None:
                    raise RuntimeError("Detector not initialized")
                # Type ignore: mypy control flow analysis doesn't understand the None check
                corners, ids, rejected = self.detector.detectMarkers(frame)  # type: ignore[unreachable]

                # Draw detection overlay
                frame = self.draw_detection_overlay(frame, corners, ids)

                # Draw status overlay
                frame = self.draw_status_overlay(frame)

                # Save frame to video if requested
                if video_writer:
                    video_writer.write(frame)

                # Check test duration
                if test_duration and (time.time() - start_time) >= test_duration:
                    logger.info(f"Test duration ({test_duration}s) completed")
                    break

                # Display frame
                if display:
                    cv2.imshow("ArUco Tag Validator", frame)

                    # Handle keyboard input
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        logger.info("User requested exit")
                        break
                    elif key == ord("c"):
                        # Clear statistics
                        self.detection_stats["tags_detected"] = 0
                        self.detection_stats["avg_distance"] = 0.0
                        logger.info(" Statistics cleared")

            logger.info(" Validation completed successfully")
            return True

        except KeyboardInterrupt:
            logger.info("Validation interrupted by user")
            return True
        except Exception as e:
            logger.error(f" Validation failed: {e}")
            return False
        finally:
            # Cleanup
            cap.release()
            if video_writer:
                video_writer.release()
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description=("Real-time ArUco tag validation with distance measurement"),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic validation without calibration
  python aruco_validator.py

  # With camera calibration for distance measurement
  python aruco_validator.py \\
      --calibration ../camera/camera_calibration.json \\
      --tag-size 10.0

  # Custom settings
  python aruco_validator.py \\
      --camera 1 --dict DICT_5X5_100 \\
      --tag-size 15.0 --save-video validation.avi

  # No display mode (for headless operation)
  python aruco_validator.py \\
      --no-display --save-video output.avi
        """,
    )

    default_calib = "../camera/camera_calibration.json"
    parser.add_argument(
        "--calibration",
        "-c",
        default=default_calib,
        help=f"Path to camera calibration JSON file (default: {default_calib})",
    )

    parser.add_argument(
        "--tag-size",
        "-s",
        type=float,
        default=10.0,
        help="Physical size of ArUco tags in millimeters (default: 10.0)",
    )

    parser.add_argument(
        "--camera", type=int, default=0, help="Camera device index (default: 0)"
    )

    parser.add_argument(
        "--dict",
        "-d",
        default="DICT_4X4_50",
        help="ArUco dictionary name (default: DICT_4X4_50)",
    )

    parser.add_argument("--save-video", help="Save output to video file")

    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Run without display (useful for headless operation)",
    )

    parser.add_argument(
        "--test-mode",
        type=int,
        metavar="SECONDS",
        help="Test mode: run for N seconds and exit (useful for CI/testing)",
    )

    args = parser.parse_args()

    # Validate calibration file exists if specified
    if args.calibration and not os.path.exists(args.calibration):
        logger.warning(f"  Calibration file not found: {args.calibration}")
        logger.warning("   Distance calculation will be unavailable")
        args.calibration = None

    # Create validator
    validator = ArucoValidator(
        calibration_file=args.calibration if args.calibration else None,
        tag_size_cm=args.tag_size / 10.0,  # Convert mm to cm
        aruco_dict=args.dict,
        camera_index=args.camera,
    )

    # Handle test mode
    if args.test_mode:
        logger.info(f" Running in test mode for {args.test_mode} seconds...")
        success = validator.run_validation(
            display=not args.no_display,
            save_video=args.save_video,
            test_duration=args.test_mode,
        )
    else:
        # Run validation
        success = validator.run_validation(
            display=not args.no_display, save_video=args.save_video
        )

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
