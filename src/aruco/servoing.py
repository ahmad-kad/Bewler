#!/usr/bin/env python3
"""
ArUco Detection System for Camera Servoing - URC 2026

Real-time ArUco marker detection optimized for camera servoing applications.
Provides delta calculations, pose estimation (6DOF), and servoing error values.

Features:
- Low-latency detection pipeline
- Delta calculation (X, Y offset from image center)
- 6DOF pose estimation (rotation and translation)
- RPY error calculation for orientation servoing
- Distance measurement
- Real-time visualization with crosshairs and pose axes
- Configurable ArUco dictionary and target orientation

Usage:
    from src.aruco.servoing import ArucoServoingDetector
    
    detector = ArucoServoingDetector(
        camera_id=0,
        resolution=(640, 480),
        calibration_file="calibrations/arm_base.json",
        marker_size_m=0.19
    )
    detector.run()

Author: URC 2026 Autonomy Team
"""

import json
import logging
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

# Try to import camera utilities (works when part of package)
try:
    from ..utils.camera import setup_camera_capture
except ImportError:
    # Fallback for direct script execution
    try:
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).parent.parent.parent))
        from src.utils.camera import setup_camera_capture
    except ImportError:
        # If still fails, we'll use direct OpenCV initialization
        setup_camera_capture = None

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class ArucoServoingDetector:
    """ArUco marker detector optimized for camera servoing applications."""

    def __init__(
        self,
        camera_id: int = 0,
        resolution: Tuple[int, int] = (640, 480),
        aruco_dict: str = "DICT_6X6_250",
        fps: int = 30,
        display: Optional[bool] = None,
        marker_size_m: float = 0.19,  # 190mm in meters
        calibration_file: Optional[str] = None,
        target_rpy: Optional[Tuple[float, float, float]] = None,  # Target roll, pitch, yaw in degrees
    ):
        """
        Initialize ArUco detector with camera.

        Args:
            camera_id: USB camera device ID (0 for default)
            resolution: Tuple of (width, height) for camera resolution
            aruco_dict: ArUco dictionary name (default: DICT_6X6_250)
            fps: Target frames per second
            display: Whether to display video feed (None = auto-detect)
            marker_size_m: Physical size of ArUco marker in meters (default: 0.19 for 190mm)
            calibration_file: Path to camera calibration JSON file (optional, for pose estimation)
            target_rpy: Target roll, pitch, yaw in degrees (default: (0, 0, 0) for perpendicular)
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.fps = fps
        self.cap: Optional[cv2.VideoCapture] = None
        self.image_center: Optional[Tuple[int, int]] = None
        self.aruco_dict_name = aruco_dict
        self.marker_size_m = marker_size_m
        self.calibration_file = calibration_file
        self.target_rpy = target_rpy if target_rpy is not None else (0.0, 0.0, 0.0)
        
        # Camera calibration parameters for pose estimation
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.calibration_loaded = False

        # Determine display capability
        if display is None:
            # Auto-detect: try GUI by default, will disable if it fails at runtime
            self.display_enabled = True
            self._auto_detect_gui = True
        else:
            self.display_enabled = display
            self._auto_detect_gui = False

        if not self.display_enabled:
            logger.info("Running in headless mode (no display)")

        # Load camera calibration if provided
        if self.calibration_file:
            self._load_calibration()

        # Initialize ArUco detector
        self._setup_aruco_detector()

    def _setup_aruco_detector(self) -> None:
        """Setup ArUco detector with optimized parameters for servoing."""
        try:
            dict_attr = getattr(cv2.aruco, self.aruco_dict_name)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_attr)
            logger.info(f"ArUco dictionary loaded: {self.aruco_dict_name}")
        except AttributeError:
            logger.error(f"Invalid ArUco dictionary: {self.aruco_dict_name}")
            raise ValueError(
                f"Invalid ArUco dictionary: {self.aruco_dict_name}. "
                "Use a predefined dictionary like DICT_6X6_250, DICT_4X4_50, or DICT_ARUCO_ORIGINAL"
            )

        # Optimize detection parameters for low-latency servoing
        # Adjusted for large markers (190mm and similar)
        self.aruco_params = cv2.aruco.DetectorParameters()
        # Type ignore: OpenCV types not fully understood by mypy
        self.aruco_params.adaptiveThreshWinSizeMin = 3  # type: ignore[attr-defined]
        self.aruco_params.adaptiveThreshWinSizeMax = 23  # type: ignore[attr-defined]
        self.aruco_params.adaptiveThreshWinSizeStep = 10  # type: ignore[attr-defined]
        # Larger markers need more lenient perimeter rates
        self.aruco_params.minMarkerPerimeterRate = 0.01  # type: ignore[attr-defined]  # Lower for large markers
        self.aruco_params.maxMarkerPerimeterRate = 8.0  # type: ignore[attr-defined]  # Higher for large markers
        self.aruco_params.polygonalApproxAccuracyRate = 0.03  # type: ignore[attr-defined]  # More lenient
        self.aruco_params.minCornerDistanceRate = 0.03  # type: ignore[attr-defined]  # More lenient
        self.aruco_params.minDistanceToBorder = 1  # type: ignore[attr-defined]  # Allow markers near edges
        self.aruco_params.minMarkerDistanceRate = 0.05  # type: ignore[attr-defined]
        # Enable corner refinement for better accuracy on large markers
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # type: ignore[attr-defined]
        self.aruco_params.cornerRefinementWinSize = 5  # type: ignore[attr-defined]
        self.aruco_params.cornerRefinementMaxIterations = 30  # type: ignore[attr-defined]
        self.aruco_params.cornerRefinementMinAccuracy = 0.1  # type: ignore[attr-defined]

        # Create detector (OpenCV 4.7+ API)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        if self.detector is None:
            raise RuntimeError("Failed to create ArucoDetector")

    def initialize_camera(self) -> None:
        """Initialize and configure the USB camera."""
        logger.info(f"Initializing camera {self.camera_id}...")

        # Try using project's camera setup utility first (if available)
        cap = None
        if setup_camera_capture is not None:
            try:
                cap = setup_camera_capture(
                    camera_index=self.camera_id,
                    width=self.resolution[0],
                    height=self.resolution[1],
                    fps=self.fps,
                )
            except Exception as e:
                logger.warning(f"Camera utility failed: {e}, trying direct OpenCV initialization")

        if cap is not None:
            self.cap = cap
        else:
            # Fallback to direct OpenCV initialization
            logger.info("Using direct OpenCV initialization")
            backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
            for backend in backends:
                self.cap = cv2.VideoCapture(self.camera_id, backend)
                if self.cap.isOpened():
                    logger.info(f"Camera opened with backend: {backend}")
                    break
                if self.cap:
                    self.cap.release()

        if not self.cap or not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.camera_id}")

        # Set camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency for servoing

        # Get actual resolution (may differ from requested)
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.image_center = (actual_width // 2, actual_height // 2)

        logger.info(f"Camera initialized: {actual_width}x{actual_height}")
        logger.info(f"Image center: {self.image_center}")

    def calculate_marker_center(self, corners: np.ndarray) -> Tuple[int, int]:
        """
        Calculate the center point of an ArUco marker.

        Args:
            corners: Array of marker corner coordinates (shape: (1, 4, 2))

        Returns:
            Tuple of (center_x, center_y)
        """
        # Corners are in shape (1, 4, 2), extract the 4 corner points
        corner_points = corners[0]
        center_x = int(np.mean(corner_points[:, 0]))
        center_y = int(np.mean(corner_points[:, 1]))
        return (center_x, center_y)

    def calculate_delta(
        self, marker_center: Tuple[int, int]
    ) -> Dict[str, float]:
        """
        Calculate delta (offset) from image center.

        Args:
            marker_center: Tuple of (x, y) coordinates of marker center

        Returns:
            Dictionary containing delta information:
            - delta_x: Pixel offset in X direction
            - delta_y: Pixel offset in Y direction
            - norm_delta_x: Normalized X offset (-1 to 1)
            - norm_delta_y: Normalized Y offset (-1 to 1)
            - distance: Euclidean distance from center in pixels
        """
        if self.image_center is None:
            raise RuntimeError("Image center not initialized. Call initialize_camera() first.")

        delta_x = float(marker_center[0] - self.image_center[0])
        delta_y = float(marker_center[1] - self.image_center[1])

        # Normalized delta values (-1 to 1 range)
        # Divide by half-width/height to get normalized range
        norm_delta_x = delta_x / self.image_center[0]
        norm_delta_y = delta_y / self.image_center[1]

        # Calculate distance from center
        distance_from_center = float(np.sqrt(delta_x**2 + delta_y**2))

        return {
            "delta_x": delta_x,
            "delta_y": delta_y,
            "norm_delta_x": norm_delta_x,
            "norm_delta_y": norm_delta_y,
            "distance": distance_from_center,
        }

    def _load_calibration(self) -> None:
        """Load camera calibration parameters from JSON file."""
        if not self.calibration_file or not os.path.exists(self.calibration_file):
            logger.warning(
                f"Calibration file not found: {self.calibration_file} - "
                "pose estimation will be unavailable"
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
                elif isinstance(dist_data, list) and len(dist_data) > 0:
                    # Handle nested list format [[...]]
                    if isinstance(dist_data[0], list):
                        self.dist_coeffs = np.array(dist_data[0], dtype=np.float32).flatten()
                    else:
                        self.dist_coeffs = np.array(dist_data, dtype=np.float32).flatten()
                else:
                    self.dist_coeffs = np.array(dist_data, dtype=np.float32).flatten()
            elif "D" in calib_data:
                self.dist_coeffs = np.array(calib_data["D"], dtype=np.float32).flatten()

            # Check if both camera matrix and distortion coefficients are loaded
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                self.calibration_loaded = True
                logger.info(f"Camera calibration loaded from {self.calibration_file}")
                logger.info(f"  Camera matrix shape: {self.camera_matrix.shape}")
                logger.info(f"  Distortion coeffs shape: {self.dist_coeffs.shape}")
            else:
                logger.warning(
                    "Camera calibration data incomplete - "
                    "pose estimation unavailable"
                )

        except Exception as e:
            logger.error(f"Failed to load calibration file {self.calibration_file}: {e}")

    def calculate_pose(
        self, corners: np.ndarray
    ) -> Optional[Dict[str, Any]]:
        """
        Calculate 6DOF pose (rotation and translation) of ArUco marker.

        Args:
            corners: Detected marker corners (shape: (1, 4, 2))

        Returns:
            Dictionary with pose information or None if calibration not available:
            - rvec: Rotation vector (3x1)
            - tvec: Translation vector (3x1) in meters
            - rotation_matrix: 3x3 rotation matrix
            - euler_angles: (roll, pitch, yaw) in degrees
            - distance: Distance from camera in meters
            - position: (x, y, z) position in camera coordinates (meters)
        """
        if not self.calibration_loaded:
            return None

        try:
            # Define 3D object points (marker corners in world coordinates)
            # Marker is in XY plane, centered at origin
            half_size = self.marker_size_m / 2.0
            obj_points = np.array(
                [
                    [-half_size, half_size, 0.0],   # Top-left
                    [half_size, half_size, 0.0],    # Top-right
                    [half_size, -half_size, 0.0],  # Bottom-right
                    [-half_size, -half_size, 0.0], # Bottom-left
                ],
                dtype=np.float32,
            )

            # Extract corner points (corners shape is (1, 4, 2))
            image_points = corners[0].astype(np.float32)

            # Solve PnP to get rotation and translation vectors
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )

            if not success:
                return None

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Extract Euler angles (ZYX convention)
            sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
            singular = sy < 1e-6

            if not singular:
                roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            else:
                roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
                pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                yaw = 0

            # Convert to degrees
            euler_angles = np.degrees([roll, pitch, yaw])

            # Calculate distance (magnitude of translation vector)
            distance = float(np.linalg.norm(tvec))

            # Extract position (x, y, z in camera coordinates)
            position = {
                "x": float(tvec[0, 0]),
                "y": float(tvec[1, 0]),
                "z": float(tvec[2, 0]),
            }

            # Calculate perpendicularity (angle from camera's view direction)
            # When camera is perpendicular to marker, the marker's Z-axis points at camera
            # The angle between camera view direction (0,0,1) and marker Z-axis gives skew
            # Marker Z-axis in camera coordinates is the third column of rotation matrix
            marker_z_axis = rotation_matrix[:, 2]  # Z-axis direction in camera frame
            camera_view = np.array([0, 0, 1])  # Camera looks along +Z
            
            # Calculate angle between camera view and marker normal
            cos_angle = np.clip(np.dot(marker_z_axis, camera_view), -1.0, 1.0)
            angle_from_perpendicular = np.degrees(np.arccos(cos_angle))
            
            # Perpendicularity score (0-1, where 1 is perfectly perpendicular)
            perpendicularity = float(np.cos(np.radians(angle_from_perpendicular)))

            # Calculate RPY errors (target - current) for servoing
            rpy_errors = {
                "roll": float(self.target_rpy[0] - euler_angles[0]),
                "pitch": float(self.target_rpy[1] - euler_angles[1]),
                "yaw": float(self.target_rpy[2] - euler_angles[2]),
            }
            
            return {
                "rvec": rvec.flatten().tolist(),
                "tvec": tvec.flatten().tolist(),
                "rotation_matrix": rotation_matrix.tolist(),
                "euler_angles": {
                    "roll": float(euler_angles[0]),
                    "pitch": float(euler_angles[1]),
                    "yaw": float(euler_angles[2]),
                },
                "rpy_errors": rpy_errors,
                "target_rpy": {
                    "roll": float(self.target_rpy[0]),
                    "pitch": float(self.target_rpy[1]),
                    "yaw": float(self.target_rpy[2]),
                },
                "distance": distance,
                "position": position,
                "angle_from_perpendicular": float(angle_from_perpendicular),
                "perpendicularity": perpendicularity,
            }

        except Exception as e:
            logger.warning(f"Pose calculation failed: {e}")
            return None

    def draw_pose_axes(
        self, frame: np.ndarray, corners: np.ndarray, pose: Dict[str, Any]
    ) -> None:
        """
        Draw 3D pose axes on the frame.

        Args:
            frame: Image frame to draw on
            corners: Marker corner coordinates
            pose: Pose dictionary from calculate_pose()
        """
        if not self.calibration_loaded or self.camera_matrix is None:
            return

        try:
            # Define axis points in 3D (in marker coordinate system)
            axis_length = self.marker_size_m * 0.5
            axis_points = np.array(
                [
                    [0, 0, 0],  # Origin
                    [axis_length, 0, 0],  # X axis (red)
                    [0, axis_length, 0],  # Y axis (green)
                    [0, 0, -axis_length],  # Z axis (blue)
                ],
                dtype=np.float32,
            )

            # Project 3D points to 2D image plane
            rvec = np.array(pose["rvec"], dtype=np.float32).reshape(3, 1)
            tvec = np.array(pose["tvec"], dtype=np.float32).reshape(3, 1)
            image_points, _ = cv2.projectPoints(
                axis_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
            )

            # Draw axes
            origin = tuple(map(int, image_points[0].ravel()))
            x_end = tuple(map(int, image_points[1].ravel()))
            y_end = tuple(map(int, image_points[2].ravel()))
            z_end = tuple(map(int, image_points[3].ravel()))

            # X axis (red)
            cv2.line(frame, origin, x_end, (0, 0, 255), 3)
            # Y axis (green)
            cv2.line(frame, origin, y_end, (0, 255, 0), 3)
            # Z axis (blue)
            cv2.line(frame, origin, z_end, (255, 0, 0), 3)

        except Exception as e:
            logger.debug(f"Failed to draw pose axes: {e}")

    def draw_detection_info(
        self,
        frame: np.ndarray,
        corners: np.ndarray,
        marker_id: int,
        marker_center: Tuple[int, int],
        delta_info: Dict[str, float],
        pose: Optional[Dict[str, Any]] = None,
    ) -> None:
        """
        Draw detection visualization on the frame with organized information panel.

        Args:
            frame: Image frame to draw on
            corners: Marker corner coordinates
            marker_id: ID of the detected marker
            marker_center: Center point of the marker
            delta_info: Dictionary with delta information
            pose: Optional pose information dictionary
        """
        # Draw marker boundaries
        cv2.polylines(
            frame, [corners[0].astype(int)], True, (0, 255, 0), 2
        )

        # Draw marker center
        cv2.circle(frame, marker_center, 5, (0, 0, 255), -1)

        # Draw line from image center to marker center
        if self.image_center:
            cv2.line(
                frame, self.image_center, marker_center, (255, 0, 0), 2
            )

        # Display marker ID near marker
        text_pos = (marker_center[0] - 40, marker_center[1] - 20)
        cv2.putText(
            frame,
            f"ID: {marker_id}",
            text_pos,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )

        # Information display starting position - simplified for servoing
        panel_x, panel_y = 10, 10
        y_offset = panel_y + 20
        line_height = 22
        font_scale = 0.6
        font_thickness = 2

        # Essential servoing information only
        
        # Delta (pixel offset) - essential for 2D servoing
        delta_text = (
            f"Delta: ({delta_info['delta_x']:+5.0f}, {delta_info['delta_y']:+5.0f})px"
        )
        cv2.putText(
            frame,
            delta_text,
            (panel_x, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 255, 255),
            font_thickness,
        )
        y_offset += line_height

        # Distance - essential for distance control
        if pose and "distance" in pose:
            distance = pose["distance"]
            distance_cm = distance * 100.0
            distance_text = f"Distance: {distance:.3f}m ({distance_cm:.1f}cm)"
            cv2.putText(
                frame,
                distance_text,
                (panel_x, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (0, 255, 0),
                font_thickness,
            )
            y_offset += line_height

        # RPY Error - essential for orientation servoing
        if pose and "rpy_errors" in pose:
            errors = pose["rpy_errors"]
            max_error = max(abs(errors["roll"]), abs(errors["pitch"]), abs(errors["yaw"]))
            
            # Color code errors
            if max_error < 2.0:
                error_color = (0, 255, 0)  # Green
            elif max_error < 5.0:
                error_color = (0, 255, 255)  # Yellow
            else:
                error_color = (0, 0, 255)  # Red
            
            error_text = (
                f"RPY Error: ({errors['roll']:+.2f}, {errors['pitch']:+.2f}, {errors['yaw']:+.2f})°"
            )
            cv2.putText(
                frame,
                error_text,
                (panel_x, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                error_color,
                font_thickness,
            )
            y_offset += line_height

        # Position (x, y, z) - useful for 3D servoing
        if pose and "position" in pose:
            pos = pose["position"]
            pos_text = f"Position: ({pos['x']:+.3f}, {pos['y']:+.3f}, {pos['z']:+.3f})m"
            cv2.putText(
                frame,
                pos_text,
                (panel_x, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale - 0.1,
                (200, 200, 200),
                font_thickness - 1,
            )

    def draw_crosshairs(self, frame: np.ndarray) -> None:
        """Draw crosshairs at image center."""
        if self.image_center is None:
            return

        color = (128, 128, 128)
        thickness = 1
        size = 20

        # Horizontal line
        cv2.line(
            frame,
            (self.image_center[0] - size, self.image_center[1]),
            (self.image_center[0] + size, self.image_center[1]),
            color,
            thickness,
        )

        # Vertical line
        cv2.line(
            frame,
            (self.image_center[0], self.image_center[1] - size),
            (self.image_center[0], self.image_center[1] + size),
            color,
            thickness,
        )

        # Center circle
        cv2.circle(frame, self.image_center, 3, color, -1)

    def detect_and_display(
        self,
    ) -> Tuple[bool, Optional[List[Dict[str, Any]]]]:
        """
        Main detection loop with visualization.

        Returns:
            Tuple of (success, detection_results)
            detection_results is a list of dictionaries, each containing:
            - id: Marker ID
            - center: (x, y) center coordinates
            - delta: Dictionary with delta information
        """
        if self.cap is None:
            return False, None

        ret, frame = self.cap.read()
        if not ret:
            return False, None

        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        if self.detector is None:
            raise RuntimeError("Detector not initialized")
        # Type ignore: mypy control flow analysis doesn't understand the None check
        corners, ids, rejected = self.detector.detectMarkers(gray)  # type: ignore[unreachable]

        # Debug: Log detection status
        if ids is None or len(ids) == 0:
            if rejected is not None and len(rejected) > 0:
                logger.debug(
                    f"No markers detected, {len(rejected)} candidates rejected "
                    f"(dict: {self.aruco_dict_name})"
                )
            else:
                logger.debug(f"No markers detected (dict: {self.aruco_dict_name})")

        # Draw crosshairs at image center
        self.draw_crosshairs(frame)

        detection_results: List[Dict[str, Any]] = []

        # Process detected markers
        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids):
                marker_id = int(marker_id[0])
                marker_corners = corners[i]

                # Calculate marker center
                marker_center = self.calculate_marker_center(marker_corners)

                # Calculate delta from image center
                delta_info = self.calculate_delta(marker_center)

                # Calculate pose (rotation and translation) if calibration available
                pose = self.calculate_pose(marker_corners)

                # Store results
                result = {
                    "id": marker_id,
                    "center": marker_center,
                    "delta": delta_info,
                }
                if pose:
                    result["pose"] = pose
                detection_results.append(result)

                # Draw detection info on frame (pass pose for skew display)
                self.draw_detection_info(
                    frame, marker_corners, marker_id, marker_center, delta_info, pose if pose else None
                )

                # Draw pose axes if available
                if pose:
                    self.draw_pose_axes(frame, marker_corners, pose)
        else:
            # No markers detected - show helpful message
            cv2.putText(
                frame,
                "No ArUco markers detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )
            # Show dictionary being used
            cv2.putText(
                frame,
                f"Dict: {self.aruco_dict_name}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (128, 128, 128),
                1,
            )

        # Display frame if GUI is available
        if self.display_enabled:
            try:
                cv2.imshow("ArUco Detection - Press Q to quit", frame)
                # If auto-detect and first successful display, silently continue
                if self._auto_detect_gui:
                    self._auto_detect_gui = False
            except cv2.error as e:
                # GUI not available - disable display and continue
                error_msg = str(e).lower()
                if "not implemented" in error_msg or "gtk" in error_msg or "cocoa" in error_msg:
                    if self._auto_detect_gui:
                        # Auto-detection: GUI not available, switch to headless
                        logger.warning("GUI not available - switching to headless mode")
                        self.display_enabled = False
                        self._auto_detect_gui = False
                    else:
                        # User explicitly requested display but it failed
                        logger.warning("GUI not available - switching to headless mode")
                        self.display_enabled = False
                else:
                    # Other errors - log but don't disable (might be transient)
                    logger.debug(f"Display error (non-fatal): {e}")

        return True, detection_results if detection_results else None

    def run(self) -> None:
        """Run the detection system."""
        try:
            self.initialize_camera()

            logger.info("\n=== ArUco Detection Started ===")
            if self.display_enabled:
                logger.info("Press 'Q' to quit\n")
            else:
                logger.info("Running in headless mode (no display)")
                logger.info("Press Ctrl+C to quit\n")

            while True:
                success, results = self.detect_and_display()

                if not success:
                    logger.warning("Failed to read frame from camera")
                    break

                # Print detection results to console
                if results:
                    for result in results:
                        msg = (
                            f"Marker {result['id']}: "
                            f"Delta=({result['delta']['delta_x']:+4.0f}, "
                            f"{result['delta']['delta_y']:+4.0f})px, "
                            f"Normalized=({result['delta']['norm_delta_x']:+.3f}, "
                            f"{result['delta']['norm_delta_y']:+.3f})"
                        )
                        if "pose" in result:
                            pose = result["pose"]
                            euler = pose["euler_angles"]
                            pos = pose["position"]
                            msg += (
                                f" | Dist: {pose['distance']:.3f}m, "
                                f"Pos: ({pos['x']:+.3f}, {pos['y']:+.3f}, {pos['z']:+.3f})m"
                            )
                            # Display RPY and errors as tuples
                            msg += (
                                f" | RPY: ({euler['roll']:+.2f}, {euler['pitch']:+.2f}, {euler['yaw']:+.2f})°"
                            )
                            if "rpy_errors" in pose:
                                errors = pose["rpy_errors"]
                                msg += (
                                    f" | Error: ({errors['roll']:+.2f}, {errors['pitch']:+.2f}, {errors['yaw']:+.2f})°"
                                )
                        logger.info(msg)

                # Check for quit key (only if display is enabled)
                if self.display_enabled:
                    try:
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break
                    except cv2.error:
                        # GUI became unavailable during runtime
                        self.display_enabled = False
                        logger.warning("GUI became unavailable, continuing in headless mode")

        except KeyboardInterrupt:
            logger.info("\nDetection interrupted by user")
        except Exception as e:
            logger.error(f"Detection failed: {e}", exc_info=True)
            raise
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        """Release camera and close windows."""
        if self.cap is not None:
            self.cap.release()
        if self.display_enabled:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                # GUI not available, ignore
                pass
        logger.info("\n=== ArUco Detection Stopped ===")

    def get_detection_results(
        self,
    ) -> Optional[List[Dict[str, Any]]]:
        """
        Get detection results without display (for programmatic use).

        Returns:
            List of detection dictionaries or None if no frame available
        """
        if self.cap is None:
            return None

        ret, frame = self.cap.read()
        if not ret:
            return None

        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        if self.detector is None:
            raise RuntimeError("Detector not initialized")
        # Type ignore: mypy control flow analysis doesn't understand the None check
        corners, ids, rejected = self.detector.detectMarkers(gray)  # type: ignore[unreachable]

        detection_results: List[Dict[str, Any]] = []

        # Process detected markers
        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids):
                marker_id = int(marker_id[0])
                marker_corners = corners[i]

                # Calculate marker center
                marker_center = self.calculate_marker_center(marker_corners)

                # Calculate delta from image center
                delta_info = self.calculate_delta(marker_center)

                # Calculate pose (rotation and translation) if calibration available
                pose = self.calculate_pose(marker_corners)

                # Store results
                result = {
                    "id": marker_id,
                    "center": marker_center,
                    "delta": delta_info,
                }
                if pose:
                    result["pose"] = pose
                detection_results.append(result)

        return detection_results if detection_results else None


def main() -> int:
    """Main entry point for command-line usage."""
    import argparse

    parser = argparse.ArgumentParser(
        description="ArUco Detection System for Camera Servoing",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage with default camera
  python -m src.aruco.servoing

  # Custom camera and resolution
  python -m src.aruco.servoing --camera 1 --resolution 1280 720

  # Different ArUco dictionary (common for 190mm tags)
  python -m src.aruco.servoing --dict DICT_4X4_50
  
  # With debug logging to troubleshoot detection
  python -m src.aruco.servoing --dict DICT_4X4_50 --debug
  
  # With pose estimation (requires calibration file)
  python -m src.aruco.servoing --calibration calibrations/camera.json --marker-size 0.19
        """,
    )

    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="USB camera device ID (default: 0)",
    )
    parser.add_argument(
        "--resolution",
        nargs=2,
        type=int,
        metavar=("WIDTH", "HEIGHT"),
        default=[640, 480],
        help="Camera resolution (default: 640 480)",
    )
    parser.add_argument(
        "--dict",
        default="DICT_6X6_250",
        help="ArUco dictionary name (default: DICT_6X6_250). Common: DICT_6X6_250, DICT_4X4_50, DICT_ARUCO_ORIGINAL",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Target frames per second (default: 30)",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Run without display (useful for headless operation)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging for detection troubleshooting",
    )
    parser.add_argument(
        "--marker-size",
        type=float,
        default=0.19,
        help="Physical size of ArUco marker in meters (default: 0.19 for 190mm)",
    )
    parser.add_argument(
        "--calibration",
        "-c",
        default="calibrations/arm_base.json",
        help="Path to camera calibration JSON file (for pose estimation, default: calibrations/arm_base.json)",
    )
    parser.add_argument(
        "--target-rpy",
        nargs=3,
        type=float,
        metavar=("ROLL", "PITCH", "YAW"),
        default=[0.0, 0.0, 0.0],
        help="Target roll, pitch, yaw in degrees (default: 0 0 0 for perpendicular)",
    )

    args = parser.parse_args()
    
    # Set logging level based on debug flag
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    try:
        # Create detector
        detector = ArucoServoingDetector(
            camera_id=args.camera,
            resolution=(args.resolution[0], args.resolution[1]),
            aruco_dict=args.dict,
            fps=args.fps,
            display=not args.no_display,
            marker_size_m=args.marker_size,
            calibration_file=args.calibration,
            target_rpy=(args.target_rpy[0], args.target_rpy[1], args.target_rpy[2]),
        )

        # Run the detection system
        detector.run()

        return 0

    except Exception as e:
        logger.error(f"Failed to run detector: {e}", exc_info=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
