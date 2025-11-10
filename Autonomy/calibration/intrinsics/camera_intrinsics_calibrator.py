#!/usr/bin/env python3
"""
Camera Intrinsics Calibration System - URC 2026
================================================

Supports three capture modes:
1. MANUAL: Image-by-image with full control
2. VIDEO: Continuous extraction with uniform sampling
3. CONSERVATIVE: Video + quality thresholds for guaranteed quality

Features:
- Multiple cameras support
- CharUco board detection
- Quality metrics and validation
- Automatic parameter export (JSON, YAML, PKL)
- Integration with state machine calibration mode
"""

import cv2
import numpy as np
import json
import yaml
import pickle
import os
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Tuple, List, Optional, Dict
from datetime import datetime
from enum import Enum
import logging

# ============================================================================
# LOGGING & CONFIGURATION
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class CaptureMode(Enum):
    """Capture mode enumeration"""
    MANUAL = "manual"
    VIDEO = "video"
    CONSERVATIVE = "conservative"


@dataclass
class CameraConfig:
    """Camera specification"""
    name: str
    camera_index: int
    resolution: Tuple[int, int]
    fps: int = 30
    sensor_type: str = "rgb"
    focal_length_estimate_mm: float = 4.0


@dataclass
class CharUcoBoardConfig:
    """ChArUco board specification"""
    name: str
    aruco_dict_name: str
    size: Tuple[int, int]  # (cols, rows)
    checker_size_mm: float
    marker_size_mm: float

    @property
    def aruco_dictionary(self) -> cv2.aruco.Dictionary:
        """Convert string dictionary name to OpenCV dictionary object"""
        dict_map = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        }

        if self.aruco_dict_name not in dict_map:
            raise ValueError(f"Unknown ArUco dictionary: {self.aruco_dict_name}")

        return cv2.aruco.getPredefinedDictionary(dict_map[self.aruco_dict_name])


@dataclass
class CalibrationResult:
    """Complete calibration result with metadata"""
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    rvecs: List[np.ndarray]
    tvecs: List[np.ndarray]
    reprojection_error: float
    camera_config: CameraConfig
    board_config: CharUcoBoardConfig
    num_images: int
    capture_mode: str
    timestamp: str
    images_used: List[str]
    quality_metrics: Dict


class CameraIntrinsicsCalibrator:
    """Main calibrator for camera intrinsic parameters"""

    def __init__(self, camera_config: CameraConfig, board_config: CharUcoBoardConfig):
        self.camera_config = camera_config
        self.board_config = board_config
        self.detector = self._setup_detector()
        self.board = self._create_board()

    def _setup_detector(self) -> cv2.aruco.ArucoDetector:
        """Setup ArUco detector"""
        return cv2.aruco.ArucoDetector(self.board_config.aruco_dictionary, cv2.aruco.DetectorParameters())

    def _create_board(self) -> cv2.aruco.CharucoBoard:
        """Create CharUco board"""
        return cv2.aruco.CharucoBoard(
            size=self.board_config.size,
            squareLength=self.board_config.checker_size_mm / 1000.0,
            markerLength=self.board_config.marker_size_mm / 1000.0,
            dictionary=self.board_config.aruco_dictionary
        )

    def capture_manual(
        self,
        target_images: int = 50,
        output_dir: str = None
    ) -> List[str]:
        """Manual image-by-image capture with user control"""
        if output_dir is None:
            output_dir = self._get_dataset_dir("manual")

        Path(output_dir).mkdir(parents=True, exist_ok=True)
        logger.info(f"Manual capture started. Target: {target_images} images")

        cap = cv2.VideoCapture(self.camera_config.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_config.resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_config.resolution[1])

        captured_images = []
        skipped = 0

        print(f"\n{'='*70}")
        print(f"MANUAL INTRINSIC CAPTURE: {self.camera_config.name}")
        print(f"{'='*70}")
        print(f"Target: {target_images} images")
        print(f"Board: {self.board_config.name}")
        print(f"\nControls:")
        print(f"  SPACE: Capture frame (when 4+ markers visible)")
        print(f"  s:     Skip frame")
        print(f"  q:     Quit")
        print(f"\nStrategy:")
        print(f"  • Vary distance (20cm, 60cm, 150cm)")
        print(f"  • Vary angle (±15°, ±30°)")
        print(f"  • Wait for sharp image\n")

        while len(captured_images) < target_images:
            ret, frame = cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            display_frame = frame.copy()
            if ids is not None:
                display_frame = cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
                marker_count = len(ids)
            else:
                marker_count = 0

            # Display info
            info = f"Manual: {len(captured_images)}/{target_images} | Markers: {marker_count} | Skipped: {skipped}"
            cv2.putText(display_frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if marker_count >= 4:
                cv2.putText(display_frame, "READY (SPACE)", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, "Waiting for markers...", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

            cv2.imshow('Manual Intrinsic Capture', display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' ') and marker_count >= 4:
                filename = f"{output_dir}/frame_{len(captured_images):04d}.png"
                cv2.imwrite(filename, frame)
                captured_images.append(filename)
                logger.info(f"Captured {len(captured_images)}/{target_images}")
            elif key == ord('s'):
                skipped += 1
            elif key == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

        logger.info(f"Manual capture complete: {len(captured_images)}/{target_images}")
        return captured_images

    def capture_video(
        self,
        target_images: int = 50,
        sampling_rate: int = 5,
        output_dir: str = None
    ) -> List[str]:
        """Video mode: continuous recording with uniform extraction"""
        if output_dir is None:
            output_dir = self._get_dataset_dir("video")

        Path(output_dir).mkdir(parents=True, exist_ok=True)
        logger.info(f"Video capture started. Target: {target_images}, sampling: every {sampling_rate}th frame")

        cap = cv2.VideoCapture(self.camera_config.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_config.resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_config.resolution[1])

        # Wait for recording start
        print(f"\n{'='*70}")
        print(f"VIDEO INTRINSIC CAPTURE: {self.camera_config.name}")
        print(f"{'='*70}")
        print(f"Target: {target_images} images")
        print(f"Sampling: every {sampling_rate}th frame")
        print(f"\nPreview live feed. Press 'r' to start recording...")

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            cv2.imshow('Ready? (press r to record)', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):
                break

        cv2.destroyAllWindows()
        print("Recording... Move board through poses. Press 'q' to stop.\n")

        # Record frames
        recorded_frames = []
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            recorded_frames.append(frame.copy())

            # Live display
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            display_frame = frame.copy()
            if ids is not None:
                display_frame = cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)

            info = f"Recording: {len(recorded_frames)} frames"
            cv2.putText(display_frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow('Recording...', display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()
        print(f"✓ Recorded {len(recorded_frames)} frames\n")

        # Extract frames
        print("Extracting frames at regular intervals...")
        captured_images = []
        frame_step = max(1, len(recorded_frames) // target_images)

        for i in range(0, len(recorded_frames), frame_step):
            if len(captured_images) >= target_images:
                break

            frame = recorded_frames[i]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None and len(ids) >= 4:
                filename = f"{output_dir}/frame_{len(captured_images):04d}.png"
                cv2.imwrite(filename, frame)
                captured_images.append(filename)

        cap.release()
        logger.info(f"Video capture complete: {len(captured_images)}/{target_images} extracted")
        return captured_images

    def capture_conservative(
        self,
        target_images: int = 50,
        min_markers: int = 4,
        max_blur_ratio: float = 0.3,
        min_detection_rate: float = 0.7,
        output_dir: str = None
    ) -> List[str]:
        """Conservative mode: video + quality thresholds"""
        if output_dir is None:
            output_dir = self._get_dataset_dir("conservative")

        Path(output_dir).mkdir(parents=True, exist_ok=True)
        logger.info(f"Conservative capture: min_markers={min_markers}, max_blur={max_blur_ratio}, min_detect={min_detection_rate}")

        cap = cv2.VideoCapture(self.camera_config.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_config.resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_config.resolution[1])

        # Preview
        print(f"\n{'='*70}")
        print(f"CONSERVATIVE INTRINSIC CAPTURE: {self.camera_config.name}")
        print(f"{'='*70}")
        print(f"Quality thresholds:")
        print(f"  Min markers: {min_markers}")
        print(f"  Max blur ratio: {max_blur_ratio}")
        print(f"  Min detection rate: {min_detection_rate:.0%}\n")

        print("Preview. Press 'r' to start recording...")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            cv2.imshow('Ready? (press r)', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):
                break

        cv2.destroyAllWindows()
        print("Recording... Press 'q' to stop.\n")

        # Record and analyze
        recorded_frames = []
        frame_quality_scores = []

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            # Calculate quality
            if ids is None or len(ids) < min_markers:
                quality_score = 0.0
                status = "✗ Too few markers"
            else:
                # Blur detection
                laplacian = cv2.Laplacian(gray, cv2.CV_64F)
                blur_var = laplacian.var()
                blur_score = min(1.0, blur_var / 500.0)
                blur_ratio = 1.0 - blur_score

                if blur_ratio > max_blur_ratio:
                    quality_score = 0.0
                    status = f"✗ Too blurry"
                else:
                    # Detection rate
                    expected_markers = self.board_config.size[0] * self.board_config.size[1]
                    detection_rate = len(ids) / expected_markers

                    if detection_rate < min_detection_rate:
                        quality_score = 0.0
                        status = f"✗ Low detection"
                    else:
                        quality_score = blur_score * detection_rate
                        status = f"✓ Good"

            recorded_frames.append(frame.copy())
            frame_quality_scores.append((quality_score, status))

            # Display
            display_frame = frame.copy()
            if ids is not None:
                display_frame = cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)

            info = f"Recording: {len(recorded_frames)} | Quality: {quality_score:.2f} | {status}"
            cv2.putText(display_frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if quality_score > 0.5 else (0, 165, 255), 2)
            cv2.imshow('Conservative Recording', display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()
        print(f"✓ Recorded {len(recorded_frames)} frames\n")

        # Select best frames
        print("Selecting best frames by quality score...")
        indexed_scores = [(i, score) for i, (score, _) in enumerate(frame_quality_scores)]
        indexed_scores.sort(key=lambda x: x[1], reverse=True)

        # Select top N, spread across time
        selected_indices = sorted([idx for idx, _ in indexed_scores[:target_images * 2]])
        if len(selected_indices) > target_images:
            step = len(selected_indices) // target_images
            selected_indices = selected_indices[::step][:target_images]

        # Save selected
        captured_images = []
        for selected_idx in selected_indices:
            frame = recorded_frames[selected_idx]
            quality_score, _ = frame_quality_scores[selected_idx]
            filename = f"{output_dir}/frame_{len(captured_images):04d}_q{quality_score:.2f}.png"
            cv2.imwrite(filename, frame)
            captured_images.append(filename)

        cap.release()
        logger.info(f"Conservative capture complete: {len(captured_images)}/{target_images}")
        return captured_images

    def detect_charuco_corners(
        self,
        image_path: str
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], bool]:
        """Detect and refine CharUco corners in image"""
        img = cv2.imread(image_path)
        if img is None:
            return None, None, False

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_corners, aruco_ids, _ = self.detector.detectMarkers(gray)

        if aruco_ids is None or len(aruco_ids) < 4:
            return None, None, False

        charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            aruco_corners, aruco_ids, gray, self.board
        )

        if charuco_corners is None or len(charuco_corners) < 4:
            return None, None, False

        # Refine corners
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
        for i in range(len(charuco_corners)):
            cv2.cornerSubPix(gray, charuco_corners[i:i+1], (5, 5), (-1, -1), criteria)

        return charuco_corners, charuco_ids, True

    def process_dataset(
        self,
        image_paths: List[str]
    ) -> Tuple[List[np.ndarray], List[np.ndarray], int]:
        """Extract corners from all images"""
        all_corners = []
        all_ids = []
        valid_count = 0

        for img_path in image_paths:
            corners, ids, success = self.detect_charuco_corners(img_path)
            if success:
                all_corners.append(corners)
                all_ids.append(ids)
                valid_count += 1

        logger.info(f"Processed {len(image_paths)} images: {valid_count} valid")
        return all_corners, all_ids, valid_count

    def calibrate(
        self,
        all_corners: List[np.ndarray],
        all_ids: List[np.ndarray],
        image_paths: List[str],
        capture_mode: str
    ) -> CalibrationResult:
        """Compute intrinsic parameters"""
        if len(all_corners) < 3:
            raise ValueError("Need at least 3 valid images")

        logger.info(f"Calibrating from {len(all_corners)} images")

        # Calibrate
        camera_matrix, dist_coeffs, rvecs, tvecs, std_dev, per_view_errors = \
            cv2.aruco.calibrateCameraCharuco(
                all_corners,
                all_ids,
                self.board,
                self.camera_config.resolution,
                cameraMatrix=None,
                distCoeffs=None,
                flags=cv2.CALIB_FIX_ASPECT_RATIO,
            )

        # Calculate reprojection error
        total_error = 0.0
        for i in range(len(all_corners)):
            board_3d = self.board.getChessboardCorners()
            imgpts, _ = cv2.projectPoints(
                np.float32(board_3d[all_ids[i].flatten()]),
                rvecs[i],
                tvecs[i],
                camera_matrix,
                dist_coeffs
            )
            error = cv2.norm(all_corners[i], imgpts, cv2.NORM_L2) / len(imgpts)
            total_error += error

        mean_error = total_error / len(all_corners)

        # Quality metrics
        fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        fov_x = 2 * np.arctan(self.camera_config.resolution[0] / (2 * fx)) * 180 / np.pi
        fov_y = 2 * np.arctan(self.camera_config.resolution[1] / (2 * fy)) * 180 / np.pi

        quality_metrics = {
            'fx': float(fx),
            'fy': float(fy),
            'cx': float(cx),
            'cy': float(cy),
            'fov_x_deg': float(fov_x),
            'fov_y_deg': float(fov_y),
            'reprojection_error': float(mean_error),
            'num_images': len(all_corners),
            'distortion_model': 'radial-tangential'
        }

        result = CalibrationResult(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            rvecs=rvecs,
            tvecs=tvecs,
            reprojection_error=mean_error,
            camera_config=self.camera_config,
            board_config=self.board_config,
            num_images=len(all_corners),
            capture_mode=capture_mode,
            timestamp=datetime.now().isoformat(),
            images_used=image_paths,
            quality_metrics=quality_metrics
        )

        return result

    def save_calibration(
        self,
        result: CalibrationResult,
        output_dir: str = None
    ) -> Dict[str, str]:
        """Save calibration in multiple formats"""
        if output_dir is None:
            output_dir = self._get_artifacts_dir()

        Path(output_dir).mkdir(parents=True, exist_ok=True)

        base_name = f"{self.camera_config.name.replace(' ', '_')}_intrinsics_{result.capture_mode}"
        files_saved = {}

        # PKL format
        pkl_file = f"{output_dir}/{base_name}.pkl"
        with open(pkl_file, 'wb') as f:
            pickle.dump(result, f)
        files_saved['pkl'] = pkl_file
        logger.info(f"Saved: {pkl_file}")

        # JSON format
        json_file = f"{output_dir}/{base_name}.json"
        json_data = {
            'camera_matrix': result.camera_matrix.tolist(),
            'dist_coeffs': result.dist_coeffs.flatten().tolist(),
            'reprojection_error': result.reprojection_error,
            'camera': asdict(result.camera_config),
            'board': asdict(result.board_config),
            'num_images': result.num_images,
            'capture_mode': result.capture_mode,
            'timestamp': result.timestamp,
            'quality_metrics': result.quality_metrics
        }
        with open(json_file, 'w') as f:
            json.dump(json_data, f, indent=2)
        files_saved['json'] = json_file
        logger.info(f"Saved: {json_file}")

        # YAML format for ROS2
        yaml_file = f"{output_dir}/{base_name}.yaml"
        yaml_data = {
            'camera_name': self.camera_config.name,
            'image_width': self.camera_config.resolution[0],
            'image_height': self.camera_config.resolution[1],
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': result.camera_matrix.flatten().tolist()
            },
            'distortion_coefficients': {
                'rows': 1,
                'cols': len(result.dist_coeffs.flatten()),
                'data': result.dist_coeffs.flatten().tolist()
            },
            'reprojection_error': float(result.reprojection_error),
            'timestamp': result.timestamp
        }
        with open(yaml_file, 'w') as f:
            yaml.dump(yaml_data, f)
        files_saved['yaml'] = yaml_file
        logger.info(f"Saved: {yaml_file}")

        return files_saved

    def print_results(self, result: CalibrationResult):
        """Print calibration results"""
        print(f"\n{'='*70}")
        print(f"INTRINSIC CALIBRATION RESULTS: {self.camera_config.name}")
        print(f"{'='*70}")
        print(f"\nCamera Matrix K:")
        print(f"  fx = {result.quality_metrics['fx']:.2f} px")
        print(f"  fy = {result.quality_metrics['fy']:.2f} px")
        print(f"  cx = {result.quality_metrics['cx']:.2f} px")
        print(f"  cy = {result.quality_metrics['cy']:.2f} px")
        print(f"  FOV: {result.quality_metrics['fov_x_deg']:.1f}° × {result.quality_metrics['fov_y_deg']:.1f}°")

        print(f"\nDistortion Coefficients:")
        labels = ['k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6']
        for i, (label, coeff) in enumerate(zip(labels, result.dist_coeffs.flatten())):
            if i < len(result.dist_coeffs.flatten()):
                print(f"  {label} = {coeff:.6f}")

        error = result.reprojection_error
        print(f"\nReprojection Error: {error:.4f} px", end=" → ")
        if error < 0.5:
            print("Excellent ✓")
        elif error < 1.0:
            print("Good ✓")
        elif error < 2.0:
            print("Acceptable")
        else:
            print("Poor ✗")

        print(f"\nCapture Details:")
        print(f"  Mode: {result.capture_mode}")
        print(f"  Images used: {result.num_images}")
        print(f"  Timestamp: {result.timestamp}")
        print(f"{'='*70}\n")

    def _get_dataset_dir(self, mode: str) -> str:
        """Get dataset directory for capture mode"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        camera_safe = self.camera_config.name.replace(' ', '_').lower()
        return f"calibration_images/intrinsics_{mode}_{camera_safe}_{timestamp}"

    def _get_artifacts_dir(self) -> str:
        """Get artifacts directory"""
        return "artifacts/intrinsics"


if __name__ == "__main__":
    # Example usage
    camera_cfg = CameraConfig(
        name="Test Camera",
        camera_index=0,
        resolution=(1920, 1080)
    )

    board_cfg = CharUcoBoardConfig(
        name="board_5x7",
        aruco_dict_name="DICT_4X4_50",
        size=(5, 7),
        checker_size_mm=30.0,
        marker_size_mm=18.0
    )

    calibrator = CameraIntrinsicsCalibrator(camera_cfg, board_cfg)
    print("Calibrator initialized. Ready for capture.")
