#!/usr/bin/env python3
"""
Extrinsic Calibration System - URC 2026
========================================

Supports:
1. HAND-EYE CALIBRATION: Camera-to-arm transformation (eye-on-hand, eye-to-hand)
2. MULTI-CAMERA CALIBRATION: Relative poses between cameras
3. IMU CALIBRATION: Bias, scale factor, and alignment

Used in state machine "CALIBRATION" mode via input streams.

TODO: CONNECT INPUT STREAMS
- TODO: ROS2 topic for robot poses (tf2 transforms)
- TODO: ROS2 service for image capture during hand-eye
- TODO: ROS2 subscriber for IMU raw data
- TODO: Parameter server for camera intrinsics
"""

import cv2
import numpy as np
import json
import yaml
import pickle
from pathlib import Path
from dataclasses import dataclass, asdict, field
from typing import Tuple, List, Optional, Dict
from datetime import datetime
from enum import Enum
import logging

# ============================================================================
# LOGGING
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ============================================================================
# DATA CLASSES
# ============================================================================

@dataclass
class HandEyePose:
    """Robot pose during hand-eye calibration capture"""
    pose_id: int
    robot_base_to_hand: np.ndarray  # 4x4 transformation matrix
    timestamp: str
    confidence: float = 1.0


@dataclass
class HandEyeResult:
    """Hand-eye calibration result"""
    setup_type: str  # "eye_on_hand" or "eye_to_hand"
    hand_eye_transform: np.ndarray  # 4x4 transformation matrix
    reprojection_error: float
    num_poses: int
    timestamp: str
    method: str = "Tsai"
    board_config: Optional[Dict] = None
    camera_intrinsics: Optional[Dict] = None


@dataclass
class MultiCameraCalibration:
    """Multi-camera stereo calibration result"""
    camera1_id: str
    camera2_id: str
    relative_rotation: np.ndarray  # 3x3
    relative_translation: np.ndarray  # 3x1
    baseline_distance_mm: float
    stereo_error: float
    essential_matrix: np.ndarray
    fundamental_matrix: np.ndarray
    timestamp: str


@dataclass
class IMUCalibrationData:
    """IMU calibration parameters"""
    imu_id: str
    # Accelerometer
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel_scale: np.ndarray = field(default_factory=lambda: np.eye(3))
    accel_noise_std: float = 0.0
    
    # Gyroscope
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_scale: np.ndarray = field(default_factory=lambda: np.eye(3))
    gyro_noise_std: float = 0.0
    
    # Alignment (camera-to-IMU)
    camera_to_imu_rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    
    # Metadata
    num_samples: int = 0
    temperature_c: float = 25.0
    timestamp: str = ""


# ============================================================================
# HAND-EYE CALIBRATION
# ============================================================================

class HandEyeCalibrator:
    """Hand-eye calibration for camera-to-arm transformation"""

    def __init__(self, camera_intrinsics: Dict, board_config: Dict):
        """
        Initialize hand-eye calibrator
        
        Args:
            camera_intrinsics: Camera matrix and distortion coefficients
            board_config: ChArUco board configuration
        """
        self.camera_intrinsics = camera_intrinsics
        self.board_config = board_config
        self.poses_collected = []
        self.image_detections = []

    def add_pose_observation(
        self,
        pose_id: int,
        robot_base_to_hand: np.ndarray,
        board_corners: np.ndarray,
        board_ids: np.ndarray,
        timestamp: Optional[str] = None
    ):
        """
        Add a pose observation for hand-eye calibration
        
        Args:
            pose_id: Unique identifier for this pose
            robot_base_to_hand: 4x4 transformation from robot base to end-effector
            board_corners: Detected ChArUco board corners
            board_ids: ChArUco marker IDs
            timestamp: Observation timestamp
        """
        if timestamp is None:
            timestamp = datetime.now().isoformat()

        # Estimate board pose from image
        K = np.array(self.camera_intrinsics['camera_matrix'])
        dist = np.array(self.camera_intrinsics['dist_coeffs'])
        
        # TODO: Board 3D points from config
        board_3d_points = self._get_board_3d_points()
        
        success, rvec, tvec = cv2.solvePnP(
            board_3d_points,
            board_corners,
            K,
            dist
        )

        if not success:
            logger.warning(f"Failed to estimate pose for observation {pose_id}")
            return

        # Store pose
        pose = HandEyePose(
            pose_id=pose_id,
            robot_base_to_hand=robot_base_to_hand,
            timestamp=timestamp
        )
        self.poses_collected.append(pose)
        
        # Store detection
        self.image_detections.append({
            'pose_id': pose_id,
            'rvec': rvec,
            'tvec': tvec,
            'board_corners': board_corners,
            'board_ids': board_ids
        })

        logger.info(f"Added pose observation {pose_id} (total: {len(self.poses_collected)})")

    def calibrate(
        self,
        setup_type: str = "eye_on_hand",
        method: str = cv2.CALIB_HAND_EYE_TSAI
    ) -> HandEyeResult:
        """
        Compute hand-eye transformation
        
        Args:
            setup_type: "eye_on_hand" or "eye_to_hand"
            method: Calibration method (TSAI, PARK, HORAUD, ANDREFF)
        
        Returns:
            HandEyeResult with transformation matrix
        """
        if len(self.poses_collected) < 3:
            raise ValueError("Need at least 3 pose observations")

        logger.info(f"Computing hand-eye calibration ({setup_type}) with {len(self.poses_collected)} poses")

        # Extract transformations
        robot_poses = []
        camera_poses = []

        for i, detection in enumerate(self.image_detections):
            rvec = detection['rvec']
            tvec = detection['tvec']
            
            # Camera to board transformation
            R_camera_to_board, _ = cv2.Rodrigues(rvec)
            t_camera_to_board = tvec.flatten()
            
            T_camera_to_board = self._create_transform(R_camera_to_board, t_camera_to_board)
            camera_poses.append(T_camera_to_board)
            
            # Robot transformation
            pose = self.poses_collected[i]
            robot_poses.append(pose.robot_base_to_hand)

        # Calibrate
        try:
            R_hand_eye, t_hand_eye = cv2.calibrateHandEye(
                [R[:3, :3] for R in robot_poses],
                [t[:3, 3:] for t in robot_poses],
                [R[:3, :3] for R in camera_poses],
                [t[:3, 3:] for t in camera_poses],
                method=method
            )

            # Create 4x4 transformation
            T_hand_eye = self._create_transform(R_hand_eye, t_hand_eye.flatten())
            
            # Calculate reprojection error
            error = self._calculate_reprojection_error(robot_poses, camera_poses, T_hand_eye)

            result = HandEyeResult(
                setup_type=setup_type,
                hand_eye_transform=T_hand_eye,
                reprojection_error=error,
                num_poses=len(self.poses_collected),
                timestamp=datetime.now().isoformat(),
                method=method,
                board_config=self.board_config,
                camera_intrinsics=self.camera_intrinsics
            )

            logger.info(f"Hand-eye calibration complete. Error: {error:.6f}")
            return result

        except Exception as e:
            logger.error(f"Hand-eye calibration failed: {e}")
            raise

    def save_calibration(
        self,
        result: HandEyeResult,
        output_dir: str = "artifacts/extrinsics"
    ) -> Dict[str, str]:
        """Save hand-eye calibration"""
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        base_name = f"hand_eye_{result.setup_type}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        files_saved = {}

        # PKL
        pkl_file = f"{output_dir}/{base_name}.pkl"
        with open(pkl_file, 'wb') as f:
            pickle.dump(result, f)
        files_saved['pkl'] = pkl_file

        # JSON
        json_file = f"{output_dir}/{base_name}.json"
        json_data = {
            'setup_type': result.setup_type,
            'hand_eye_transform': result.hand_eye_transform.tolist(),
            'reprojection_error': float(result.reprojection_error),
            'num_poses': result.num_poses,
            'method': result.method,
            'timestamp': result.timestamp
        }
        with open(json_file, 'w') as f:
            json.dump(json_data, f, indent=2)
        files_saved['json'] = json_file

        logger.info(f"Saved hand-eye calibration: {pkl_file}")
        return files_saved

    @staticmethod
    def _create_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
        """Create 4x4 transformation matrix"""
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.flatten()
        return T

    def _get_board_3d_points(self) -> np.ndarray:
        """TODO: Get 3D board points from configuration"""
        # PLACEHOLDER: Needs board configuration
        # Should return Nx3 array of board corner 3D coordinates
        logger.warning("TODO: Implement _get_board_3d_points with board config")
        return np.zeros((35, 3))  # 5x7 board = 35 corners

    @staticmethod
    def _calculate_reprojection_error(
        robot_poses: List[np.ndarray],
        camera_poses: List[np.ndarray],
        T_hand_eye: np.ndarray
    ) -> float:
        """Calculate hand-eye reprojection error"""
        errors = []
        for R_robot, R_camera in zip(robot_poses, camera_poses):
            error = np.linalg.norm(
                R_robot @ T_hand_eye - R_camera @ R_robot
            )
            errors.append(error)
        return float(np.mean(errors))


# ============================================================================
# MULTI-CAMERA CALIBRATION
# ============================================================================

class MultiCameraCalibrator:
    """Calibrate relative poses between multiple cameras"""

    def __init__(
        self,
        camera1_intrinsics: Dict,
        camera2_intrinsics: Dict,
        board_config: Dict
    ):
        self.camera1_intrinsics = camera1_intrinsics
        self.camera2_intrinsics = camera2_intrinsics
        self.board_config = board_config
        self.pose_pairs = []

    def add_observation(
        self,
        corners1: np.ndarray,
        ids1: np.ndarray,
        corners2: np.ndarray,
        ids2: np.ndarray
    ):
        """Add synchronized observation from both cameras"""
        self.pose_pairs.append({
            'corners1': corners1,
            'ids1': ids1,
            'corners2': corners2,
            'ids2': ids2
        })
        logger.info(f"Added observation pair (total: {len(self.pose_pairs)})")

    def calibrate(self) -> MultiCameraCalibration:
        """Calibrate stereo configuration"""
        if len(self.pose_pairs) < 3:
            raise ValueError("Need at least 3 observation pairs")

        logger.info(f"Calibrating stereo setup with {len(self.pose_pairs)} pairs")

        # TODO: Implement stereo calibration with image points from both cameras
        # PLACEHOLDER RESULT
        result = MultiCameraCalibration(
            camera1_id="cam1",
            camera2_id="cam2",
            relative_rotation=np.eye(3),
            relative_translation=np.array([[0.1], [0.0], [0.0]]),
            baseline_distance_mm=100.0,
            stereo_error=0.5,
            essential_matrix=np.eye(3),
            fundamental_matrix=np.eye(3),
            timestamp=datetime.now().isoformat()
        )

        return result

    def save_calibration(
        self,
        result: MultiCameraCalibration,
        output_dir: str = "artifacts/extrinsics"
    ) -> Dict[str, str]:
        """Save multi-camera calibration"""
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        base_name = f"multicam_{result.camera1_id}_to_{result.camera2_id}"
        files_saved = {}

        # JSON
        json_file = f"{output_dir}/{base_name}.json"
        json_data = {
            'camera1_id': result.camera1_id,
            'camera2_id': result.camera2_id,
            'baseline_mm': float(result.baseline_distance_mm),
            'stereo_error': float(result.stereo_error),
            'relative_rotation': result.relative_rotation.tolist(),
            'relative_translation': result.relative_translation.tolist(),
            'timestamp': result.timestamp
        }
        with open(json_file, 'w') as f:
            json.dump(json_data, f, indent=2)
        files_saved['json'] = json_file

        logger.info(f"Saved multi-camera calibration: {json_file}")
        return files_saved


# ============================================================================
# IMU CALIBRATION
# ============================================================================

class IMUCalibrator:
    """Calibrate IMU biases, scales, and alignment"""

    def __init__(self, imu_id: str = "imu_0"):
        self.imu_id = imu_id
        self.accel_samples = []
        self.gyro_samples = []
        self.calibration = IMUCalibrationData(imu_id=imu_id)

    def add_accel_sample(
        self,
        x: float,
        y: float,
        z: float,
        gravity_magnitude: float = 9.81
    ):
        """
        Add accelerometer sample during static pose
        
        Note: Perform calibration by placing IMU in 6 orientations (±X, ±Y, ±Z)
        """
        self.accel_samples.append(np.array([x, y, z]))

    def add_gyro_sample(self, x: float, y: float, z: float):
        """Add gyroscope sample during rest (zero rotation)"""
        self.gyro_samples.append(np.array([x, y, z]))

    def calibrate_accelerometer(self):
        """
        Calibrate accelerometer bias and scale
        
        TODO: Implement 6-position calibration method
        Requires placing IMU in 6 static orientations and recording averages
        """
        logger.warning("TODO: Implement 6-position accelerometer calibration")

        if len(self.accel_samples) < 6:
            logger.warning(f"Insufficient accel samples: {len(self.accel_samples)} (need 6+)")
            return

        # PLACEHOLDER: Simple bias calculation
        accel_data = np.array(self.accel_samples)
        bias = np.mean(accel_data, axis=0)
        self.calibration.accel_bias = bias - np.array([0, 0, 9.81])

        logger.info(f"Accelerometer bias: {self.calibration.accel_bias}")

    def calibrate_gyroscope(self):
        """
        Calibrate gyroscope bias
        
        TODO: Implement temperature-compensated gyro bias calculation
        """
        logger.warning("TODO: Implement temperature-compensated gyro calibration")

        if len(self.gyro_samples) < 100:
            logger.warning(f"Insufficient gyro samples: {len(self.gyro_samples)} (need 100+)")
            return

        # PLACEHOLDER: Simple bias calculation
        gyro_data = np.array(self.gyro_samples)
        self.calibration.gyro_bias = np.mean(gyro_data, axis=0)
        self.calibration.gyro_noise_std = float(np.std(gyro_data))

        logger.info(f"Gyroscope bias: {self.calibration.gyro_bias}")
        logger.info(f"Gyroscope noise std: {self.calibration.gyro_noise_std}")

    def set_camera_to_imu_alignment(
        self,
        rotation_matrix: np.ndarray
    ):
        """
        Set camera-to-IMU rotation alignment
        
        TODO: Auto-detect from synchronized vision and IMU rotation
        """
        self.calibration.camera_to_imu_rotation = rotation_matrix
        logger.info("Camera-to-IMU alignment set")

    def save_calibration(
        self,
        output_dir: str = "artifacts/extrinsics"
    ) -> str:
        """Save IMU calibration"""
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        self.calibration.timestamp = datetime.now().isoformat()
        self.calibration.num_samples = len(self.accel_samples) + len(self.gyro_samples)

        # JSON
        json_file = f"{output_dir}/imu_{self.imu_id}_calibration.json"
        json_data = {
            'imu_id': self.calibration.imu_id,
            'accelerometer': {
                'bias': self.calibration.accel_bias.tolist(),
                'scale': self.calibration.accel_scale.tolist(),
                'noise_std': float(self.calibration.accel_noise_std)
            },
            'gyroscope': {
                'bias': self.calibration.gyro_bias.tolist(),
                'scale': self.calibration.gyro_scale.tolist(),
                'noise_std': float(self.calibration.gyro_noise_std)
            },
            'camera_to_imu_rotation': self.calibration.camera_to_imu_rotation.tolist(),
            'num_samples': self.calibration.num_samples,
            'temperature_c': self.calibration.temperature_c,
            'timestamp': self.calibration.timestamp
        }
        with open(json_file, 'w') as f:
            json.dump(json_data, f, indent=2)

        logger.info(f"Saved IMU calibration: {json_file}")
        return json_file


# ============================================================================
# TODO: ROS2 INTEGRATION PLACEHOLDERS
# ============================================================================

"""
TODO: Implement ROS2 integration for state machine calibration mode

1. HAND-EYE CALIBRATION INPUT STREAM:
   - Subscribe to /robot/state/end_effector_pose (tf2 TransformStamped)
   - Subscribe to /camera/detections (custom msg with board corners)
   - Service: /calibration/capture_hand_eye_pose (request + store observation)

2. MULTI-CAMERA INPUT STREAM:
   - Subscribe to /camera1/detections (board detection)
   - Subscribe to /camera2/detections (board detection)
   - Auto-sync observations by timestamp
   - Service: /calibration/add_stereo_pair

3. IMU CALIBRATION INPUT STREAM:
   - Subscribe to /imu/data_raw (sensor_msgs/Imu)
   - Service: /calibration/start_imu_static_calibration
   - Service: /calibration/finish_imu_calibration

4. STATE MACHINE INTEGRATION:
   - On enter "CALIBRATION" state:
     ├─ Initialize calibrator based on parameter
     └─ Subscribe to relevant topics
   - On exit "CALIBRATION" state:
     └─ Save calibration artifacts
   - Parameter: /calibration/mode = "hand_eye" | "stereo" | "imu"
"""


if __name__ == "__main__":
    # Example: Hand-eye calibration setup
    camera_intrinsics = {
        'camera_matrix': np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]]),
        'dist_coeffs': np.array([0.1, -0.1, 0, 0, 0])
    }

    board_config = {
        'size': (5, 7),
        'checker_size_mm': 30.0,
        'marker_size_mm': 18.0
    }

    calibrator = HandEyeCalibrator(camera_intrinsics, board_config)
    print("Hand-eye calibrator ready for pose observations")

