"""
Extrinsic Calibration Module

Provides tools for computing transformations between cameras and sensors:
- Hand-Eye Calibration: Camera-to-arm transformation
- Multi-Camera Calibration: Relative poses between cameras
- IMU Calibration: Accelerometer and gyroscope biases

Used in state machine "CALIBRATION" mode for full system calibration.
"""

from .hand_eye_imu_calibrator import (
    HandEyeCalibrator,
    HandEyeResult,
    HandEyePose,
    MultiCameraCalibrator,
    MultiCameraCalibration,
    IMUCalibrator,
    IMUCalibrationData
)

__all__ = [
    'HandEyeCalibrator',
    'HandEyeResult',
    'HandEyePose',
    'MultiCameraCalibrator',
    'MultiCameraCalibration',
    'IMUCalibrator',
    'IMUCalibrationData'
]
