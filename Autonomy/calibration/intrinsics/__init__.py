"""
Camera Intrinsics Calibration Module

Provides tools for computing per-camera intrinsic parameters (focal length,
principal point, distortion coefficients) using ChArUco boards.

Supports three capture modes:
- MANUAL: Image-by-image with full control (best for precision)
- VIDEO: Continuous recording with automatic extraction (best for speed)
- CONSERVATIVE: Video + quality filtering (best for production)
"""

from .camera_intrinsics_calibrator import (
    CameraConfig,
    CharUcoBoardConfig,
    CameraIntrinsicsCalibrator,
    CalibrationResult,
    CaptureMode
)

__all__ = [
    'CameraConfig',
    'CharUcoBoardConfig',
    'CameraIntrinsicsCalibrator',
    'CalibrationResult',
    'CaptureMode'
]
