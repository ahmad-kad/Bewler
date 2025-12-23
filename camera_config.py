#!/usr/bin/env python3
"""
Camera Configuration - URC 2026
Shared configuration for camera calibration scripts
"""

# ChArUco board configuration
# User's board: 8 rows x 6 columns = (6, 8) in OpenCV
# Square size: 29mm, Marker size: 19mm
CHARUCO_CONFIG = {
    "board_size": (6, 8),  # (columns, rows) in OpenCV
    "square_size": 0.029,  # 29mm in meters
    "marker_size": 0.019,  # 19mm in meters
    "dictionary": "DICT_4X4_50",
}

# Default calibration settings
CALIBRATION_CONFIG = {
    "duration": 30,  # seconds
    "target_frames": 15,  # frames to select
    "min_frames": 5,  # minimum frames needed
    "quality_threshold": 3,  # minimum markers for partial detection
}
