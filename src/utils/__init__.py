"""
Shared utilities for the calibration system.
"""

from .camera import (
    detect_camera_sensor,
    setup_camera_capture,
    get_camera_resolution,
    generate_sensor_based_camera_id,
    save_calibration_file,
    load_calibration_file,
    list_calibrated_cameras,
    rename_calibration_file,
    validate_camera_connection,
)
from .file_ops import (
    ensure_directory,
    safe_file_operation,
    safe_write_json,
    safe_read_json,
)
from .system import (
    run_system_command,
    check_command_available,
    get_system_info,
    get_camera_devices,
    get_usb_devices,
    validate_system_requirements,
)

__all__ = [
    # Camera operations
    "detect_camera_sensor",
    "setup_camera_capture",
    "get_camera_resolution",
    "generate_sensor_based_camera_id",
    "save_calibration_file",
    "load_calibration_file",
    "list_calibrated_cameras",
    "rename_calibration_file",
    "validate_camera_connection",
    # File operations
    "ensure_directory",
    "safe_file_operation",
    "safe_write_json",
    "safe_read_json",
    # System operations
    "run_system_command",
    "check_command_available",
    "get_system_info",
    "get_camera_devices",
    "get_usb_devices",
    "validate_system_requirements",
]
