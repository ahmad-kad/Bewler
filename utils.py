"""
Camera utilities for sensor detection and camera setup.
Consolidated from src/utils/camera.py and src/utils/file_ops.py
"""

import os
import subprocess
from typing import Optional, Tuple, Any
from contextlib import contextmanager
from datetime import datetime

# Optional OpenCV support
try:
    import cv2

    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None  # type: ignore
    OPENCV_AVAILABLE = False


# File operations from src/utils/file_ops.py
def ensure_directory(file_path: str):
    """Ensure the directory for a file path exists."""
    directory = os.path.dirname(file_path) if os.path.dirname(file_path) else "."
    os.makedirs(directory, exist_ok=True)


@contextmanager
def safe_file_operation(file_path: str, mode: str = "r", **kwargs):
    """Context manager for safe file operations with error handling."""
    try:
        with open(file_path, mode, **kwargs) as f:
            yield f
    except IOError as e:
        print(f"File operation failed for '{file_path}': {e}")
        raise
    except Exception as e:
        print(f"Unexpected error with file '{file_path}': {e}")
        raise


def safe_read_json(file_path: str, default_value: Any = None) -> Any:
    """Safely read a JSON file with fallback to default value."""
    try:
        with safe_file_operation(file_path, "r") as f:
            import json

            return json.load(f)
    except (IOError, ValueError):
        return default_value


def safe_write_json(file_path: str, data: Any, indent: int = 2):
    """Safely write data to a JSON file."""
    ensure_directory(file_path)
    try:
        with safe_file_operation(file_path, "w") as f:
            import json

            json.dump(data, f, indent=indent)
    except Exception as e:
        print(f"Failed to write JSON to '{file_path}': {e}")
        raise


# Camera utilities from src/utils/camera.py
def detect_camera_sensor() -> Optional[str]:
    """Detect the camera sensor model (IMX219, IMX477, etc.)."""
    try:
        # Method 1: libcamera-still (most reliable for Pi cameras)
        result = subprocess.run(
            ["libcamera-still", "--list-cameras"],
            capture_output=True,
            text=True,
            timeout=10,
        )

        if result.returncode == 0:
            output = result.stdout.lower()

            # Parse sensor models
            if "imx219" in output:
                return "imx219"
            elif "imx477" in output:
                return "imx477"
            elif "imx708" in output:
                return "imx708"
            elif "ov5647" in output:
                return "ov5647"

    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    # Method 2: Fallback to vcgencmd
    try:
        result = subprocess.run(["vcgencmd", "get_camera"], capture_output=True, text=True, timeout=5)

        if result.returncode == 0:
            output = result.stdout.lower()
            if "detected=1" in output:
                # Try to parse sensor from /proc/device-tree
                try:
                    with open("/proc/device-tree/model", "r") as f:
                        model = f.read().strip().lower()

                    if "camera" in model and ("imx219" in model or "v2" in model):
                        return "imx219"
                    elif "camera" in model and ("imx477" in model or "v3" in model):
                        return "imx477"
                except:
                    pass

                return "unknown_raspberry_pi_camera"

    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    return "unknown_sensor"


def get_camera_info() -> dict:
    """Get comprehensive camera information."""
    info: dict = {
        "sensor": detect_camera_sensor(),
        "libcamera_available": False,
        "opencv_available": OPENCV_AVAILABLE,
        "cameras_detected": 0,
        "camera_details": [],
    }

    # Check libcamera
    try:
        import picamera2

        cameras = picamera2.Picamera2.global_camera_info()
        info["libcamera_available"] = True
        info["cameras_detected"] = len(cameras)
        info["camera_details"] = cameras
    except ImportError:
        pass

    return info


def save_calibration_file(calib_data: dict, camera_name: str, output_dir: str = "calibrations") -> str:
    """Save camera calibration data to individual JSON file."""

    # Add timestamp and sensor info if not present
    if "calibrated_at" not in calib_data:
        calib_data["calibrated_at"] = datetime.now().isoformat()

    if "sensor" not in calib_data:
        sensor = detect_camera_sensor()
        if sensor and sensor != "unknown_sensor":
            calib_data["sensor"] = sensor

    # Create filename
    filename = f"{camera_name}.json"
    filepath = os.path.join(output_dir, filename)

    # Save the calibration data
    safe_write_json(filepath, calib_data)

    print(f"Saved calibration for '{camera_name}' to {filepath}")
    return filepath


def load_calibration_file(camera_name: str, calibration_dir: str = "calibrations") -> dict:
    """Load camera calibration data from JSON file."""
    filepath = os.path.join(calibration_dir, f"{camera_name}.json")

    calib_data = safe_read_json(filepath)

    if not calib_data:
        raise FileNotFoundError(f"Calibration file not found: {filepath}")

    return calib_data


def list_calibrated_cameras(calibration_dir: str = "calibrations") -> list:
    """List all calibrated cameras by scanning calibration directory."""
    if not os.path.exists(calibration_dir):
        return []

    cameras = []
    for filename in os.listdir(calibration_dir):
        if filename.endswith(".json"):
            camera_name = filename[:-5]  # Remove .json extension
            cameras.append(camera_name)

    return sorted(cameras)


def rename_calibration_file(old_name: str, new_name: str, calibration_dir: str = "calibrations"):
    """Rename a camera calibration file."""
    old_path = os.path.join(calibration_dir, f"{old_name}.json")
    new_path = os.path.join(calibration_dir, f"{new_name}.json")

    if not os.path.exists(old_path):
        raise FileNotFoundError(f"Calibration file not found: {old_path}")

    if os.path.exists(new_path):
        raise FileExistsError(f"Calibration file already exists: {new_path}")

    os.rename(old_path, new_path)
    print(f"Renamed camera: '{old_name}' -> '{new_name}'")


def validate_calibration_data(calib_data: dict) -> bool:
    """Validate that calibration data contains required fields."""
    required_fields = ["camera_matrix", "distortion_coefficients", "image_width", "image_height"]

    for field in required_fields:
        if field not in calib_data:
            print(f"Missing required field: {field}")
            return False

    # Check camera matrix dimensions
    if len(calib_data["camera_matrix"]) != 3 or len(calib_data["camera_matrix"][0]) != 3:
        print("Invalid camera matrix dimensions")
        return False

    # Check distortion coefficients length
    if len(calib_data["distortion_coefficients"]) != 5:
        print("Invalid distortion coefficients length")
        return False

    return True
