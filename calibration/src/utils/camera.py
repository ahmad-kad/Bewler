"""
Camera utilities for sensor detection and camera setup.
"""

import os
import subprocess
from typing import Optional, Tuple

# Optional OpenCV support
try:
    import cv2

    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None  # type: ignore
    OPENCV_AVAILABLE = False


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
        result = subprocess.run(
            ["vcgencmd", "get_camera"], capture_output=True, text=True, timeout=5
        )
        if "detected=1" in result.stdout:
            return "pi_camera_generic"
    except Exception:
        pass

    # Method 3: Try to infer from resolution (fallback)
    if OPENCV_AVAILABLE:
        try:
            cap = cv2.VideoCapture(0)
            if cap.isOpened():
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                cap.release()

                # Common resolutions for different sensors
                if width == 3280 and height == 2464:  # Pi Camera v2
                    return "imx219"
                elif width == 4056 and height == 3040:  # Pi HQ Camera
                    return "imx477"
                elif width == 4608 and height == 2592:  # Pi Camera v3
                    return "imx708"
                elif width == 2592 and height == 1944:  # Pi Camera v1
                    return "ov5647"
        except Exception:
            pass

    return None


def setup_camera_capture(
    camera_index: int = 0, width: int = 1280, height: int = 720, fps: int = 30
) -> Optional[object]:
    """Set up camera capture with standard configuration."""
    if not OPENCV_AVAILABLE:
        print("OpenCV not available for camera setup")
        return None

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_index}")
        return None

    # Configure camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # Verify settings
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Camera {camera_index} configured: {actual_width}x{actual_height}")
    return cap


def get_camera_resolution(camera_index: int = 0) -> Tuple[int, int]:
    """Get the current camera resolution."""
    if not OPENCV_AVAILABLE:
        return (0, 0)

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        return (0, 0)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()

    return (width, height)


def generate_sensor_based_camera_id(calibration_dir: str = "calibrations") -> str:
    """Generate sequential camera ID based on sensor (imx219_1, imx219_2, etc.)."""

    # Detect current sensor
    sensor = detect_camera_sensor()
    if not sensor:
        sensor = "camera"

    # Get existing camera IDs from calibration directory
    existing_cameras = list_calibrated_cameras(calibration_dir)

    # Find all IDs for this sensor type
    sensor_numbers = []
    for cam_id in existing_cameras:
        if cam_id.startswith(f"{sensor}_"):
            try:
                num = int(cam_id.split("_")[-1])
                sensor_numbers.append(num)
            except (ValueError, IndexError):
                continue

    # Generate next number for this sensor
    next_num = max(sensor_numbers) + 1 if sensor_numbers else 1
    return f"{sensor}_{next_num}"


def validate_camera_connection(camera_index: int = 0) -> bool:
    """Validate that a camera is connected and accessible."""
    if not OPENCV_AVAILABLE:
        return False

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        return False

    # Try to read a frame
    ret, frame = cap.read()
    cap.release()

    return ret and frame is not None and frame.size > 0


def save_calibration_file(
    calib_data: dict, camera_name: str, output_dir: str = "calibrations"
) -> str:
    """Save camera calibration data to individual JSON file."""
    from datetime import datetime
    from .file_ops import safe_write_json

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


def load_calibration_file(
    camera_name: str, calibration_dir: str = "calibrations"
) -> dict:
    """Load camera calibration data from JSON file."""
    from .file_ops import safe_read_json

    filepath = os.path.join(calibration_dir, f"{camera_name}.json")
    calib_data = safe_read_json(filepath, default_value={})

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


def rename_calibration_file(
    old_name: str, new_name: str, calibration_dir: str = "calibrations"
):
    """Rename a camera calibration file."""
    old_path = os.path.join(calibration_dir, f"{old_name}.json")
    new_path = os.path.join(calibration_dir, f"{new_name}.json")

    if not os.path.exists(old_path):
        raise FileNotFoundError(f"Calibration file not found: {old_path}")

    if os.path.exists(new_path):
        raise FileExistsError(f"Calibration file already exists: {new_path}")

    os.rename(old_path, new_path)
    print(f"Renamed camera: '{old_name}' -> '{new_name}'")
