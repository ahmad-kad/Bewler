"""
Camera utilities for sensor detection and camera setup.
"""

import glob
import os
import subprocess
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# Optional OpenCV support
try:
    import cv2

    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None  # type: ignore
    OPENCV_AVAILABLE = False


class RPiCameraCapture:
    """Capture class that mimics cv2.VideoCapture using rpicam-vid pipe.
    
    Reliable way to get frames on Pi 5 when standard V4L2 backend fails.
    """
    def __init__(self, camera_index: int = 0, width: int = 1280, height: int = 720, fps: int = 30):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_size = width * height * 3 // 2  # YUV420
        self.proc = None
        
        # Check for rpicam-vid or libcamera-vid
        self.cmd_name = "rpicam-vid"
        try:
            subprocess.run(["rpicam-vid", "--version"], capture_output=True)
        except FileNotFoundError:
            self.cmd_name = "libcamera-vid"

        self._start_capture()

    def _start_capture(self):
        cmd = [
            self.cmd_name,
            "-t", "0",  # Continuous
            "--camera", str(self.camera_index),
            "--width", str(self.width),
            "--height", str(self.height),
            "--framerate", str(self.fps),
            "--nopreview",
            "--codec", "yuv420",
            "-o", "-"  # Pipe to stdout
        ]
        self.proc = subprocess.Popen(
            cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.DEVNULL,
            bufsize=self.frame_size * 2
        )

    def isOpened(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        if not self.isOpened():
            return False, None
        
        try:
            raw = self.proc.stdout.read(self.frame_size)
            if len(raw) != self.frame_size:
                return False, None
            
            yuv = np.frombuffer(raw, dtype=np.uint8).reshape((self.height * 3 // 2, self.width))
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            return True, bgr
        except Exception:
            return False, None

    def release(self):
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None

    def get(self, prop_id):
        if prop_id == cv2.CAP_PROP_FRAME_WIDTH:
            return self.width
        if prop_id == cv2.CAP_PROP_FRAME_HEIGHT:
            return self.height
        if prop_id == cv2.CAP_PROP_FPS:
            return self.fps
        return 0

    def set(self, prop_id, value):
        # Limited support for setting props after start
        return False


def find_video_devices() -> List[str]:
    """Find all available video capture devices on the system.
    
    Filters out encoders, decoders, and other non-capture devices by attempting
    to open them with OpenCV and checking for valid camera properties.
    
    Returns:
        List of device paths for actual cameras (e.g., ['/dev/video0', '/dev/video1', ...])
    """
    import json, time as time_module
    
    if not OPENCV_AVAILABLE:
        return []
    
    video_devices = sorted(glob.glob("/dev/video*"))
    # Filter to only numeric devices (exclude video4linux, etc.)
    numeric_devices = [
        d for d in video_devices 
        if d.replace("/dev/video", "").isdigit()
    ]
    
    # #region agent log - Device finding start
    log_entry = {
        "timestamp": int(time_module.time() * 1000),
        "location": "camera.py:find_video_devices_start",
        "message": "Starting device finding",
        "data": {"numeric_devices": numeric_devices},
        "sessionId": "debug-session",
        "runId": "device_debug",
        "hypothesisId": "device_filtering"
    }
    try:
        with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
            f.write(json.dumps(log_entry) + "\n")
    except: pass
    # #endregion

    # Filter to only devices that can actually capture video
    capture_devices = []
    for device in numeric_devices:
        try:
            # Use integer index for standard V4L2 devices to avoid OpenCV warnings
            idx = int(device.replace("/dev/video", ""))
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            
            is_opened = cap.isOpened()
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH) if is_opened else -1
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) if is_opened else -1
            
            # #region agent log - Device check
            log_entry = {
                "timestamp": int(time_module.time() * 1000),
                "location": "camera.py:device_check",
                "message": f"Checking device {device}",
                "data": {"device": device, "is_opened": is_opened, "width": width, "height": height},
                "sessionId": "debug-session",
                "runId": "device_debug",
                "hypothesisId": "device_filtering"
            }
            try:
                with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                    f.write(json.dumps(log_entry) + "\n")
            except: pass
            # #endregion

            if is_opened:
                # Real cameras should have valid dimensions
                if width > 0 and height > 0:
                    capture_devices.append(device)
                cap.release()
        except Exception as e:
            # #region agent log - Device check error
            log_entry = {
                "timestamp": int(time_module.time() * 1000),
                "location": "camera.py:device_check_error",
                "message": f"Error checking device {device}",
                "data": {"device": device, "error": str(e)},
                "sessionId": "debug-session",
                "runId": "device_debug",
                "hypothesisId": "device_filtering"
            }
            try:
                with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                    f.write(json.dumps(log_entry) + "\n")
            except: pass
            # #endregion
            pass
    
    # #region agent log - Capture devices found
    log_entry = {
        "timestamp": int(time_module.time() * 1000),
        "location": "camera.py:capture_devices_found",
        "message": "Found capture devices",
        "data": {"capture_devices": capture_devices},
        "sessionId": "debug-session",
        "runId": "device_debug",
        "hypothesisId": "device_filtering"
    }
    try:
        with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
            f.write(json.dumps(log_entry) + "\n")
    except: pass
    # #endregion

    return capture_devices


def get_camera_device_path(camera_index: int) -> Optional[str]:
    """Get the actual device path for a camera index.
    
    Maps logical camera indices (0, 1, 2...) to actual device paths
    (/dev/video19, /dev/video20, etc.) based on available devices.
    
    Args:
        camera_index: Logical camera index (0-based)
        
    Returns:
        Device path (e.g., '/dev/video19') or None if not found
    """
    devices = find_video_devices()
    if camera_index < len(devices):
        return devices[camera_index]
    return None


def detect_camera_sensor() -> Optional[str]:
    """Detect the camera sensor model (IMX219, IMX477, etc.)."""
    import json, time as time_module
    
    # #region agent log - Sensor detection start
    log_entry = {
        "timestamp": int(time_module.time() * 1000),
        "location": "camera.py:detect_camera_sensor_start",
        "message": "Starting sensor detection",
        "data": {},
        "sessionId": "debug-session",
        "runId": "sensor_debug",
        "hypothesisId": "sensor_detection"
    }
    try:
        with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
            f.write(json.dumps(log_entry) + "\n")
    except: pass
    # #endregion

    try:
        # Method 1: rpicam-still (newer) or libcamera-still (older) - most reliable for Pi cameras
        # Try rpicam-still first (newer Raspberry Pi OS)
        commands = ["rpicam-still", "libcamera-still"]
        result = None
        for cmd in commands:
            try:
                result = subprocess.run(
                    [cmd, "--list-cameras"],
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                
                # #region agent log - Command result
                log_entry = {
                    "timestamp": int(time_module.time() * 1000),
                    "location": f"camera.py:cmd_{cmd}",
                    "message": f"Command {cmd} result",
                    "data": {"returncode": result.returncode, "stdout": result.stdout[:200], "stderr": result.stderr[:200]},
                    "sessionId": "debug-session",
                    "runId": "sensor_debug",
                    "hypothesisId": "sensor_detection"
                }
                try:
                    with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                        f.write(json.dumps(log_entry) + "\n")
                except: pass
                # #endregion

                if result and result.returncode == 0:
                    break
            except FileNotFoundError:
                continue

        if result and result.returncode == 0:
            output = result.stdout.lower()

            # Parse sensor models
            sensor_found = None
            if "imx219" in output:
                sensor_found = "imx219"
            elif "imx477" in output:
                sensor_found = "imx477"
            elif "imx708" in output:
                sensor_found = "imx708"
            elif "ov5647" in output:
                sensor_found = "ov5647"
            
            if sensor_found:
                # #region agent log - Sensor found
                log_entry = {
                    "timestamp": int(time_module.time() * 1000),
                    "location": "camera.py:sensor_found",
                    "message": f"Sensor detected via libcamera: {sensor_found}",
                    "data": {"sensor": sensor_found},
                    "sessionId": "debug-session",
                    "runId": "sensor_debug",
                    "hypothesisId": "sensor_detection"
                }
                try:
                    with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                        f.write(json.dumps(log_entry) + "\n")
                except: pass
                # #endregion
                return sensor_found

    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    # Method 2: Fallback to vcgencmd
    try:
        result = subprocess.run(
            ["vcgencmd", "get_camera"], capture_output=True, text=True, timeout=5
        )
        # #region agent log - vcgencmd result
        log_entry = {
            "timestamp": int(time_module.time() * 1000),
            "location": "camera.py:vcgencmd",
            "message": "vcgencmd result",
            "data": {"stdout": result.stdout.strip()},
            "sessionId": "debug-session",
            "runId": "sensor_debug",
            "hypothesisId": "sensor_detection"
        }
        try:
            with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                f.write(json.dumps(log_entry) + "\n")
        except: pass
        # #endregion
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
                
                # #region agent log - OpenCV fallback check
                log_entry = {
                    "timestamp": int(time_module.time() * 1000),
                    "location": "camera.py:opencv_fallback",
                    "message": "OpenCV fallback detection",
                    "data": {"width": width, "height": height},
                    "sessionId": "debug-session",
                    "runId": "sensor_debug",
                    "hypothesisId": "sensor_detection"
                }
                try:
                    with open("/home/malaysia/Bewler/.cursor/debug.log", "a") as f:
                        f.write(json.dumps(log_entry) + "\n")
                except: pass
                # #endregion

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
    """Set up camera capture with standard configuration.
    
    Tries multiple methods:
    1. Standard V4L2 backend
    2. RPi-specific rpicam-vid pipe (for Pi 5 compatibility)
    """
    if not OPENCV_AVAILABLE:
        print("OpenCV not available for camera setup")
        return None

    # Try method 1: Standard V4L2
    # On Pi 5, we prefer mapped device paths if available
    device_path = get_camera_device_path(camera_index)
    source = device_path if device_path else camera_index
    
    print(f"Attempting V4L2 capture on {source}...")
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        
        # TEST if we can actually read a frame
        ret, _ = cap.read()
        if ret:
            print(f"V4L2 capture successful on {source}")
            return cap
        else:
            print(f"V4L2 opened but failed to read from {source}. Trying RPi pipe fallback...")
            cap.release()
    else:
        print(f"V4L2 failed to open {source}. Trying RPi pipe fallback...")

    # Try method 2: RPi-specific pipe capture
    try:
        print(f"Starting RPi pipe capture on camera {camera_index}...")
        cap = RPiCameraCapture(camera_index, width, height, fps)
        if cap.isOpened():
            # Test frame
            ret, _ = cap.read()
            if ret:
                print(f"RPi pipe capture successful on camera {camera_index}")
                return cap
            else:
                print(f"RPi pipe opened but failed to read. Camera might be busy.")
                cap.release()
    except Exception as e:
        print(f"RPi pipe capture failed: {e}")

    return None


def get_camera_resolution(camera_index: int = 0) -> Tuple[int, int]:
    """Get the current camera resolution."""
    if not OPENCV_AVAILABLE:
        return (0, 0)

    # Try to get actual device path, fallback to index
    device_path = get_camera_device_path(camera_index)
    camera_source = device_path if device_path else camera_index
    
    cap = cv2.VideoCapture(camera_source)
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

    # Try to get actual device path, fallback to index
    device_path = get_camera_device_path(camera_index)
    camera_source = device_path if device_path else camera_index
    
    cap = cv2.VideoCapture(camera_source)
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
