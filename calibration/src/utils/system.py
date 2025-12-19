"""
System utilities for command execution and system interaction.
"""

import subprocess
import shlex
from pathlib import Path
from typing import Tuple, List, Union, Dict, Any


def run_command(cmd: str, description: str) -> Tuple[bool, str]:
    """
    Run a system command and return success status and output.

    Args:
        cmd: Command to run
        description: Description of the command for logging

    Returns:
        Tuple of (success: bool, output: str)
    """
    try:
        result = subprocess.run(
            shlex.split(cmd), capture_output=True, text=True, timeout=30
        )
        success = result.returncode == 0
        output = result.stdout.strip() if success else result.stderr.strip()
        return success, output
    except (subprocess.TimeoutExpired, subprocess.SubprocessError) as e:
        return False, f"Command failed: {str(e)}"


def run_system_command(
    command: Union[str, List[str]], timeout: int = 10, shell: bool = False
) -> Tuple[bool, str, str]:
    """
    Run a system command with standardized error handling.

    Args:
        command: Command to run (string or list)
        timeout: Timeout in seconds
        shell: Whether to run through shell

    Returns:
        Tuple of (success: bool, stdout: str, stderr: str)
    """
    try:
        if isinstance(command, str) and not shell:
            # Parse command string into list
            command = shlex.split(command)

        result = subprocess.run(
            command, shell=shell, capture_output=True, text=True, timeout=timeout
        )

        success = result.returncode == 0
        return success, result.stdout.strip(), result.stderr.strip()

    except subprocess.TimeoutExpired:
        return False, "", f"Command timed out after {timeout} seconds"
    except FileNotFoundError:
        return (
            False,
            "",
            f"Command not found: {command[0] if isinstance(command, list) else command.split()[0]}",
        )
    except Exception as e:
        return False, "", f"Command execution failed: {str(e)}"


def check_command_available(command: str) -> bool:
    """Check if a system command is available."""
    success, _, _ = run_system_command(f"which {command}", timeout=5)
    return success


def check_system_setup() -> Dict[str, Any]:
    """
    Check system setup for camera calibration.

    Returns:
        Dictionary with setup status
    """
    results = {
        "video_devices": False,
        "user_in_video_group": False,
        "v4l2_utils": False,
        "camera_enabled": False,
    }

    # Check for video devices
    try:
        video_devices = list(Path("/dev").glob("video*"))
        results["video_devices"] = len(video_devices) > 0
    except Exception:
        results["video_devices"] = False

    # Check if user is in video group
    success, output = run_command("groups", "check user groups")
    results["user_in_video_group"] = "video" in output if success else False

    # Check v4l2-utils availability
    results["v4l2_utils"] = check_command_available("v4l2-ctl")

    # Check if camera is enabled (Raspberry Pi specific)
    success, output = run_command("vcgencmd get_camera", "check camera status")
    if success and "detected=1" in output:
        results["camera_enabled"] = True

    return results


def list_camera_devices() -> None:
    """
    List available camera devices with detailed information.
    """
    print("🔍 Scanning for camera devices...\n")

    # Check /dev/video* devices
    try:
        video_devices = list(Path("/dev").glob("video*"))
        if video_devices:
            print(f"📹 Found {len(video_devices)} video device(s):")
            for device in sorted(video_devices):
                print(f"   {device}")

                # Try to get device info with v4l2-ctl
                success, output = run_command(
                    f"v4l2-ctl -d {device} --info", f"get info for {device}"
                )
                if success:
                    # Extract driver name
                    for line in output.split("\n"):
                        if "Driver name" in line:
                            driver = line.split(":")[1].strip()
                            print(f"      Driver: {driver}")
                            break
                else:
                    print("      (Could not query device info)")
        else:
            print("❌ No video devices found in /dev/")
    except Exception as e:
        print(f"❌ Error scanning devices: {e}")

    print()


def get_system_info() -> Dict[str, Any]:
    """Get basic system information."""
    info: Dict[str, Any] = {}

    # OS information
    success, stdout, _ = run_system_command("uname -a")
    info["os"] = stdout if success else "Unknown"

    # Python version
    import sys

    info["python_version"] = (
        f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    )

    # Available commands
    commands_to_check = ["libcamera-still", "vcgencmd", "v4l2-ctl"]
    info["commands"] = {}
    for cmd in commands_to_check:
        info["commands"][cmd] = check_command_available(cmd)

    return info


def get_camera_devices() -> List[str]:
    """Get list of available camera device paths."""
    success, stdout, _ = run_system_command("ls /dev/video* 2>/dev/null", shell=True)
    if success:
        return stdout.split()
    return []


def get_usb_devices() -> List[dict]:
    """Get information about USB devices (limited parsing)."""
    devices = []

    success, stdout, _ = run_system_command("lsusb", timeout=5)
    if success:
        for line in stdout.split("\n"):
            if line.strip():
                # Basic parsing: "Bus XXX Device XXX: ID XXXX:XXXX Description"
                parts = line.split(":", 1)
                if len(parts) >= 2:
                    bus_device = parts[0].strip()
                    id_and_desc = parts[1].strip()

                    device_info = {"bus_device": bus_device, "info": id_and_desc}
                    devices.append(device_info)

    return devices


def validate_system_requirements() -> Dict[str, Any]:
    """Validate that system meets calibration requirements."""
    requirements = {
        "python_version": False,
        "opencv": False,
        "v4l2_utils": False,
        "libcamera": False,
        "camera_devices": False,
    }

    # Check Python version
    import sys

    requirements["python_version"] = sys.version_info >= (3, 8)

    # Check OpenCV
    try:
        import cv2

        requirements["opencv"] = True
    except ImportError:
        pass

    # Check system tools
    requirements["v4l2_utils"] = check_command_available("v4l2-ctl")
    requirements["libcamera"] = check_command_available("libcamera-still")

    # Check camera devices
    camera_devices = get_camera_devices()
    requirements["camera_devices"] = len(camera_devices) > 0

    return requirements
