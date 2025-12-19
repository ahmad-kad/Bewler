"""
Camera validation and system check utilities.
"""

from typing import Dict, Any

from ..utils.camera import validate_camera_connection
from ..utils.system import validate_system_requirements


def validate_camera_setup(camera_index: int = 0) -> Dict[str, Any]:
    """
    Validate complete camera setup including hardware and software.

    Args:
        camera_index: Camera device index to test

    Returns:
        Dictionary with validation results
    """
    results: Dict[str, Any] = {
        "camera_connection": False,
        "system_requirements": {},
        "overall_status": False,
    }

    # Check camera connection
    results["camera_connection"] = validate_camera_connection(camera_index)

    # Check system requirements
    results["system_requirements"] = validate_system_requirements()

    # Overall status - camera works and basic requirements met
    camera_ok = results["camera_connection"]
    opencv_ok = results["system_requirements"].get("opencv", False)
    camera_devices = results["system_requirements"].get("camera_devices", False)

    results["overall_status"] = camera_ok and opencv_ok and camera_devices

    return results


def check_camera_connection(camera_index: int = 0) -> bool:
    """
    Check if a camera is connected and accessible.

    Args:
        camera_index: Camera device index

    Returns:
        True if camera is accessible
    """
    return validate_camera_connection(camera_index)
