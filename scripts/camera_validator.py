#!/usr/bin/env python3
"""
Camera Validator for Raspberry Pi - URC 2026

Validates camera presence, connectivity, and functionality before calibration.
Essential for reliable Pi OS calibration setup.

Usage:
    python camera_validator.py --check
    python camera_validator.py --auto-detect
    python camera_validator.py --validate 0
    python camera_validator.py --setup

Author: URC 2026 Autonomy Team
"""

import argparse
import logging
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import cv2
except ImportError:
    print("  OpenCV not available - some features disabled")
    raise

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)


class CameraValidator:
    """Comprehensive camera validation for Raspberry Pi systems."""

    def __init__(self):
        self.max_cameras = 10  # Test up to 10 camera indices
        self.test_duration = 2  # Seconds to test each camera
        self.required_fps = 15  # Minimum acceptable FPS
        self.required_resolution = (640, 480)  # Minimum resolution

    def check_system_prerequisites(self) -> Dict[str, bool]:
        """Check if Raspberry Pi system is properly configured for cameras."""
        logger.info(" Checking Raspberry Pi camera prerequisites...")

        results = {
            "v4l2_utils": False,
            "opencv": False,
            "video_group": False,
            "camera_devices": False,
            "libcamera": False,
        }

        # Check v4l2-utils (video4linux tools)
        try:
            result = subprocess.run(
                ["v4l2-ctl", "--version"], capture_output=True, text=True, timeout=5
            )
            results["v4l2_utils"] = result.returncode == 0
            if results["v4l2_utils"]:
                logger.info("    v4l2-utils installed")
            else:
                logger.warning("    v4l2-utils not found")
        except (subprocess.TimeoutExpired, FileNotFoundError):
            logger.warning("    v4l2-utils not available")

        # Check OpenCV camera support
        try:
            import cv2

            # Test basic OpenCV camera functionality
            test_cap = cv2.VideoCapture(-1)  # Invalid index should fail gracefully
            test_cap.release()
            results["opencv"] = True
            logger.info("    OpenCV camera support available")
        except Exception as e:
            logger.error(f"    OpenCV camera support failed: {e}")

        # Check if user is in video group
        try:
            result = subprocess.run(
                ["groups"], capture_output=True, text=True, timeout=5
            )
            groups = result.stdout.strip().split()
            results["video_group"] = "video" in groups
            if results["video_group"]:
                logger.info("    User in video group")
            else:
                logger.warning(
                    "    User not in video group (run: sudo usermod -a -G video $USER)"
                )
        except Exception as e:
            logger.warning(f"    Could not check video group: {e}")

        # Check for camera devices
        camera_devices = list(Path("/dev").glob("video*"))
        results["camera_devices"] = len(camera_devices) > 0
        if results["camera_devices"]:
            device_names = [str(d) for d in camera_devices]
            logger.info(
                f"    Found {len(camera_devices)} camera device(s): {device_names}"
            )
        else:
            logger.warning("    No camera devices found in /dev/")

        # Check libcamera (Raspberry Pi camera stack)
        try:
            result = subprocess.run(
                ["libcamera-hello", "--version"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            results["libcamera"] = result.returncode == 0
            if results["libcamera"]:
                logger.info("    libcamera available")
            else:
                logger.info(
                    "   ℹ libcamera not available (may not be needed for USB cameras)"
                )
        except (subprocess.TimeoutExpired, FileNotFoundError):
            logger.info(
                "   ℹ libcamera not available (may not be needed for USB cameras)"
            )

        return results

    def detect_cameras(self) -> List[Dict]:
        """Auto-detect all available cameras on the system."""
        logger.info(" Auto-detecting cameras...")

        detected_cameras = []

        for camera_index in range(self.max_cameras):
            logger.info(f"   Testing camera {camera_index}...")

            camera_info: Dict[str, Any] = {
                "index": camera_index,
                "available": False,
                "backend": None,
                "resolution": None,
                "fps": None,
                "error": None,
            }

            # Try different backends (important for Raspberry Pi)
            backends = [
                ("V4L2", cv2.CAP_V4L2),
                ("ANY", cv2.CAP_ANY),
                ("GSTREAMER", cv2.CAP_GSTREAMER),
            ]

            for backend_name, backend in backends:
                try:
                    cap = cv2.VideoCapture(camera_index, backend)

                    if cap.isOpened():
                        # Test by reading a few frames
                        frames_read = 0
                        start_time = time.time()

                        for _ in range(10):  # Test 10 frames
                            ret, frame = cap.read()
                            if ret:
                                frames_read += 1
                                if frames_read == 1:
                                    # Get camera properties
                                    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                                    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                                    fps = cap.get(cv2.CAP_PROP_FPS)
                                    camera_info["resolution"] = (width, height)
                                    camera_info["fps"] = fps
                            else:
                                break

                        elapsed = time.time() - start_time
                        actual_fps = frames_read / elapsed if elapsed > 0 else 0

                        if frames_read >= 5:  # Require at least 5 successful frames
                            camera_info["available"] = True
                            camera_info["backend"] = backend_name
                            camera_info["fps"] = actual_fps
                            msg = (
                                f"      Camera {camera_index}: {width}x{height} @ "
                                f"{actual_fps:.1f} FPS ({backend_name})"
                            )
                            logger.info(msg)
                            detected_cameras.append(camera_info)
                            break
                        else:
                            cap.release()
                            continue

                    cap.release()

                except Exception as e:
                    camera_info["error"] = str(e)
                    continue

            if not camera_info["available"]:
                logger.info(f"      Camera {camera_index} not available")

        return detected_cameras

    def validate_camera(
        self, camera_index: int, backend: Optional[int] = None
    ) -> Dict[str, Any]:
        """Comprehensive validation of a specific camera."""
        logger.info(f" Validating camera {camera_index}...")

        validation_results: Dict[str, Any] = {
            "index": camera_index,
            "available": False,
            "stable": False,
            "resolution_ok": False,
            "fps_ok": False,
            "backend": None,
            "resolution": None,
            "fps": None,
            "errors": [],
        }

        # Try to open camera
        cap = None
        if backend:
            cap = cv2.VideoCapture(camera_index, backend)
            backend_name = "CUSTOM"
        else:
            # Try backends in order
            backends = [("V4L2", cv2.CAP_V4L2), ("ANY", cv2.CAP_ANY)]
            for backend_name, backend_val in backends:
                cap = cv2.VideoCapture(camera_index, backend_val)
                if cap.isOpened():
                    break
                cap.release()

        if not cap or not cap.isOpened():
            validation_results["errors"].append("Cannot open camera")
            return validation_results

        validation_results["available"] = True
        validation_results["backend"] = backend_name

        # Configure camera
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)

        # Get actual properties
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)

        validation_results["resolution"] = (width, height)
        validation_results["fps"] = fps

        # Check resolution
        validation_results["resolution_ok"] = (
            width >= self.required_resolution[0]
            and height >= self.required_resolution[1]
        )
        if not validation_results["resolution_ok"]:
            validation_results["errors"].append(
                f"Resolution {width}x{height} below minimum {self.required_resolution}"
            )

        # Test stability over time
        logger.info("   Testing camera stability...")
        frames_read = 0
        failed_frames = 0
        start_time = time.time()

        try:
            while time.time() - start_time < self.test_duration:
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    frames_read += 1
                else:
                    failed_frames += 1

            elapsed = time.time() - start_time
            actual_fps = frames_read / elapsed if elapsed > 0 else 0

            # Camera is stable if <10% frame failures and reasonable FPS
            stability_ratio = (
                failed_frames / (frames_read + failed_frames)
                if (frames_read + failed_frames) > 0
                else 1
            )
            validation_results["stable"] = stability_ratio < 0.1 and actual_fps > 5

            validation_results["fps_ok"] = actual_fps >= self.required_fps

            if validation_results["stable"]:
                msg = f"      Camera stable: {actual_fps:.1f} FPS, {failed_frames} failed frames"
                logger.info(msg)
            else:
                msg = f"      Camera unstable: {actual_fps:.1f} FPS, {failed_frames} failed frames"
                logger.warning(msg)
                validation_results["errors"].append(
                    f"Camera unstable: {failed_frames} failed frames"
                )

            if not validation_results["fps_ok"]:
                validation_results["errors"].append(
                    f"FPS {actual_fps:.1f} below minimum {self.required_fps}"
                )

        finally:
            cap.release()

        return validation_results

    def generate_setup_script(self) -> str:
        """Generate a setup script for Raspberry Pi camera configuration."""
        setup_script = """#!/bin/bash
# Raspberry Pi Camera Setup Script - URC 2026
# Run with: sudo bash setup_cameras.sh

echo " Setting up Raspberry Pi for camera calibration..."

# Update system
echo " Updating system packages..."
apt update && apt upgrade -y

# Install camera utilities
echo " Installing camera utilities..."
apt install -y v4l-utils libv4l-dev

# Install Python dependencies for computer vision
echo " Installing Python CV dependencies..."
pip3 install opencv-python numpy

# Configure user permissions for cameras
echo " Configuring camera permissions..."
usermod -a -G video $SUDO_USER

# Enable camera interface (if using Raspberry Pi camera)
echo " Enabling camera interface..."
raspi-config nonint do_camera 1

# Create udev rule for consistent device naming (optional)
echo " Creating udev rule for camera devices..."
cat > /etc/udev/rules.d/99-camera.rules << 'EOF'
# UDEV rule for consistent camera device naming
SUBSYSTEM=="video4linux", ATTR{name}=="*Camera*", SYMLINK+="camera_$attr{idVendor}_$attr{idProduct}"
EOF

# Reload udev rules
udevadm control --reload-rules
udevadm trigger

echo "Setup complete! Please reboot for camera permissions to take effect."
echo " After reboot, run: python3 camera_validator.py --check"
"""

        return setup_script

    def print_validation_report(self, cameras: List[Dict]) -> None:
        """Print a comprehensive validation report."""
        print("\n" + "=" * 60)
        print(" CAMERA VALIDATION REPORT")
        print("=" * 60)

        if not cameras:
            print("No cameras detected!")
            print("\n Troubleshooting steps:")
            print("1. Check camera connections")
            print("2. Run: python3 camera_validator.py --setup")
            print("3. Reboot system")
            print("4. Check: ls /dev/video*")
            return

        print(f"Found {len(cameras)} camera(s):\n")

        for camera in cameras:
            status = (
                "READY"
                if (
                    camera["available"]
                    and camera["stable"]
                    and camera["resolution_ok"]
                    and camera["fps_ok"]
                )
                else "ISSUES"
            )

            print(f"Camera {camera['index']}: {status}")
            print(f"   Backend: {camera['backend']}")
            print(f"   Resolution: {camera['resolution'][0]}x{camera['resolution'][1]}")
            print(f"   FPS: {camera['fps']:.1f}")
            print(f"   Stable: {'YES' if camera['stable'] else 'NO'}")

            if camera["errors"]:
                print(f"   Issues: {', '.join(camera['errors'])}")

            print()

        # Summary
        ready_count = sum(
            1
            for c in cameras
            if c["available"] and c["stable"] and c["resolution_ok"] and c["fps_ok"]
        )

        print(f" Summary: {ready_count}/{len(cameras)} cameras ready for calibration")

        if ready_count == len(cameras):
            print(" All cameras validated! Ready for calibration.")
        else:
            print("  Some cameras need attention before calibration.")


def main():
    """Main entry point for camera validation."""
    parser = argparse.ArgumentParser(description="Camera Validator for Raspberry Pi")
    parser.add_argument(
        "--check", action="store_true", help="Check system prerequisites"
    )
    parser.add_argument(
        "--auto-detect", action="store_true", help="Auto-detect all cameras"
    )
    parser.add_argument("--validate", type=int, help="Validate specific camera index")
    parser.add_argument("--setup", action="store_true", help="Generate setup script")
    parser.add_argument("--all", action="store_true", help="Run full validation suite")

    args = parser.parse_args()

    validator = CameraValidator()

    if args.check:
        # Check system prerequisites
        results = validator.check_system_prerequisites()

        print("\n" + "=" * 50)
        print(" SYSTEM PREREQUISITES CHECK")
        print("=" * 50)

        for check, passed in results.items():
            status = "PASS" if passed else "FAIL"
            print(f"{status} {check.replace('_', ' ').title()}")

        all_passed = all(results.values())
        print(f"\n Overall: {'READY' if all_passed else 'NEEDS SETUP'}")

    elif args.auto_detect:
        # Auto-detect cameras
        cameras = validator.detect_cameras()
        validator.print_validation_report(cameras)

    elif args.validate is not None:
        # Validate specific camera
        result = validator.validate_camera(args.validate)

        cameras = [result]
        validator.print_validation_report(cameras)

    elif args.setup:
        # Generate setup script
        script = validator.generate_setup_script()

        with open("setup_cameras.sh", "w") as f:
            f.write(script)

        print(" Generated setup script: setup_cameras.sh")
        print("   Run with: sudo bash setup_cameras.sh")
        print("   Then reboot your Raspberry Pi")

    elif args.all:
        # Full validation suite
        print(" Running complete camera validation suite...\n")

        # 1. System check
        print("1. System Prerequisites:")
        results = validator.check_system_prerequisites()

        # 2. Camera detection
        print("\n2. Camera Detection:")
        cameras = validator.detect_cameras()

        # 3. Detailed validation
        if cameras:
            print("\n3. Detailed Validation:")
            validated_cameras = []
            for camera in cameras:
                result = validator.validate_camera(camera["index"])
                validated_cameras.append(result)

            validator.print_validation_report(validated_cameras)
        else:
            print("No cameras detected - cannot proceed with validation")

    else:
        # Default: show help
        parser.print_help()


if __name__ == "__main__":
    main()
