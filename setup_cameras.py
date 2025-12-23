#!/usr/bin/env python3
"""
Camera Setup Automation - URC 2026
Automated camera detection and configuration for Raspberry Pi 5
"""

import os
import sys
import subprocess
import time
from pathlib import Path
from typing import Dict, List, Tuple

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))


class CameraSetup:
    """Automated camera setup for Raspberry Pi."""

    def __init__(self):
        self.system_info = self.get_system_info()

    def get_system_info(self) -> Dict:
        """Get system information."""
        info = {
            "is_raspberry_pi": False,
            "model": "Unknown",
            "has_cameras": False,
            "camera_count": 0,
            "picamera_available": False,
            "opencv_available": False,
        }

        # Check if Raspberry Pi
        try:
            with open("/proc/device-tree/model", "r") as f:
                model = f.read().strip()
                info["is_raspberry_pi"] = "Raspberry Pi" in model
                info["model"] = model
        except:
            pass

        # Check camera libraries
        try:
            import picamera2

            info["picamera_available"] = True
        except ImportError:
            pass

        try:
            import cv2

            info["opencv_available"] = True
        except ImportError:
            pass

        return info

    def run_command(self, cmd: str, _description: str = "") -> Tuple[bool, str]:
        """Run system command and return result."""
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30)
            return result.returncode == 0, result.stdout.strip()
        except Exception as e:
            return False, str(e)

    def check_system_requirements(self) -> Dict[str, bool]:
        """Check all system requirements."""
        print(" Checking system requirements...")

        checks = {
            "raspberry_pi": self.system_info["is_raspberry_pi"],
            "picamera2": self.system_info["picamera_available"],
            "opencv": self.system_info["opencv_available"],
            "v4l_utils": False,
            "permissions": False,
            "config_txt": False,
        }

        # Check v4l-utils
        success, _ = self.run_command("which v4l2-ctl")
        checks["v4l_utils"] = success

        # Check permissions
        success, output = self.run_command("groups")
        checks["permissions"] = "video" in output

        # Check config.txt
        config_file = Path("/boot/firmware/config.txt")
        if config_file.exists():
            content = config_file.read_text()
            checks["config_txt"] = "camera_auto_detect=1" in content

        return checks

    def install_missing_packages(self) -> bool:
        """Install missing packages."""
        print(" Installing missing packages...")

        packages = []

        if not self.system_info["picamera_available"]:
            packages.extend(["python3-picamera2", "python3-libcamera"])

        if not self.system_info["opencv_available"]:
            packages.extend(["python3-opencv", "libopencv-dev"])

        if not self.run_command("which v4l2-ctl")[0]:
            packages.append("v4l-utils")

        if packages:
            cmd = f"sudo apt update && sudo apt install -y {' '.join(packages)}"
            success, output = self.run_command(cmd)
            if not success:
                print(f" Failed to install packages: {output}")
                return False

        print(" Packages installed successfully!")
        return True

    def fix_permissions(self) -> bool:
        """Fix camera permissions."""
        print(" Fixing camera permissions...")

        # Add user to video group
        success, output = self.run_command("sudo usermod -a -G video $USER")
        if not success:
            print(f" Failed to add user to video group: {output}")
            return False

        print(" User added to video group")
        print(" You may need to logout/login or reboot for changes to take effect")
        return True

    def configure_camera(self) -> bool:
        """Configure camera settings in config.txt."""
        print("  Configuring camera settings...")

        config_file = Path("/boot/firmware/config.txt")

        if not config_file.exists():
            print(" config.txt not found!")
            return False

        content = config_file.read_text()

        # Enable camera auto-detection
        if "camera_auto_detect=0" in content:
            content = content.replace("camera_auto_detect=0", "camera_auto_detect=1")
            print(" Enabled camera auto-detection")
        elif "camera_auto_detect=1" not in content:
            content += "\ncamera_auto_detect=1\n"
            print(" Added camera auto-detection")

        # Add camera overlay for Pi 5
        if "dtoverlay=imx708" not in content and "dtoverlay=imx219" not in content:
            content += "dtoverlay=imx708\n"
            print(" Added IMX708 camera overlay")

        # Write back
        try:
            config_file.write_text(content)
            print(" Configuration saved")
            return True
        except Exception as e:
            print(f" Failed to save configuration: {e}")
            return False

    def detect_cameras(self) -> List[Dict]:
        """Detect available cameras."""
        print(" Detecting cameras...")

        cameras = []

        # Try libcamera first (Pi 5)
        if self.system_info["picamera_available"]:
            try:
                import picamera2

                camera_info = picamera2.Picamera2.global_camera_info()
                for i, cam in enumerate(camera_info):
                    cameras.append({"index": i, "name": cam.get("Model", f"Camera {i}"), "method": "libcamera"})
            except Exception as e:
                print(f"  Libcamera detection failed: {e}")

        # Fallback to OpenCV
        if not cameras and self.system_info["opencv_available"]:
            try:
                import cv2

                for i in range(3):
                    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                    if cap.isOpened():
                        cameras.append({"index": i, "name": f"Camera {i} (V4L2)", "method": "v4l2"})
                        cap.release()
            except:
                pass

        return cameras

    def run_calibration_wizard(self) -> bool:
        """Launch the calibration wizard."""
        print(" Launching calibration wizard...")

        wizard_script = Path(__file__).parent / "camera_wizard.py"
        if wizard_script.exists():
            cmd = f"python3 {wizard_script}"
            os.system(cmd)
            return True
        else:
            print(" Calibration wizard not found!")
            return False

    def run_setup(self):
        """Run the complete setup process."""
        print(" URC 2026 Camera Setup")
        print("=" * 40)

        # Step 1: Check system
        print("\n Step 1: System Check")
        checks = self.check_system_requirements()

        print("System Status:")
        for check, status in checks.items():
            status_icon = "" if status else ""
            print(f"   {status_icon} {check.replace('_', ' ').title()}")

        # Step 2: Install missing packages
        missing = [k for k, v in checks.items() if not v and k in ["picamera2", "opencv", "v4l_utils"]]
        if missing:
            print(f"\n Step 2: Installing missing packages...")
            if not self.install_missing_packages():
                print(" Setup failed at package installation")
                return False

        # Step 3: Fix permissions
        if not checks["permissions"]:
            print(f"\n Step 3: Fixing permissions...")
            if not self.fix_permissions():
                print(" Setup failed at permissions")
                return False

        # Step 4: Configure camera
        if not checks["config_txt"]:
            print(f"\n  Step 4: Configuring camera...")
            if not self.configure_camera():
                print(" Setup failed at configuration")
                return False

        # Step 5: Reboot notice
        print(f"\n Step 5: Reboot Required")
        print("The system needs to reboot to apply camera configuration changes.")
        print("After reboot, run: python3 scripts/camera_wizard.py")

        reboot_now = input("Reboot now? (y/N): ").strip().lower()
        if reboot_now == "y":
            print(" Rebooting in 3 seconds...")
            time.sleep(3)
            os.system("sudo reboot")

        return True

    def run_diagnostics(self):
        """Run camera diagnostics."""
        print(" Camera Diagnostics")
        print("=" * 30)

        # System info
        print("System Information:")
        print(f"   Model: {self.system_info['model']}")
        print(f"   Is Raspberry Pi: {self.system_info['is_raspberry_pi']}")
        print(f"   PiCamera2: {self.system_info['picamera_available']}")
        print(f"   OpenCV: {self.system_info['opencv_available']}")
        print()

        # Camera detection
        cameras = self.detect_cameras()
        print(f"Camera Detection: {len(cameras)} cameras found")

        if cameras:
            for cam in cameras:
                print(f"    {cam['name']} (via {cam['method']})")
        else:
            print("    No cameras detected")
            print()
            print("Troubleshooting:")
            print("   1. Check physical camera connection")
            print("   2. Verify CSI cable is properly seated")
            print("   3. Ensure camera has power")
            print("   4. Run: python3 scripts/setup_cameras.py")
            print("   5. Reboot system")

        print()

        # Configuration check
        config_file = Path("/boot/firmware/config.txt")
        if config_file.exists():
            content = config_file.read_text()
            camera_configured = "camera_auto_detect=1" in content
            print(f"Camera Configuration: {' Enabled' if camera_configured else ' Disabled'}")

            if "dtoverlay=imx" in content:
                print("   Camera overlay:  Configured")
            else:
                print("   Camera overlay:  Missing")
        else:
            print("   Config file:  Not found")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Camera Setup Automation")
    parser.add_argument("--diagnostics", action="store_true", help="Run diagnostics only")
    parser.add_argument("--wizard", action="store_true", help="Launch calibration wizard")

    args = parser.parse_args()

    setup = CameraSetup()

    if args.diagnostics:
        setup.run_diagnostics()
    elif args.wizard:
        setup.run_calibration_wizard()
    else:
        # Run full setup
        success = setup.run_setup()
        if success:
            print("\n Setup completed! Run diagnostics to verify:")
            print("   python3 scripts/setup_cameras.py --diagnostics")


if __name__ == "__main__":
    main()
