#!/usr/bin/env python3
"""
Installation Verification Script - URC 2026 Camera Calibration

Verifies that all required libraries and system components are properly installed.
Run this after installation to ensure everything is working correctly.

Usage:
    python3 verify_installation.py
"""

import importlib
import subprocess
import sys
from pathlib import Path


class InstallationVerifier:
    """Comprehensive installation verification."""

    def __init__(self):
        self.results = {}
        self.errors = []

    def check_python_version(self):
        """Check Python version compatibility."""
        version = sys.version_info
        required = (3, 8)

        if version >= required:
            self.results["python_version"] = (
                f"Python {version.major}.{version.minor}.{version.micro} OK"
            )
            return True
        else:
            self.results["python_version"] = (
                f"Python {version.major}.{version.minor}.{version.micro} ✗ (need 3.8+)"
            )
            self.errors.append(f"Python {required[0]}.{required[1]}+ required")
            return False

    def check_python_package(self, package_name, import_name=None, min_version=None):
        """Check if Python package is installed and meets version requirements."""
        if import_name is None:
            import_name = package_name

        try:
            module = importlib.import_module(import_name)

            if hasattr(module, "__version__"):
                version = module.__version__
                if min_version and version < min_version:
                    self.results[package_name] = (
                        f"{package_name} {version} FAILED (need {min_version}+)"
                    )
                    self.errors.append(f"{package_name} version {version} too old")
                    return False
                else:
                    self.results[package_name] = f"{package_name} {version} OK"
            else:
                self.results[package_name] = f"{package_name} OK (version unknown)"

            return True

        except ImportError:
            self.results[package_name] = f"{package_name} ✗ (not installed)"
            self.errors.append(f"{package_name} not installed")
            return False

    def check_system_command(self, command, description):
        """Check if system command is available."""
        try:
            result = subprocess.run(
                command.split(), capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                self.results[description] = f"{description} OK"
                return True
            else:
                self.results[description] = f"{description} ✗"
                self.errors.append(f"{description} failed")
                return False
        except (subprocess.TimeoutExpired, FileNotFoundError):
            self.results[description] = f"{description} ✗ (not found)"
            self.errors.append(f"{description} not available")
            return False

    def check_file_permissions(self, file_path, description):
        """Check if file/directory has correct permissions."""
        path = Path(file_path)
        if path.exists():
            # Check if readable/writable as needed
            readable = path.stat().st_mode & 0o444
            if readable:
                self.results[description] = f"{description} OK"
                return True
            else:
                self.results[description] = f"{description} ✗ (permission denied)"
                self.errors.append(f"No permission for {file_path}")
                return False
        else:
            self.results[description] = f"{description} ✗ (not found)"
            self.errors.append(f"{file_path} not found")
            return False

    def run_all_checks(self):
        """Run comprehensive installation verification."""
        print("Verifying Camera Calibration Installation")
        print("=" * 50)

        # Python version
        self.check_python_version()

        # Python packages
        python_packages = [
            ("numpy", "numpy", "1.21.0"),
            ("cv2", "cv2", "4.5.0"),
        ]

        for package_name, import_name, min_version in python_packages:
            self.check_python_package(package_name, import_name, min_version)

        # System commands
        system_commands = [
            ("v4l2-ctl --version", "v4l2-utils"),
            ("ls /dev/video0", "Camera devices"),
        ]

        for command, description in system_commands:
            self.check_system_command(command, description)

        # File permissions
        file_checks = [
            ("/dev/video0", "Camera device access"),
        ]

        for file_path, description in file_checks:
            self.check_file_permissions(file_path, description)

        # Custom OpenCV checks
        self.check_opencv_functionality()

    def check_opencv_functionality(self):
        """Check OpenCV functionality."""
        try:
            import cv2
            import numpy as np

            # Test basic functionality
            img = np.zeros((100, 100, 3), dtype=np.uint8)
            cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Test conversion

            # Test camera opening (without actual camera)
            cap = cv2.VideoCapture(-1)  # Invalid index
            cap.release()

            self.results["OpenCV functionality"] = "OpenCV functionality OK"
            return True

        except Exception as e:
            self.results["OpenCV functionality"] = (
                f"OpenCV functionality FAILED ({str(e)})"
            )
            self.errors.append(f"OpenCV functionality failed: {str(e)}")
            return False

    def print_report(self):
        """Print verification report."""
        print("\nVERIFICATION RESULTS")
        print("=" * 30)

        for component, status in self.results.items():
            print(f"{status}")

        print("\n" + "=" * 50)

        if self.errors:
            print("ISSUES FOUND:")
            for error in self.errors:
                print(f"   • {error}")

            print("\n🔧 FIXES:")
            print("   1. Run: python3 camera_validator.py --setup")
            print("   2. Run: sudo bash setup_cameras.sh")
            print("   3. Reboot: sudo reboot")
            print("   4. Test: python3 camera_check.py")

        else:
            print("ALL CHECKS PASSED!")
            print("Ready for camera calibration")
            print("\nNext steps:")
            print("   • Connect cameras")
            print("   • Run: python3 camera_validator.py --auto-detect")
            print("   • Calibrate: python3 quick_calibration.py --camera 0")

    def get_success_rate(self):
        """Calculate success rate of installation."""
        total_checks = len(self.results)
        passed_checks = sum(1 for status in self.results.values() if "✓" in status)
        return passed_checks, total_checks


def main():
    """Main verification function."""
    verifier = InstallationVerifier()
    verifier.run_all_checks()
    verifier.print_report()

    passed, total = verifier.get_success_rate()
    print(f"\n📈 Success Rate: {passed}/{total} checks passed")

    # Exit with appropriate code
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
