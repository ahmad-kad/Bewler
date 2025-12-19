#!/usr/bin/env python3
"""
Simple Camera Check for Raspberry Pi - URC 2026

Basic camera validation without OpenCV dependency.
Perfect for initial setup verification.

Usage:
    python3 camera_check.py
"""

import subprocess


def run_command(cmd, description):
    """Run a command and return success status."""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=10
        )
        success = result.returncode == 0
        output = result.stdout.strip() if success else result.stderr.strip()
        return success, output
    except Exception as e:
        return False, str(e)


def check_system_setup():
    """Check Raspberry Pi system setup for cameras."""
    print(" Raspberry Pi Camera System Check")
    print("=" * 40)

    checks = [
        (
            "Video devices present",
            "ls /dev/video* 2>/dev/null | wc -l",
            lambda x: int(x) > 0,
        ),
        (
            "User in video group",
            "groups | grep -q video && echo 'yes' || echo 'no'",
            lambda x: x == "yes",
        ),
        (
            "v4l2-utils installed",
            "which v4l2-ctl >/dev/null && echo 'yes' || echo 'no'",
            lambda x: x == "yes",
        ),
        (
            "Camera interface enabled",
            "raspi-config nonint get_camera",
            lambda x: x.strip() == "1",
        ),
    ]

    results = []
    for description, command, validator in checks:
        print(f"Checking {description}...", end=" ")
        success, output = run_command(command, description)

        if success and validator(output):
            print("")
            results.append(True)
        else:
            print("")
            results.append(False)

    # Summary
    print("\n" + "=" * 40)
    passed = sum(results)
    total = len(results)
    print(f"System Check: {passed}/{total} checks passed")

    if passed == total:
        print(" System ready for camera calibration!")
    else:
        print("  Some issues found. Run setup script:")
        print("   python3 camera_validator.py --setup")

    return passed == total


def list_camera_devices():
    """List all camera devices detected."""
    print("\n Detected Camera Devices:")
    print("-" * 30)

    success, output = run_command("ls /dev/video*", "list video devices")
    if success and output:
        devices = output.split()
        for device in devices:
            print(f"   {device}")

            # Try to get device info
            info_success, info = run_command(
                f"v4l2-ctl -d {device} --info 2>/dev/null | head -3", "device info"
            )
            if info_success:
                for line in info.split("\n")[:2]:  # First 2 lines
                    if line.strip():
                        print(f"     {line.strip()}")
            print()
    else:
        print("   No camera devices found")
        print("\n Troubleshooting:")
        print("   1. Check camera connections")
        print("   2. Try different USB ports")
        print("   3. Run: sudo usermod -a -G video $USER")
        print("   4. Reboot system")


def check_opencv():
    """Check if OpenCV is available."""
    print("\n OpenCV Status:")
    try:
        import cv2

        print("    OpenCV available")
        version = cv2.__version__
        print(f"   Version: {version}")

        # Test camera access
        print("\n Testing Camera Access:")
        for i in range(3):  # Test first 3 indices
            try:
                cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                if cap.isOpened():
                    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    print(f"   Camera {i}:  {width}x{height}")
                    cap.release()
                else:
                    print(f"   Camera {i}:  Not accessible")
            except Exception:
                print(f"   Camera {i}:  Error accessing")

    except ImportError:
        print("    OpenCV not installed")
        print("   Install with: pip3 install opencv-python")


def main():
    """Main function."""
    print("Raspberry Pi Camera Check - URC 2026")
    print("=====================================\n")

    # Basic system checks
    system_ready = check_system_setup()

    # List devices
    list_camera_devices()

    # OpenCV check
    check_opencv()

    # Final guidance
    print("\n" + "=" * 50)
    print(" Next Steps:")

    if system_ready:
        print("1. Cameras detected: Run calibration")
        print("   python3 quick_calibration.py --camera 0")
    else:
        print("1. Fix system issues first")
        print("   python3 camera_validator.py --setup")
        print("2. Reboot system")
        print("3. Run this check again")

    print("\n For detailed validation:")
    print("   python3 camera_validator.py --all")


if __name__ == "__main__":
    main()
