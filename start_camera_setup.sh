#!/bin/bash
# URC 2026 Camera Setup Launcher
# One-command camera setup and calibration

echo "🎥 URC 2026 Camera Setup Launcher"
echo "=================================="
echo

# Check if we're in the right directory
if [ ! -f "scripts/setup_cameras.py" ]; then
    echo "❌ Error: Please run this script from the Bewler project root directory"
    echo "   cd /home/malaysia/Bewler"
    echo "   ./start_camera_setup.sh"
    exit 1
fi

echo "🔍 Checking system..."

# Check if we're on Raspberry Pi
if [ ! -f "/proc/device-tree/model" ]; then
    echo "❌ Error: This appears to not be a Raspberry Pi system"
    echo "   Camera calibration requires Raspberry Pi hardware"
    exit 1
fi

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Error: Python 3 is not installed"
    echo "   Install with: sudo apt install python3"
    exit 1
fi

echo "✅ System check passed"
echo

# Detect cameras
echo "📹 Checking for cameras..."
CAMERA_COUNT=$(python3 -c "import sys; sys.path.insert(0, 'src'); import picamera2; print(len(picamera2.Picamera2.global_camera_info()))" 2>/dev/null || echo "0")

if [ "$CAMERA_COUNT" -gt 0 ]; then
    echo "✅ Found $CAMERA_COUNT camera(s)"
    echo
    echo "🚀 Launching calibration wizard..."
    python3 scripts/camera_wizard.py
else
    echo "⚠️  No cameras detected"
    echo
    echo "🔧 Starting automated setup to fix camera detection..."
    echo

    # Run automated setup
    python3 scripts/setup_cameras.py

    echo
    echo "🔄 After setup completes, reboot and run this script again:"
    echo "   sudo reboot"
    echo "   ./start_camera_setup.sh"
fi

