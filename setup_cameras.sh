#!/bin/bash
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
