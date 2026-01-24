# Camera Calibration Installation - URC 2026

## TL;DR - Get Started in 5 Minutes

```bash
# 1. Generate setup script
python3 camera_validator.py --setup

# 2. Run system setup (requires sudo)
sudo bash setup_cameras.sh

# 3. Reboot to apply permissions
sudo reboot

# 4. Verify installation
python3 verify_installation.py

# 5. Test camera validation
python3 camera_validator.py --auto-detect

# 6. Run calibration
python3 quick_calibration.py --camera 0
```

---

## 📦 What Gets Installed

### System Packages
- `v4l-utils` - Video device control
- `libv4l-dev` - Video development libraries
- `build-essential` - Compilation tools
- Camera permissions and udev rules

### Python Libraries
- `numpy` - Numerical computing
- `opencv-python` - Computer vision
- Camera validation and calibration scripts

### Camera Support
- USB cameras (Logitech, ArduCam, etc.)
- Raspberry Pi Camera modules (CSI/Ribbon cable cameras)
- V4L2-compatible devices

---

## Installation Methods

### Method 1: Automated (Recommended)
```bash
python3 camera_validator.py --setup && sudo bash setup_cameras.sh && sudo reboot
```

**Note for CSI/Ribbon Cable Cameras:** The setup script automatically enables the camera interface and auto-detection. After rebooting, your Raspberry Pi cameras should be detected automatically.

### Method 2: Manual
```bash
# System packages
sudo apt update && sudo apt install -y v4l-utils libv4l-dev python3-pip

# Python libraries
pip3 install numpy opencv-python

# Permissions
sudo usermod -a -G video $USER && sudo reboot
```

### Method 3: Virtual Environment
```bash
python3 -m venv ~/calibration_env
source ~/calibration_env/bin/activate
pip install -r requirements.txt
```

---

## Verification Steps

### 1. System Check
```bash
python3 camera_check.py
# Should show: System ready for camera calibration!
```

### 2. Installation Verification
```bash
python3 verify_installation.py
# Should show: ALL CHECKS PASSED!
```

### 3. Camera Validation
```bash
python3 camera_validator.py --auto-detect
# Should show: Camera 0: READY
```

### 4. Calibration Test
```bash
python3 quick_calibration.py --camera 0
# Should successfully calibrate camera
```

---

## 📋 Files Created

```
calibration/camera/
├── camera_validator.py      # Comprehensive camera validation
├── camera_check.py          # Basic system check
├── verify_installation.py   # Installation verification
├── installation_guide.md    # Detailed installation guide
├── pi_camera_setup.md       # Raspberry Pi specific setup
├── requirements.txt         # Python dependencies
├── setup_cameras.sh         # System setup script (generated)
├── quick_calibration.py     # Camera calibration
├── check_calibration.py     # Quality assessment
└── INSTALL.md              # This file
```

---

## Troubleshooting

### Raspberry Pi CSI Camera Setup

If you have cameras connected via ribbon cable (CSI interface), ensure they're enabled:

```bash
# 1. Enable camera interface
sudo raspi-config nonint do_camera 0

# 2. Enable camera auto-detection (for multiple cameras)
CONFIG_FILE="/boot/firmware/config.txt"
[ ! -f "$CONFIG_FILE" ] && CONFIG_FILE="/boot/config.txt"
sudo sed -i 's/camera_auto_detect=0/camera_auto_detect=1/' "$CONFIG_FILE" 2>/dev/null || \
echo "camera_auto_detect=1" | sudo tee -a "$CONFIG_FILE" > /dev/null

# 3. Reboot
sudo reboot

# 4. After reboot, verify cameras are detected
rpicam-still --list-cameras
```

**Important:** Changes to camera settings require a reboot to take effect.

### Common Issues

**"Permission denied"**
```bash
sudo usermod -a -G video $USER
sudo reboot
```

**"Module not found"**
```bash
pip3 install -r requirements.txt
```

**"No cameras detected"**

For USB cameras:
```bash
ls /dev/video*
# Check camera connections and try different USB ports
```

For Raspberry Pi CSI cameras (ribbon cable):
```bash
# Check if camera interface is enabled
raspi-config nonint get_camera
# Should return 0 (enabled). If it returns 1, enable it:
sudo raspi-config nonint do_camera 0

# Check camera auto-detection setting
grep camera_auto_detect /boot/firmware/config.txt || grep camera_auto_detect /boot/config.txt
# Should show: camera_auto_detect=1

# If not enabled, add it:
sudo sed -i 's/camera_auto_detect=0/camera_auto_detect=1/' /boot/firmware/config.txt 2>/dev/null || \
sudo sed -i 's/camera_auto_detect=0/camera_auto_detect=1/' /boot/config.txt 2>/dev/null

# Reboot required for changes to take effect
sudo reboot

# After reboot, test cameras:
rpicam-still --list-cameras
```

**"OpenCV not available"**
```bash
pip3 install opencv-python
# Or for headless: pip3 install opencv-python-headless
```

---

## Success Criteria

After installation, you should be able to:

1. Run `python3 verify_installation.py` without errors
2. Run `python3 camera_validator.py --auto-detect` and see cameras
3. Run `python3 quick_calibration.py --camera 0` successfully
4. Generate calibration files in JSON format

---

## 📚 Additional Resources

- `camera/pi_camera_setup.md` - Complete Raspberry Pi camera setup guide (hardware & software)
- `camera/README.md` - Camera calibration usage documentation
- `README.md` - Project overview

---

**Ready to calibrate?** Your Raspberry Pi is now equipped for professional camera calibration! 📸

**Status**: Production Ready
**Tested on**: Raspberry Pi 4B/5 with Raspberry Pi OS Bookworm
**Last Updated**: December 2024
