#  URC 2026 Camera Setup Guide

## Clean & User-Friendly Camera Calibration

This guide provides multiple easy ways to set up and calibrate your cameras for robotics applications.

---

##  Quick Start (Recommended)

### Option 1: Automated Setup (Easiest)
```bash
# Run automated setup
python3 setup_cameras.py

# Follow the prompts, then reboot if needed
sudo reboot

# Launch the interactive wizard
python3 auto_calibrate.py  # Interactive calibration
```

### Option 2: Graphical Interface (Most User-Friendly)
```bash
# Install tkinter if needed
sudo apt install python3-tk

# Launch the GUI
python3 camera_preview.py  # GUI-style preview
```

### Option 3: Command Line (Original Method)
```bash
# Check system status
python3 test_detection.py  # Camera detection test

# Test camera and detection (5-second test)
python3 test_detection.py --camera 0

# Test live preview functionality (shows video window)
python3 camera_preview.py  # Live preview test --camera 0

# Live preview with detection overlay (full featured)
python3 camera_preview.py --camera 0

# Run calibration
python3 auto_calibrate.py  # Quick calibration --camera 0
```

### Option 4: Generate Test Materials
```bash
# Generate ChArUco test board image
python3 test_detection.py --generate-board

# Print the generated charuco_test_board_8x6.png for testing

# Automatic calibration with live preview
python3 auto_calibrate.py --camera 0 --duration 20 --name front_camera
# Features: Live preview, auto frame collection, intelligent selection
```

---

##  What Each Tool Does

###  `setup_cameras.py` - Automated Setup
- **Automatically detects** your Raspberry Pi model
- **Installs missing packages** (PiCamera2, OpenCV, etc.)
- **Fixes permissions** (adds user to video group)
- **Configures camera settings** in `/boot/firmware/config.txt`
- **Guides you through** the entire setup process

###  `camera_wizard.py` - Interactive Calibration
- **Step-by-step menus** for camera selection
- **Automatic camera detection** with sensor identification
- **Real-time progress** during calibration
- **Visual feedback** with status messages
- **Error handling** with helpful troubleshooting tips

###  `camera_gui.py` - Graphical Interface
- **Full GUI application** with tabs and buttons
- **Live system status** checks
- **Progress bars** during calibration
- **Camera management** with detailed information
- **Point-and-click** operation

---

##  Step-by-Step Setup Process

### Step 1: Initial Setup
```bash
cd /home/malaysia/Bewler

# Run automated setup (handles everything)
python3 setup_cameras.py
```

The setup will:
-  Check your system requirements
-  Install any missing packages
-  Fix user permissions
-  Configure camera settings
-  Request reboot if needed

### Step 2: Camera Connection
After reboot, ensure your camera is properly connected:

**Raspberry Pi Camera:**
- Connect to the **CSI connector** on your Pi 5
- Ensure the **ribbon cable** is fully seated
- **Blue side** of cable faces the HDMI ports

**USB Cameras:**
- Plug into any **USB port**
- Ensure the camera has **power** (if external)

### Step 3: Verify Detection
```bash
# Check if camera is detected
python3 -c "import picamera2; print('Cameras:', len(picamera2.Picamera2.global_camera_info()))"
```

Should show: `Cameras: 1` (or more)

### Step 4: Test Camera & Detection
```bash
# Quick detection test (5 seconds)
python3 test_detection.py --camera 0

# Live preview with detection overlay
python3 camera_preview.py --camera 0
```

**What to look for:**
-  **Green status**: "Board Detected" with quality percentage
-  **Markers highlighted**: White squares around detected markers
-  **Corners marked**: Pink dots on ChArUco corners
-  **Quality score**: 60%+ for good calibration
-  **Progress bar**: Shows calibration time remaining
-  **Frame counter**: Shows how many frames collected

### Step 5: Run Calibration

#### Using the Wizard (Recommended):
```bash
python3 auto_calibrate.py  # Interactive calibration
```
Follow the interactive menus to:
1. Check system status
2. Select your camera
3. Configure calibration settings
4. Run calibration with live feedback

#### Using the GUI:
```bash
python3 camera_preview.py  # GUI-style preview
```
Click through the tabs to:
- Run system checks
- Select and calibrate cameras
- View calibration results

---

##  Understanding the Output

### System Check Results
```
 Raspberry Pi: Raspberry Pi 5 Model B Rev 1.0
 PiCamera2 available
 OpenCV 4.12.0 available
 User in video group
 1 camera(s) detected
 System ready for calibration!
```

### Calibration Results
After successful calibration, you'll get:
- **Calibration file**: `calibrations/camera_0.json`
- **Camera matrix**: Intrinsic parameters
- **Distortion coefficients**: Lens correction data
- **Quality metrics**: Frame count, reprojection error

---

##  Troubleshooting

### "No cameras detected"
```bash
# Check physical connections
python3 setup_cameras.py --diagnostics

# Verify camera is powered and connected
python3 -c "import picamera2; print(picamera2.Picamera2.global_camera_info())"
```

### "Permission denied" or "Video group" errors
```bash
# Fix permissions
sudo usermod -a -G video $USER

# Logout and login again, or reboot
sudo reboot
```

### "Import errors" or "Module not found"
```bash
# Install missing packages
sudo apt install python3-picamera2 python3-opencv

# Or run automated setup
python3 setup_cameras.py
```

### Detection Not Working (Red Status)
```bash
# Test detection specifically
python3 test_detection.py --camera 0

# Launch live preview to debug
python3 camera_preview.py --camera 0
```

**Common Detection Issues:**
- **No markers detected**: ChArUco board not visible or lighting too poor
- **Markers but no board**: Board partially visible, need better angle
- **Low quality (<60%)**: Board too far, motion blur, or poor focus
- **Inconsistent detection**: Unstable board position or changing lighting

**Solutions:**
- Use `python3 camera_preview.py` to see live detection
- Adjust lighting for better contrast
- Hold board steady with good focus
- Try different distances (30-100cm typical)
- Ensure board is clean and undamaged

### GUI won't start
```bash
# Install tkinter
sudo apt install python3-tk

# Try again
python3 camera_preview.py  # GUI-style preview
```

---

##  Advanced Usage

### Custom Calibration Directory
```bash
# Wizard
python3 auto_calibrate.py  # Interactive calibration
# Then select "Settings" and change directory

# Command line
python3 auto_calibrate.py  # Quick calibration --calibration-dir my_calibrations
```

### Batch Calibration (Multiple Cameras)
```bash
# Connect first camera, calibrate
python3 auto_calibrate.py  # Interactive calibration

# Connect second camera, calibrate again
python3 auto_calibrate.py  # Interactive calibration

# View all calibrated cameras
python3 auto_calibrate.py  # Interactive calibration
# Select "Show Calibrated Cameras"
```

### Manual Calibration Control
```bash
# Short calibration (30 seconds)
python3 auto_calibrate.py  # Quick calibration --camera 0 --duration 30

# Custom camera name
python3 auto_calibrate.py  # Quick calibration --camera 0 --name front_left

# Manual capture mode (press space to capture frames)
python3 auto_calibrate.py  # Quick calibration --camera 0 --manual
```

---

##  File Structure

```
calibrations/
 camera_0.json          # First camera calibration
 front_left.json        # Named camera calibration
 imx708_1.json         # Sensor-based naming

./  # Root directory
 camera_wizard.py       # Interactive text wizard
 camera_gui.py         # Graphical interface
 setup_cameras.py      # Automated setup
 quick_calibration.py  # Original command-line tool
 camera_check.py       # System diagnostics
```

---

##  Best Practices

###  Camera Setup
- **Secure connections**: Ensure CSI cable is firmly seated
- **Power first**: Connect camera power before data cable
- **Clean connectors**: Dust can interfere with connections
- **Test after changes**: Always verify detection after hardware changes

###  Calibration Tips
- **Good lighting**: Bright, even lighting improves detection
- **Board movement**: Move ChArUco board through different angles
- **Steady hold**: Keep board still when corners are detected
- **Multiple distances**: Calibrate at different distances from camera
- **Quality check**: Use 60+ seconds for best results

###  Data Management
- **Backup calibrations**: Copy `calibrations/` folder regularly
- **Version control**: Track calibration changes with git
- **Naming convention**: Use descriptive names (front_left, overhead, etc.)
- **Documentation**: Note calibration conditions (lighting, distance, etc.)

---

##  Emergency Recovery

If something goes wrong:

### Reset Camera Configuration
```bash
# Reset config.txt camera settings
sudo sed -i '/camera_auto_detect/d' /boot/firmware/config.txt
sudo sed -i '/dtoverlay=imx/d' /boot/firmware/config.txt

# Reboot
sudo reboot
```

### Reinstall Packages
```bash
# Remove and reinstall camera packages
sudo apt remove --purge python3-picamera2 python3-libcamera libcamera-*
sudo apt autoremove
sudo apt install python3-picamera2 python3-libcamera

# Reboot
sudo reboot
```

### Start Fresh
```bash
# Remove all calibrations
rm -rf calibrations/

# Reset configuration
rm -f ~/.urc_camera_config.json

# Reboot and start over
sudo reboot
```

---

##  Support

If you encounter issues:

1. **Run diagnostics**: `python3 setup_cameras.py --diagnostics`
2. **Check logs**: Look for error messages in terminal output
3. **Verify hardware**: Test camera with different cable/port
4. **Update system**: `sudo apt update && sudo apt upgrade`

For hardware-specific issues, consult the [Raspberry Pi Camera Documentation](https://www.raspberrypi.com/documentation/computers/camera_software.html).

---

** Happy calibrating! Your cameras are now ready for robotics applications.**

