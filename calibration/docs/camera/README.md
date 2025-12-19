# Sensor-Based Camera Calibration - URC 2026

**Intelligent camera calibration with automatic sensor detection and individual file storage.**

---

##  Quick Start

### Single Camera (Auto Sensor Detection)
```bash
python quick_calibration.py --camera 0
# → Detects: IMX219 sensor
# → Assigns: imx219_1
```

### Custom Camera Name
```bash
python quick_calibration.py --camera 0 --name "front_left"
```

### Batch Calibration (Sensor-Based IDs)
```bash
python quick_calibration.py --batch 3
# → imx219_1, imx219_2, imx219_3 (if all IMX219)
```

### List Calibrated Cameras
```bash
python quick_calibration.py --list-cameras
```

### Rename Cameras
```bash
python quick_calibration.py --rename-camera imx219_1 front_left
```

---

##  What It Does

1. **Sensor Detection**: Automatically detects camera sensors (IMX219, IMX477, etc.)
2. **Sequential IDs**: Generates `sensor_1`, `sensor_2`, etc. per sensor type
3. **File Storage**: Saves calibrations as individual JSON files
4. **Visual Feedback**: Real-time calibration progress with ChArUco detection
5. **Quality Assessment**: Automatic evaluation of calibration quality

---

##  Files

- `quick_calibration.py` - Main calibration script with sensor detection & file storage
- `check_calibration.py` - Quality assessment tool for calibration files
- `camera_validator.py` - Camera detection and validation utilities
- `README.md` - This documentation

##  File Storage Features

### Automatic Sensor Detection
- **IMX219**: Raspberry Pi Camera v2 (8MP)
- **IMX477**: Raspberry Pi HQ Camera (12.3MP)
- **IMX708**: Raspberry Pi Camera v3 (12MP)
- **OV5647**: Raspberry Pi Camera v1 (5MP)

### Sequential ID Generation
```
First IMX219 camera  → imx219_1
Second IMX219 camera → imx219_2
First IMX477 camera  → imx477_1
```

### File Storage
- **JSON Format**: `camera_calibrations.json` (default)
- **YAML Format**: `camera_calibrations.yaml` (optional)
- **Each calibration** stored in separate JSON file
- **Sensor tracking** and calibration history

---

##  How It Works

1. **Sensor Detection**: Automatically identifies camera sensor model
2. **ID Generation**: Creates sequential ID (sensor_1, sensor_2, etc.)
3. **Calibration**: Uses ChArUco board for precise camera calibration
4. **File Storage**: Saves to individual JSON files with metadata
5. **Quality Check**: Validates calibration accuracy

##  CLI Options

```bash
# Basic calibration
python quick_calibration.py                    # Camera 0, auto ID
python quick_calibration.py --camera 1        # Specific camera
python quick_calibration.py --name front_left # Custom name

# Batch operations
python quick_calibration.py --batch 3         # Calibrate 3 cameras
python quick_calibration.py --list-cameras    # Show all calibrated
python quick_calibration.py --rename-camera imx219_1 front_left

# Custom directory
python quick_calibration.py --calibration-dir my_cals    # Custom output directory
python quick_calibration.py --manual                  # Manual capture mode
```

---

##  Visual Feedback

- ** Green Status**: Good detection (ready to capture)
- ** Orange Status**: Partial detection (adjust angle)
- ** Red Status**: Searching for markers
- **Progress Bar**: Shows frames captured (target: 15)
- **Live Counter**: Frames captured and time remaining

---

##  Output Format

Files saved as: `camera_{id}_{timestamp}.json`

```json
{
  "camera_id": 0,
  "timestamp": "20241219_143000",
  "camera_matrix": [[669.13, 0.0, 644.66], [0.0, 668.50, 357.35], [0.0, 0.0, 1.0]],
  "distortion_coefficients": [2.54, 6.38, -0.018, -0.226, -249.18],
  "image_width": 1280,
  "image_height": 720,
  "frames_used": 15,
  "reprojection_error": 0.0
}
```

---

##  Quality Assessment

Run `python check_calibration.py` to assess quality:

- **Excellent**: 12+ frames → 95/100 score
- **Good**: 8+ frames → 80/100 score
- **Acceptable**: 5+ frames → 60/100 score
- **Poor**: <5 frames → recalibrate

---

##  Troubleshooting

### Camera Not Detected
```bash
# Check devices
ls /dev/video*

# Check permissions
sudo usermod -a -G video $USER
# Logout/login for changes
```

### Poor Quality Results
- Ensure good lighting
- Move board through different angles
- Keep board visible and steady
- Try longer capture time

### Permission Issues
```bash
# Add to video group
sudo usermod -a -G video $USER
# Reboot or logout/login
```

---

##  Installation & Setup

### Quick Setup (Recommended)
```bash
cd calibration/camera
python3 camera_validator.py --setup    # Generate setup script
sudo bash setup_cameras.sh             # Run system setup
sudo reboot                            # Apply permission changes
python3 verify_installation.py         # Verify everything works
```

### Manual Installation
See `installation_guide.md` for detailed installation instructions.

### Requirements Summary

#### System Requirements
- **Raspberry Pi**: 4B or 5 with Raspberry Pi OS Bookworm
- **Python**: 3.8+
- **RAM**: 2GB minimum, 4GB recommended
- **Storage**: 16GB SD card minimum

#### Python Libraries
- OpenCV 4.5.0+ (`opencv-python`)
- NumPy 1.21.0+ (`numpy`)

#### System Packages
- `v4l-utils` - Video device utilities
- `libv4l-dev` - Video development libraries
- Camera permissions (user in `video` group)

#### Supported Cameras
- USB cameras (Logitech C920/C930e, ArduCam)
- Raspberry Pi Camera modules (v2/v3)
- Any V4L2-compatible camera

---

** Status**: Simple & Effective
** Last Updated**: December 2024
** For**: URC 2026 One-time Setup
