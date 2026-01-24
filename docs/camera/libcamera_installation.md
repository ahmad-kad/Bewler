# libcamera Installation Guide - Quick Reference

**libcamera** is the modern camera stack for Raspberry Pi Camera Modules (CSI). This guide covers installation and verification.

---

## Quick Installation

### For Raspberry Pi OS Bookworm (2023+) - Recommended

libcamera is **pre-installed** on Raspberry Pi OS Bookworm. Simply verify it's available:

```bash
# Check if installed
libcamera-hello --version

# If not found, install
sudo apt update
sudo apt install -y libcamera-apps libcamera-tools python3-picamera2
```

### For Raspberry Pi OS Bullseye (2021-2022)

libcamera should be pre-installed, but if missing:

```bash
sudo apt update
sudo apt install -y libcamera-apps libcamera-tools python3-picamera2
```

### For Older Raspberry Pi OS (Buster or earlier)

**Upgrade to Bullseye or Bookworm** - libcamera is not available on older versions.

---

## What Gets Installed

### Core Packages

- **`libcamera-apps`**: Command-line applications
  - `libcamera-hello` - Quick camera test/preview
  - `libcamera-still` - Capture still images
  - `libcamera-vid` - Record video
  - `libcamera-raw` - Capture raw images

- **`libcamera-tools`**: Additional utilities
  - `libcamera-jpeg` - JPEG encoding utilities
  - Additional debugging and testing tools

- **`python3-picamera2`**: Python interface
  - Modern replacement for deprecated `picamera` library
  - Works with both GUI and headless systems

### System Dependencies (automatically installed)

- `libcamera0` - Core library (usually pre-installed)
- `libcamera-dev` - Development headers (if building from source)

---

## Verification Steps

### Step 1: Check Installation

```bash
# Check if commands are available
which libcamera-hello
which libcamera-still
which libcamera-vid

# Check versions
libcamera-hello --version
libcamera-still --version
```

### Step 2: Enable Camera Interface

libcamera requires the camera interface to be enabled:

```bash
# Check current status
raspi-config nonint get_camera
# 0 = enabled, 1 = disabled

# Enable if disabled
sudo raspi-config nonint do_camera 1
sudo reboot
```

### Step 3: Test Camera Access

```bash
# Quick preview test (2 seconds)
libcamera-hello -t 2000

# List available cameras
libcamera-still --list-cameras

# Capture test image
libcamera-still -o test.jpg

# Record test video (5 seconds)
libcamera-vid -t 5000 -o test.h264
```

---

## When Do You Need libcamera?

### Required For:
- **Raspberry Pi Camera Modules** (CSI interface)
  - Camera v1 (OV5647)
  - Camera v2 (IMX219)
  - Camera v3 (IMX708)
  - HQ Camera (IMX477)

### Not Required For:
- **USB cameras** (use V4L2 directly via OpenCV)
- **Legacy camera applications** (use old `raspistill`/`raspivid` if still available)

### Recommended For:
- **Sensor detection** (this project's `detect_camera_sensor()` function)
- **Hardware-accelerated encoding** (H.264 video)
- **Modern camera features** (better performance, more control)

---

## Common Use Cases

### Capture Still Image

```bash
# Basic capture
libcamera-still -o image.jpg

# High resolution
libcamera-still -o image.jpg --width 3280 --height 2464

# With preview
libcamera-still -o image.jpg --preview

# Raw capture
libcamera-still -o image.jpg --raw
```

### Record Video

```bash
# Basic video (5 seconds)
libcamera-vid -t 5000 -o video.h264

# H.264 encoding
libcamera-vid -t 0 --codec h264 -o video.h264

# With specific resolution
libcamera-vid -t 5000 --width 1920 --height 1080 -o video.h264
```

### List Cameras

```bash
# Show all detected cameras
libcamera-still --list-cameras
```

**Example output:**
```
Available cameras:
0: imx219 [3280x2464] (/base/soc/i2c0mux/i2c@1/imx219@10)
    Modes: 'SRGGB10_CSI2P' : 3280x2464 [0.05, 30.0] fps
           'SRGGB10_CSI2P' : 1920x1080 [0.05, 30.0] fps
```

### Python Interface (picamera2)

```python
from picamera2 import Picamera2

# Initialize camera
picam2 = Picamera2()

# Configure
config = picam2.create_preview_configuration()
picam2.configure(config)

# Start and capture
picam2.start()
array = picam2.capture_array()
picam2.stop()
```

---

## Troubleshooting

### Issue: "command not found"

**Solution:**
```bash
sudo apt update
sudo apt install -y libcamera-apps libcamera-tools
```

### Issue: "No cameras available"

**Check:**
1. Camera physically connected (CSI ribbon cable secure)
2. Camera interface enabled: `sudo raspi-config nonint do_camera 1`
3. System rebooted after enabling camera
4. Check detection: `vcgencmd get_camera` (should show `detected=1`)

### Issue: "Permission denied"

**Solution:**
```bash
sudo usermod -a -G video $USER
# Logout/login or reboot
```

### Issue: libcamera works but OpenCV can't access camera

**Explanation:** libcamera and OpenCV use different interfaces:
- **libcamera**: Direct access to CSI cameras (better performance)
- **OpenCV**: Uses V4L2 interface (`/dev/video*`)

**Solution:** For OpenCV access, ensure camera interface is enabled and V4L2 device exists:
```bash
ls /dev/video*
# Should show /dev/video0 (and possibly /dev/video10, /dev/video11)
```

---

## Integration with This Project

### Sensor Detection

The project's `detect_camera_sensor()` function uses libcamera:

```python
from src.utils.camera import detect_camera_sensor

sensor = detect_camera_sensor()
# Returns: "imx219", "imx477", "imx708", "ov5647", or None
```

**How it works:**
1. Calls `libcamera-still --list-cameras`
2. Parses output for sensor model
3. Falls back to `vcgencmd` if libcamera unavailable

### Camera Validation

The `camera_validator.py` script checks for libcamera:

```bash
python3 camera_validator.py --check
```

**Output includes:**
```
libcamera available
```

If libcamera is not found, it's noted as optional (USB cameras don't need it).

---

## Advanced: Building from Source

**Note:** Building from source is rarely necessary. Use `apt install` unless you need specific features or latest development version.

If you must build from source, see the [official libcamera documentation](https://www.raspberrypi.com/documentation/computers/camera_software.html).

---

## Summary

**Quick Install:**
```bash
sudo apt update
sudo apt install -y libcamera-apps libcamera-tools python3-picamera2
sudo raspi-config nonint do_camera 1
sudo reboot
```

**Verify:**
```bash
libcamera-hello -t 2000
libcamera-still --list-cameras
```

**When needed:** Raspberry Pi Camera Modules (CSI)  
**When optional:** USB cameras (but recommended for sensor detection)

---

**Status**: Production Ready  
**Tested on**: Raspberry Pi OS Bookworm  
**Last Updated**: December 2024












