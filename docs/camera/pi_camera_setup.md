# Raspberry Pi Camera Setup Guide - URC 2026

Complete guide for setting up Raspberry Pi camera modules (CSI) and USB cameras on Raspberry Pi systems.

---

## Table of Contents

1. [Hardware Setup](#hardware-setup)
2. [Software Configuration](#software-configuration)
3. [Camera Interface Types](#camera-interface-types)
4. [Testing & Validation](#testing--validation)
5. [Troubleshooting](#troubleshooting)
6. [Advanced Configuration](#advanced-configuration)

---

## Hardware Setup

### Raspberry Pi Camera Module (CSI)

**Supported Models:**
- Raspberry Pi Camera v1 (OV5647 sensor, 5MP)
- Raspberry Pi Camera v2 (IMX219 sensor, 8MP)
- Raspberry Pi Camera v3 (IMX708 sensor, 12MP)
- Raspberry Pi HQ Camera (IMX477 sensor, 12.3MP)

**Physical Connection Steps:**

1. **Power down the Raspberry Pi** (important to prevent damage)
   ```bash
   sudo shutdown -h now
   ```

2. **Locate the CSI connector** on your Raspberry Pi
   - Raspberry Pi 4/5: Between the HDMI ports and the audio jack
   - Raspberry Pi Zero: Small connector on the board

3. **Connect the camera ribbon cable:**
   - Lift the plastic tab on the CSI connector
   - Insert the ribbon cable with the metal contacts facing away from the Ethernet/USB ports
   - The blue/teal side should face the USB ports
   - Press down the plastic tab to secure the cable

4. **Verify connection:**
   - Cable should be flush and secure
   - No exposed metal contacts visible

5. **Power on the Raspberry Pi**

### USB Cameras

**Supported Models:**
- Logitech C920, C930e, C270
- ArduCam USB cameras
- Any UVC (USB Video Class) compatible camera

**Connection Steps:**

1. **Connect USB camera** to any available USB port
2. **Use USB 3.0 ports** (blue) for better performance if available
3. **Avoid USB hubs** for critical applications (direct connection preferred)

---

## Software Configuration

### Method 1: Automated Setup (Recommended)

The project includes an automated setup script that handles all configuration:

```bash
# Generate and run setup script
python3 camera_validator.py --setup
sudo bash setup_cameras.sh
sudo reboot
```

**What the script does:**
- Installs required system packages (`v4l-utils`, `libv4l-dev`)
- Installs Python dependencies (`opencv-python`, `numpy`)
- Adds user to `video` group for camera permissions
- Enables camera interface via `raspi-config`
- Creates udev rules for consistent device naming

### Method 2: Manual Setup

If you prefer manual configuration or need to troubleshoot:

#### Step 1: Enable Camera Interface

For **Raspberry Pi Camera Modules (CSI)**, you must enable the camera interface:

```bash
# Interactive method
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable → Finish → Reboot

# Non-interactive method (from script)
sudo raspi-config nonint do_camera 0  # 0 = enable, 1 = disable
sudo reboot
```

**Why this matters:** The camera interface is disabled by default for security. Enabling it loads the necessary kernel modules and makes the camera available to applications.

#### Step 1.5: Enable Camera Auto-Detection (Multiple Cameras)

If you have **multiple CSI cameras** connected, enable auto-detection:

```bash
# Edit config file
CONFIG_FILE="/boot/firmware/config.txt"
[ ! -f "$CONFIG_FILE" ] && CONFIG_FILE="/boot/config.txt"

# Enable camera auto-detection
sudo sed -i 's/camera_auto_detect=0/camera_auto_detect=1/' "$CONFIG_FILE" 2>/dev/null || \
echo "camera_auto_detect=1" | sudo tee -a "$CONFIG_FILE" > /dev/null

# Verify the setting
grep camera_auto_detect "$CONFIG_FILE"
# Should show: camera_auto_detect=1

# Reboot for changes to take effect
sudo reboot
```

**Note:** `camera_auto_detect=1` tells the system to automatically detect and configure all connected CSI cameras. This is especially important when using multiple cameras.

#### Step 2: Install System Packages

```bash
sudo apt update
sudo apt install -y v4l-utils libv4l-dev build-essential
```

**Package explanations:**
- `v4l-utils`: Command-line tools for video device control (`v4l2-ctl`, `v4l2-compliance`)
- `libv4l-dev`: Development libraries for Video4Linux2 (used by OpenCV)
- `build-essential`: Compilation tools (may be needed for some camera drivers)

#### Step 2.5: Install libcamera (Raspberry Pi Camera Modules)

**When is libcamera needed?**
- **Required for:** Raspberry Pi Camera Modules (CSI) - v1, v2, v3, HQ Camera
- **Not needed for:** USB cameras (they use V4L2 directly)

**Installation:**

On **Raspberry Pi OS Bookworm** (recommended), libcamera is typically pre-installed. Verify and install if needed:

```bash
# Check if libcamera is already installed
# Note: On newer Raspberry Pi OS, commands use 'rpicam-' prefix instead of 'libcamera-'
rpicam-hello --version || libcamera-hello --version

# If not installed, install libcamera applications
sudo apt install -y rpicam-apps libcamera-tools python3-picamera2
```

**What gets installed:**
- `rpicam-apps` (or `libcamera-apps` on older systems): Command-line tools
  - **Newer systems:** `rpicam-still`, `rpicam-vid`, `rpicam-hello`
  - **Older systems:** `libcamera-still`, `libcamera-vid`, `libcamera-hello`
- `libcamera-tools`: Additional utilities
- `python3-picamera2`: Python interface to libcamera (modern replacement for picamera)

**Verify installation:**
```bash
# Test camera access (try new command first, fallback to old)
rpicam-hello -t 2000 || libcamera-hello -t 2000
# Should show camera preview for 2 seconds

# List available cameras
rpicam-still --list-cameras || rpicam-still --list-cameras || libcamera-still --list-cameras

# Capture a test image
rpicam-still -o test.jpg || libcamera-still -o test.jpg
```

**Why libcamera matters:**
- Modern camera stack for Raspberry Pi (replaced legacy `raspistill`/`raspivid`)
- Better performance and features than legacy tools
- Required for sensor detection in this project (`detect_camera_sensor()` function)
- Provides hardware-accelerated encoding (H.264) for video streaming

**Note:** If you're using **only USB cameras**, you can skip libcamera installation. However, it's recommended to install it anyway for future flexibility.

#### Step 3: Install Python Dependencies

```bash
# System-wide installation
pip3 install opencv-python numpy

# Or in virtual environment (recommended for development)
python3 -m venv ~/camera_env
source ~/camera_env/bin/activate
pip install opencv-python numpy
```

**Note:** For headless systems (no display), use `opencv-python-headless` instead:
```bash
pip3 install opencv-python-headless numpy
```

#### Step 4: Configure Permissions

Add your user to the `video` group to access camera devices:

```bash
sudo usermod -a -G video $USER
```

**Important:** You must **logout and login again** (or reboot) for group membership to take effect. Simply opening a new terminal is not sufficient.

Verify membership:
```bash
groups
# Should show 'video' in the list
```

#### Step 5: Reboot

```bash
sudo reboot
```

Rebooting ensures:
- Camera interface is enabled
- Group membership is active
- Kernel modules are loaded
- udev rules are applied

---

## Camera Interface Types

### CSI Camera (Raspberry Pi Camera Module)

**Device Path:** `/dev/video0` (or higher if multiple cameras)

**Detection:**
```bash
# Check if camera is detected
vcgencmd get_camera
# Should show: supported=1 detected=1

# List video devices
ls -l /dev/video*
# Should show: /dev/video0 (and possibly /dev/video10, /dev/video11 for H.264 encoder)
```

**Using libcamera (modern approach):**
```bash
# Test camera with libcamera
rpicam-still -o test.jpg || libcamera-still -o test.jpg
libcamera-vid -t 5000 -o test.h264

# List available cameras
rpicam-still --list-cameras || libcamera-still --list-cameras
```

**Using OpenCV:**
```python
import cv2
cap = cv2.VideoCapture(0)  # Index 0 for first camera
ret, frame = cap.read()
```

### USB Camera

**Device Path:** `/dev/video0`, `/dev/video1`, etc. (depends on connection order)

**Detection:**
```bash
# List all video devices
ls -l /dev/video*

# Get detailed information
v4l2-ctl --list-devices
# Shows: device name, driver, capabilities

# Check device capabilities
v4l2-ctl --device=/dev/video0 --all
```

**Using OpenCV:**
```python
import cv2
# Try different indices if multiple cameras
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera found at index {i}")
        ret, frame = cap.read()
        if ret:
            print(f"Camera {i} working!")
        cap.release()
```

---

## Testing & Validation

### Quick System Check

```bash
# Run the project's camera check script
python3 camera_check.py
```

Expected output:
```
System ready for camera calibration!
```

### Comprehensive Validation

```bash
# Auto-detect all cameras
python3 camera_validator.py --auto-detect
```

Expected output:
```
Camera 0: READY
  Resolution: 1280x720
  FPS: 30
  Sensor: IMX219
```

### Manual Testing

#### Test 1: Device Detection
```bash
# Check if video devices exist
ls /dev/video*

# Should show at least /dev/video0
```

#### Test 2: Camera Access
```bash
# Test with v4l2-ctl
v4l2-ctl --device=/dev/video0 --list-formats

# Capture a test image (if libcamera available)
rpicam-still -o test.jpg || libcamera-still -o test.jpg
```

#### Test 3: OpenCV Access
```python
import cv2

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("ERROR: Cannot open camera")
else:
    print("SUCCESS: Camera opened")
    ret, frame = cap.read()
    if ret:
        print(f"SUCCESS: Frame captured ({frame.shape})")
    else:
        print("ERROR: Cannot read frame")
    cap.release()
```

#### Test 4: Calibration Test
```bash
# Run a quick calibration test
python3 quick_calibration.py --camera 0
```

---

## Troubleshooting

### Issue: "Permission denied" when accessing camera

**Symptoms:**
- `PermissionError` or `IOError` when opening camera
- `v4l2-ctl` shows permission denied

**Solution:**
```bash
# Add user to video group
sudo usermod -a -G video $USER

# Verify membership
groups

# Reboot or logout/login
sudo reboot
```

**Why this happens:** Camera devices (`/dev/video*`) are owned by the `video` group. Your user must be a member to access them.

---

### Issue: "No cameras detected" or "Cannot open camera"

**Symptoms:**
- `ls /dev/video*` shows no devices
- OpenCV `VideoCapture` returns `False` for `isOpened()`
- `vcgencmd get_camera` shows `detected=0`

**Solutions:**

1. **Check physical connection:**
   ```bash
   # For CSI cameras, verify ribbon cable is secure
   # Power cycle the Pi after connecting
   ```

2. **Verify camera interface is enabled:**
   ```bash
   # Check current status
   raspi-config nonint get_camera
   # 0 = enabled, 1 = disabled
   
   # Enable if disabled
   sudo raspi-config nonint do_camera 1
   sudo reboot
   ```

3. **Check kernel modules:**
   ```bash
   # For CSI cameras
   lsmod | grep bcm2835
   # Should show bcm2835-v4l2 or similar
   
   # For USB cameras
   lsusb
   # Should list your USB camera device
   ```

4. **Try different USB port** (for USB cameras)

5. **Check dmesg for errors:**
   ```bash
   dmesg | tail -20
   # Look for camera-related errors
   ```

---

### Issue: "OpenCV not available" or "Module not found"

**Symptoms:**
- `ImportError: No module named 'cv2'`
- OpenCV functions fail

**Solution:**
```bash
# Install OpenCV
pip3 install opencv-python

# For headless systems
pip3 install opencv-python-headless

# Verify installation
python3 -c "import cv2; print(cv2.__version__)"
```

---

### Issue: Camera works but produces black/blank frames

**Symptoms:**
- Camera opens successfully
- `cap.read()` returns `True` but frame is all black
- No image data captured

**Solutions:**

1. **Check camera focus/exposure:**
   ```python
   import cv2
   cap = cv2.VideoCapture(0)
   cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual exposure
   cap.set(cv2.CAP_PROP_EXPOSURE, -6)  # Adjust exposure value
   ```

2. **Wait for camera initialization:**
   ```python
   import time
   cap = cv2.VideoCapture(0)
   time.sleep(2)  # Give camera time to initialize
   ret, frame = cap.read()
   ```

3. **Try different resolution:**
   ```python
   cap = cv2.VideoCapture(0)
   cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
   ```

4. **Check lighting conditions** (camera may need more light)

---

### Issue: Multiple cameras - wrong camera selected

**Symptoms:**
- Multiple cameras connected but wrong one opens
- Camera indices inconsistent

**Solution:**
```bash
# List all video devices with details
v4l2-ctl --list-devices

# This shows the mapping:
# /dev/video0: USB Camera (Logitech)
# /dev/video1: USB Camera (ArduCam)
# /dev/video10: bcm2835-codec (CSI camera encoder)

# Use the correct index in your code
cap = cv2.VideoCapture(1)  # Use index 1 for second camera
```

**Note:** Device indices can change based on connection order. Consider using device paths or udev rules for consistent naming.

---

### Issue: libcamera not found or "command not found"

**Symptoms:**
- `libcamera-hello: command not found` or `rpicam-hello: command not found`
- `libcamera-still: command not found` or `rpicam-still: command not found`

**Note:** On newer Raspberry Pi OS (Bookworm+), commands use the `rpicam-*` prefix. On older systems, they use `libcamera-*`. Try the `rpicam-*` commands first.
- Sensor detection fails in `camera_validator.py`

**Solutions:**

1. **Install libcamera applications:**
   ```bash
   sudo apt update
   sudo apt install -y libcamera-apps libcamera-tools python3-picamera2
   ```

2. **Verify installation:**
   ```bash
   # Check for new command first, then old
   which rpicam-hello || which libcamera-hello
   # Should show: /usr/bin/rpicam-hello (newer) or /usr/bin/libcamera-hello (older)

   rpicam-hello --version || libcamera-hello --version
   # Should show version information
   ```

3. **Check if camera interface is enabled:**
   ```bash
   # libcamera requires camera interface to be enabled
   sudo raspi-config nonint do_camera 1
   sudo reboot
   ```

4. **For older Raspberry Pi OS versions:**
   If you're on an older OS (Buster or earlier), you may need to upgrade to Bullseye or Bookworm, as libcamera is not available on older versions.

**Why this happens:** libcamera is the modern camera stack for Raspberry Pi. It's pre-installed on Raspberry Pi OS Bullseye (2021) and later, but may be missing on older installations or minimal images.

---

### Issue: Low frame rate or performance issues

**Symptoms:**
- Camera works but FPS is very low
- High CPU usage
- Dropped frames

**Solutions:**

1. **Reduce resolution:**
   ```python
   cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
   ```

2. **Use hardware acceleration** (if available):
   ```bash
   # For CSI cameras, use libcamera directly
   libcamera-vid -t 0 --codec h264 --inline -o - | cv2.VideoCapture(...)
   ```

3. **Check USB bandwidth** (for USB cameras):
   - Use USB 3.0 ports if available
   - Avoid USB hubs
   - Disconnect other USB devices if possible

4. **Optimize OpenCV capture:**
   ```python
   # Use threading for better performance
   from threading import Thread
   
   class VideoStream:
       def __init__(self, src=0):
           self.cap = cv2.VideoCapture(src)
           self.ret, self.frame = self.cap.read()
           self.stopped = False
       
       def start(self):
           Thread(target=self.update, args=()).start()
           return self
       
       def update(self):
           while not self.stopped:
               self.ret, self.frame = self.cap.read()
   ```

---

## Advanced Configuration

### Consistent Device Naming with udev Rules

Create custom device names that don't change based on connection order:

```bash
# Create udev rule
sudo nano /etc/udev/rules.d/99-camera.rules
```

Add rule (example for USB camera):
```
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0825", SYMLINK+="camera_logitech"
```

Find vendor/product IDs:
```bash
lsusb
# Find your camera, note the ID (e.g., 046d:0825)
```

Reload rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Now access camera via:
```python
cap = cv2.VideoCapture("/dev/camera_logitech")
```

### Camera-Specific Settings

#### Logitech C920/C930e
```python
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus for consistent calibration
```

#### Raspberry Pi Camera v2 (IMX219)
```python
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1232)  # Native resolution
cap.set(cv2.CAP_PROP_FPS, 30)
```

### Multiple Camera Setup

For systems with multiple cameras:

```python
import cv2

cameras = []
for i in range(4):  # Test up to 4 cameras
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            cameras.append((i, cap))
            print(f"Camera {i} ready: {frame.shape}")
        else:
            cap.release()
    else:
        break

print(f"Found {len(cameras)} working cameras")
```

---

## Verification Checklist

After setup, verify all items:

- [ ] Camera physically connected (CSI ribbon secure or USB connected)
- [ ] Camera interface enabled (`raspi-config` or `vcgencmd get_camera` shows `detected=1`)
- [ ] System packages installed (`v4l2-ctl --version` works)
- [ ] Python dependencies installed (`python3 -c "import cv2"` works)
- [ ] User in video group (`groups` shows `video`)
- [ ] System rebooted after permission changes
- [ ] Video device exists (`ls /dev/video*` shows devices)
- [ ] Camera opens in OpenCV (`cv2.VideoCapture(0).isOpened()` returns `True`)
- [ ] Frames can be captured (`cap.read()` returns valid frame)
- [ ] Validation script passes (`python3 camera_validator.py --auto-detect`)

---

## Next Steps

Once your camera is set up and verified:

1. **Run calibration:** `python3 quick_calibration.py --camera 0`
2. **Check calibration quality:** `python3 check_calibration.py`
3. **Use in your application:** Load calibration files and use for computer vision tasks

---

## Additional Resources

- [Raspberry Pi Camera Documentation](https://www.raspberrypi.com/documentation/computers/camera_software.html)
- [OpenCV Camera Documentation](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html)
- [Video4Linux2 Documentation](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html)

---

**Status**: Production Ready  
**Tested on**: Raspberry Pi 4B/5 with Raspberry Pi OS Bookworm  
**Last Updated**: December 2024

