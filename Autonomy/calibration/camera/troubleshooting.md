# 🔍 Troubleshooting Guide - Common Issues & Solutions

**Comprehensive troubleshooting for camera calibration problems with visual diagnostics.**

---

## 📋 Quick Reference

| Issue | Quick Fix | Section |
|-------|-----------|---------|
| "Cannot open camera" | Check USB connection | [Camera Connection](#camera-connection-issues) |
| "Need at least 5 frames" | Move board more | [Insufficient Frames](#insufficient-calibration-frames) |
| Distance values wrong | Check `--tag-size` units | [Distance Measurement](#distance-measurement-issues) |
| Poor calibration quality | Improve lighting/motion | [Calibration Quality](#calibration-quality-issues) |
| Multi-camera conflicts | Use sequential mode | [Multi-Camera Issues](#multi-camera-issues) |

---

## 📷 Camera Connection Issues

### Issue: "Cannot open camera X"

#### Symptoms
```
❌ Cannot open camera 0
❌ Cannot open camera 1
[ERROR] cv2.VideoCapture(0) failed
```

#### Root Causes & Solutions

##### **Cause 1: Camera Not Connected**
```bash
# Check connected cameras (Linux)
ls /dev/video*

# Expected output:
/dev/video0
/dev/video1
/dev/video2
/dev/video3
/dev/video4

# If missing, check USB connections
lsusb | grep -i camera
```

**Solution:**
1. Ensure cameras are properly connected
2. Try different USB ports
3. Use powered USB hub if needed
4. Check camera power requirements

##### **Cause 2: Permission Issues**
```bash
# Check permissions (Linux)
ls -la /dev/video0

# If no permissions, add user to video group
sudo usermod -a -G video $USER
# Then logout/login or run: newgrp video
```

##### **Cause 3: Camera Already in Use**
```bash
# Check what processes are using cameras
lsof /dev/video* 2>/dev/null || echo "No processes using cameras"

# Kill competing processes
pkill -f "python.*video"
pkill -f "opencv"
```

##### **Cause 4: Wrong Camera Index**
```bash
# List all cameras with info
v4l2-ctl --list-devices

# Test specific camera
python -c "
import cv2
for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f'Camera {i}: OK')
        cap.release()
    else:
        print(f'Camera {i}: FAIL')
"
```

**Solution:** Use correct camera index in commands.

### Issue: Camera Opens But Shows Black/Static

#### Symptoms
- Camera opens successfully
- Window shows black screen or static
- No video feed visible

#### Solutions

##### **Adjust Camera Settings**
```bash
# Set camera resolution explicitly
python calibrate_from_markers.py --camera-width 1280 --camera-height 720

# Try different resolutions
python calibrate_from_markers.py --camera-width 640 --camera-height 480
```

##### **Check Camera Controls**
```bash
# List camera controls
v4l2-ctl -d /dev/video0 --list-ctrls

# Adjust brightness/exposure
v4l2-ctl -d /dev/video0 --set-ctrl brightness=128
v4l2-ctl -d /dev/video0 --set-ctrl exposure_auto=1
```

##### **Test with Simple Viewer**
```bash
# Test with basic OpenCV viewer
python -c "
import cv2
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('Test', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
cv2.destroyAllWindows()
cap.release()
"
```

---

## 🎯 Calibration Quality Issues

### Issue: "Need at least 5 frames" or "Calibration failed"

#### Symptoms
```
❌ Need at least 5 frames, got 2
❌ Calibration failed
⚠️  Only 3 good frames collected
```

#### Root Cause: Insufficient Marker Detection

##### **Lighting Problems**
```
✅ Good: Even, diffuse lighting
❌ Bad: Harsh shadows, direct sunlight
❌ Bad: Low light, poor contrast
```

**Solutions:**
- Use LED panel lights or natural diffuse lighting
- Avoid direct sunlight through windows
- Ensure board is well-lit from all angles
- Increase room lighting

##### **Board Movement Issues**
```
✅ Good: Smooth, continuous motion
✅ Good: Covers entire field of view
❌ Bad: Jerky movements
❌ Bad: Only moves in one plane
❌ Bad: Too fast/too slow
```

**Movement Pattern:**
```
1. Start centered, face-on to camera
2. Rotate ±30° in all directions
3. Move closer (0.3m) and farther (1.2m)
4. Tilt at various angles (0°, 30°, 45°, 60°)
5. Cover all corners of camera view
6. Maintain smooth motion for full duration
```

##### **Board Quality Issues**
```
✅ Good: High-contrast print
✅ Good: Rigid mounting
❌ Bad: Faded/reflective paper
❌ Bad: Flexible/hand-held
❌ Bad: Low resolution print
```

**Board Checks:**
```bash
# Verify board exists and is readable
ls -la test_board.pdf

# Print at high quality (600+ DPI recommended)
# Use matte paper, not glossy
# Mount rigidly (no flexing)
```

##### **Camera Focus/Angle Issues**
```
✅ Good: Board fills 1/4 to 1/2 of frame
✅ Good: All markers clearly visible
❌ Bad: Board too small (< 10% of frame)
❌ Bad: Board too large (> 80% of frame)
❌ Bad: Markers blurred/unfocused
```

**Camera Setup:**
- Focus camera properly on board
- Position board at appropriate distance
- Ensure camera lens is clean
- Check for autofocus issues

### Issue: Poor Reprojection Error (> 1.0 pixels)

#### Symptoms
```
Calibration Results:
Reprojection Error: 2.34 pixels (poor)
Quality: poor
```

#### Causes & Solutions

##### **Inconsistent Board Position**
- Board moved during capture
- Hand-held instead of mounted
- Vibration from movement

**Solution:** Mount board rigidly on stable surface

##### **Lens Distortion**
- Fisheye lens effects
- Poor quality camera lens
- Lens not properly focused

**Solution:** Use camera with good lens, ensure proper focus

##### **Timing Issues**
- Not enough time for stable captures
- Too fast movement
- Frame drops during capture

**Solution:** Increase `--duration`, slow down board movement

---

## 📏 Distance Measurement Issues

### Issue: Distance Values Unrealistic

#### Symptoms
```
ID:0 45000.5mm ❌
ID:4 0.02mm ❌
Distance: -1250.3mm ❌
```

#### Common Causes

##### **Wrong Tag Size Units**
```bash
# WRONG - millimeters interpreted as meters
python detect_with_multiple_calibrations.py --tag-size 18

# CORRECT - specify millimeters
python detect_with_multiple_calibrations.py --tag-size 18  # 18mm
```

**Tag Size Reference:**
- **18mm**: Standard ArUco tags in this project
- **20mm**: Common industrial tags
- **50mm**: Large outdoor tags
- **Always in millimeters!**

##### **Incorrect Board Parameters**
```bash
# Check board matches calibration
python calibrate_from_markers.py \
  --cols 7 --rows 5 \
  --square-size 0.030 \
  --marker-size 0.018  # Must match actual board!
```

##### **Calibration Quality Issues**
- Poor calibration reprojection error
- Insufficient calibration frames
- Wrong camera matrix

**Solution:** Re-run calibration with better conditions

### Issue: Inconsistent Distance Readings

#### Symptoms
- Same tag shows different distances
- Measurements jump erratically
- Distance changes when camera doesn't move

#### Causes & Solutions

##### **Pose Estimation Instability**
- Tag partially occluded
- Poor lighting on tag
- Tag at extreme angle

**Solution:** Ensure full tag visibility, good lighting

##### **Coordinate System Issues**
- Different coordinate systems between calibrations
- Hand-eye calibration not performed
- Base frame misalignment

**Solution:** Verify coordinate system consistency

##### **Numerical Instability**
- Very close or very far distances
- Extreme viewing angles
- Poor camera calibration

**Solution:** Operate within calibrated range (0.2-2.0m typical)

---

## 🎬 Multi-Camera Issues

### Issue: Cameras Conflict During Calibration

#### Symptoms
- Only first camera works
- Subsequent cameras fail to open
- "Device or resource busy" errors

#### Solutions

##### **Sequential Calibration (Recommended)**
```bash
# Use the multi-camera script (handles sequencing automatically)
python calibrate_multiple_cameras.py --num-cameras 5

# Script automatically:
# 1. Calibrates camera 0
# 2. Prompts to swap cameras
# 3. Calibrates camera 1
# 4. Repeats for all cameras
```

##### **Manual Sequential Approach**
```bash
# Calibrate each camera individually
for cam in {0..4}; do
    python calibrate_from_markers.py --camera $cam --output camera_$cam.json
    echo "Swap to camera $((cam+1)) and press ENTER"
    read
done
```

##### **USB Port Management**
```bash
# Use different USB ports for each camera
# Avoid USB hubs initially
# Use powered USB hubs if needed
# Ensure sufficient power for all cameras
```

### Issue: Parallel Detection Fails

#### Symptoms
- Cameras work individually
- Fail when running simultaneously
- Frame drops or corrupted frames

#### Solutions

##### **Memory/Resource Issues**
```bash
# Check system resources
top -p $(pgrep -f detect_with_multiple)
free -h

# Reduce resolution if needed
python detect_with_multiple_calibrations.py \
  --calibration-dir ./calibrations \
  --camera-width 640 \
  --camera-height 480
```

##### **USB Bandwidth Issues**
- Too many cameras on same USB controller
- Insufficient USB bandwidth
- USB 2.0 vs 3.0 differences

**Solutions:**
- Distribute cameras across multiple USB controllers
- Use USB 3.0 ports when available
- Reduce frame rate or resolution

##### **Use Sequential Mode**
```bash
# Switch to sequential processing
python detect_with_multiple_calibrations.py \
  --calibration-dir ./calibrations \
  --sequential \
  --tag-size 18
```

### Issue: Camera Index Changes

#### Symptoms
- Camera indices change after reboot
- `/dev/video0` becomes `/dev/video2`
- Calibration files don't match physical cameras

#### Solutions

##### **Persistent Device Names**
```bash
# Use udev rules for persistent naming (Linux)
sudo nano /etc/udev/rules.d/99-camera.rules

# Add rules like:
SUBSYSTEM=="video4linux", ATTR{idVendor}=="046d", ATTR{idProduct}=="0825", SYMLINK+="camera_front"
SUBSYSTEM=="video4linux", ATTR{idVendor}=="046d", ATTR{idProduct}=="0826", SYMLINK+="camera_rear"
```

##### **Camera Serial Number Mapping**
```bash
# Map by serial number instead of index
python calibrate_by_serial.py --serial ABC123 --output front_camera.json
```

##### **Document Physical Positions**
```bash
# Create mapping file
cat > camera_mapping.txt << EOF
Physical Position | Camera Index | Purpose
-----------------|-------------|---------
Front Left       | 0           | Navigation
Front Right      | 1           | Obstacle detection
Rear             | 2           | Following
Left Side        | 3           | Terrain analysis
Right Side       | 4           | Terrain analysis
EOF
```

---

## 🔧 Advanced Diagnostics

### Calibration Quality Analysis

#### Detailed Quality Report
```python
import json
import numpy as np

def analyze_calibration_quality(calib_file):
    with open(calib_file) as f:
        calib = json.load(f)

    # Extract parameters
    camera_matrix = np.array(calib['camera_matrix']['data']).reshape(3, 3)
    dist_coeffs = np.array(calib['distortion_coefficients']['data'])
    frames_used = calib['frames_used']

    # Quality metrics
    focal_length = (camera_matrix[0,0] + camera_matrix[1,1]) / 2
    principal_point = camera_matrix[:2, 2]
    distortion_magnitude = np.linalg.norm(dist_coeffs)

    print(f"Frames Used: {frames_used}")
    print(f"Focal Length: {focal_length:.1f} pixels")
    print(f"Principal Point: ({principal_point[0]:.1f}, {principal_point[1]:.1f})")
    print(f"Distortion Magnitude: {distortion_magnitude:.4f}")

    # Quality assessment
    if frames_used >= 20 and distortion_magnitude < 1.0:
        print("Quality: EXCELLENT")
    elif frames_used >= 15 and distortion_magnitude < 2.0:
        print("Quality: GOOD")
    elif frames_used >= 10:
        print("Quality: FAIR")
    else:
        print("Quality: POOR - Recalibrate")

# Usage
analyze_calibration_quality('camera_0_calibration.json')
```

### Performance Profiling

#### FPS and Latency Analysis
```python
import time
import cv2

def profile_camera_performance(camera_index=0, duration=10):
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    frames_captured = 0
    start_time = time.time()

    while time.time() - start_time < duration:
        ret, frame = cap.read()
        if ret:
            frames_captured += 1

    cap.release()

    elapsed = time.time() - start_time
    fps = frames_captured / elapsed

    print(f"Duration: {elapsed:.1f}s")
    print(f"Frames: {frames_captured}")
    print(f"FPS: {fps:.1f}")

    if fps >= 25:
        print("Performance: EXCELLENT")
    elif fps >= 15:
        print("Performance: GOOD")
    else:
        print("Performance: POOR - Check camera/connection")

profile_camera_performance(0)
```

### System Health Check

#### Complete Diagnostic Script
```bash
#!/bin/bash
echo "=== Camera Calibration System Health Check ==="
echo

# Check system resources
echo "📊 System Resources:"
free -h | head -2
echo "CPU Load: $(uptime | awk '{print $NF}')"
echo

# Check cameras
echo "📷 Connected Cameras:"
ls /dev/video* 2>/dev/null || echo "No cameras found"
echo

# Check USB devices
echo "🔌 USB Devices:"
lsusb | grep -i camera || echo "No camera USB devices found"
echo

# Check processes
echo "⚙️  Running Camera Processes:"
pgrep -f "python.*calibrat|opencv|video" || echo "No camera processes running"
echo

# Check disk space
echo "💾 Disk Space:"
df -h . | tail -1
echo

echo "=== Health Check Complete ==="
```

---

## 🚑 Emergency Recovery

### Complete System Reset
```bash
# 1. Kill all camera processes
pkill -9 -f "python.*calibrat"
pkill -9 -f "opencv"
pkill -9 -f "video"

# 2. Reset USB devices
sudo modprobe -r uvcvideo
sudo modprobe uvcvideo

# 3. Clear calibration cache
rm -rf __pycache__
rm -rf .pytest_cache

# 4. Test basic camera access
python -c "import cv2; print('OpenCV version:', cv2.__version__)"
python -c "import cv2; cap = cv2.VideoCapture(0); print('Camera 0:', 'OK' if cap.isOpened() else 'FAIL'); cap.release()"
```

### Fresh Start Checklist
- [ ] Unplug and reconnect all cameras
- [ ] Restart computer if needed
- [ ] Verify camera permissions (`ls -la /dev/video*`)
- [ ] Test individual cameras with simple script
- [ ] Clear old calibration files
- [ ] Start with single camera calibration first
- [ ] Gradually add complexity (multi-camera, etc.)

---

## 📞 Getting Help

### Diagnostic Information to Provide

When reporting issues, include:

```bash
# System information
uname -a
lsb_release -a 2>/dev/null || cat /etc/os-release

# Python/OpenCV versions
python --version
python -c "import cv2; print('OpenCV:', cv2.__version__)"

# Camera information
ls /dev/video*
v4l2-ctl --list-devices

# USB information
lsusb | head -10

# Error logs (if available)
# Copy the full error output
```

### Community Resources

- **GitHub Issues**: Report bugs with full diagnostic info
- **Wiki**: Extended troubleshooting guides
- **Discussions**: Community solutions for common problems
- **Documentation**: Updated troubleshooting procedures

---

**🎯 Remember:** Most calibration issues stem from three root causes:
1. **Poor lighting** → Ensure even, diffuse illumination
2. **Insufficient board movement** → Follow the recommended motion pattern
3. **Camera connection problems** → Verify hardware and permissions

*"Debugging is twice as hard as writing code in the first place."* - Brian Kernighan
