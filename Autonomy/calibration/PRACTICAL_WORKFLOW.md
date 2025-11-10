# Complete Webcam Calibration & ArUco Distance Detection Workflow

## 🎯 Overview

This guide walks you through calibrating your webcam and then using that calibration to measure distances to ArUco tags in real-time.

**Total Time: ~15-30 minutes**
- Calibration: 10-15 minutes
- Testing: 5-15 minutes

---

## 📋 Prerequisites

### Hardware
- Webcam connected to your computer
- **Recommended**: Printed ChArUco board (8x6 pattern) for calibration
  - Already available: `charuco_board/charuco_board_100.pdf`
  - Can also be generated: See `charuco_board/generate_chessboard.py`
- Printed ArUco tags for distance testing
  - Available: `aruco_tags/aruco_sheets.pdf` or `aruco_tags/aruco_sheets_large.pdf`

### Software
```bash
pip install opencv-python numpy
```

---

## 🚀 Step 1: Calibrate Your Webcam

```bash
cd Autonomy/calibration/camera/

# Run the quick calibration script
python quick_calibration.py --duration 30 --output my_camera.json
```

**During calibration:**
1. Point your webcam at the ChArUco board
2. Slowly move the board around the camera view
3. Keep the board visible for at least 30 seconds
4. Hold steady - avoid jerky movements
5. Press 'q' to stop early or let it complete automatically

**Expected output:**
```
✅ Calibration successful!
💾 Calibration saved to: my_camera.json
📊 Calibration Results:
   Image size: 1280x720
   Frames used: 15
   Camera Matrix:
   [[...]]
   Distortion Coefficients:
   [...]
```

---

## 🎯 Step 2: Test ArUco Tag Distance Detection

### With Calibrated Camera

```bash
cd Autonomy/calibration/aruco_tags/

# Test with your calibrated camera
python aruco_validator.py \
  --calibration ../camera/my_camera.json \
  --tag-size 10.0

# Press 'q' to quit
```

### With Demo Calibration (Testing Only)

```bash
cd Autonomy/calibration/aruco_tags/

# Test with demo calibration
python aruco_validator.py \
  --calibration ../camera/demo_calibration.json \
  --tag-size 10.0

# Press 'q' to quit
```

### Test Mode (Automatic Duration)

```bash
# Run for exactly 5 seconds then exit
python aruco_validator.py \
  --calibration ../camera/demo_calibration.json \
  --tag-size 10.0 \
  --test-mode 5

# Also works headless (no display)
python aruco_validator.py \
  --calibration ../camera/demo_calibration.json \
  --test-mode 10 \
  --no-display
```

---

## 📊 What You'll See

### Real-Time Display

```
┌─────────────────────────────────────────┐
│  [Webcam Feed]                          │
│                                         │
│     ╔════════════════╗                 │
│     ║  ✓ ID:42       ║                 │
│     ║    30.5cm      ║                 │
│     ╚════════════════╝                 │
│                                         │
│  ─── Status Bar ───                    │
│  ArUco Tag Validator - URC 2026        │
│  Camera: Calibrated                    │
│  FPS: 29.8                             │
│  Frames: 145                           │
│  Detections: 3                         │
│  Avg Distance: 30.2cm                  │
└─────────────────────────────────────────┘
```

### Status Indicators

| Symbol | Meaning | Color |
|--------|---------|-------|
| ✓ | Good detection (>80% confidence) | Green |
| ~ | Moderate detection (50-80% confidence) | Yellow |
| ✗ | Poor detection (<50% confidence) | Red |

### Distance Information

- **ID**: ArUco tag identifier
- **Distance**: Measurement in centimeters (if calibrated)
- **FPS**: Frames per second processing speed
- **Detections**: Total tags detected in this session
- **Avg Distance**: Average distance to all detected tags

---

## 🔧 Troubleshooting

### Issue: "Failed to open camera"

**Solution**: Check camera access
```bash
python -c "
import cv2
for i in range(3):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f'Camera {i}: Available')
        cap.release()
    else:
        print(f'Camera {i}: Not available')
"
```

### Issue: No tags detected

**Possible causes:**
1. **Wrong tag size**: Use actual printed size
2. **Poor lighting**: Ensure good, even lighting
3. **Tag too small**: Should be at least 1/10th of frame
4. **Tag at angle**: Point camera directly at tag

**Solution:**
```bash
# Make sure you're using the correct tag size
# 1cm: --tag-size 1.0
# 10cm: --tag-size 10.0
# 20cm: --tag-size 20.0

python aruco_validator.py \
  --calibration ../camera/my_camera.json \
  --tag-size <actual_size_in_cm>
```

### Issue: Distance measurements seem wrong

**Likely causes:**
1. **Uncalibrated camera**: Use actual camera calibration
2. **Mismatched tag size**: Specify actual physical tag size
3. **Camera at angle**: Distance most accurate head-on

**Solution:**
```bash
# Verify calibration quality
python -c "
import json
with open('../camera/my_camera.json') as f:
    data = json.load(f)
print(f'Calibration Quality: {data[\"calibration_quality\"]}')
print(f'Reprojection Error: {data[\"reprojection_error\"]}')
"

# Recalibrate if error > 1.0
python quick_calibration.py --duration 60 --output my_camera.json
```

---

## 📁 Quick Reference

### File Locations

| File | Purpose |
|------|---------|
| `camera/quick_calibration.py` | Fast webcam calibration |
| `camera/demo_calibration.json` | Demo calibration for testing |
| `aruco_tags/aruco_validator.py` | Real-time detection & distance |
| `aruco_tags/test_aruco_detection.py` | Static image testing |
| `charuco_board/charuco_board_100.pdf` | ChArUco board for calibration |
| `aruco_tags/aruco_sheets.pdf` | ArUco tags for detection |

### Common Commands

```bash
# Calibrate webcam
cd Autonomy/calibration/camera/
python quick_calibration.py --duration 30 --output my_camera.json

# Test detection with calibration
cd Autonomy/calibration/aruco_tags/
python aruco_validator.py --calibration ../camera/my_camera.json --tag-size 10.0

# Test detection with demo (no calibration)
python aruco_validator.py --tag-size 10.0 --test-mode 10

# Batch test on static images
python test_aruco_detection.py --batch ../aruco_tags_png/ --save-results results.json

# Custom camera and settings
python aruco_validator.py \
  --camera 1 \
  --calibration ../camera/my_camera.json \
  --tag-size 10.0 \
  --dict DICT_5X5_100 \
  --save-video output.avi
```

---

## 🎓 Understanding the Results

### Camera Matrix

```
[fx   0  cx]
[ 0  fy  cy]
[ 0   0   1]
```

- **fx, fy**: Focal lengths (pixels)
- **cx, cy**: Principal point (image center)
- Higher values = more zoomed in

### Distortion Coefficients

`[k1, k2, p1, p2, k3]`

- **k1, k2, k3**: Radial distortion
- **p1, p2**: Tangential distortion
- Values closer to 0 = less distortion

### Reprojection Error

- **< 0.3**: Excellent calibration
- **< 0.5**: Good calibration
- **< 1.0**: Acceptable calibration
- **> 1.0**: Recalibrate recommended

---

## 🚀 Next Steps

### For Raspberry Pi Deployment

```bash
# Transfer calibration to Pi
scp Autonomy/calibration/camera/my_camera.json pi@raspberrypi:~/

# On Raspberry Pi
python aruco_validator.py \
  --calibration ./my_camera.json \
  --tag-size 10.0 \
  --no-display
```

### For ROS2 Integration

```bash
# Load calibration into ROS2 parameter server
ros2 run autonomy_calibration calibration_manager \
  --calibration my_camera.json

# Subscribe to detected tags topic
ros2 topic echo /aruco/detections
```

---

## 📞 Support

### Quick Fixes

| Problem | Fix |
|---------|-----|
| Camera not opening | Try different camera index: `--camera 0` (or 1, 2, etc.) |
| Low FPS | Reduce resolution or use faster CPU |
| Tags not detected | Improve lighting or increase tag size in frame |
| Distance wrong | Check tag size specified matches printed size |
| Calibration poor | Move board more during calibration, recapture more frames |

### Testing Tools

```bash
# Test camera detection on static images
python test_aruco_detection.py --image <image_path>

# Test single tag generation
python generate_aruco_tag.py --id 42 --size 10 --output test_tag.pdf

# Validate calibration quality
python calibration_validator.py --calibration my_camera.json
```

---

## ✅ Checklist

- [ ] Webcam connected and accessible
- [ ] Calibration completed or demo file exists
- [ ] ArUco tags printed (or use demo images)
- [ ] Quick validation test passed
- [ ] Distance measurements reasonable
- [ ] FPS performance acceptable

---

**🎉 You're ready to detect and measure ArUco tags with calibrated distances!**
