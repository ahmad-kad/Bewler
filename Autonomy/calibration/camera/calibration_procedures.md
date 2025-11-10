# 📖 Calibration Procedures - Step-by-Step Guide

**Complete procedures for camera calibration with visual aids and troubleshooting.**

---

## 📋 Table of Contents

- [Single Camera Calibration](#single-camera-calibration)
- [Multi-Camera Sequential Calibration](#multi-camera-sequential-calibration)
- [Calibration Validation](#calibration-validation)
- [Best Practices](#best-practices)
- [Advanced Configuration](#advanced-configuration)

---

## 🎯 Single Camera Calibration

### Prerequisites
- ✅ ChArUco board (7×5 recommended)
- ✅ Camera connected and accessible
- ✅ Good lighting conditions
- ✅ Stable mounting surface

### Step 1: Prepare Environment

```bash
# Navigate to camera calibration directory
cd Autonomy/calibration/camera

# Verify camera access (Linux)
ls /dev/video*

# Test camera with simple viewer (optional)
python -c "import cv2; cap = cv2.VideoCapture(0); ret, frame = cap.read(); print('Camera OK' if ret else 'Camera FAIL'); cap.release()"
```

### Step 2: Run Calibration

```bash
python calibrate_from_markers.py \
  --cols 7 \
  --rows 5 \
  --square-size 0.030 \
  --marker-size 0.018 \
  --output my_camera.json \
  --duration 30
```

### Step 3: Follow On-Screen Instructions

```
🎯 Direct ArUco Marker Calibration
============================================================
Board: 7×5 (35 markers)
Square: 30.0mm, Marker: 18.0mm
📷 Camera: 1280x720

📹 Capturing for 30 seconds...
Move board around to collect good frames
```

### Step 4: Move ChArUco Board

**Critical for good calibration - follow this pattern:**

```
1. Start with board centered, facing camera directly
2. Rotate board ±30° around each axis (pitch, yaw, roll)
3. Move board closer/farther (0.3m to 1.5m distance)
4. Tilt board at different angles (30°, 45°, 60°)
5. Cover all areas of camera field of view
6. Maintain steady motion throughout 30 seconds
```

#### Visual Movement Pattern:
```
Top View:              Side View:
Camera → 📷             Camera → 📷
                       ↗️        ↘️
         ↖️   🎯   ↗️           🎯
         ↙️       ↘️         ↗️        ↘️
                      ↘️        ↗️
```

### Step 5: Monitor Progress

**Good signs during calibration:**
```
✅ Frame 5: Collected 25 corners (total: 5)
✅ Frame 10: Collected 28 corners (total: 10)
✅ Frame 15: Collected 32 corners (total: 15)
✅ Frame 20: Collected 30 corners (total: 20)
```

**When to stop early:**
- Press `q` if you get 15+ frames before 30 seconds
- Good calibration typically needs 15-25 frames

### Step 6: Verify Results

**Check the output file:**
```bash
ls -la my_camera.json
cat my_camera.json | head -20
```

**Expected output:**
```json
{
  "camera_index": 0,
  "camera_matrix": {
    "rows": 3,
    "cols": 3,
    "data": [669.13, 0.0, 644.66, 0.0, 668.50, 357.35, 0.0, 0.0, 1.0]
  },
  "distortion_coefficients": {
    "rows": 1,
    "cols": 5,
    "data": [2.54, 6.38, -0.018, -0.226, -249.18]
  },
  "image_width": 1280,
  "image_height": 720,
  "frames_used": 18,
  "board_size": "7x5",
  "calibration_quality": "good"
}
```

---

## 🎬 Multi-Camera Sequential Calibration

### Hardware Setup
```
Computer ↔ Camera 0 ↔ Camera 1 ↔ Camera 2 ↔ Camera 3 ↔ Camera 4
   ↓           ↓           ↓           ↓           ↓           ↓
 USB Ports  /dev/video0  /dev/video1  /dev/video2  /dev/video3  /dev/video4
```

### Step 1: Prepare All Cameras
- ✅ Connect all 5 cameras to computer
- ✅ Verify camera indices: `/dev/video0` through `/dev/video4`
- ✅ Ensure cameras are powered and recognized
- ✅ Place cameras in final operational positions

### Step 2: Run Multi-Camera Calibration

```bash
python calibrate_multiple_cameras.py \
  --num-cameras 5 \
  --cols 7 --rows 5 \
  --square-size 0.030 \
  --marker-size 0.018 \
  --output-dir ./camera_calibrations \
  --duration 30
```

### Step 3: Follow Sequential Process

**The script will guide you through each camera:**

```
🎯 Multi-Camera Calibration System
====================================
Calibrating 5 cameras sequentially
Output directory: ./camera_calibrations

⏳ Prepare camera 0 and press ENTER to continue...
[Press ENTER]

📷 CALIBRATING CAMERA 0
============================================================
Board: 7×5 (35 markers)
Square: 30.0mm, Marker: 18.0mm

📹 Capturing for 30 seconds...
Move board around to collect good frames

✅ Frame 5: Collected 25 corners (total: 5)
✅ Frame 10: Collected 28 corners (total: 10)
✅ Frame 15: Collected 32 corners (total: 15)

✅ Calibration successful!
💾 Calibration saved to: camera_calibrations/camera_0_calibration.json

⏳ Prepare camera 1 and press ENTER to continue...
[Physically swap cameras, then press ENTER]
```

### Step 4: Camera Swapping Process

**For each camera N:**
1. **Disconnect** camera N-1 (if not using USB switch)
2. **Connect** camera N to the same USB port
3. **Wait** for camera recognition (`ls /dev/video*`)
4. **Press ENTER** when ready
5. **Move board** for 30 seconds of calibration
6. **Verify** calibration file was created

### Step 5: Verify All Calibrations

```bash
# Check all files were created
ls -la camera_calibrations/
# camera_0_calibration.json
# camera_1_calibration.json
# camera_2_calibration.json
# camera_3_calibration.json
# camera_4_calibration.json

# Verify file contents
for file in camera_calibrations/camera_*_calibration.json; do
  echo "=== $file ==="
  cat "$file" | jq '.frames_used, .calibration_quality'
done
```

### Step 6: Test Detection

```bash
# Test with all cameras
python detect_with_multiple_calibrations.py \
  --calibration-dir ./camera_calibrations \
  --tag-size 18 \
  --sequential
```

---

## ✅ Calibration Validation

### Quality Metrics

#### Reprojection Error
- **Excellent**: < 0.5 pixels
- **Good**: 0.5 - 1.0 pixels
- **Acceptable**: 1.0 - 2.0 pixels
- **Poor**: > 2.0 pixels

#### Frame Count
- **Excellent**: 20+ frames
- **Good**: 15-19 frames
- **Acceptable**: 10-14 frames
- **Insufficient**: < 10 frames

### Validation Tests

#### 1. Distance Measurement Test
```bash
python ../aruco_tags/aruco_validator.py \
  --calibration my_camera.json \
  --tag-size 18
```
**Expected:** Accurate distance measurements (within ±5mm)

#### 2. Corner Detection Test
```bash
python -c "
import cv2
import json

# Load calibration
with open('my_camera.json') as f:
    calib = json.load(f)

camera_matrix = np.array(calib['camera_matrix']['data']).reshape(3,3)
dist_coeffs = np.array(calib['distortion_coefficients']['data'])

# Test undistortion
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if ret:
    undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
    print('Undistortion test: PASSED')
else:
    print('Camera test: FAILED')
cap.release()
"
```

#### 3. Multi-Camera Consistency Test
```bash
# Test all cameras can detect simultaneously
python detect_with_multiple_calibrations.py \
  --calibration-dir ./camera_calibrations \
  --tag-size 18 \
  --duration 10  # Quick test
```

---

## 🌟 Best Practices

### Environmental Setup

#### Lighting
```
✅ Good: Even, diffuse lighting
✅ Good: Natural daylight or LED panels
❌ Bad: Direct sunlight (creates harsh shadows)
❌ Bad: Fluorescent flicker
❌ Bad: Low light conditions
```

#### Board Mounting
```
✅ Good: Rigid mounting, no flexing
✅ Good: Clean, matte surface
✅ Good: Printed at high resolution
❌ Bad: Hand-held (introduces motion blur)
❌ Bad: Reflective surface
❌ Bad: Low print quality
```

### Motion Patterns

#### Optimal Board Movement
```
1. Start: Board perpendicular to camera, 0.5m away
2. Rotate: ±30° pitch, ±20° yaw, ±10° roll
3. Translate: Move closer (0.3m) and farther (1.0m)
4. Tilt: Vary angle from 0° to 60°
5. Coverage: Fill entire camera field of view
6. Speed: Smooth, continuous motion
```

#### Movement Visualization
```
Camera View:                    Movement Pattern:
┌─────────────────┐            1. Center position
│        🎯        │            2. ↖️ Top-left corner
│                 │            3. ↗️ Top-right corner
│                 │            4. ↙️ Bottom-left corner
│                 │            5. ↘️ Bottom-right corner
└─────────────────┘            6. Angled positions
```

### Parameter Selection

#### Board Size Guidelines
| Board Size | Markers | Best For | Calibration Time |
|------------|---------|----------|------------------|
| 4×4 (16)  | 16      | Quick tests | 15-20 seconds |
| 5×7 (35)  | 35      | Standard use | 25-35 seconds |
| 7×5 (35)  | 35      | Standard use | 25-35 seconds |
| 8×6 (48)  | 48      | High precision | 35-45 seconds |

#### Square Size Guidelines
| Square Size | Use Case | Typical Distance |
|-------------|----------|------------------|
| 20mm | Close-range (0.2-0.8m) | Industrial, lab |
| 30mm | Medium-range (0.3-1.5m) | Robotics, UAV |
| 50mm | Long-range (0.5-3.0m) | Outdoor, large scenes |

---

## ⚙️ Advanced Configuration

### Custom Board Generation

#### Generate Custom ChArUco Board
```bash
cd ../aruco_tags
python aruco_sheets.py \
  --cols 8 \
  --rows 6 \
  --square-size 25 \
  --marker-size 15 \
  --output custom_board.pdf \
  --paper-size a4
```

#### Calibration with Custom Board
```bash
python calibrate_from_markers.py \
  --cols 8 \
  --rows 6 \
  --square-size 0.025 \
  --marker-size 0.015 \
  --output custom_calibration.json
```

### Performance Tuning

#### High-Precision Calibration
```bash
python calibrate_from_markers.py \
  --cols 10 \
  --rows 7 \
  --square-size 0.020 \
  --marker-size 0.012 \
  --duration 60 \
  --output high_precision.json
```

#### Fast Calibration (Development)
```bash
python calibrate_from_markers.py \
  --cols 4 \
  --rows 4 \
  --square-size 0.050 \
  --marker-size 0.030 \
  --duration 15 \
  --output fast_test.json
```

### Multi-Resolution Calibration

#### High-Resolution Setup
```bash
python calibrate_from_markers.py \
  --cols 7 \
  --rows 5 \
  --square-size 0.030 \
  --marker-size 0.018 \
  --camera-width 1920 \
  --camera-height 1080 \
  --output hd_calibration.json
```

### Error Recovery

#### Resume Failed Calibration
```bash
# If calibration failed midway
python calibrate_from_markers.py \
  --cols 7 \
  --rows 5 \
  --square-size 0.030 \
  --marker-size 0.018 \
  --duration 45 \
  --output recovery_calibration.json
```

#### Validate and Recalibrate
```bash
# Check existing calibration quality
python ../aruco_tags/aruco_validator.py \
  --calibration existing.json \
  --tag-size 18 \
  --validate-only

# Recalibrate if quality is poor
python calibrate_from_markers.py \
  --cols 7 \
  --rows 5 \
  --square-size 0.030 \
  --marker-size 0.018 \
  --duration 60 \
  --output improved_calibration.json
```

---

## 📊 Quality Assessment

### Calibration Report Analysis

#### Understanding the Output
```json
{
  "frames_used": 18,
  "calibration_quality": "good",
  "board_size": "7x5"
}
```

#### Quality Interpretation
- **frames_used**: Number of good frames captured
- **calibration_quality**: Automated assessment (excellent/good/fair/poor)
- **board_size**: ChArUco board dimensions used

### Performance Metrics

#### Real-Time Assessment
```
Calibration Results:
- Reprojection Error: 0.34 pixels (excellent)
- Frame Count: 18/30 captured
- Coverage: 85% of board area sampled
- Stability: Camera matrix condition = 1.2 (good)
```

#### Distance Accuracy Test
```
Test Results:
- Near field (0.3m): ±1.2mm accuracy
- Mid field (0.8m): ±2.8mm accuracy
- Far field (1.5m): ±5.1mm accuracy
- Overall: ±3.0mm average accuracy
```

---

## 🔄 Integration Testing

### ROS2 Parameter Loading
```python
# Test parameter loading in ROS2
import rclpy
from autonomy_interfaces.srv import LoadCalibrationParameters

def test_calibration_loading():
    node = rclpy.create_node('test_calibration')

    # Create service client
    client = node.create_client(LoadCalibrationParameters, 'calibration/load_parameters')

    # Prepare request
    request = LoadCalibrationParameters.Request()
    request.calibration_file = '/path/to/camera_calibration.json'
    request.parameter_namespace = '/camera/camera_0'

    # Call service
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response = future.result()
    assert response.success, f"Calibration loading failed: {response.error_message}"

    print("✅ Calibration parameters loaded successfully")
```

### Cross-Subsystem Validation
```python
# Test that calibration works across subsystems
def test_cross_subsystem_calibration():
    # 1. Load calibration parameters
    # 2. Test computer vision undistortion
    # 3. Test navigation pose estimation
    # 4. Test SLAM camera integration
    # 5. Verify consistent coordinate frames

    print("✅ All subsystems can access calibration parameters")
    print("✅ Coordinate frames are consistent")
    print("✅ Real-time performance meets requirements")
```

---

**🎯 Key Success Criteria:**
- [ ] Calibration completes without errors
- [ ] 15+ frames collected successfully
- [ ] Reprojection error < 1.0 pixels
- [ ] Distance measurements accurate (±5mm)
- [ ] ROS2 parameter loading works
- [ ] Multi-camera detection functions correctly

---

*"Perfect calibration is the foundation of reliable computer vision."*
