# 🎨 Visual Guides - Screenshots & Diagrams

**Step-by-step visual walkthrough of camera calibration procedures.**

---

## 📋 Table of Contents

- [Calibration Interface](#calibration-interface)
- [Board Movement Patterns](#board-movement-patterns)
- [Detection Results](#detection-results)
- [Error Visualization](#error-visualization)
- [Multi-Camera Setup](#multi-camera-setup)

---

## 📷 Calibration Interface

### Starting Calibration

![Calibration Start](https://via.placeholder.com/800x600/4CAF50/FFFFFF?text=Calibration+Start+Screen)

**Command executed:**
```bash
python calibrate_from_markers.py --cols 7 --rows 5 --square-size 0.030 --marker-size 0.018
```

**What you see:**
- Terminal shows calibration parameters
- Camera window opens automatically
- Live preview shows your camera feed
- Instructions displayed in terminal

### Calibration Progress

![Calibration Progress](https://via.placeholder.com/800x600/2196F3/FFFFFF?text=Calibration+Progress)

**Progress indicators:**
```
🎯 Direct ArUco Marker Calibration
Board: 7×5 (35 markers)
Square: 30.0mm, Marker: 18.0mm

📹 Capturing for 30 seconds...
Move board around to collect good frames
```

**Live feedback:**
- Frame counter increases: `Frame 5: Collected 25 corners (total: 5)`
- Visual markers on detected corners
- Green overlays on successfully detected markers

### Successful Completion

![Calibration Success](https://via.placeholder.com/800x600/4CAF50/FFFFFF?text=Calibration+Complete)

**Success indicators:**
```
✅ Frame 18: Collected 32 corners (total: 18)
✅ Calibration successful!
💾 Calibration saved to: my_camera.json
```

**Output verification:**
```json
{
  "camera_index": 0,
  "frames_used": 18,
  "calibration_quality": "good",
  "camera_matrix": [...],
  "distortion_coefficients": [...]
}
```

---

## 🎯 Board Movement Patterns

### Correct Movement Pattern

#### Step 1: Start Position (0 seconds)
![Board Start Position](https://via.placeholder.com/800x600/FF9800/000000?text=Step+1:+Center+Position)

**Board placement:**
- Centered in camera view
- Facing directly toward camera
- Approximately 0.5m from lens
- All markers clearly visible

#### Step 2: Rotation Around Pitch (5 seconds)
![Board Pitch Rotation](https://via.placeholder.com/800x600/FF9800/000000?text=Step+2:+Pitch+Rotation)

**Movement:**
- Tilt board up/down ±30°
- Keep board centered
- Maintain same distance
- Smooth, continuous motion

#### Step 3: Rotation Around Yaw (10 seconds)
![Board Yaw Rotation](https://via.placeholder.com/800x600/FF9800/000000?text=Step+3:+Yaw+Rotation)

**Movement:**
- Rotate board left/right ±20°
- Combine with pitch for 3D rotation
- Cover all marker orientations

#### Step 4: Distance Variation (15 seconds)
![Board Distance Change](https://via.placeholder.com/800x600/FF9800/000000?text=Step+4:+Distance+Variation)

**Movement:**
- Move closer: 0.3m from camera
- Move farther: 1.2m from camera
- Combine with rotations
- Fill different parts of frame

#### Step 5: Angular Variation (20-30 seconds)
![Board Angular Movement](https://via.placeholder.com/800x600/FF9800/000000?text=Step+5:+Angular+Variation)

**Movement:**
- Tilt at 30°, 45°, 60° angles
- Move to corners of camera view
- Combine all previous motions
- Ensure full coverage

### Movement Pattern Summary

```
Camera View Coverage:
┌─────────────────┐
│        1        │  ← Center position
│       ↗↖↙↘      │  ← Corner coverage
│      ↗   ↖      │  ← Diagonal movement
│     ↗  🎯  ↖     │  ← Full range
│    ↗       ↖    │
│   ↗    📦    ↖   │  ← Board at various angles
└─────────────────┘
```

### Common Movement Mistakes

#### ❌ Too Fast Movement
![Too Fast Movement](https://via.placeholder.com/800x600/F44336/FFFFFF?text=ERROR:+Too+Fast+Movement)

**Problems:**
- Markers become blurred
- Insufficient stable captures
- Low frame collection rate

#### ❌ Insufficient Range
![Insufficient Range](https://via.placeholder.com/800x600/F44336/FFFFFF?text=ERROR:+Insufficient+Range)

**Problems:**
- Only center of view covered
- Poor calibration in periphery
- Inaccurate distortion correction

#### ❌ Hand-Held Instability
![Hand Held Instability](https://via.placeholder.com/800x600/F44336/FFFFFF?text=ERROR:+Hand+Held+Instability)

**Problems:**
- Board shakes during capture
- Motion blur on markers
- Inconsistent measurements

---

## 👁️ Detection Results

### Single Tag Detection

#### Successful Detection
![Single Tag Success](https://via.placeholder.com/800x600/4CAF50/FFFFFF?text=Single+Tag+Detection+Success)

**Visual indicators:**
- Green square around detected marker
- Tag ID displayed: `ID: 42`
- Distance measurement: `156.3mm`
- Confidence indicator: ✓ (checkmark)

**Data output:**
```
ID:42 156.3mm ✓
```

#### Partial Detection
![Partial Tag Detection](https://via.placeholder.com/800x600/FF9800/FFFFFF?text=Partial+Tag+Detection)

**Visual indicators:**
- Yellow/orange outline (partial detection)
- Tag ID visible but uncertain
- Distance estimate with warning
- Confidence indicator: ⚠️

### Multi-Tag Detection

#### Multiple Tags in View
![Multi Tag Detection](https://via.placeholder.com/800x600/2196F3/FFFFFF?text=Multi+Tag+Detection)

**Visual indicators:**
- Multiple green squares
- Each tag numbered: `ID:0`, `ID:4`, `ID:12`
- Individual distances displayed
- Performance metrics shown

**Data output:**
```
ID:0  245.6mm ✓
ID:4  156.3mm ✓
ID:12  89.2mm ✓
FPS: 42.3
```

### Distance Visualization

#### Distance Color Coding
![Distance Color Coding](https://via.placeholder.com/800x600/9C27B0/FFFFFF?text=Distance+Color+Coding)

**Color scheme:**
- 🔵 Blue: 0-200mm (close range)
- 🟢 Green: 200-500mm (medium range)
- 🟡 Yellow: 500-1000mm (far range)
- 🔴 Red: >1000mm (distant)

#### 3D Distance Map
![3D Distance Visualization](https://via.placeholder.com/800x600/607D8B/FFFFFF?text=3D+Distance+Visualization)

**Features:**
- Top-down view of detected tags
- Distance represented by circle size
- Color-coded distance ranges
- Camera position marked

---

## ❌ Error Visualization

### Camera Connection Issues

#### Camera Not Found
![Camera Not Found](https://via.placeholder.com/800x600/F44336/FFFFFF?text=ERROR:+Camera+Not+Found)

**Error message:**
```
❌ Cannot open camera 0
[ERROR] cv2.VideoCapture(0) failed
```

**Troubleshooting steps:**
1. Check USB connection
2. Verify camera permissions
3. Try different camera index
4. Test with system camera app

#### Camera Access Denied
![Camera Permission Error](https://via.placeholder.com/800x600/F44336/FFFFFF?text=ERROR:+Camera+Permission+Denied)

**Error message:**
```
❌ Cannot open camera 0
[ERROR] Permission denied
```

**Solution visualization:**
```bash
# Check permissions
ls -la /dev/video0
# crw-rw---- 1 root video 81, 0 Oct 21 10:30 /dev/video0

# Add user to video group
sudo usermod -a -G video $USER

# Logout and login again, or:
newgrp video
```

### Calibration Quality Issues

#### Insufficient Frames
![Insufficient Frames](https://via.placeholder.com/800x600/F44336/FFFFFF?text=ERROR:+Insufficient+Calibration+Frames)

**Error message:**
```
❌ Need at least 5 frames, got 2
❌ Calibration failed
```

**Visual diagnosis:**
- Very few markers detected (red circles)
- Board barely moved during capture
- Only 2-3 frames collected instead of 15+

#### Poor Marker Detection
![Poor Marker Detection](https://via.placeholder.com/800x600/F44336/FFFFFF?text=ERROR:+Poor+Marker+Detection)

**Symptoms:**
- Markers not detected (no green squares)
- Only corners detected occasionally
- Low frame collection rate

**Common causes:**
- Poor lighting conditions
- Marker damage/wear
- Incorrect marker size parameters
- Camera focus issues

### Distance Measurement Errors

#### Unrealistic Distances
![Unrealistic Distances](https://via.placeholder.com/800x600/F44336/FFFFFF?text=ERROR:+Unrealistic+Distances)

**Error output:**
```
ID:0 45000.5mm ❌
ID:4 -1250.3mm ❌
```

**Root causes:**
- Wrong tag size specification
- Calibration parameter errors
- Coordinate system issues

#### Inconsistent Readings
![Inconsistent Readings](https://via.placeholder.com/800x600/FF9800/FFFFFF?text=WARNING:+Inconsistent+Readings)

**Symptoms:**
- Distance jumps erratically
- Same tag shows different distances
- Measurements change without movement

**Diagnostic visualization:**
```
Time → Distance Plot:
     │         *
     │       *   *
     │     *       *
     │   *           *
     │ *               *
─────┼──────────────────── Time
   0s                  5s
```

---

## 🎬 Multi-Camera Setup

### Sequential Calibration Process

#### Camera 0 Calibration
![Sequential Camera 0](https://via.placeholder.com/800x600/4CAF50/FFFFFF?text=Sequential:+Camera+0+Calibration)

**Process:**
1. Camera 0 window opens
2. User moves ChArUco board
3. Calibration completes
4. Prompt for camera 1 appears

#### Camera Switching Prompt
![Camera Switch Prompt](https://via.placeholder.com/800x600/FF9800/FFFFFF?text=Camera+Switch+Prompt)

**Terminal display:**
```
✅ Calibration successful!
💾 Calibration saved to: camera_calibrations/camera_0_calibration.json

⏳ Prepare camera 1 and press ENTER to continue...
```

**Physical actions:**
1. Disconnect camera 0
2. Connect camera 1 to same USB port
3. Press ENTER when ready

### Parallel Detection Setup

#### 5-Camera View Layout
![5 Camera Layout](https://via.placeholder.com/800x600/2196F3/FFFFFF?text=5+Camera+Parallel+Detection)

**Window arrangement:**
```
┌─────┬─────┐
│Cam 0│Cam 1│
├─────┼─────┤
│Cam 2│Cam 3│
├─────┴─────┤
│   Cam 4   │
└───────────┘
```

**Data flow visualization:**
```
Cameras → Detection → Distance Calc → Display
    ↓         ↓            ↓          ↓
  Frames →   IDs   →   Measurements → UI
```

### Multi-Camera Performance Monitor

#### Real-Time Statistics
![Multi Camera Stats](https://via.placeholder.com/800x600/607D8B/FFFFFF?text=Multi+Camera+Performance+Stats)

**Performance metrics:**
```
Camera | FPS | Tags | Avg Distance | Status
-------|-----|------|-------------|--------
0      | 38  | 3    | 245mm       | ✓ OK
1      | 42  | 1    | 156mm       | ✓ OK
2      | 35  | 2    | 312mm       | ✓ OK
3      | 39  | 0    | -           | ⚠️ No tags
4      | 41  | 4    | 189mm       | ✓ OK
-----------------------------------
Total  | 39  | 10   | 226mm avg   | ✓ System OK
```

---

## 🛠️ Configuration Visualizations

### Board Parameter Effects

#### Board Size Comparison
![Board Size Comparison](https://via.placeholder.com/800x600/9C27B0/FFFFFF?text=Board+Size+Effects)

**Visual comparison:**
- **4×4 board**: Fast calibration, lower accuracy
- **7×5 board**: Balanced speed/accuracy
- **10×7 board**: High precision, slower

#### Marker Size Effects
![Marker Size Effects](https://via.placeholder.com/800x600/9C27B0/FFFFFF?text=Marker+Size+Effects)

**Detection range:**
- Small markers (12mm): Close range only
- Medium markers (18mm): Standard range
- Large markers (30mm): Long range capable

### Camera Resolution Impact

#### Resolution vs Quality
![Resolution Comparison](https://via.placeholder.com/800x600/FF5722/FFFFFF?text=Resolution+Impact)

**Trade-offs:**
```
640×480:   Fast (58 FPS) ←→ Lower accuracy (±4mm)
1280×720:  Balanced (42 FPS) ←→ Good accuracy (±3mm)
1920×1080: Precise (28 FPS) ←→ Best accuracy (±2mm)
```

---

## 📊 Quality Assessment Tools

### Calibration Quality Dashboard

#### Quality Metrics Display
![Quality Dashboard](https://via.placeholder.com/800x600/4CAF50/FFFFFF?text=Calibration+Quality+Dashboard)

**Quality indicators:**
- ✅ **Reprojection Error**: 0.34px (Excellent)
- ✅ **Frame Count**: 18/30 collected
- ✅ **Coverage**: 85% of board area
- ✅ **Stability**: Matrix condition = 1.2

#### Quality Trend Analysis
![Quality Trends](https://via.placeholder.com/800x600/2196F3/FFFFFF?text=Quality+Trend+Analysis)

**Historical tracking:**
```
Calibration Date | Error (px) | Frames | Quality
-----------------|-----------|--------|---------
2024-01-15       | 0.34      | 18     | Excellent
2024-01-20       | 0.42      | 16     | Good
2024-01-25       | 0.38      | 19     | Excellent
2024-01-30       | 0.51      | 15     | Good
```

### Error Distribution Analysis

#### Error Heatmap
![Error Heatmap](https://via.placeholder.com/800x600/F44336/FFFFFF?text=Calibration+Error+Heatmap)

**Error distribution:**
- Green: Low error regions
- Yellow: Medium error regions
- Red: High error regions
- Shows where calibration is strongest/weakest

---

## 🎯 Best Practices Summary

### ✅ Do This
- **Even lighting** throughout capture
- **Smooth board movement** covering full range
- **Rigid board mounting** (no flexing)
- **Complete motion pattern** (30+ seconds)
- **Verify results** before proceeding

### ❌ Avoid This
- **Direct sunlight** or harsh shadows
- **Hand-held boards** (causes shake)
- **Incomplete movement** (only center coverage)
- **Wrong parameters** (board size, marker size)
- **Poor focus** or camera shake

---

## 🚀 Advanced Visualization

### 3D Calibration Visualization

#### Camera Coordinate System
![3D Calibration View](https://via.placeholder.com/800x600/607D8B/FFFFFF?text=3D+Calibration+Visualization)

**3D representation:**
- Camera position and orientation
- Calibration target positions
- Field of view visualization
- Error vectors and magnitudes

### Real-Time Performance Graphing

#### FPS and Latency Monitoring
![Performance Graph](https://via.placeholder.com/800x600/FF9800/FFFFFF?text=Real+Time+Performance+Graph)

**Live metrics:**
- Frame rate over time
- Detection latency per frame
- Memory usage trends
- CPU utilization graphs

---

*"A picture is worth a thousand calibration parameters."*
