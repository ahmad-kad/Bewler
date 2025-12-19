# Camera Calibration System - URC 2026

**Sensor-based camera calibration with automatic ID generation and individual file storage.**

---

## Quick Start (5 minutes)

### 1. Setup System
```bash
cd calibration
python3 scripts/camera_validator.py --setup
sudo bash tools/setup_cameras.sh
sudo reboot
```

### 2. Test Camera Detection
```bash
python3 scripts/camera_check.py                    # System validation
python3 scripts/camera_validator.py --auto-detect  # Find cameras
```

### 3. Calibrate Cameras
```bash
# Single camera (auto sensor detection)
python3 scripts/quick_calibration.py --camera 0
# → Detects: IMX219 sensor
# → Assigns: imx219_1

# Custom name
python3 scripts/quick_calibration.py --camera 0 --name "front_left"

# Batch calibration
python3 scripts/quick_calibration.py --batch 3
```

### 4. Manage Calibrations
```bash
python3 scripts/quick_calibration.py --list-cameras    # View all
python3 scripts/quick_calibration.py --rename-camera imx219_1 front_left
```

---

## System Architecture

### Project Structure

```
calibration/
├── src/                    # Core library modules (importable)
│   ├── __init__.py
│   ├── aruco/             # ArUco tag functionality
│   │   ├── __init__.py
│   │   ├── generator.py
│   │   └── validator.py
│   ├── charuco/           # ChArUco board functionality
│   │   ├── __init__.py
│   │   ├── generator.py
│   │   └── calibrator.py
│   ├── camera/            # Camera calibration utilities
│   │   ├── __init__.py
│   │   ├── calibration.py
│   │   ├── validation.py
│   │   └── system.py
│   └── utils/             # Shared utilities
│       ├── __init__.py
│       ├── camera.py
│       ├── file_ops.py
│       └── system.py
├── scripts/               # CLI tools and executables
│   ├── quick_calibration.py
│   ├── camera_validator.py
│   ├── verify_installation.py
│   └── camera_check.py
├── docs/                  # Documentation
│   ├── README.md
│   ├── INSTALL.md
│   └── camera/
├── tests/                 # Test suite
│   ├── __init__.py
│   └── unit/
├── tools/                 # Setup scripts and config
│   ├── setup_cameras.sh
│   ├── requirements.txt
│   └── requirements-test.txt
└── .flake8               # Linting configuration
```
    ├── requirements-test.txt
    └── pytest.ini
```

---

## Key Features

### Sensor-Based Calibration
- **Automatic sensor detection**: IMX219, IMX477, IMX708, OV5647
- **Sequential IDs**: `imx219_1`, `imx219_2`, `imx477_1`, etc.
- **Individual file storage**: JSON files with calibration metadata

### Calibration Quality
- **Automatic quality assessment** with reprojection error metrics
- **Frame counting** and validation statistics
- **File-based tracking** of calibration history and sensor information

---

## Usage Examples

### Generate Calibration Target
```bash
# ChArUco board for camera calibration
cd calibration/charuco_board
# As library:
# from src.charuco.generator import CharucoGenerator
# gen = CharucoGenerator()
# gen.save_board_png(rows=5, cols=7, output_path="board.png")
```

### Calibrate Camera
```bash
cd calibration/camera

# Auto-detect sensor and assign ID
python3 quick_calibration.py --camera 0
# → Detects: IMX219 sensor
# → Assigns: imx219_1

# Manual naming
python3 quick_calibration.py --camera 0 --name "front_left"

# Batch calibration
python3 quick_calibration.py --batch 3
```

### Manage Calibrations
```bash
# List all calibrated cameras
python3 quick_calibration.py --list-cameras

# Rename camera
python3 quick_calibration.py --rename-camera imx219_1 front_left

# Use custom directory
python3 quick_calibration.py --calibration-dir my_calibrations --camera 0
```

---

## File Format

### Individual JSON Files
Each camera calibration is saved as a separate JSON file:

```
calibrations/
├── imx219_1.json
├── imx219_2.json
└── front_left.json
```

**Example calibration file** (`calibrations/imx219_1.json`):
```json
{
  "sensor": "imx219",
  "calibrated_at": "2024-12-19T14:30:22Z",
  "camera_matrix": [[669.13, 0.0, 644.66], [0.0, 668.50, 357.35], [0.0, 0.0, 1.0]],
  "distortion_coefficients": [2.54, 6.38, -0.018, -0.226, -249.18],
  "image_width": 1280,
  "image_height": 720,
  "frames_used": 15,
  "opencv_version": "4.8.0",
  "calibration_method": "charuco"
}
```

---

## Testing

### Run Unit Tests
```bash
cd calibration
pip install -r tools/requirements-test.txt
python -m pytest tests/
```

### Test Components
```bash
# Test library imports
python -c "from src.aruco.generator import ArucoGenerator; print('ArUco generator OK')"
python -c "from src.charuco.generator import CharucoGenerator; print('ChArUco generator OK')"
python -c "from src.camera.validation import validate_camera_setup; print('Camera validation OK')"
```

---

## Status

**Production Ready**
- Sensor-based automatic camera identification
- Sequential ID generation per sensor type
- Individual file storage with JSON format
- Comprehensive testing framework
- Raspberry Pi optimized

**Tested Sensors:**
- IMX219 (Pi Camera v2)
- IMX477 (Pi HQ Camera)
- IMX708 (Pi Camera v3)
- OV5647 (Pi Camera v1)

---

**Ready for deployment on Raspberry Pi systems!** 📸🤖

## API Reference

# Setup
camera = CameraConfig("Front Camera", camera_index=0, resolution=(1920, 1080))
board = CharUcoBoardConfig("board_5x7", "DICT_4X4_50", (5, 7), 30.0, 18.0)
calibrator = CameraIntrinsicsCalibrator(camera, board)

# Capture
images = calibrator.capture_manual(target_images=50)

# Process
corners, ids, rejected = calibrator.process_dataset(images)

# Calibrate
result = calibrator.calibrate(corners, ids, images, "manual")

# Save (automatic 3-format output)
files = calibrator.save_calibration(result)
print(f"Saved to: {files}")

# Display
calibrator.print_results(result)
```

### 2. Multi-Camera Calibration (Fast)

```python
cameras = [
    ("Front", 0),
    ("Left", 1),
    ("Right", 2),
]

board = CharUcoBoardConfig("board_5x7", "DICT_4X4_50", (5, 7), 30.0, 18.0)

for name, index in cameras:
    camera = CameraConfig(name, camera_index=index, resolution=(1920, 1080))
    calibrator = CameraIntrinsicsCalibrator(camera, board)
    
    # VIDEO mode: fast for multiple cameras
    images = calibrator.capture_video(target_images=30)
    corners, ids, _ = calibrator.process_dataset(images)
    result = calibrator.calibrate(corners, ids, images, "video")
    calibrator.save_calibration(result)
    
    print(f" {name} calibrated")
```

### 3. Hand-Eye Calibration for Arm

```python
from extrinsics.hand_eye_imu_calibrator import HandEyeCalibrator

calibrator = HandEyeCalibrator(camera_intrinsics, board_config)

# Collect 10 pose observations
for pose_id in range(10):
    T_robot = get_arm_pose()          # Get from robot state
    frame = capture_image()           # Capture from camera
    corners, ids = detect_board(frame)
    calibrator.add_pose_observation(pose_id, T_robot, corners, ids)

# Calibrate
result = calibrator.calibrate(setup_type="eye_on_hand")
calibrator.save_calibration(result)
```

### 4. CLI Usage

```bash
# List cameras
python3 calibration_cli.py list

# Calibrate camera 0 with various options
python3 calibration_cli.py calibrate --camera 0 --mode manual
python3 calibration_cli.py calibrate --camera 0 --mode video --count 30
python3 calibration_cli.py calibrate --camera 0 --mode conservative --count 40

# With custom resolution
python3 calibration_cli.py calibrate --camera 0 --resolution 1280x720 --mode video

# With different board
python3 calibration_cli.py calibrate --camera 0 --board board_small --mode manual

# Test system
python3 calibration_cli.py test
```

---

## ROS2 Integration

### Launch Calibration Node

```bash
ros2 run autonomy_calibration calibration_node \
  --ros-args \
  -p calibration_mode:=intrinsic \
  -p target_images:=50 \
  -p capture_mode:=manual \
  -p camera_index:=0 \
  -p board_name:=board_5x7
```

### Topics

**Publish:**
- `/calibration/status` (String) - Status updates
- `/calibration/progress` (Float32) - Progress 0-100%

**Subscribe:**
- `/state_machine/state` (String) - State machine state
- `/calibration/command` (String) - Commands (start, cancel, get_results)

### Example: Start Calibration via Command

```bash
# Terminal 1: Start node
ros2 run autonomy_calibration calibration_node

# Terminal 2: Send command
ros2 topic pub /calibration/command std_msgs/String "data: start"

# Monitor progress
ros2 topic echo /calibration/progress
ros2 topic echo /calibration/status
```

---

## Testing

### Unit Tests

```bash
cd tests/unit
python3 -m pytest test_intrinsics.py -v

# Output:
# test_camera_config_creation PASSED
# test_board_config_creation PASSED
# test_calibration_result_creation PASSED
# ... (11 total tests)
```

### Integration Tests

```bash
cd tests/integration
python3 -m pytest test_end_to_end_calibration.py -v

# Includes:
# - Configuration validation
# - Results serialization
# - Quality metrics
# - Multi-camera workflows
# - Output directory structure
# - Parameter validation
# - Data processing pipeline
# - Error handling
```

### System Test

```bash
python3 calibration_cli.py test

# Output:
# Test 1: Module Imports # Test 2: Camera Detection # Test 3: Configuration Validation # Test 4: Output Directory Setup # All tests passed! System is ready to use.
```

---

## Quality Assessment

### Reprojection Error Standards

| Quality Level | Error Range | Confidence |
|---------------|-------------|-----------|
| Excellent | < 0.5 px | Very high |
| Good | 0.5-1.0 px | High |
| Acceptable | 1.0-2.0 px | Medium |
| Poor | > 2.0 px | Low (not recommended) |

### Example Output

```
========================================================================
  CALIBRATION RESULTS
========================================================================

Reprojection Error: 0.3450 px
Focal Length (fx): 1920.23
Focal Length (fy): 1920.15
Principal Point (cx): 959.87
Principal Point (cy): 539.92

Distortion Coefficients: [-0.102  0.051  0.001 -0.002  0.000]
Image Quality Score: 95.20%

Quality Assessment: EXCELLENT
```

---

## Output Files

### JSON Format (Human-Readable)

```json
{
  "camera_name": "Camera_0",
  "camera_matrix": [
    [1920.23, 0, 959.87],
    [0, 1920.15, 539.92],
    [0, 0, 1]
  ],
  "distortion": [-0.102, 0.051, 0.001, -0.002, 0.0],
  "reprojection_error": 0.345,
  "quality_score": 0.952,
  "capture_mode": "manual",
  "image_count": 50,
  "timestamp": "2025-01-15T14:32:00"
}
```

### YAML Format (ROS2 Compatible)

```yaml
camera_name: Camera_0
camera_matrix:
  - [1920.23, 0, 959.87]
  - [0, 1920.15, 539.92]
  - [0, 0, 1]
distortion: [-0.102, 0.051, 0.001, -0.002, 0.0]
reprojection_error: 0.345
quality_score: 0.952
capture_mode: manual
image_count: 50
timestamp: '2025-01-15T14:32:00'
```

### Pickle Format (Python)

```python
import pickle

with open('camera_0_intrinsics_manual.pkl', 'rb') as f:
    result = pickle.load(f)

<<<<<<< HEAD
### Target Metrics (Competition Ready)
- **Reprojection Error:** < 0.5 pixels
- **Calibration Time:** < 30 seconds
- **Validation Time:** < 10 seconds
- **Memory Usage:** < 500MB

### Quality Thresholds
- **Excellent:** Reprojection error < 0.3 pixels
- **Good:** Reprojection error < 0.5 pixels
- **Acceptable:** Reprojection error < 1.0 pixels
- **Poor:** Reprojection error > 1.0 pixels

## Support and Documentation

### Quick Start Guides
- **[Practical Workflow Guide](PRACTICAL_WORKFLOW.md)** - Step-by-step guide for camera calibration and ArUco testing

### Related Documents
- [`CameraCalibrationGuide.md`](../reference/CameraCalibrationGuide.md) - Detailed concepts and procedures
- [`CalibrationGuide.md`](../reference/CalibrationGuide.md) - Hardware-specific calibration
- [`SensorGuide.md`](../reference/SensorGuide.md) - Sensor integration details

### Getting Help
1. Check the troubleshooting section above
2. Review the detailed guides for specific issues
3. Run the test suite to diagnose problems
4. Check calibration quality metrics
=======
print(result.camera_matrix)
print(result.distortion)
print(result.reprojection_error)
```
>>>>>>> ba52654723152f45e46dfaeabf2aedadc10c49f2

---

## Common Workflows

### Workflow 1: Quick 5-Camera Calibration (for Raspberry Pi)

```bash
# Time: ~20-30 minutes total

for i in {0..4}; do
  echo "Calibrating camera $i..."
  python3 calibration_cli.py calibrate \
    --camera $i \
    --mode video \
    --count 30 \
    --board board_5x7
done

echo "All cameras calibrated!"
```

### Workflow 2: Production-Grade Single Camera

```bash
# Time: ~15 minutes, highest quality

python3 calibration_cli.py calibrate \
  --camera 0 \
  --mode conservative \
  --count 50 \
  --board board_5x7

# Verify results
python3 -c "
import json
with open('artifacts/intrinsics/camera_0_intrinsics_conservative.json') as f:
    data = json.load(f)
    error = data['reprojection_error']
    if error < 0.5:
        print(f' Excellent quality: {error:.3f} px')
    else:
        print(f'Retry calibration for better quality')
"
```

### Workflow 3: Automated Multi-Camera Setup

```python
#!/usr/bin/env python3
"""Calibrate all cameras in a single run."""

import subprocess
import json
from pathlib import Path

cameras = [
    ("front_left", 0),
    ("front_center", 1),
    ("front_right", 2),
    ("rear_left", 3),
    ("rear_right", 4),
]

for name, index in cameras:
    print(f"\n{'='*60}")
    print(f"Calibrating {name} (camera {index})...")
    print(f"{'='*60}")
    
    result = subprocess.run([
        "python3", "calibration_cli.py", "calibrate",
        "--camera", str(index),
        "--mode", "video",
        "--count", "30"
    ])
    
    if result.returncode != 0:
        print(f" Failed to calibrate {name}")
        continue
    
    # Verify quality
    calib_file = Path(f"artifacts/intrinsics/camera_{index}_intrinsics_video.json")
    if calib_file.exists():
        with open(calib_file) as f:
            data = json.load(f)
            error = data["reprojection_error"]
            quality = data["quality_score"]
            print(f" {name}: error={error:.3f}px, quality={quality:.1%}")

print(f"\n{'='*60}")
print("All cameras calibrated!")
print(f"{'='*60}")
```

---

## Troubleshooting

### Issue: No cameras detected

```bash
# Check camera permissions
ls -la /dev/video*

# Try with sudo (not recommended)
sudo python3 calibration_cli.py list

# Alternative: Check with OpenCV
python3 -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"
```

### Issue: Low reprojection error (>2 px)

**Solutions:**
1. Ensure good lighting
2. Keep board perpendicular to camera
3. Vary distance and angles more
4. Use more images (increase --count)
5. Switch to MANUAL mode for better control

### Issue: Insufficient data after capture

```bash
# Means: Less than 5 frames with detected markers

# Solutions:
# 1. More images
python3 calibration_cli.py calibrate --camera 0 --count 100

# 2. Better board visibility
# - Increase lighting
# - Clean camera lens
# - Ensure markers are clearly visible

# 3. Manual mode for best control
python3 calibration_cli.py calibrate --camera 0 --mode manual
```

### Issue: Module import errors

```bash
# Ensure you're in the right directory
cd Autonomy/calibration/intrinsics

# Or add to Python path
export PYTHONPATH="${PYTHONPATH}:$(pwd)/../"

# Then run
python3 calibration_cli.py list
```

---

## Performance Characteristics

| Task | Time | Quality | Notes |
|------|------|---------|-------|
| Single camera (MANUAL) | 12-15 min | Excellent | Full control, best quality |
| Single camera (VIDEO) | 3-5 min | Good | Fast, reasonable quality |
| Single camera (CONSERVATIVE) | 5-8 min | Excellent | Fast + quality checks |
| 5 cameras (VIDEO) | 20-30 min | Good | Parallel possible |
| Hand-eye (10 poses) | 10-15 min | Good | Requires arm movement |
| Multi-camera stereo | 15-20 min | Good | 2+ cameras synchronized |

---

## Next Steps

### Immediate (Ready now)
-  Use CLI tool to calibrate cameras
-  Run unit tests (`pytest`)
-  Integrate with ROS2 workflow

### Short-term (2-3 weeks)
- ⏳ Create generation module (board generators)
- ⏳ Add more integration tests
- ⏳ Create field validation scripts

### Medium-term (1-2 months)
-  Temperature-compensated IMU calibration
-  Auto-detection of camera-to-IMU alignment
-  Individual calibration files for tracking

### Long-term (Future)
-  Web dashboard for calibration management
-  Cloud-based calibration storage
-  Machine learning for quality prediction

---

## Files Summary

| File | Lines | Purpose |
|------|-------|---------|
| `intrinsics/camera_intrinsics_calibrator.py` | 840 | Core intrinsics calibration |
| `intrinsics/calibration_cli.py` | 600 | CLI tool for easy usage |
| `extrinsics/hand_eye_imu_calibrator.py` | 600 | Extrinsics & IMU calibration |
| `extrinsics/ros2_calibration_node.py` | 500 | ROS2 integration node |
| `tests/unit/test_intrinsics.py` | 250 | Unit tests |
| `tests/integration/test_end_to_end_calibration.py` | 600 | Integration tests |
| Documentation | 1000+ | Guides and examples |

**Total**: 1,700+ lines of code, 1,000+ lines of documentation

---

## Support

For issues or questions:
1. Check `CALIBRATION_SYSTEM.md` for detailed documentation
2. Review `QUICK_START_NEW_SYSTEM.md` for examples
3. Run tests: `pytest tests/ -v`
4. Check troubleshooting section above

---

**Last Updated**: November 2025
**Status**: Production Ready **Version**: 1.0
