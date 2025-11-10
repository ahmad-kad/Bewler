# Calibration System - Quick Start Guide

## New Organization

```
Autonomy/calibration/
├── intrinsics/          ← Camera intrinsics (NEW: organized here)
├── extrinsics/          ← Hand-eye, multi-camera, IMU (NEW: organized here)
├── generation/          ← Generate calibration targets (ChArUco, ArUco)
├── artifacts/           ← Saved calibrations (output directory)
├── tests/               ← Unit and integration tests
└── CALIBRATION_SYSTEM.md ← Full documentation
```

## Quick Start (5 min)

### Step 1: Understand Your Hardware

What do you need to calibrate?

1. **Single camera intrinsics?** → Use `intrinsics/` module
2. **Multi-camera setup?** → Use `extrinsics/` for camera-to-camera
3. **Arm + camera?** → Use `extrinsics/` for hand-eye calibration
4. **IMU sensor?** → Use `extrinsics/` for IMU calibration

### Step 2: Choose Capture Mode

| Mode | Time | Quality | Best For |
|------|------|---------|----------|
| MANUAL | 12-15 min | 0.3-0.6 px | 1-2 cameras, precision |
| VIDEO | 3-5 min | 0.5-1.0 px | 3+ cameras, speed |
| CONSERVATIVE | 5-8 min | 0.3-0.7 px | Production systems |

### Step 3: Run Calibration

#### Example: Camera Intrinsics (MANUAL mode)

```python
from autonomy_calibration.intrinsics import (
    CameraConfig, CharUcoBoardConfig, CameraIntrinsicsCalibrator
)

# 1. Configure
camera = CameraConfig(
    name="Front Camera",
    camera_index=0,
    resolution=(1920, 1080)
)

board = CharUcoBoardConfig(
    name="board_5x7",
    aruco_dict_name="DICT_4X4_50",
    size=(5, 7),
    checker_size_mm=30.0,
    marker_size_mm=18.0
)

# 2. Initialize
calibrator = CameraIntrinsicsCalibrator(camera, board)

# 3. Capture (manual mode - press SPACE for each frame)
images = calibrator.capture_manual(target_images=50)

# 4. Process
corners, ids, valid_count = calibrator.process_dataset(images)
print(f"Valid images: {valid_count}")

# 5. Calibrate
result = calibrator.calibrate(corners, ids, images, capture_mode="manual")

# 6. Save
files = calibrator.save_calibration(result)

# 7. View results
calibrator.print_results(result)
print(f"Saved to: {files}")
```

#### Output Files

After calibration, you'll have 3 files in `artifacts/intrinsics/`:

1. **PKL** - Load easily in Python
   ```python
   import pickle
   with open('front_camera_intrinsics_manual.pkl', 'rb') as f:
       result = pickle.load(f)
   ```

2. **JSON** - Human-readable
   ```json
   {
     "camera_matrix": [...],
     "dist_coeffs": [...],
     "reprojection_error": 0.45
   }
   ```

3. **YAML** - ROS2 compatible
   ```yaml
   camera_name: Front Camera
   camera_matrix:
     data: [...]
   distortion_coefficients:
     data: [...]
   ```

## Common Tasks

### Task 1: Calibrate All 5 Cameras

```python
from autonomy_calibration.intrinsics import CameraConfig, CharUcoBoardConfig, CameraIntrinsicsCalibrator

cameras = [
    ("Front Left", 0),
    ("Front Center", 1),
    ("Front Right", 2),
    ("Rear Left", 3),
    ("Rear Right", 4),
]

board = CharUcoBoardConfig("board", "DICT_4X4_50", (5, 7), 30.0, 18.0)

for name, index in cameras:
    camera = CameraConfig(name, camera_index=index, resolution=(1920, 1080))
    calibrator = CameraIntrinsicsCalibrator(camera, board)
    
    # VIDEO mode for speed (3-5 min per camera)
    images = calibrator.capture_video(target_images=50)
    corners, ids, _ = calibrator.process_dataset(images)
    result = calibrator.calibrate(corners, ids, images, "video")
    calibrator.save_calibration(result)
    
    print(f"✓ {name} calibrated")
```

### Task 2: Hand-Eye Calibration for Arm

```python
from autonomy_calibration.extrinsics import HandEyeCalibrator
import numpy as np

# Load camera calibration from previous step
camera_intrinsics = {
    'camera_matrix': np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]]),
    'dist_coeffs': np.array([0.1, -0.1, 0, 0, 0])
}

board_config = {'size': (5, 7), 'checker_size_mm': 30.0, 'marker_size_mm': 18.0}

calibrator = HandEyeCalibrator(camera_intrinsics, board_config)

# Move arm to 10 different poses and collect observations
for pose_id in range(10):
    T_robot = get_arm_pose()  # Get from robot controller
    frame = capture_image()
    corners, ids = detect_board(frame)
    
    calibrator.add_pose_observation(pose_id, T_robot, corners, ids)
    print(f"Pose {pose_id} captured")

# Calibrate and save
result = calibrator.calibrate(setup_type="eye_on_hand")
calibrator.save_calibration(result)
print("Hand-eye calibration saved")
```

### Task 3: Multi-Camera Stereo Calibration

```python
from autonomy_calibration.extrinsics import MultiCameraCalibrator

# Load both camera calibrations
cam1_intrinsics = {...}
cam2_intrinsics = {...}
board_config = {...}

calibrator = MultiCameraCalibrator(cam1_intrinsics, cam2_intrinsics, board_config)

# Collect 10 synchronized observations
for i in range(10):
    corners1, ids1 = detect_board(frame1)
    corners2, ids2 = detect_board(frame2)
    
    calibrator.add_observation(corners1, ids1, corners2, ids2)
    print(f"Stereo pair {i} captured")

# Calibrate and save
result = calibrator.calibrate()
calibrator.save_calibration(result)
print("Multi-camera calibration saved")
```

## Testing

### Run Unit Tests

```bash
cd Autonomy/calibration/tests/unit/
python -m pytest test_intrinsics.py -v
```

### Run All Tests

```bash
cd Autonomy/calibration/tests/
python -m pytest . -v
```

## Directory Structure

After running some calibrations, you'll have:

```
artifacts/
├── intrinsics/
│   ├── front_camera_intrinsics_manual.pkl
│   ├── front_camera_intrinsics_manual.json
│   ├── front_camera_intrinsics_manual.yaml
│   ├── front_center_intrinsics_video.pkl
│   └── ...
└── extrinsics/
    ├── hand_eye_eye_on_hand_*.pkl
    ├── hand_eye_eye_on_hand_*.json
    ├── multicam_front_left_to_front_center.json
    └── imu_imu_0_calibration.json

calibration_images/
├── intrinsics_manual_front_camera_20250101_120000/
│   ├── frame_0000.png
│   ├── frame_0001.png
│   └── ...
├── intrinsics_video_front_center_20250101_130000/
│   └── ...
└── ...
```

## Quality Standards

Before deployment:

- [ ] Reprojection error < 1.0 px
- [ ] All 3 formats (PKL, JSON, YAML) generated
- [ ] Source images archived
- [ ] Verified with actual hardware

## Troubleshooting

**"No markers detected"**
- Ensure ChArUco board is visible
- Check lighting (well-lit environment)
- Verify camera focus

**"High reprojection error (>2 px)"**
- Recapture with more diverse poses
- Vary distance: 20cm, 60cm, 150cm
- Check board flatness/damage

**"Hand-eye calibration fails"**
- Ensure robot poses are synchronized with images
- Collect at least 10 diverse poses
- Verify robot pose accuracy

## Next Steps

1. Read `CALIBRATION_SYSTEM.md` for complete documentation
2. Run intrinsics calibration for your cameras
3. If needed, run extrinsics for multi-camera/arm/IMU
4. Integrate calibrations into state machine "CALIBRATION" mode
5. Validate on actual hardware

---

**Questions?** See `CALIBRATION_SYSTEM.md` for comprehensive documentation, examples, and workflow guides.

