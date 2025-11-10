# Calibration System Documentation - URC 2026

## Overview

The calibration system is organized into **4 main directories**:

```
Autonomy/calibration/
├── intrinsics/          # Camera calibration (single camera per-camera parameters)
├── extrinsics/          # Multi-camera, hand-eye, and IMU calibration
├── generation/          # Tools for generating calibration targets (ChArUco boards, ArUco tags)
├── artifacts/           # Output directory for saved calibrations
│   ├── intrinsics/      # Saved intrinsic parameters (JSON, YAML, PKL)
│   └── extrinsics/      # Saved extrinsic parameters
├── tests/               # Test suite
│   ├── unit/            # Unit tests
│   └── integration/     # Integration tests
└── calibration_images/  # Captured images during calibration
    ├── intrinsics_manual_*
    ├── intrinsics_video_*
    └── intrinsics_conservative_*
```

---

## Part 1: INTRINSICS CALIBRATION

### What Are Intrinsics?

Camera intrinsic parameters define how a camera transforms 3D world points to 2D image pixels:

```
[u]     [fx  0  cx]  [X]
[v]  =  [ 0 fy cy]  [Y]  (after projection)
[1]     [ 0  0  1]  [Z]
```

**Parameters:**
- `fx, fy`: Focal lengths (pixels)
- `cx, cy`: Principal point (image center)
- `k1, k2, p1, p2, k3`: Distortion coefficients

### Three Capture Modes

#### **MODE 1: MANUAL** (Best for 1-2 cameras, high precision)
- Image-by-image with full manual control
- You press SPACE to capture each frame
- **Time:** 12-15 min/camera
- **Quality:** 0.3-0.6 px (excellent)
- **Best for:** Precision work, careful selection

```python
from intrinsics.camera_intrinsics_calibrator import (
    CameraConfig, CharUcoBoardConfig, CameraIntrinsicsCalibrator
)

camera_cfg = CameraConfig(
    name="Front Camera",
    camera_index=0,
    resolution=(1920, 1080)
)

board_cfg = CharUcoBoardConfig(
    name="board_5x7",
    aruco_dict_name="DICT_4X4_50",
    size=(5, 7),
    checker_size_mm=30.0,
    marker_size_mm=18.0
)

calibrator = CameraIntrinsicsCalibrator(camera_cfg, board_cfg)

# Capture 50 images manually
images = calibrator.capture_manual(target_images=50)

# Process images
corners, ids, valid_count = calibrator.process_dataset(images)

# Calibrate
result = calibrator.calibrate(corners, ids, images, capture_mode="manual")

# Save in multiple formats
files = calibrator.save_calibration(result)

# Display results
calibrator.print_results(result)
```

#### **MODE 2: VIDEO** (Best for speed, 3+ cameras)
- Continuous recording with automatic extraction
- **Time:** 3-5 min/camera
- **Quality:** 0.5-1.0 px (good)
- **Best for:** Fast capture, multiple cameras

```python
# Record ~30-60 seconds of continuous motion
# System automatically extracts 50 evenly-spaced frames
images = calibrator.capture_video(
    target_images=50,
    sampling_rate=5  # Every 5th frame
)
```

#### **MODE 3: CONSERVATIVE** (Best for production systems)
- Video + automatic quality filtering
- Analyzes blur, marker detection, and frame quality
- **Time:** 5-8 min/camera
- **Quality:** 0.3-0.7 px (excellent)
- **Best for:** Guaranteed quality, production

```python
# Record video with auto-quality filtering
images = calibrator.capture_conservative(
    target_images=50,
    min_markers=4,
    max_blur_ratio=0.3,
    min_detection_rate=0.7
)
```

### Output Formats

Calibrations are automatically saved as:

1. **PKL** - Python pickle (easiest to load)
   ```python
   import pickle
   with open('camera_intrinsics_manual.pkl', 'rb') as f:
       result = pickle.load(f)
       K = result.camera_matrix
       dist = result.dist_coeffs
   ```

2. **JSON** - Human-readable
   ```json
   {
     "camera_matrix": [[800.5, 0, 320], [0, 800.5, 240], [0, 0, 1]],
     "dist_coeffs": [0.1, -0.1, 0, 0, 0],
     "reprojection_error": 0.45
   }
   ```

3. **YAML** - ROS2 compatible
   ```yaml
   camera_name: Front Camera
   image_width: 1920
   image_height: 1080
   camera_matrix:
     rows: 3
     cols: 3
     data: [800.5, 0, 320, 0, 800.5, 240, 0, 0, 1]
   distortion_coefficients:
     rows: 1
     cols: 5
     data: [0.1, -0.1, 0, 0, 0]
   ```

### Quality Assessment

```
Reprojection Error    | Assessment
---------------------|-------------------
< 0.5 px             | Excellent ✓
0.5 - 1.0 px         | Good ✓
1.0 - 2.0 px         | Acceptable
> 2.0 px             | Poor ✗ (recalibrate)
```

---

## Part 2: EXTRINSICS CALIBRATION

### What Are Extrinsics?

Extrinsic parameters define transformations **between** cameras and other sensors.

#### **TYPE 1: HAND-EYE CALIBRATION** (Camera-to-Arm)

Computes transformation from robot end-effector to camera:

```
T_hand_eye = transformation from hand to camera
```

**Setup types:**
- **Eye-on-hand:** Camera mounted on robot arm
- **Eye-to-hand:** Camera fixed, looking at robot

**Usage:**

```python
from extrinsics.hand_eye_imu_calibrator import HandEyeCalibrator

camera_intrinsics = {
    'camera_matrix': np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]]),
    'dist_coeffs': np.array([0.1, -0.1, 0, 0, 0])
}

board_config = {
    'size': (5, 7),
    'checker_size_mm': 30.0,
    'marker_size_mm': 18.0
}

calibrator = HandEyeCalibrator(camera_intrinsics, board_config)

# Collect pose observations
for pose_id in range(10):
    # Get robot pose (4x4 matrix from robot controller)
    T_robot = get_robot_pose()
    
    # Detect board in image
    corners, ids = detect_board(frame)
    
    # Add observation
    calibrator.add_pose_observation(
        pose_id=pose_id,
        robot_base_to_hand=T_robot,
        board_corners=corners,
        board_ids=ids
    )

# Calibrate
result = calibrator.calibrate(setup_type="eye_on_hand")

# Save
files = calibrator.save_calibration(result)
```

#### **TYPE 2: MULTI-CAMERA STEREO** (Camera-to-Camera)

Computes relative pose between two cameras:

```
T_cam1_to_cam2 = [R | t] transformation from camera 1 to camera 2
```

**Usage:**

```python
from extrinsics.hand_eye_imu_calibrator import MultiCameraCalibrator

calibrator = MultiCameraCalibrator(
    camera1_intrinsics=cam1_K,
    camera2_intrinsics=cam2_K,
    board_config=board_config
)

# Collect synchronized observations
for observation_id in range(10):
    corners1, ids1 = detect_board(frame1)
    corners2, ids2 = detect_board(frame2)
    
    calibrator.add_observation(corners1, ids1, corners2, ids2)

# Calibrate
result = calibrator.calibrate()

# Save
files = calibrator.save_calibration(result)
```

#### **TYPE 3: IMU CALIBRATION** (Inertial Sensor)

Calibrates accelerometer and gyroscope biases:

```python
from extrinsics.hand_eye_imu_calibrator import IMUCalibrator

calibrator = IMUCalibrator(imu_id="imu_0")

# Collect static accelerometer data (6 orientations)
calibrator.add_accel_sample(0.1, 0.05, 9.81)
# ... more samples ...

# Collect gyro data (at rest)
calibrator.add_gyro_sample(0.01, 0.02, 0.01)
# ... more samples ...

# Calibrate
calibrator.calibrate_accelerometer()
calibrator.calibrate_gyroscope()

# Save
calibrator.save_calibration()
```

### ROS2 Integration

**TODO: Connect to state machine "CALIBRATION" mode**

When state machine enters "CALIBRATION":

```
1. Initialize calibrator based on /calibration/mode parameter
2. Subscribe to relevant topics:
   - Hand-eye: /robot/state/end_effector_pose + /camera/detections
   - Multi-camera: /camera1/detections + /camera2/detections
   - IMU: /imu/data_raw
3. Service endpoints:
   - /calibration/capture_pose
   - /calibration/add_stereo_pair
   - /calibration/finalize_calibration
4. Save artifacts on state exit
```

---

## Part 3: GENERATION TOOLS

### Generate ChArUco Boards

For camera intrinsic calibration:

```bash
cd generation/
python generate_charuco_board.py --rows 7 --cols 5 --checker-size 30 --marker-size 18 --output board.pdf
```

### Generate ArUco Tags

For environment markers:

```bash
cd generation/
python generate_aruco_tags.py --num-tags 50 --size 10 --output tags.pdf
```

---

## Part 4: TESTING

### Run Unit Tests

```bash
cd tests/unit/
python -m pytest test_intrinsics.py -v

# Or with unittest
python test_intrinsics.py
```

### Run Integration Tests

```bash
cd tests/integration/
python test_end_to_end_calibration.py
```

---

## Workflow Examples

### Example 1: Single Camera Intrinsic Calibration

```python
# 1. Setup
from intrinsics.camera_intrinsics_calibrator import *

camera_cfg = CameraConfig("Camera", camera_index=0, resolution=(1920, 1080))
board_cfg = CharUcoBoardConfig("board", "DICT_4X4_50", (5, 7), 30.0, 18.0)
calibrator = CameraIntrinsicsCalibrator(camera_cfg, board_cfg)

# 2. Capture (choose one mode)
# Option A: Manual
images = calibrator.capture_manual(target_images=50)

# Option B: Video
# images = calibrator.capture_video(target_images=50)

# Option C: Conservative
# images = calibrator.capture_conservative(target_images=50)

# 3. Process and calibrate
corners, ids, _ = calibrator.process_dataset(images)
result = calibrator.calibrate(corners, ids, images, "manual")

# 4. Save
files = calibrator.save_calibration(result)

# 5. Results
calibrator.print_results(result)
print(f"Saved to: {files}")
```

### Example 2: Multi-Camera System

```python
# Calibrate intrinsics for each camera
for camera_id, camera_config in CAMERAS.items():
    calibrator = CameraIntrinsicsCalibrator(camera_config, board_cfg)
    images = calibrator.capture_video(target_images=50)
    corners, ids, _ = calibrator.process_dataset(images)
    result = calibrator.calibrate(corners, ids, images, "video")
    calibrator.save_calibration(result)

# Then calibrate extrinsics between cameras
stereo_calibrator = MultiCameraCalibrator(
    cam1_intrinsics, cam2_intrinsics, board_cfg
)
# ... collect observations ...
result = stereo_calibrator.calibrate()
stereo_calibrator.save_calibration(result)
```

### Example 3: Hand-Eye for Arm

```python
# First calibrate camera intrinsics (see Example 1)

# Then calibrate hand-eye
calibrator = HandEyeCalibrator(camera_intrinsics, board_config)

# Move arm to 10 different poses
for i in range(10):
    # Get current arm pose from robot controller
    T_robot = get_arm_pose_from_controller()
    
    # Capture image and detect board
    frame = capture_image()
    corners, ids = detect_board(frame)
    
    # Add observation
    calibrator.add_pose_observation(i, T_robot, corners, ids)
    
    print(f"Pose {i} captured")

# Calibrate
result = calibrator.calibrate(setup_type="eye_on_hand")
files = calibrator.save_calibration(result)
print(f"Hand-eye transform saved: {files}")
```

---

## Directory Structure

```
Autonomy/calibration/

intrinsics/
├── camera_intrinsics_calibrator.py    # Main intrinsics calibrator
├── README.md                           # Usage guide
└── __init__.py

extrinsics/
├── hand_eye_imu_calibrator.py         # Hand-eye, multi-camera, IMU
├── ros2_calibration_node.py           # ROS2 integration (TODO)
└── README.md

generation/
├── generate_charuco_board.py           # ChArUco board generator
├── generate_aruco_tags.py              # ArUco tag generator
└── README.md

artifacts/
├── intrinsics/
│   ├── front_camera_intrinsics_manual.pkl
│   ├── front_camera_intrinsics_manual.json
│   ├── front_camera_intrinsics_manual.yaml
│   └── ...
└── extrinsics/
    ├── hand_eye_eye_on_hand_*.pkl
    ├── multicam_cam1_to_cam2.json
    └── imu_imu_0_calibration.json

calibration_images/
├── intrinsics_manual_front_camera_20250101_000000/
├── intrinsics_video_front_camera_20250101_000100/
└── ...

tests/
├── unit/
│   ├── test_intrinsics.py
│   ├── test_extrinsics.py
│   └── __init__.py
├── integration/
│   ├── test_end_to_end_calibration.py
│   └── __init__.py
└── README.md
```

---

## State Machine Integration

### Entering "CALIBRATION" State

```python
# In state_machine_director.py
if new_state == "CALIBRATION":
    calibration_mode = get_parameter("/calibration/mode")
    # "intrinsic" or "hand_eye" or "stereo" or "imu"
    
    if calibration_mode == "hand_eye":
        initialize_hand_eye_calibrator()
        subscribe_to_robot_poses()
        subscribe_to_camera_detections()
```

### Exiting "CALIBRATION" State

```python
# Save all collected data
result = calibrator.calibrate()
calibrator.save_calibration(result)

# Publish success/failure
publish_calibration_result(result)
```

---

## Quality Checklist

Before deploying calibration:

- [ ] Reprojection error < 1.0 px
- [ ] All 3 output formats generated (PKL, JSON, YAML)
- [ ] Timestamp recorded
- [ ] Source images archived
- [ ] Quality metrics documented
- [ ] Tested with actual hardware

---

## Troubleshooting

### Problem: "No markers detected"
- **Cause:** ChArUco board not visible, bad lighting, or camera out of focus
- **Solution:** Ensure board is in frame, improve lighting, check focus

### Problem: "High reprojection error (> 2 px)"
- **Cause:** Not enough diverse poses, poor image quality, or board damage
- **Solution:** Recapture with more varied angles/distances, check board flatness

### Problem: "Hand-eye calibration fails"
- **Cause:** Robot poses not synchronized with image captures
- **Solution:** Ensure timestamps match, collect more diverse poses

---

## Performance Targets

| Metric | Target | Acceptable | Poor |
|--------|--------|-----------|------|
| Reprojection Error | < 0.5 px | 0.5-1.0 px | > 2.0 px |
| Collection Time (Manual) | 12-15 min | 15-20 min | > 30 min |
| Collection Time (Video) | 3-5 min | 5-10 min | > 15 min |
| Collection Time (Conservative) | 5-8 min | 8-12 min | > 20 min |

---

## Next Steps

1. **Implement ROS2 node** for state machine integration
2. **Add temperature compensation** for IMU calibration
3. **Implement auto-alignment detection** for camera-IMU
4. **Add validation script** for field verification
5. **Create calibration database** for tracking all calibrations

