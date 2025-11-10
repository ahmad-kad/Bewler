==================
Camera Calibration
==================

Camera calibration procedures for the URC 2026 rover.

Calibration Overview
====================

Camera calibration is essential for accurate computer vision and navigation. The system supports multiple calibration methods:

- **Intrinsic calibration**: Correct lens distortion and focal length
- **Extrinsic calibration**: Determine camera position relative to rover
- **Hand-eye calibration**: Coordinate system alignment

Tools Available
===============

**Python Calibration Scripts:**

- ``calibrate_from_markers.py``: ArUco marker-based calibration
- ``calibrate_multiple_cameras.py``: Multi-camera setup
- ``quick_calibration.py``: Rapid calibration for testing

**ROS2 Services:**

- ``CalibrateCamera``: Camera intrinsic calibration
- ``LoadCalibrationParameters``: Load saved calibration data
- ``ValidateCalibration``: Check calibration quality

Procedure
=========

1. **Prepare calibration target**: Print ChArUco board or use ArUco markers
2. **Position camera**: Ensure good lighting and stable setup
3. **Capture frames**: Move target around to get diverse viewpoints
4. **Run calibration**: Use appropriate calibration script
5. **Validate results**: Check reprojection error and coverage
6. **Save parameters**: Store calibration data for runtime use

Quality Metrics
===============

- **Reprojection error**: < 1.0 pixels ideal
- **Coverage**: All image regions should be calibrated
- **Stability**: Results should be consistent across runs

Troubleshooting
===============

**Poor calibration results:**

- Check lighting conditions
- Ensure target is in focus
- Increase number of calibration frames
- Verify target dimensions are correct

**Runtime issues:**

- Check calibration file paths
- Verify camera intrinsics are loaded correctly
- Monitor for calibration drift over time
