"""
Camera calibration functions for the URC 2026 system.
Supports both standalone calibration and ROS 2 CameraInfo publishing for distributed architecture.
"""

import time
from typing import Optional, Dict, Any

try:
    import cv2
    import numpy as np

    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None  # type: ignore
    numpy = None  # type: ignore
    OPENCV_AVAILABLE = False

# ROS 2 imports (optional for standalone calibration)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo
    from builtin_interfaces.msg import Time
    ROS2_AVAILABLE = True
except ImportError:
    # ROS 2 not available - create dummy classes for type hints
    rclpy = None
    Node = type  # Use type as dummy base class
    CameraInfo = type
    Time = type
    ROS2_AVAILABLE = False


def calibrate_camera(
    camera_index: int = 0,
    duration: int = 30,
    auto_capture: bool = True,
    camera_name: Optional[str] = None,
    calibration_dir: str = "calibrations",
) -> bool:
    """
    Calibrate single camera with visual feedback.

    Args:
        camera_index: Camera device index
        duration: Calibration duration in seconds
        auto_capture: Whether to auto-capture frames
        camera_name: Optional camera name (auto-generated if None)
        calibration_dir: Directory to save calibrations

    Returns:
        True if calibration successful, False otherwise
    """
    if not OPENCV_AVAILABLE:
        print("OpenCV not available for camera calibration")
        return False

    from ..utils.camera import (
        detect_camera_sensor,
        generate_sensor_based_camera_id,
        save_calibration_file,
    )

    print(f"Calibrating Camera {camera_index}")
    print("=" * 40)

    # Detect sensor before opening camera
    sensor = detect_camera_sensor()
    if sensor and sensor != "unknown_sensor":
        print(f"Detected sensor: {sensor.upper()}")
    else:
        print("Could not detect camera sensor")

    # Generate camera name if not provided
    if camera_name is None:
        camera_name = generate_sensor_based_camera_id(calibration_dir)
        print(f"Auto-assigned camera ID: {camera_name}")

    print(f"Calibrating Camera {camera_index} ({camera_name})")
    print("=" * 50)

    # Open camera
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("Cannot open camera")
        return False

    # Configure camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Resolution: {width}x{height}")

    # Setup detection
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard((7, 5), 0.030, 0.018, aruco_dict)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    charuco_detector = cv2.aruco.CharucoDetector(board)

    # Storage for calibration data
    all_corners = []
    all_ids = []
    captured = 0

    print(f"Move ChArUco board around for {duration}s")
    print("   Auto-capture: ON" if auto_capture else "   Press SPACE to capture")
    print("   Press Q to quit\n")

    start_time = time.time()
    frame_count = 0

    try:
        while time.time() - start_time < duration and captured < 15:
            ret, frame = cap.read()
            if not ret:
                break

            frame_count += 1
            marker_count = 0
            status = "Searching..."
            color = (0, 0, 255)  # Red

            # Detect markers
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                marker_count = len(ids)

                # Check for good board detection
                if marker_count >= 4:
                    charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(
                        frame
                    )
                    if charuco_corners is not None and len(charuco_corners) >= 6:
                        status = f"{captured + 1}/15 Ready!"
                        color = (0, 255, 0)  # Green

            # Show status on frame
            cv2.putText(
                frame,
                f"Status: {status}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )
            cv2.putText(
                frame,
                f"Captured: {captured}/15",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                f"Time: {int(time.time() - start_time)}/{duration}s",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            cv2.imshow("Camera Calibration", frame)

            # Auto capture or manual
            should_capture = False
            if auto_capture and status == f"{captured + 1}/15 Ready!":
                should_capture = True
            elif not auto_capture:
                key = cv2.waitKey(1) & 0xFF
                if key == ord(" "):  # Spacebar
                    should_capture = True
                elif key == ord("q"):
                    break

            if (
                should_capture
                and charuco_corners is not None
                and charuco_ids is not None
            ):
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)
                captured += 1
                print(f"Captured frame {captured}/15")
                time.sleep(0.5)  # Brief pause between captures

        cv2.destroyAllWindows()
        cap.release()

        if captured < 5:
            print(f"Insufficient frames captured: {captured}/15")
            return False

        # Perform calibration
        print("Performing calibration...")
        ret, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
            all_corners, all_ids, board, (width, height), None, None
        )

        if not ret:
            print("Calibration failed")
            return False

        # Prepare calibration data
        calib_data = {
            "camera_index": camera_index,
            "sensor": sensor,
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": dist_coeffs.flatten().tolist(),
            "image_width": width,
            "image_height": height,
            "frames_used": captured,
            "reprojection_error": 0.0,
            "opencv_version": cv2.__version__,
            "calibration_method": "charuco",
        }

        # Save calibration file
        save_calibration_file(calib_data, camera_name, calibration_dir)

        print("Calibration complete!")
        print(f"    Camera: {camera_name}")
        print(f"    Quality: {captured} frames used")
        print(f"    Directory: {calibration_dir}")
        return True

    except Exception as e:
        print(f"Error: {e}")
        cv2.destroyAllWindows()
        if "cap" in locals():
            cap.release()
        return False


def batch_calibrate(
    num_cameras: int = 5, calibration_dir: str = "calibrations"
) -> bool:
    """
    Calibrate multiple cameras in sequence.

    Args:
        num_cameras: Number of cameras to calibrate
        calibration_dir: Directory to save calibrations

    Returns:
        True if all calibrations successful
    """
    print(f"Batch calibrating {num_cameras} cameras")
    print("=" * 50)

    results = []
    for i in range(num_cameras):
        print(f"\n{'='*50}")
        print(f"Camera {i+1}/{num_cameras} (Device {i})")
        print(f"{'='*50}")

        success = calibrate_camera(
            camera_index=i, duration=25, calibration_dir=calibration_dir
        )
        results.append((i, success))

        if success:
            print(f"Camera {i}: SUCCESS")
        else:
            print(f"Camera {i}: FAILED")

        if i < num_cameras - 1:
            input("\nConnect next camera and press ENTER...")

    # Summary
    print(f"\n{'='*50}")
    print("CALIBRATION SUMMARY")
    print(f"{'='*50}")

    successful = sum(1 for _, success in results if success)
    print(f"Total cameras: {num_cameras}")
    print(f"Successful: {successful}")
    print(f"Failed: {num_cameras - successful}")

    from ..utils.camera import list_calibrated_cameras

    if successful > 0:
        print(f"\nCalibrations saved to: {calibration_dir}/")
        cameras = list_calibrated_cameras(calibration_dir)
        print(f"Calibrated cameras: {', '.join(cameras)}")

    return successful == num_cameras


class CalibrationPublisher(Node):
    """
    ROS 2 node for publishing camera calibration data as CameraInfo messages.
    Used in distributed architecture where Pi 5 calibrates and Pi Zero 2 W loads intrinsics.
    """

    def __init__(self):
        super().__init__('calibration_publisher')
        self.calib_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 1)
        self.get_logger().info('Calibration publisher initialized')

    def publish_calibration(self, calib_data: Dict[str, Any], frame_id: str = "camera_link") -> bool:
        """
        Publish calibration data as ROS CameraInfo message.

        Args:
            calib_data: Calibration data dictionary from calibration
            frame_id: ROS TF frame ID for the camera

        Returns:
            True if published successfully
        """
        if not ROS2_AVAILABLE:
            print("ROS 2 not available for CameraInfo publishing")
            return False

        try:
            msg = CameraInfo()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.height = calib_data['image_height']
            msg.width = calib_data['image_width']
            msg.distortion_model = "plumb_bob"
            msg.d = calib_data['distortion_coefficients']
            msg.k = calib_data['camera_matrix']

            # Add identity rectification matrices if not present
            if 'rectification_matrix' not in calib_data:
                msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            else:
                msg.r = calib_data['rectification_matrix']

            if 'projection_matrix' not in calib_data:
                # Create basic projection matrix from intrinsics
                fx = msg.k[0]
                fy = msg.k[4]
                cx = msg.k[2]
                cy = msg.k[5]
                msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            else:
                msg.p = calib_data['projection_matrix']

            self.calib_pub.publish(msg)
            self.get_logger().info(f'Published calibration for {frame_id}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to publish calibration: {e}')
            return False


def publish_calibration_to_ros(
    calib_data: Dict[str, Any],
    camera_name: str,
    frame_id: Optional[str] = None,
    ros_domain_id: int = 0
) -> bool:
    """
    Publish calibration data to ROS 2 network for distributed architecture.

    Args:
        calib_data: Calibration data dictionary
        camera_name: Name of the camera
        frame_id: ROS TF frame ID (auto-generated if None)
        ros_domain_id: ROS domain ID for isolation

    Returns:
        True if published successfully
    """
    if not ROS2_AVAILABLE:
        print("ROS 2 not available for calibration publishing")
        return False

    # Set ROS domain for isolation
    import os
    os.environ['ROS_DOMAIN_ID'] = str(ros_domain_id)

    # Initialize ROS 2
    rclpy.init()

    try:
        # Create publisher node
        node = CalibrationPublisher()

        # Auto-generate frame_id if not provided
        if frame_id is None:
            frame_id = f"{calib_data.get('sensor', 'camera')}_{camera_name}_link"

        # Publish calibration
        success = node.publish_calibration(calib_data, frame_id)

        # Keep node alive briefly to ensure message is sent
        rclpy.spin_once(node, timeout_sec=0.1)

        return success

    except Exception as e:
        print(f"Error publishing calibration to ROS: {e}")
        return False
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def calibrate_and_publish(
    camera_index: int = 0,
    duration: int = 30,
    auto_capture: bool = True,
    camera_name: Optional[str] = None,
    calibration_dir: str = "calibrations",
    publish_to_ros: bool = False,
    ros_domain_id: int = 0
) -> bool:
    """
    Calibrate camera and optionally publish calibration to ROS 2 network.

    This function combines calibration with ROS 2 publishing for the distributed
    architecture where Pi 5 calibrates cameras and Pi Zero 2 W loads intrinsics.

    Args:
        camera_index: Camera device index
        duration: Calibration duration in seconds
        auto_capture: Whether to auto-capture frames
        camera_name: Optional camera name
        calibration_dir: Directory to save calibrations
        publish_to_ros: Whether to publish calibration to ROS network
        ros_domain_id: ROS domain ID for isolation

    Returns:
        True if calibration successful and publishing succeeded (if requested)
    """
    # Perform calibration
    calib_success = calibrate_camera(
        camera_index=camera_index,
        duration=duration,
        auto_capture=auto_capture,
        camera_name=camera_name,
        calibration_dir=calibration_dir
    )

    if not calib_success:
        return False

    # Publish to ROS if requested
    if publish_to_ros and calib_success:
        from ..utils.camera import load_calibration_file

        # Load the calibration data we just saved
        if camera_name is None:
            from ..utils.camera import generate_sensor_based_camera_id
            camera_name = generate_sensor_based_camera_id(calibration_dir)

        try:
            calib_data = load_calibration_file(camera_name, calibration_dir)
            ros_success = publish_calibration_to_ros(
                calib_data, camera_name, ros_domain_id=ros_domain_id
            )

            if ros_success:
                print(f"Calibration published to ROS 2 network (domain {ros_domain_id})")
            else:
                print("Warning: Calibration completed but ROS publishing failed")

            return ros_success

        except Exception as e:
            print(f"Error loading/publishing calibration: {e}")
            return False

    return calib_success
