#!/usr/bin/env python3
"""
Keyboard Localization Module

Detects ArUco markers on keyboard corners, estimates keyboard pose,
and broadcasts coordinate transforms for keyboard frame.
"""

import os
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class KeyboardLocalizationNode(Node):
    """
    ROS 2 node for keyboard marker detection and pose estimation.

    Publishes:
    - /keyboard_pose: PoseStamped with keyboard position in camera frame
    - TF: camera_optical_frame -> keyboard_frame transform

    Subscribes:
    - /camera/color/image_raw: Camera image stream
    - /camera/color/camera_info: Camera intrinsic parameters
    """

    def __init__(self):
        super().__init__("keyboard_localization_node")

        # Parameters
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("marker_dict", "DICT_4X4_50")
        self.declare_parameter("marker_size", 0.03)  # 3cm markers
        self.declare_parameter("config_file", "keyboard_config.yaml")

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # TF broadcaster and listener
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera calibration
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_info_received = False

        # Marker configuration (map marker_id -> 3D position in keyboard frame)
        self.marker_positions: Dict[int, np.ndarray] = {
            0: np.array([0.0, 0.0, 0.0]),  # Top-left corner
            1: np.array([0.3, 0.0, 0.0]),  # Top-right corner
            2: np.array([0.0, 0.15, 0.0]),  # Bottom-left corner
            3: np.array([0.3, 0.15, 0.0]),  # Bottom-right corner
        }

        # Load configuration from file if available
        self._load_configuration()

        # Publishers
        self.keyboard_pose_pub = self.create_publisher(PoseStamped, "keyboard_pose", 10)
        self.debug_image_pub = self.create_publisher(Image, "keyboard_detection_image", 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, self.get_parameter("camera_topic").value, self._image_callback, 10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_callback,
            10,
        )

        # State tracking
        self.last_keyboard_pose: Optional[Dict] = None
        self.detection_count = 0
        self.failed_detection_count = 0

        self.get_logger().info("Keyboard localization node initialized")

    def _load_configuration(self) -> None:
        """Load keyboard marker configuration from YAML file."""
        try:
            config_file = self.get_parameter("config_file").value

            # Try multiple paths
            possible_paths = [
                config_file,
                os.path.expanduser(f"~/{config_file}"),
                f"/tmp/{config_file}",
            ]

            config_path = None
            for path in possible_paths:
                if os.path.exists(path):
                    config_path = path
                    break

            if config_path:
                with open(config_path, "r") as f:
                    config = yaml.safe_load(f)

                # Load marker positions if available
                if "markers" in config:
                    self.marker_positions = {int(k): np.array(v) for k, v in config["markers"].items()}
                    self.get_logger().info(f"Loaded {len(self.marker_positions)} marker positions from config")
            else:
                self.get_logger().warn(f"Configuration file not found, using default marker positions")

        except Exception as e:
            self.get_logger().warn(f"Failed to load configuration: {e}")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Extract camera calibration from CameraInfo message."""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            self.camera_info_received = True

            self.get_logger().info(
                "Camera calibration received:\n" f"  Matrix: {self.camera_matrix}\n" f"  Distortion: {self.dist_coeffs}"
            )

    def _image_callback(self, msg: Image) -> None:
        """Process incoming camera image for keyboard marker detection."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Detect ArUco markers
            corners, ids, rejected = self.detector.detectMarkers(cv_image)

            # Estimate keyboard pose if markers detected
            if ids is not None and len(ids) > 0:
                keyboard_pose = self._estimate_keyboard_pose(corners, ids, cv_image)

                if keyboard_pose is not None:
                    self.last_keyboard_pose = keyboard_pose
                    self.detection_count += 1

                    # Publish keyboard pose
                    self._publish_keyboard_pose(keyboard_pose, msg.header)

                    # Broadcast TF transform
                    self._broadcast_keyboard_transform(keyboard_pose, msg.header)

                    # Publish annotated image for debugging
                    annotated = cv_image.copy()
                    cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
                    self._publish_debug_image(annotated, msg.header)
            else:
                self.failed_detection_count += 1

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def _estimate_keyboard_pose(self, corners: List, ids: np.ndarray, image: np.ndarray) -> Optional[Dict]:
        """
        Estimate keyboard pose from detected markers using PnP.

        Returns dict with 'position' and 'orientation' (as quaternion)
        """
        try:
            if self.camera_matrix is None:
                self.get_logger().warn("Camera calibration not yet received")
                return None

            # Collect 3D-2D correspondences from detected markers
            object_points = []
            image_points = []

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in self.marker_positions:
                    # Use first corner of marker as 2D image point
                    corner_2d = corners[i][0][0]
                    image_points.append(corner_2d)

                    # Use marker position in keyboard frame as 3D point
                    object_points.append(self.marker_positions[marker_id])

            if len(object_points) < 3:
                self.get_logger().warn(f"Not enough marker correspondences: {len(object_points)}/3")
                return None

            object_points = np.array(object_points, dtype=np.float32)
            image_points = np.array(image_points, dtype=np.float32)

            # Solve PnP to get keyboard pose (rotation + translation)
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            if not success:
                self.get_logger().warn("PnP solver failed")
                return None

            # Convert rotation vector to quaternion
            R, _ = cv2.Rodrigues(rvec)
            quaternion = self._rotation_matrix_to_quaternion(R)

            return {
                "position": tvec.flatten(),
                "orientation": quaternion,
                "rotation_matrix": R,
                "rvec": rvec,
                "tvec": tvec,
                "num_markers": len(object_points),
            }

        except Exception as e:
            self.get_logger().error(f"Pose estimation error: {e}")
            return None

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = np.trace(R)
        q = np.zeros(4)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s

        return q

    def _publish_keyboard_pose(self, pose_data: Dict, header) -> None:
        """Publish keyboard pose as PoseStamped."""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "camera_color_optical_frame"

        # Position
        pose_msg.pose.position.x = float(pose_data["position"][0])
        pose_msg.pose.position.y = float(pose_data["position"][1])
        pose_msg.pose.position.z = float(pose_data["position"][2])

        # Orientation (quaternion)
        pose_msg.pose.orientation.x = float(pose_data["orientation"][0])
        pose_msg.pose.orientation.y = float(pose_data["orientation"][1])
        pose_msg.pose.orientation.z = float(pose_data["orientation"][2])
        pose_msg.pose.orientation.w = float(pose_data["orientation"][3])

        self.keyboard_pose_pub.publish(pose_msg)

    def _broadcast_keyboard_transform(self, pose_data: Dict, header) -> None:
        """Broadcast camera -> keyboard transform."""
        transform = TransformStamped()
        transform.header = header
        transform.header.frame_id = "camera_color_optical_frame"
        transform.child_frame_id = "keyboard_frame"

        # Translation
        transform.transform.translation.x = float(pose_data["position"][0])
        transform.transform.translation.y = float(pose_data["position"][1])
        transform.transform.translation.z = float(pose_data["position"][2])

        # Rotation (quaternion)
        transform.transform.rotation.x = float(pose_data["orientation"][0])
        transform.transform.rotation.y = float(pose_data["orientation"][1])
        transform.transform.rotation.z = float(pose_data["orientation"][2])
        transform.transform.rotation.w = float(pose_data["orientation"][3])

        self.tf_broadcaster.sendTransform(transform)

    def _publish_debug_image(self, image: np.ndarray, header) -> None:
        """Publish annotated image for debugging."""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            ros_image.header = header
            self.debug_image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Debug image publishing error: {e}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = KeyboardLocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
