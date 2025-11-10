"""
Computer Vision Subsystem Node.

URC 2026 Computer Vision implementation providing ArUco marker detection,
competition object recognition, and real-time vision processing.
"""

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from autonomy_interfaces.msg import VisionDetection
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String


class ComputerVisionNode(Node):
    """
    Computer vision processing for URC 2026 competition.

    Provides ArUco marker detection, competition object recognition,
    and real-time vision processing for navigation assistance.
    """

    def __init__(self) -> None:
        """Initialize computer vision node with camera interfaces."""
        super().__init__('computer_vision_node')

        # Publishers
        self.detection_publisher = self.create_publisher(
            VisionDetection, 'vision/detections', 10)
        self.status_publisher = self.create_publisher(
            String, 'computer_vision/status', 10)
        self.debug_image_publisher = self.create_publisher(
            Image, 'vision/debug_image', 10)

        # Camera data republishers (for data flow and monitoring)
        self.camera_image_republisher = self.create_publisher(
            Image, 'camera/image_raw', 10)
        self.camera_depth_republisher = self.create_publisher(
            Image, 'camera/depth/image_raw', 10)
        self.camera_info_republisher = self.create_publisher(
            CameraInfo, 'camera/camera_info', 10)

        # Subscribers
        self.image_subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, 'camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.camera_info_callback, 10)

        # CV Bridge for ROS/OpenCV conversion
        self.bridge = CvBridge()

        # ArUco marker detection setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera parameters (placeholder - would be calibrated)
        self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # Competition targets
        self.target_objects = {
            'mallet': {'color': 'orange', 'shape': 'cylindrical'},
            'water_bottle': {'color': 'various', 'shape': 'cylindrical'}
        }

        # Processing state
        self.last_image: Optional[np.ndarray] = None
        self.current_detections: List[VisionDetection] = []

        # Control timer
        self.processing_timer = self.create_timer(0.1, self.process_image)  # 10 Hz
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('Computer Vision node initialized')

    def image_callback(self, msg: Image):
        """Handle incoming camera images."""
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Republish camera image for other nodes
            self.camera_image_republisher.publish(msg)

            # Create and publish camera info (placeholder)
            camera_info = CameraInfo()
            camera_info.header = msg.header
            camera_info.width = msg.width
            camera_info.height = msg.height
            camera_info.distortion_model = "plumb_bob"
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
            camera_info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]  # Camera matrix
            camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Rectification matrix
            camera_info.p = [600.0, 0.0, 320.0, 0.0, 0.0, 600.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]  # Projection matrix

            self.camera_info_republisher.publish(camera_info)

        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def depth_callback(self, msg: Image):
        """Handle incoming depth images."""
        # Republish depth image for other nodes
        self.camera_depth_republisher.publish(msg)

    def camera_info_callback(self, msg: CameraInfo):
        """Handle incoming camera info."""
        # Republish camera info for other nodes
        self.camera_info_republisher.publish(msg)

    def process_image(self):
        """Main vision processing loop."""
        if self.last_image is None:
            return

        # Clear previous detections
        self.current_detections.clear()

        # Process ArUco markers
        self.detect_aruco_markers()

        # Process competition objects
        self.detect_competition_objects()

        # Publish detections
        for detection in self.current_detections:
            self.detection_publisher.publish(detection)

        # Publish debug image (optional)
        if self.debug_image_publisher.get_subscription_count() > 0:
            self.publish_debug_image()

    def detect_aruco_markers(self):
        """Detect ArUco markers in the image."""
        if self.last_image is None:
            return

        # Convert to grayscale
        gray = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            # Estimate pose for each marker
            for i, marker_id in enumerate(ids.flatten()):
                # Pose estimation
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], 0.05, self.camera_matrix, self.dist_coeffs)

                if rvecs is not None and tvecs is not None:
                    # Convert rotation vector to quaternion
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
                    quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

                    # Create detection message
                    detection = VisionDetection()
                    detection.header = self.create_header()
                    detection.object_type = f'aruco_{marker_id}'
                    detection.confidence = 0.95
                    detection.position.x = tvecs[0][0][0]
                    detection.position.y = tvecs[0][0][1]
                    detection.position.z = tvecs[0][0][2]
                    detection.orientation.x = quaternion[0]
                    detection.orientation.y = quaternion[1]
                    detection.orientation.z = quaternion[2]
                    detection.orientation.w = quaternion[3]

                    self.current_detections.append(detection)

                    # Draw marker on debug image
                    cv2.aruco.drawDetectedMarkers(self.last_image, [corners[i]], ids[i:i+1])
                    cv2.aruco.drawAxis(self.last_image, self.camera_matrix, self.dist_coeffs,
                                     rvecs[0], tvecs[0], 0.1)

    def detect_competition_objects(self):
        """Detect competition objects (mallet, water bottle)."""
        if self.last_image is None:
            return

        # Convert to HSV for color-based detection
        hsv = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2HSV)

        # Detect orange mallet (placeholder implementation)
        # In practice, this would use trained ML models or sophisticated CV algorithms
        orange_lower = np.array([5, 50, 50])
        orange_upper = np.array([15, 255, 255])
        orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)

        # Find contours in mask
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)

                # Estimate position (simplified - would use depth sensing in practice)
                distance = 2.0  # Placeholder distance estimate
                angle = ((x + w/2) - self.last_image.shape[1]/2) * 0.002  # Rough angle estimate

                # Create detection message
                detection = VisionDetection()
                detection.header = self.create_header()
                detection.object_type = 'mallet'
                detection.confidence = 0.7  # Lower confidence for color-based detection
                detection.position.x = distance * math.sin(angle)
                detection.position.y = 0.0
                detection.position.z = distance * math.cos(angle)
                detection.orientation.w = 1.0  # No rotation info

                self.current_detections.append(detection)

                # Draw bounding box on debug image
                cv2.rectangle(self.last_image, (x, y), (x+w, y+h), (0, 255, 255), 2)
                cv2.putText(self.last_image, 'Mallet', (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    def rotation_matrix_to_quaternion(self, R: np.ndarray) -> Tuple[float, float, float, float]:
        """Convert rotation matrix to quaternion."""
        q = np.zeros(4)
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return tuple(q)

    def publish_debug_image(self):
        """Publish debug image with detections overlaid."""
        if self.last_image is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(self.last_image, encoding='bgr8')
            debug_msg.header = self.create_header()
            self.debug_image_publisher.publish(debug_msg)

    def create_header(self) -> Header:
        """Create a standard ROS2 header."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera'
        return header

    def status_callback(self):
        """Publish vision system status."""
        status_msg = String()
        num_detections = len(self.current_detections)
        status_msg.data = f'Vision operational - {num_detections} detections'
        self.status_publisher.publish(status_msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ComputerVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
