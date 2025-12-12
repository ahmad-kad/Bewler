#!/usr/bin/env python3
"""
Follow-Me Mission - ROS2 Node

Executes follow-me missions using ArUco marker tracking:
- Real-time ArUco marker detection and pose estimation
- Adaptive following behavior with safety margins
- SLAM integration for global localization
- Velocity control for smooth following

Author: URC 2026 Autonomy Team
"""

# Standard Library
import math
import time
from enum import Enum
from typing import Optional

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy

# ROS2 Messages
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class FollowMeState(Enum):
    """States for follow-me mission"""

    IDLE = "idle"
    SEARCHING = "searching"
    FOLLOWING = "following"
    LOST = "lost"
    STOPPED = "stopped"
    FAILED = "failed"


class FollowMeMission(Node):
    """
    Follow-Me Mission Node

    Subscribes to:
    - /camera/image_raw: RGB camera feed
    - /camera/camera_info: Camera calibration info
    - /slam/pose: SLAM pose for global localization
    - /odom: Odometry for velocity feedback

    Publishes:
    - /mission/follow/status: Mission status
    - /mission/follow/target_pose: Target marker pose
    - /mission/follow/cmd_vel: Following velocity commands
    - /mission/follow/markers: Visualization markers
    - /mission/follow/distance: Distance to target

    Services:
    - /mission/follow/start: Start following (tag_id)
    - /mission/follow/stop: Stop following
    - /mission/follow/status: Get current status
    """

    def __init__(self):
        super().__init__("follow_me_mission")

        # Mission state
        self.state = FollowMeState.IDLE
        self.target_tag_id = None
        self.target_pose: Optional[PoseStamped] = None
        self.last_detection_time = None
        self.mission_start_time = None

        # Following parameters
        self.declare_parameter("target_tag_id", 42)
        self.declare_parameter("safety_distance", 2.0)  # meters
        self.declare_parameter("max_follow_distance", 5.0)  # meters
        self.declare_parameter("min_follow_distance", 1.0)  # meters
        self.declare_parameter("lost_timeout", 3.0)  # seconds
        self.declare_parameter("max_linear_velocity", 1.0)  # m/s
        self.declare_parameter("max_angular_velocity", 0.5)  # rad/s
        self.declare_parameter("position_tolerance", 0.2)  # meters
        self.declare_parameter("angle_tolerance", 0.1)  # radians

        self.target_tag_id = self.get_parameter("target_tag_id").value
        self.safety_distance = self.get_parameter("safety_distance").value
        self.max_follow_distance = self.get_parameter("max_follow_distance").value
        self.min_follow_distance = self.get_parameter("min_follow_distance").value
        self.lost_timeout = self.get_parameter("lost_timeout").value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.angle_tolerance = self.get_parameter("angle_tolerance").value

        # ArUco setup (simplified for compatibility)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        # Note: ArucoDetector not available in this OpenCV version, using detectMarkers directly

        # Publishers
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.status_pub = self.create_publisher(
            String, "/mission/follow/status", qos_reliable
        )
        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/mission/follow/target_pose", qos_reliable
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/mission/follow/cmd_vel", qos_reliable
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, "/mission/follow/markers", qos_reliable
        )
        self.distance_pub = self.create_publisher(
            Float32, "/mission/follow/distance", qos_reliable
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.camera_info_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/slam/pose", self.pose_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Mission control services
        self.start_srv = self.create_service(
            Trigger, "/mission/follow/start", self.start_mission_callback
        )
        self.stop_srv = self.create_service(
            Trigger, "/mission/follow/stop", self.stop_mission_callback
        )
        self.status_srv = self.create_service(
            Trigger, "/mission/follow/status", self.get_status_callback
        )

        # Current sensor data
        self.current_image: Optional[np.ndarray] = None
        self.camera_info: Optional[CameraInfo] = None
        self.current_pose: Optional[PoseStamped] = None
        self.current_odom: Optional[Odometry] = None

        # Status update timer
        self.status_timer = self.create_timer(0.5, self.status_update)
        self.control_timer = self.create_timer(0.1, self.control_update)

        # Mission execution
        self.stop_execution = False

        self.get_logger().info("Follow-Me Mission initialized")

    # Sensor data callbacks
    def image_callback(self, msg: Image):
        """Process camera image and detect ArUco markers"""
        try:
            # Convert ROS Image to OpenCV
            self.current_image = self.ros_image_to_cv2(msg)

            # Detect ArUco markers
            self.detect_aruco_markers()

        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera calibration info"""
        self.camera_info = msg

    def pose_callback(self, msg: PoseStamped):
        """Update SLAM pose"""
        self.current_pose = msg

    def odom_callback(self, msg: Odometry):
        """Update odometry"""
        self.current_odom = msg

    # Mission control callbacks
    def start_mission_callback(self, request, response):
        """Start follow-me mission"""
        if self.state == FollowMeState.IDLE:
            self.target_tag_id = self.get_parameter("target_tag_id").value
            self.start_mission()
            response.success = True
            response.message = f"Started following ArUco tag {self.target_tag_id}"
        else:
            response.success = False
            response.message = f"Cannot start mission in state {self.state.value}"
        return response

    def stop_mission_callback(self, request, response):
        """Stop follow-me mission"""
        self.stop_execution = True
        self.state = FollowMeState.IDLE
        self.target_pose = None

        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        response.success = True
        response.message = "Follow-me mission stopped"
        return response

    def get_status_callback(self, request, response):
        """Get current mission status"""
        response.success = True
        distance = self.get_target_distance()
        response.message = f"State: {self.state.value}, Tag: {self.target_tag_id}, Distance: {distance:.2f}m"
        return response

    # ArUco detection
    def detect_aruco_markers(self):
        """Detect ArUco markers in current image"""
        if self.current_image is None or self.state == FollowMeState.IDLE:
            return

        try:
            # Detect markers (using direct detectMarkers function)
            corners, ids, rejected = aruco.detectMarkers(
                self.current_image, self.aruco_dict, parameters=self.aruco_params
            )

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == self.target_tag_id:
                        # Found target marker
                        self.last_detection_time = time.time()

                        # Estimate pose if camera info available
                        if self.camera_info:
                            pose = self.estimate_marker_pose(corners[i])
                            if pose:
                                self.target_pose = pose
                                self.target_pose_pub.publish(pose)

                                if self.state == FollowMeState.SEARCHING:
                                    self.state = FollowMeState.FOLLOWING
                                    self.get_logger().info(
                                        f"Found target tag {self.target_tag_id}, starting follow"
                                    )

                        # Visualize marker
                        self.visualize_marker(corners[i], marker_id)
                        break

            # Check if target is lost
            if (
                self.last_detection_time
                and time.time() - self.last_detection_time > self.lost_timeout
            ):
                if self.state == FollowMeState.FOLLOWING:
                    self.state = FollowMeState.LOST
                    self.get_logger().warning(f"Lost target tag {self.target_tag_id}")

        except Exception as e:
            self.get_logger().error(f"ArUco detection error: {str(e)}")

    def estimate_marker_pose(self, corners) -> Optional[PoseStamped]:
        """Estimate 3D pose of ArUco marker"""
        if not self.camera_info:
            return None

        try:
            # Camera matrix and distortion coefficients
            camera_matrix = np.array(self.camera_info.k).reshape(3, 3)
            dist_coeffs = np.array(self.camera_info.d)

            # Marker size (assume 0.1m)
            marker_size = 0.1

            # Estimate pose
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, marker_size, camera_matrix, dist_coeffs
            )

            if rvecs is not None and len(rvecs) > 0:
                # Convert rotation vector to quaternion
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                # Create pose message
                pose = PoseStamped()
                pose.header.frame_id = "camera_link"
                pose.header.stamp = self.get_clock().now().to_msg()

                pose.pose.position.x = float(tvec[0])
                pose.pose.position.y = float(tvec[1])
                pose.pose.position.z = float(tvec[2])

                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]

                return pose

        except Exception as e:
            self.get_logger().error(f"Pose estimation error: {str(e)}")

        return None

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """Convert rotation matrix to quaternion"""
        trace = np.trace(rotation_matrix)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
            y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
            z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
        else:
            if (
                rotation_matrix[0, 0] > rotation_matrix[1, 1]
                and rotation_matrix[0, 0] > rotation_matrix[2, 2]
            ):
                s = 2.0 * np.sqrt(
                    1.0
                    + rotation_matrix[0, 0]
                    - rotation_matrix[1, 1]
                    - rotation_matrix[2, 2]
                )
                w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                x = 0.25 * s
                y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(
                    1.0
                    + rotation_matrix[1, 1]
                    - rotation_matrix[0, 0]
                    - rotation_matrix[2, 2]
                )
                w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                y = 0.25 * s
                z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(
                    1.0
                    + rotation_matrix[2, 2]
                    - rotation_matrix[0, 0]
                    - rotation_matrix[1, 1]
                )
                w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                z = 0.25 * s

        return [x, y, z, w]

    # Control logic
    def control_update(self):
        """Update following control commands"""
        if self.state != FollowMeState.FOLLOWING or not self.target_pose:
            return

        try:
            # Calculate following velocity
            cmd_vel = self.calculate_follow_velocity()

            # Publish velocity command
            self.cmd_vel_pub.publish(cmd_vel)

        except Exception as e:
            self.get_logger().error(f"Control update error: {str(e)}")

    def calculate_follow_velocity(self) -> Twist:
        """Calculate velocity commands for following behavior"""
        cmd = Twist()

        if not self.target_pose:
            return cmd

        try:
            # Get target position in robot frame (simplified)
            target_x = self.target_pose.pose.position.x
            target_y = self.target_pose.pose.position.y

            # Calculate distance and angle to target
            distance = math.sqrt(target_x**2 + target_y**2)
            angle = math.atan2(target_y, target_x)

            # Publish distance
            distance_msg = Float32()
            distance_msg.data = distance
            self.distance_pub.publish(distance_msg)

            # Following logic
            if distance > self.max_follow_distance:
                # Too far, speed up
                cmd.linear.x = min(self.max_linear_velocity, distance * 0.3)
            elif distance < self.min_follow_distance:
                # Too close, slow down or stop
                cmd.linear.x = max(0.0, distance * 0.1 - 0.1)
            else:
                # Good distance, maintain speed
                cmd.linear.x = 0.2  # m/s

            # Angular control to face target
            if abs(angle) > self.angle_tolerance:
                cmd.angular.z = max(
                    -self.max_angular_velocity,
                    min(self.max_angular_velocity, angle * 2.0),
                )

            # Stop if too close to target
            if distance < self.safety_distance:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        except Exception as e:
            self.get_logger().error(f"Velocity calculation error: {str(e)}")

        return cmd

    def get_target_distance(self) -> float:
        """Get distance to target"""
        if not self.target_pose:
            return float("inf")

        target_x = self.target_pose.pose.position.x
        target_y = self.target_pose.pose.position.y
        return math.sqrt(target_x**2 + target_y**2)

    # Mission execution
    def start_mission(self):
        """Start the follow-me mission"""
        self.stop_execution = False
        self.state = FollowMeState.SEARCHING
        self.mission_start_time = time.time()
        self.target_pose = None
        self.last_detection_time = None

        self.get_logger().info(
            f"Started follow-me mission for tag {self.target_tag_id}"
        )

    # Visualization
    def visualize_marker(self, corners, marker_id):
        """Create visualization markers for detected ArUco tag"""
        marker_array = MarkerArray()

        # Marker outline
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "aruco_markers"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Line width
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 1

        # Add corners as points
        for corner in corners[0]:
            point = Point()
            point.x = float(corner[0] / 100.0)  # Normalize
            point.y = float(corner[1] / 100.0)
            point.z = 0.0
            marker.points.append(point)

        # Close the loop
        marker.points.append(marker.points[0])

        marker_array.markers.append(marker)
        self.markers_pub.publish(marker_array)

    def ros_image_to_cv2(self, ros_image: Image) -> np.ndarray:
        """Convert ROS Image message to OpenCV format"""
        if ros_image.encoding == "rgb8":
            image = np.frombuffer(ros_image.data, dtype=np.uint8)
            image = image.reshape((ros_image.height, ros_image.width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif ros_image.encoding == "bgr8":
            image = np.frombuffer(ros_image.data, dtype=np.uint8)
            return image.reshape((ros_image.height, ros_image.width, 3))
        else:
            raise ValueError(f"Unsupported image encoding: {ros_image.encoding}")

    def status_update(self):
        """Periodic status update"""
        # Publish status
        status_msg = String()
        distance = self.get_target_distance()
        status_msg.data = f"{self.state.value}|{self.target_tag_id}|{distance:.2f}"
        self.status_pub.publish(status_msg)

        # Handle lost target
        if self.state == FollowMeState.LOST:
            # Stop robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

            # Try to recover for a short time, then stop mission
            if time.time() - self.last_detection_time > self.lost_timeout * 3:
                self.state = FollowMeState.STOPPED
                self.get_logger().error("Target lost for too long, stopping mission")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        mission = FollowMeMission()
        rclpy.spin(mission)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
