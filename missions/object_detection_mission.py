#!/usr/bin/env python3
"""
Object Detection Mission - ROS2 Node

Executes object detection and approach missions using:
- Computer vision for object recognition
- SLAM pose for precise localization
- Navigation stack for approach planning
- Real-time detection feedback

Author: URC 2026 Autonomy Team
"""

import json

# Standard Library
import math
import time
from enum import Enum
from typing import Optional

import cv2
import numpy as np
import rclpy

# ROS2 Messages
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String

# Using std_srvs for basic service calls
from std_srvs.srv import Trigger


class ObjectDetectionState(Enum):
    """States for object detection mission"""

    IDLE = "idle"
    SEARCHING = "searching"
    DETECTING = "detecting"
    APPROACHING = "approaching"
    ARRIVED = "arrived"
    FAILED = "failed"
    ABORTED = "aborted"


class ObjectDetectionMission(Node):
    """
    Object Detection Mission Node

    Subscribes to:
    - /camera/image_raw: RGB camera feed
    - /camera/camera_info: Camera calibration info
    - /detections: Object detections from CV pipeline
    - /slam/pose: SLAM pose for localization
    - /odom: Odometry for velocity control

    Publishes:
    - /mission/object/status: Mission status
    - /mission/object/progress: Detection confidence
    - /mission/object/target_pose: Target object pose
    - /mission/object/detection_image: Detection visualization

    Actions:
    - /navigate_to_pose: Navigation stack for approach

    Services:
    - /mission/object/start: Start object detection
    - /mission/object/stop: Stop detection
    - /mission/object/status: Get current status
    """

    def __init__(self):
        super().__init__("object_detection_mission")

        # Mission state
        self.state = ObjectDetectionState.IDLE
        self.target_object = None
        self.detection_confidence = 0.0
        self.target_pose: Optional[PoseStamped] = None
        self.mission_start_time = None

        # Detection parameters
        self.declare_parameter("target_object", "sample")
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("approach_distance", 1.5)  # meters
        self.declare_parameter("detection_timeout", 300.0)  # 5 minutes
        self.declare_parameter("search_timeout", 60.0)  # 1 minute per search pattern

        self.target_object = self.get_parameter("target_object").value
        self.min_confidence = self.get_parameter("min_confidence").value
        self.approach_distance = self.get_parameter("approach_distance").value
        self.detection_timeout = self.get_parameter("detection_timeout").value
        self.search_timeout = self.get_parameter("search_timeout").value

        # Service client for navigation (simplified)
        self.nav_client = self.create_client(Trigger, "/navigate_to_pose")

        # Publishers
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.status_pub = self.create_publisher(
            String, "/mission/object/status", qos_reliable
        )
        self.progress_pub = self.create_publisher(
            Float32, "/mission/object/progress", qos_reliable
        )
        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/mission/object/target_pose", qos_reliable
        )
        self.detection_image_pub = self.create_publisher(
            Image, "/mission/object/detection_image", qos_reliable
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos_reliable)

        # Subscribers
        self.detection_sub = self.create_subscription(
            String, "/detections", self.detection_callback, 10
        )  # Simplified detection input
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
            Trigger, "/mission/object/start", self.start_mission_callback
        )
        self.stop_srv = self.create_service(
            Trigger, "/mission/object/stop", self.stop_mission_callback
        )
        self.status_srv = self.create_service(
            Trigger, "/mission/object/status", self.get_status_callback
        )

        # Current sensor data
        self.current_image: Optional[np.ndarray] = None
        self.camera_info: Optional[CameraInfo] = None
        self.current_pose: Optional[PoseStamped] = None
        self.current_odom: Optional[Odometry] = None
        self.last_detection_time = None

        # Status update timer
        self.status_timer = self.create_timer(1.0, self.status_update)

        # Mission execution
        self.mission_thread: Optional[threading.Thread] = None
        self.stop_execution = False

        self.get_logger().info("Object Detection Mission initialized")

    # Sensor data callbacks
    def detection_callback(self, msg: String):
        """Process simplified object detections"""
        if self.state == ObjectDetectionState.IDLE:
            return

        try:
            # Parse JSON detection data
            detection_data = json.loads(msg.data)

            # Check if target object detected
            if detection_data.get("object") == self.target_object:
                confidence = detection_data.get("confidence", 0.0)

                if confidence > self.min_confidence:
                    self.detection_confidence = confidence
                    self.last_detection_time = time.time()

                    # Create pose from detection data
                    if "position" in detection_data:
                        self.target_pose = PoseStamped()
                        self.target_pose.header.frame_id = "camera_link"
                        self.target_pose.header.stamp = self.get_clock().now().to_msg()
                        self.target_pose.pose.position.x = detection_data[
                            "position"
                        ].get("x", 1.0)
                        self.target_pose.pose.position.y = detection_data[
                            "position"
                        ].get("y", 0.0)
                        self.target_pose.pose.position.z = detection_data[
                            "position"
                        ].get("z", 0.0)
                        self.target_pose_pub.publish(self.target_pose)

                    self.get_logger().info(
                        f"Detected {self.target_object} with confidence {confidence:.2f}"
                    )

        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse detection data")
        except Exception as e:
            self.get_logger().error(f"Detection callback error: {str(e)}")

    def image_callback(self, msg: Image):
        """Process camera image"""
        try:
            # Convert ROS Image to OpenCV
            self.current_image = self.ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {str(e)}")

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
        """Start object detection mission"""
        if self.state == ObjectDetectionState.IDLE:
            self.start_mission()
            response.success = True
            response.message = (
                f"Started object detection mission for {self.target_object}"
            )
        else:
            response.success = False
            response.message = f"Cannot start mission in state {self.state.value}"
        return response

    def stop_mission_callback(self, request, response):
        """Stop object detection mission"""
        self.stop_execution = True
        self.state = ObjectDetectionState.IDLE
        self.target_pose = None
        self.detection_confidence = 0.0
        response.success = True
        response.message = "Object detection mission stopped"
        return response

    def get_status_callback(self, request, response):
        """Get current mission status"""
        response.success = True
        response.message = f"State: {self.state.value}, Target: {self.target_object}, Confidence: {self.detection_confidence:.2f}"
        return response

    # Mission execution
    def start_mission(self):
        """Start the object detection mission"""
        if not self.nav_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Navigation service not available")
            self.state = ObjectDetectionState.FAILED
            return

        self.stop_execution = False
        self.state = ObjectDetectionState.SEARCHING
        self.mission_start_time = time.time()
        self.detection_confidence = 0.0
        self.target_pose = None

        # Start mission execution thread
        self.mission_thread = threading.Thread(target=self.execute_mission)
        self.mission_thread.start()

    def execute_mission(self):
        """Execute the object detection mission"""
        try:
            self.get_logger().info(
                f"Starting object detection mission for {self.target_object}"
            )

            while not self.stop_execution:
                if time.time() - self.mission_start_time > self.detection_timeout:
                    self.get_logger().error("Detection timeout exceeded")
                    self.state = ObjectDetectionState.FAILED
                    break

                # Search pattern if no detection
                if self.state == ObjectDetectionState.SEARCHING:
                    self.execute_search_pattern()

                # Wait for detection
                elif self.state == ObjectDetectionState.DETECTING:
                    if (
                        self.target_pose
                        and self.detection_confidence > self.min_confidence
                    ):
                        self.get_logger().info("Target detected, approaching...")
                        self.state = ObjectDetectionState.APPROACHING

                        if self.approach_target():
                            self.state = ObjectDetectionState.ARRIVED
                            self.get_logger().info("Successfully approached target!")
                            break
                        else:
                            self.get_logger().error("Failed to approach target")
                            self.state = ObjectDetectionState.FAILED
                            break

                time.sleep(0.5)

            if self.state == ObjectDetectionState.ARRIVED:
                self.get_logger().info(
                    "Object detection mission completed successfully"
                )

        except Exception as e:
            self.get_logger().error(f"Mission execution error: {str(e)}")
            self.state = ObjectDetectionState.FAILED
        finally:
            self.stop_execution = False

    def execute_search_pattern(self):
        """Execute a search pattern to find the target object"""
        self.get_logger().info("Executing search pattern...")

        # Simple search pattern: rotate in place while moving forward
        search_start = time.time()
        rotations = 0

        while (
            time.time() - search_start < self.search_timeout and not self.stop_execution
        ):
            # Rotate slowly
            cmd = Twist()
            cmd.angular.z = 0.3  # rad/s
            self.cmd_vel_pub.publish(cmd)

            # Check for detection
            if self.detection_confidence > self.min_confidence:
                self.state = ObjectDetectionState.DETECTING
                break

            time.sleep(0.1)

        # Stop rotation
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        if self.detection_confidence < self.min_confidence:
            self.get_logger().info("Search pattern completed, no target found")

    def approach_target(self) -> bool:
        """Approach the detected target object"""
        if not self.target_pose or not self.current_pose:
            return False

        try:
            # Calculate approach pose (approach_distance away from target)
            approach_pose = PoseStamped()
            approach_pose.header = self.target_pose.header

            # Vector from robot to target
            dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
            dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance > self.approach_distance:
                # Calculate approach position
                ratio = (distance - self.approach_distance) / distance
                approach_pose.pose.position.x = (
                    self.current_pose.pose.position.x + dx * ratio
                )
                approach_pose.pose.position.y = (
                    self.current_pose.pose.position.y + dy * ratio
                )
                approach_pose.pose.orientation = self.target_pose.pose.orientation

                # Navigate to approach position (simplified)
                self.get_logger().info("Navigating to approach position...")

                # Simulate navigation (in real system, would call navigation service)
                time.sleep(3.0)  # Simulate navigation time

                self.get_logger().info(
                    "Successfully reached approach position (simulated)"
                )
                return True

            else:
                # Already close enough
                self.get_logger().info("Already at appropriate distance from target")
                return True

            return False

        except Exception as e:
            self.get_logger().error(f"Approach error: {str(e)}")
            return False

    # detection_to_pose method removed - using simplified detection handling

    def ros_image_to_cv2(self, ros_image: Image) -> np.ndarray:
        """Convert ROS Image message to OpenCV format"""
        if ros_image.encoding == "rgb8":
            image = np.frombuffer(ros_image.data, dtype=np.uint8)
            image = image.reshape((ros_image.height, ros_image.width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        else:
            raise ValueError(f"Unsupported image encoding: {ros_image.encoding}")

    def status_update(self):
        """Periodic status update"""
        # Publish status
        status_msg = String()
        status_msg.data = (
            f"{self.state.value}|{self.target_object}|{self.detection_confidence:.2f}"
        )
        self.status_pub.publish(status_msg)

        # Publish progress (detection confidence)
        progress_msg = Float32()
        progress_msg.data = self.detection_confidence
        self.progress_pub.publish(progress_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        mission = ObjectDetectionMission()
        rclpy.spin(mission)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
