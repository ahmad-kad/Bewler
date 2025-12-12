"""
Follow Me behavior module for ArUco tag following.

Implements safe following behavior using ArUco tag detection with
configurable safety distances and speed limits.
"""

import time
from typing import Any, Dict, Optional, Tuple

import structlog
from autonomy_interfaces.msg import FollowMeStatus
from autonomy_interfaces.srv import DetectAruco, FollowMeControl
from geometry_msgs.msg import Point, Twist
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

logger = structlog.get_logger(__name__)


class FollowMeBehavior:
    """
    Follow Me behavior using ArUco tag detection.

    Provides safe following behavior with configurable safety distances,
    speed limits, and obstacle avoidance.
    """

    def __init__(self, node: Node):
        """
        Initialize follow me behavior.

        Args:
            node: Parent ROS2 node
        """
        self.node = node

        # Follow me state
        self.is_following = False
        self.target_tag_id: Optional[int] = None
        self.safety_distance = 2.0  # meters
        self.max_speed = 1.0  # m/s
        self.operator_id = "unknown"

        # Target tracking
        self.target_position: Optional[Point] = None
        self.target_distance = 0.0
        self.target_angle = 0.0
        self.last_detection_time = 0.0
        self.target_visible = False

        # Safety parameters
        self.safety_violation = False
        self.current_speed = 0.0

        # QoS profile for reliable communication
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Service clients
        self.aruco_detection_client: Client = node.create_client(DetectAruco, "/aruco_detection/detect")
        self.follow_control_client: Client = node.create_client(FollowMeControl, "/follow_me/control")

        # Publishers
        self.status_publisher = node.create_publisher(FollowMeStatus, "/follow_me/status", self.qos)
        self.cmd_vel_publisher = node.create_publisher(Twist, "/cmd_vel", self.qos)

        # Service server for follow me control
        self.follow_control_service = node.create_service(
            FollowMeControl,
            "/state_machine/follow_me_control",
            self._handle_follow_control,
        )

        # Timer for follow me control loop
        self.control_timer = node.create_timer(0.1, self._follow_control_loop)  # 10Hz

        # Timer for status publishing
        self.status_timer = node.create_timer(0.5, self._publish_status)  # 2Hz

        logger.info("Follow me behavior initialized")

    def start_following(
        self,
        target_tag_id: int,
        safety_distance: float = 2.0,
        max_speed: float = 1.0,
        operator_id: str = "unknown",
    ) -> bool:
        """
        Start following a specific ArUco tag.

        Args:
            target_tag_id: ID of ArUco tag to follow
            safety_distance: Safe following distance in meters
            max_speed: Maximum following speed in m/s
            operator_id: ID of operator requesting follow

        Returns:
            True if successfully started following
        """
        logger.info(
            "Starting follow me behavior",
            target_tag_id=target_tag_id,
            safety_distance=safety_distance,
            max_speed=max_speed,
            operator_id=operator_id,
        )

        # Validate parameters
        if safety_distance < 0.5:
            logger.warning("Safety distance too small, using minimum of 0.5m")
            safety_distance = 0.5
        if max_speed > 2.0:
            logger.warning("Max speed too high, limiting to 2.0 m/s")
            max_speed = 2.0

        # Update state
        self.is_following = True
        self.target_tag_id = target_tag_id
        self.safety_distance = safety_distance
        self.max_speed = max_speed
        self.operator_id = operator_id
        self.target_visible = False
        self.safety_violation = False

        # Test ArUco detection
        if not self._test_aruco_detection():
            logger.error("ArUco detection not available")
            self.is_following = False
            return False

        logger.info("Follow me behavior started successfully")
        return True

    def stop_following(self) -> None:
        """Stop following behavior."""
        logger.info("Stopping follow me behavior")

        # Stop robot movement
        self._publish_velocity(0.0, 0.0, 0.0)

        # Reset state
        self.is_following = False
        self.target_tag_id = None
        self.target_position = None
        self.target_visible = False
        self.safety_violation = False

        logger.info("Follow me behavior stopped")

    def _follow_control_loop(self) -> None:
        """Main follow me control loop."""
        if not self.is_following or self.target_tag_id is None:
            return

        # Detect target ArUco tag
        detection_result = self._detect_target_tag()

        if detection_result is None:
            # Target not visible - stop and wait
            self.target_visible = False
            self._publish_velocity(0.0, 0.0, 0.0)
            return

        # Update target information
        target_pos, distance, angle = detection_result
        self.target_position = target_pos
        self.target_distance = distance
        self.target_angle = angle
        self.target_visible = True
        self.last_detection_time = time.time()

        # Check safety distance
        self.safety_violation = distance < self.safety_distance

        if self.safety_violation:
            # Too close - stop immediately
            logger.warning("Safety distance violated - stopping")
            self._publish_velocity(0.0, 0.0, 0.0)
            return

        # Calculate desired movement
        linear_vel, angular_vel = self._calculate_movement(distance, angle)

        # Apply speed limits
        linear_vel = max(-self.max_speed, min(self.max_speed, linear_vel))
        angular_vel = max(-1.0, min(1.0, angular_vel))  # Limit angular velocity

        # Publish velocity command
        self._publish_velocity(linear_vel, 0.0, angular_vel)
        self.current_speed = abs(linear_vel)

    def _detect_target_tag(self) -> Optional[Tuple[Point, float, float]]:
        """
        Detect the target ArUco tag.

        Returns:
            Tuple of (position, distance, angle) if detected, None otherwise
        """
        if not self.aruco_detection_client.wait_for_service(timeout_sec=1.0):
            return None

        # Create detection request
        request = DetectAruco.Request()
        request.target_tag_ids = [self.target_tag_id]
        request.detection_timeout = 0.1  # 100ms timeout
        request.require_distance_estimate = True
        request.max_detection_distance = 10.0  # 10m max detection distance

        try:
            # Call detection service
            future = self.aruco_detection_client.call_async(request)
            self.node.get_executor().spin_until_future_complete(future, timeout_sec=0.2)

            if not future.done():
                return None

            response = future.result()

            if not response.success or not response.detected_tag_ids:
                return None

            # Find our target tag
            if self.target_tag_id not in response.detected_tag_ids:
                return None

            # Get position and distance
            tag_index = response.detected_tag_ids.index(self.target_tag_id)
            position = response.tag_positions[tag_index]
            distance = response.tag_distances[tag_index]
            angle = response.tag_angles[tag_index]

            return (position, distance, angle)

        except Exception as e:
            logger.error("ArUco detection failed", error=str(e))
            return None

    def _calculate_movement(self, distance: float, angle: float) -> Tuple[float, float]:
        """
        Calculate desired movement based on target position.

        Args:
            distance: Distance to target
            angle: Angle to target

        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        # Calculate desired distance (safety distance + small buffer)
        desired_distance = self.safety_distance + 0.5

        # Calculate linear velocity based on distance error
        distance_error = distance - desired_distance
        linear_vel = 0.0

        if abs(distance_error) > 0.2:  # 20cm deadband
            # Proportional control for distance
            kp_distance = 0.5
            linear_vel = kp_distance * distance_error
            linear_vel = max(-self.max_speed, min(self.max_speed, linear_vel))

        # Calculate angular velocity based on angle error
        angular_vel = 0.0
        if abs(angle) > 0.1:  # 0.1 rad deadband (~6 degrees)
            # Proportional control for angle
            kp_angle = 1.0
            angular_vel = kp_angle * angle
            angular_vel = max(-1.0, min(1.0, angular_vel))

        return linear_vel, angular_vel

    def _publish_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        """Publish velocity command."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)

    def _publish_status(self) -> None:
        """Publish follow me status."""
        msg = FollowMeStatus()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.is_following = self.is_following
        msg.target_tag_id = self.target_tag_id if self.target_tag_id else 0
        msg.target_distance = self.target_distance
        msg.target_angle = self.target_angle

        msg.safety_distance = self.safety_distance
        msg.safety_violation = self.safety_violation
        msg.current_speed = self.current_speed

        if self.target_position:
            msg.target_position = self.target_position
        msg.target_visible = self.target_visible
        msg.last_detection_time = time.time() - self.last_detection_time

        msg.max_speed = self.max_speed
        msg.operator_id = self.operator_id

        self.status_publisher.publish(msg)

    def _test_aruco_detection(self) -> bool:
        """Test if ArUco detection service is available."""
        return self.aruco_detection_client.wait_for_service(timeout_sec=2.0)

    def _handle_follow_control(
        self, request: FollowMeControl.Request, response: FollowMeControl.Response
    ) -> FollowMeControl.Response:
        """Handle follow me control service requests."""
        logger.info(
            "Follow me control request",
            target_tag_id=request.target_tag_id,
            enable_following=request.enable_following,
            operator_id=request.operator_id,
        )

        if request.enable_following:
            # Start following
            success = self.start_following(
                target_tag_id=request.target_tag_id,
                safety_distance=request.safety_distance,
                max_speed=request.max_speed,
                operator_id=request.operator_id,
            )

            response.success = success
            response.message = "Follow me started" if success else "Failed to start follow me"
        else:
            # Stop following
            self.stop_following()
            response.success = True
            response.message = "Follow me stopped"

        response.is_following = self.is_following
        response.current_target_tag = self.target_tag_id if self.target_tag_id else 0
        response.current_distance = self.target_distance

        if self.target_position:
            response.target_position = self.target_position

        return response

    def get_status(self) -> Dict[str, Any]:
        """Get current follow me status."""
        return {
            "is_following": self.is_following,
            "target_tag_id": self.target_tag_id,
            "target_distance": self.target_distance,
            "target_angle": self.target_angle,
            "safety_distance": self.safety_distance,
            "safety_violation": self.safety_violation,
            "current_speed": self.current_speed,
            "target_visible": self.target_visible,
            "operator_id": self.operator_id,
        }
