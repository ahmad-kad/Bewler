"""
Frontend interface for Follow Me mode.

Provides a simple interface for operators to control follow me behavior
through the state machine.
"""

import time
from typing import Any, Callable, Dict, Optional

import structlog
from autonomy_interfaces.msg import FollowMeStatus
from autonomy_interfaces.msg import SystemState as SystemStateMsg
from autonomy_interfaces.srv import ChangeState, FollowMeControl
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

logger = structlog.get_logger(__name__)


class FollowMeFrontend:
    """
    Frontend interface for Follow Me mode.

    Provides methods for operators to start/stop follow me behavior
    and monitor follow me status.
    """

    def __init__(self, node: Node):
        """
        Initialize follow me frontend.

        Args:
            node: Parent ROS2 node
        """
        self.node = node

        # QoS profile for reliable communication
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Service clients
        self.change_state_client: Client = node.create_client(ChangeState, "/state_machine/change_state")
        self.follow_control_client: Client = node.create_client(FollowMeControl, "/state_machine/follow_me_control")

        # Subscribers
        self.status_subscriber = node.create_subscription(
            FollowMeStatus, "/follow_me/status", self._status_callback, self.qos
        )
        self.state_subscriber = node.create_subscription(
            SystemStateMsg,
            "/state_machine/current_state",
            self._state_callback,
            self.qos,
        )

        # State tracking
        self.current_state = "UNKNOWN"
        self.current_substate = "NONE"
        self.follow_me_status: Optional[FollowMeStatus] = None
        self.status_callback: Optional[Callable[[FollowMeStatus], None]] = None
        self.state_callback: Optional[Callable[[SystemStateMsg], None]] = None

        logger.info("Follow me frontend initialized")

    def start_follow_me(
        self,
        target_tag_id: int,
        safety_distance: float = 2.0,
        max_speed: float = 1.0,
        operator_id: str = "frontend",
    ) -> bool:
        """
        Start follow me mode.

        Args:
            target_tag_id: ArUco tag ID to follow
            safety_distance: Safe following distance in meters
            max_speed: Maximum following speed in m/s
            operator_id: ID of operator requesting follow

        Returns:
            True if successfully started
        """
        logger.info(
            "Starting follow me mode",
            target_tag_id=target_tag_id,
            safety_distance=safety_distance,
            max_speed=max_speed,
            operator_id=operator_id,
        )

        # First, transition to AUTONOMOUS state with FOLLOW_ME substate
        if not self._transition_to_follow_me_state(operator_id):
            logger.error("Failed to transition to follow me state")
            return False

        # Wait a moment for state transition
        time.sleep(1.0)

        # Start follow me behavior
        if not self._start_follow_behavior(target_tag_id, safety_distance, max_speed, operator_id):
            logger.error("Failed to start follow me behavior")
            return False

        logger.info("Follow me mode started successfully")
        return True

    def stop_follow_me(self, operator_id: str = "frontend") -> bool:
        """
        Stop follow me mode.

        Args:
            operator_id: ID of operator requesting stop

        Returns:
            True if successfully stopped
        """
        logger.info("Stopping follow me mode", operator_id=operator_id)

        # Stop follow me behavior
        if not self._stop_follow_behavior(operator_id):
            logger.error("Failed to stop follow me behavior")
            return False

        # Transition back to IDLE state
        if not self._transition_to_idle_state(operator_id):
            logger.error("Failed to transition to idle state")
            return False

        logger.info("Follow me mode stopped successfully")
        return True

    def get_follow_me_status(self) -> Optional[Dict[str, Any]]:
        """
        Get current follow me status.

        Returns:
            Dictionary with follow me status or None if not available
        """
        if self.follow_me_status is None:
            return None

        return {
            "is_following": self.follow_me_status.is_following,
            "target_tag_id": self.follow_me_status.target_tag_id,
            "target_distance": self.follow_me_status.target_distance,
            "target_angle": self.follow_me_status.target_angle,
            "safety_distance": self.follow_me_status.safety_distance,
            "safety_violation": self.follow_me_status.safety_violation,
            "current_speed": self.follow_me_status.current_speed,
            "target_visible": self.follow_me_status.target_visible,
            "operator_id": self.follow_me_status.operator_id,
        }

    def is_follow_me_active(self) -> bool:
        """
        Check if follow me mode is currently active.

        Returns:
            True if follow me is active
        """
        return (
            self.current_state == "AUTONOMOUS"
            and self.current_substate == "FOLLOW_ME"
            and self.follow_me_status is not None
            and self.follow_me_status.is_following
        )

    def set_status_callback(self, callback: Callable[[FollowMeStatus], None]) -> None:
        """Set callback for follow me status updates."""
        self.status_callback = callback

    def set_state_callback(self, callback: Callable[[SystemStateMsg], None]) -> None:
        """Set callback for state updates."""
        self.state_callback = callback

    def _transition_to_follow_me_state(self, operator_id: str) -> bool:
        """Transition to AUTONOMOUS state with FOLLOW_ME substate."""
        if not self.change_state_client.wait_for_service(timeout_sec=5.0):
            logger.error("State change service not available")
            return False

        request = ChangeState.Request()
        request.desired_state = "AUTONOMOUS"
        request.desired_substate = "FOLLOW_ME"
        request.reason = "Starting follow me mode"
        request.operator_id = operator_id

        try:
            future = self.change_state_client.call_async(request)
            self.node.get_executor().spin_until_future_complete(future, timeout_sec=10.0)

            if not future.done():
                logger.error("State change request timed out")
                return False

            response = future.result()

            if not response.success:
                logger.error("State change failed", message=response.message)
                return False

            logger.info("Successfully transitioned to follow me state")
            return True

        except Exception as e:
            logger.error("State change request failed", error=str(e))
            return False

    def _transition_to_idle_state(self, operator_id: str) -> bool:
        """Transition to IDLE state."""
        if not self.change_state_client.wait_for_service(timeout_sec=5.0):
            logger.error("State change service not available")
            return False

        request = ChangeState.Request()
        request.desired_state = "IDLE"
        request.desired_substate = "NONE"
        request.reason = "Stopping follow me mode"
        request.operator_id = operator_id

        try:
            future = self.change_state_client.call_async(request)
            self.node.get_executor().spin_until_future_complete(future, timeout_sec=10.0)

            if not future.done():
                logger.error("State change request timed out")
                return False

            response = future.result()

            if not response.success:
                logger.error("State change failed", message=response.message)
                return False

            logger.info("Successfully transitioned to idle state")
            return True

        except Exception as e:
            logger.error("State change request failed", error=str(e))
            return False

    def _start_follow_behavior(
        self,
        target_tag_id: int,
        safety_distance: float,
        max_speed: float,
        operator_id: str,
    ) -> bool:
        """Start follow me behavior."""
        if not self.follow_control_client.wait_for_service(timeout_sec=5.0):
            logger.error("Follow control service not available")
            return False

        request = FollowMeControl.Request()
        request.target_tag_id = target_tag_id
        request.safety_distance = safety_distance
        request.max_speed = max_speed
        request.enable_following = True
        request.operator_id = operator_id

        try:
            future = self.follow_control_client.call_async(request)
            self.node.get_executor().spin_until_future_complete(future, timeout_sec=10.0)

            if not future.done():
                logger.error("Follow control request timed out")
                return False

            response = future.result()

            if not response.success:
                logger.error("Follow control failed", message=response.message)
                return False

            logger.info("Successfully started follow me behavior")
            return True

        except Exception as e:
            logger.error("Follow control request failed", error=str(e))
            return False

    def _stop_follow_behavior(self, operator_id: str) -> bool:
        """Stop follow me behavior."""
        if not self.follow_control_client.wait_for_service(timeout_sec=5.0):
            logger.error("Follow control service not available")
            return False

        request = FollowMeControl.Request()
        request.target_tag_id = 0  # Not used for stop
        request.safety_distance = 0.0  # Not used for stop
        request.max_speed = 0.0  # Not used for stop
        request.enable_following = False
        request.operator_id = operator_id

        try:
            future = self.follow_control_client.call_async(request)
            self.node.get_executor().spin_until_future_complete(future, timeout_sec=10.0)

            if not future.done():
                logger.error("Follow control request timed out")
                return False

            response = future.result()

            if not response.success:
                logger.error("Follow control failed", message=response.message)
                return False

            logger.info("Successfully stopped follow me behavior")
            return True

        except Exception as e:
            logger.error("Follow control request failed", error=str(e))
            return False

    def _status_callback(self, msg: FollowMeStatus) -> None:
        """Handle follow me status updates."""
        self.follow_me_status = msg

        if self.status_callback:
            self.status_callback(msg)

    def _state_callback(self, msg: SystemStateMsg) -> None:
        """Handle state updates."""
        self.current_state = msg.current_state
        self.current_substate = msg.substate

        if self.state_callback:
            self.state_callback(msg)
