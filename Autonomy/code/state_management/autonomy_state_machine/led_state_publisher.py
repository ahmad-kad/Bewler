"""
LED state publisher for state machine.

Publishes LED state information based on rover state for URC competition
requirements. The LED controller subscribes to this and decides actual colors.
"""

from typing import Optional

import structlog
from rclpy.node import Node
from std_msgs.msg import String

from .states import AutonomousMode, SystemState

logger = structlog.get_logger(__name__)


class LEDStatePublisher:
    """
    Publishes LED state information for URC competition requirements.

    Per URC 2026 rules:
    - Red: Autonomous operation
    - Blue: Teleoperation (manually driving)
    - Flashing Green: Successful arrival at target

    This class publishes state information; the LED controller decides
    actual colors and patterns.
    """

    def __init__(self, node: Node):
        """
        Initialize LED state publisher.

        Args:
            node: Parent ROS2 node
        """
        self.node = node

        # Publisher for LED state information
        self.led_info_publisher = node.create_publisher(String, "/state_machine/led_info", 10)

        logger.info("LED state publisher initialized")

    def update_state(
        self,
        state: SystemState,
        substate: Optional[AutonomousMode] = None,
    ) -> None:
        """
        Update LED information based on state.

        Args:
            state: Current system state
            substate: Current autonomous substate (if applicable)
        """
        led_info = self._determine_led_info(state, substate)
        self._publish_led_info(led_info)

        logger.debug(
            "LED state updated",
            state=str(state),
            substate=str(substate) if substate else None,
            led_info=led_info,
        )

    def signal_waypoint_success(self) -> None:
        """Signal successful arrival at waypoint (flashing green per URC rules)."""
        led_info = "WAYPOINT_SUCCESS"
        self._publish_led_info(led_info)

        logger.info("Waypoint success signaled")

    def _determine_led_info(
        self,
        state: SystemState,
        substate: Optional[AutonomousMode] = None,
    ) -> str:
        """
        Determine LED information string based on state.

        Args:
            state: System state
            substate: Autonomous substate

        Returns:
            LED information string for LED controller
        """
        # Per URC 2026 requirements:
        # - Autonomous states should show RED
        # - Teleoperation should show BLUE
        # - Safety should show RED BLINKING
        # - Others can show appropriate indicators

        if state == SystemState.AUTONOMOUS:
            # Red indicator for autonomous (URC requirement)
            return "AUTONOMOUS_RED"

        elif state == SystemState.TELEOPERATION:
            # Blue indicator for teleoperation (URC requirement)
            return "TELEOPERATION_BLUE"

        elif state == SystemState.SAFETY:
            # Red blinking for safety/emergency
            return "SAFETY_RED_BLINK"

        elif state == SystemState.BOOT:
            # Yellow/amber during boot
            return "BOOT_INITIALIZING"

        elif state == SystemState.CALIBRATION:
            # Yellow during calibration
            return "CALIBRATION_YELLOW"

        elif state == SystemState.IDLE:
            # Green solid for idle/ready
            return "IDLE_GREEN"

        elif state == SystemState.SHUTDOWN:
            # Red fade for shutdown
            return "SHUTDOWN_RED_FADE"

        # Default
        return "UNKNOWN_STATE"

    def _publish_led_info(self, led_info: str) -> None:
        """
        Publish LED information message.

        Args:
            led_info: LED information string
        """
        msg = String()
        msg.data = led_info
        self.led_info_publisher.publish(msg)

        logger.debug("LED info published", info=led_info)

    def signal_transition_start(self) -> None:
        """Signal that a state transition is starting."""
        led_info = "TRANSITION_IN_PROGRESS"
        self._publish_led_info(led_info)

        logger.debug("Transition start signaled")

    def signal_error(self, error_type: str = "GENERAL") -> None:
        """
        Signal an error condition.

        Args:
            error_type: Type of error
        """
        led_info = f"ERROR_{error_type}"
        self._publish_led_info(led_info)

        logger.warning("Error signaled", error_type=error_type)

    def signal_success(self, success_type: str = "GENERAL") -> None:
        """
        Signal a success condition.

        Args:
            success_type: Type of success
        """
        led_info = f"SUCCESS_{success_type}"
        self._publish_led_info(led_info)

        logger.info("Success signaled", success_type=success_type)
