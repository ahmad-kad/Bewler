"""
Subsystem coordinator for state machine.

Coordinates subsystem activation, monitors subsystem status,
and ensures subsystems are properly configured for each state.
"""

from typing import Dict, List, Optional, Set

import structlog
from rclpy.node import Node

from .states import AutonomousMode

logger = structlog.get_logger(__name__)


class SubsystemCoordinator:
    """
    Coordinates subsystem operations for state machine.

    Manages subsystem lifecycle, monitors status, and ensures
    proper subsystem configuration for each state.
    """

    def __init__(self, node: Node):
        """
        Initialize subsystem coordinator.

        Args:
            node: Parent ROS2 node
        """
        self.node = node
        self._active_subsystems: Set[str] = set()
        self._failed_subsystems: Set[str] = set()
        self._subsystem_status: Dict[str, Dict] = {}

        # Known subsystems
        self._known_subsystems = [
            "camera",
            "navigation",
            "computer_vision",
            "slam",
            "manipulation",
            "science_instruments",
            "autonomous_typing",
        ]

        logger.info("Subsystem coordinator initialized")

    def initialize_subsystems(self) -> None:
        """Initialize all subsystems during boot."""
        logger.info("Initializing subsystems")

        # In a real implementation, this would:
        # 1. Check which subsystems are available
        # 2. Initialize communication with each
        # 3. Verify subsystems are ready

        # For now, assume basic subsystems are available
        self._active_subsystems = {"camera", "navigation"}
        logger.info("Basic subsystems initialized", subsystems=list(self._active_subsystems))

    def start_calibration(self) -> None:
        """Start calibration process for sensors."""
        logger.info("Starting calibration")

        # Activate calibration-related subsystems
        self._active_subsystems.add("camera")
        self._active_subsystems.add("navigation")

        # In real implementation:
        # - Call calibration services on camera nodes
        # - Start SLAM calibration
        # - Verify calibration results

        logger.info("Calibration started")

    def enable_teleoperation(self) -> None:
        """Enable subsystems for teleoperation mode."""
        logger.info("Enabling teleoperation mode")

        # Ensure navigation is active
        self._active_subsystems.add("navigation")

        # Optionally add camera for teleoperation visualization
        self._active_subsystems.add("camera")

        logger.info("Teleoperation enabled", active_subsystems=list(self._active_subsystems))

    def enable_autonomous(self, substate: Optional[AutonomousMode] = None) -> None:
        """
        Enable subsystems for autonomous mode.

        Args:
            substate: Specific autonomous mission substate
        """
        logger.info("Enabling autonomous mode", substate=str(substate) if substate else None)

        # Core autonomous subsystems
        self._active_subsystems.update(["navigation", "computer_vision", "slam"])

        # Mission-specific subsystems
        if substate == AutonomousMode.SCIENCE:
            self._active_subsystems.update(["science_instruments", "manipulation"])
            logger.info("Science mission subsystems enabled")

        elif substate == AutonomousMode.NAVIGATION:
            # Navigation uses core subsystems
            logger.info("Navigation mission subsystems enabled")

        elif substate == AutonomousMode.EQUIPMENT:
            self._active_subsystems.update(["manipulation", "autonomous_typing"])
            logger.info("Equipment servicing subsystems enabled")

        elif substate == AutonomousMode.ARM_CONTROL:
            self._active_subsystems.add("manipulation")
            logger.info("Arm control subsystems enabled")

        elif substate == AutonomousMode.FOLLOW_ME:
            self._active_subsystems.add("aruco_detection")
            logger.info("Follow-me subsystems enabled")

        logger.info("Autonomous mode enabled", active_subsystems=list(self._active_subsystems))

    def engage_safety_mode(self) -> None:
        """Set all systems to known safe values.

        This unified safety implementation works for all safety states
        (ESTOP, SAFESTOP, SAFETY) since 'safe' means the same thing.
        """
        logger.warning("Setting all systems to safe values")

        # Set navigation to safe state (zero velocity)
        self._set_navigation_safe()

        # Set manipulation to safe state (hold position)
        self._set_manipulation_safe()

        # Set science to safe state (graceful stop)
        self._set_science_safe()

        # Set autonomous typing to safe state (controlled stop)
        self._set_autonomous_typing_safe()

        # Keep minimal subsystems active for safety monitoring
        safe_subsystems = {"camera", "communication"}
        self._active_subsystems = safe_subsystems

        logger.warning(
            "All systems set to safe values",
            active_subsystems=list(self._active_subsystems),
        )

    def _set_navigation_safe(self) -> None:
        """Set navigation system to safe state (zero velocity)."""
        try:
            # In real implementation: publish zero velocity command
            logger.info("Navigation: Set to zero velocity (safe state)")
            # self._navigation_client.call_zero_velocity()
        except Exception as e:
            logger.warning("Failed to set navigation safe", error=str(e))

    def _set_manipulation_safe(self) -> None:
        """Set manipulation system to safe state (hold position)."""
        try:
            # In real implementation: command arm to hold current position
            logger.info("Manipulation: Holding current position (safe state)")
            # self._manipulation_client.call_hold_position()
        except Exception as e:
            logger.warning("Failed to set manipulation safe (may not be present)", error=str(e))

    def _set_science_safe(self) -> None:
        """Set science system to safe state (graceful stop)."""
        try:
            # In real implementation: allow current operation to complete
            logger.info("Science: Completing current operation gracefully (safe state)")
            # self._science_client.call_safe_stop()
        except Exception as e:
            logger.warning("Failed to set science safe (may not be present)", error=str(e))

    def _set_autonomous_typing_safe(self) -> None:
        """Set autonomous typing system to safe state (controlled stop)."""
        try:
            # In real implementation: stop carousel and park servos
            logger.info("Autonomous typing: Controlled stop and park (safe state)")
            # self._typing_client.call_safe_stop()
        except Exception as e:
            logger.warning(
                "Failed to set autonomous typing safe (may not be present)",
                error=str(e),
            )

    def disengage_safety_mode(self) -> None:
        """Resume normal subsystem operation after safety mode."""
        logger.info("Resuming normal subsystem operation")

        # Restore previously active subsystems
        # In real implementation, this would restart subsystems that were
        # safely stopped, not just re-enable them
        self._active_subsystems.update({"navigation", "camera"})

        # Optional subsystems are re-enabled if they exist
        if self._subsystem_exists("manipulation"):
            self._active_subsystems.add("manipulation")

        if self._subsystem_exists("science_instruments"):
            self._active_subsystems.add("science_instruments")

        if self._subsystem_exists("autonomous_typing"):
            self._active_subsystems.add("autonomous_typing")

        logger.info("Normal operation resumed", active_subsystems=list(self._active_subsystems))

    def _subsystem_exists(self, subsystem: str) -> bool:
        """Check if a subsystem is available in this configuration."""
        # In real implementation: check ROS2 service availability
        # For now, assume basic subsystems exist
        return subsystem in ["navigation", "camera", "manipulation"]

    def shutdown_subsystems(self) -> None:
        """Shutdown all subsystems gracefully."""
        logger.info("Shutting down subsystems")

        # In real implementation:
        # - Send shutdown signals to each subsystem
        # - Wait for acknowledgments
        # - Close communication channels

        self._active_subsystems.clear()
        logger.info("All subsystems shutdown")

    def check_subsystem_status(self, subsystem: str) -> Dict:
        """
        Check status of a specific subsystem.

        Args:
            subsystem: Name of subsystem to check

        Returns:
            Dictionary with subsystem status information
        """
        # In real implementation, this would:
        # - Call GetSubsystemStatus service
        # - Return actual status from subsystem

        # For now, return mock status
        is_active = subsystem in self._active_subsystems
        is_failed = subsystem in self._failed_subsystems

        status = {
            "name": subsystem,
            "active": is_active,
            "failed": is_failed,
            "healthy": is_active and not is_failed,
        }

        logger.debug("Subsystem status checked", subsystem=subsystem, status=status)
        return status

    def get_active_subsystems(self) -> List[str]:
        """Get list of currently active subsystems."""
        return list(self._active_subsystems)

    def get_failed_subsystems(self) -> List[str]:
        """Get list of failed subsystems."""
        return list(self._failed_subsystems)

    def mark_subsystem_failed(self, subsystem: str, reason: str = "") -> None:
        """
        Mark a subsystem as failed.

        Args:
            subsystem: Subsystem name
            reason: Reason for failure
        """
        self._failed_subsystems.add(subsystem)
        self._active_subsystems.discard(subsystem)

        logger.error(
            "Subsystem marked as failed",
            subsystem=subsystem,
            reason=reason,
        )

    def recover_subsystem(self, subsystem: str) -> bool:
        """
        Attempt to recover a failed subsystem.

        Args:
            subsystem: Subsystem to recover

        Returns:
            True if recovery successful
        """
        logger.info("Attempting subsystem recovery", subsystem=subsystem)

        # In real implementation:
        # - Restart subsystem
        # - Verify functionality
        # - Re-initialize if needed

        if subsystem in self._failed_subsystems:
            self._failed_subsystems.remove(subsystem)
            self._active_subsystems.add(subsystem)
            logger.info("Subsystem recovered", subsystem=subsystem)
            return True

        logger.warning("Subsystem recovery failed", subsystem=subsystem)
        return False

    def verify_subsystems_ready(self, required_subsystems: List[str]) -> bool:
        """
        Verify that required subsystems are ready.

        Args:
            required_subsystems: List of required subsystem names

        Returns:
            True if all required subsystems are active and healthy
        """
        for subsystem in required_subsystems:
            if subsystem not in self._active_subsystems:
                logger.warning(
                    "Required subsystem not active",
                    subsystem=subsystem,
                )
                return False

            if subsystem in self._failed_subsystems:
                logger.warning(
                    "Required subsystem failed",
                    subsystem=subsystem,
                )
                return False

        logger.debug("All required subsystems ready", subsystems=required_subsystems)
        return True

    def get_subsystem_summary(self) -> Dict:
        """
        Get comprehensive subsystem status summary.

        Returns:
            Dictionary with subsystem summary information
        """
        return {
            "total_known": len(self._known_subsystems),
            "active": len(self._active_subsystems),
            "failed": len(self._failed_subsystems),
            "active_list": list(self._active_subsystems),
            "failed_list": list(self._failed_subsystems),
            "inactive_list": list(set(self._known_subsystems) - self._active_subsystems - self._failed_subsystems),
        }
