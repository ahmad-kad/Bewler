"""
Transition validator for state machine.

Validates state transitions based on transition matrix, preconditions,
and mission-specific requirements.
"""

from typing import List, Tuple, Dict, Optional, Set

from .states import (
    SystemState,
    AutonomousSubstate,
    get_state_metadata,
    is_valid_transition,
    get_required_subsystems,
)


class ValidationError(Exception):
    """Raised when state transition validation fails."""

    pass


class TransitionValidator:
    """
    Validates state transitions with precondition checking.

    Handles validation of state transitions including checking transition matrix,
    preconditions, subsystem readiness, and mission-specific requirements.
    """

    def __init__(self, logger=None, state_machine_ref=None):
        """Initialize the transition validator."""
        self.logger = logger  # Will be set by state machine director
        self.state_machine_ref = state_machine_ref  # Reference to state machine for live state

    def set_calibration_complete(self, complete: bool) -> None:
        """Set calibration completion status."""
        self._calibration_complete = complete
        if self.logger:
            self.logger.info(f"Calibration status updated: {complete}")

    def set_boot_complete(self, complete: bool) -> None:
        """Set boot completion status."""
        self._boot_complete = complete
        if self.logger:
            self.logger.info(f"Boot status updated: {complete}")

    def set_communication_status(self, ok: bool) -> None:
        """Set communication status."""
        self._communication_ok = ok
        if self.logger:
            self.logger.info(f"Communication status updated: {ok}")

    def set_safety_status(self, cleared: bool, verified: bool = False) -> None:
        """Set safety clearance status."""
        self._safety_cleared = cleared
        self._manual_verification = verified
        if self.logger:
            self.logger.info(f"Safety status updated: cleared={cleared}, verified={verified}")

    def update_active_subsystems(self, subsystems: List[str]) -> None:
        """Update the set of active subsystems."""
        self._active_subsystems = set(subsystems)
        if self.logger:
            self.logger.debug(f"Active subsystems updated: {subsystems}")

    def add_active_subsystem(self, subsystem: str) -> None:
        """Add a subsystem to active set."""
        self._active_subsystems.add(subsystem)
        if self.logger:
            self.logger.debug(f"Subsystem activated: {subsystem}")

    def remove_active_subsystem(self, subsystem: str) -> None:
        """Remove a subsystem from active set."""
        self._active_subsystems.discard(subsystem)
        if self.logger:
            self.logger.debug(f"Subsystem deactivated: {subsystem}")

    def set_gnss_available(self, available: bool) -> None:
        """Set GNSS availability status."""
        self._gnss_available = available
        if self.logger:
            self.logger.info(f"GNSS availability updated: {available}")

    def set_aruco_detection_available(self, available: bool) -> None:
        """Set ArUco detection availability status."""
        self._aruco_detection_available = available
        if self.logger:
            self.logger.info(f"ArUco detection availability updated: {available}")

    def validate_transition(
        self,
        from_state: SystemState,
        to_state: SystemState,
        to_substate: Optional[AutonomousSubstate] = None,
        force: bool = False,
    ) -> Tuple[bool, str, List[str]]:
        """
        Validate a state transition.

        Args:
            from_state: Current state
            to_state: Desired state
            to_substate: Optional desired substate
            force: If True, skip some validation checks

        Returns:
            Tuple of (is_valid, message, failed_preconditions)
        """
        failed_preconditions: List[str] = []

        # Check if transition is allowed in transition matrix
        if not is_valid_transition(from_state, to_state):
            message = (
                f"Transition from {from_state} to {to_state} not allowed by "
                f"transition matrix"
            )
            if self.logger:
                self.logger.warning(
                    f"Invalid transition attempt: {from_state} -> {to_state}"
                )
            return False, message, ["transition_not_allowed"]

        # If forcing, skip precondition checks
        if force:
            if self.logger:
                self.logger.warning(
                    f"Forcing transition, skipping preconditions: {from_state} -> {to_state}"
                )
            return True, "Transition forced", []

        # Check entry requirements for target state
        to_metadata = get_state_metadata(to_state)
        for requirement in to_metadata.entry_requirements:
            if not self._check_requirement(requirement):
                failed_preconditions.append(requirement)
                logger.warning(
                    "Entry requirement not met",
                    requirement=requirement,
                    to_state=str(to_state),
                )

        # Check exit requirements for current state
        from_metadata = get_state_metadata(from_state)
        for requirement in from_metadata.exit_requirements:
            if not self._check_requirement(requirement):
                failed_preconditions.append(requirement)
                logger.warning(
                    "Exit requirement not met",
                    requirement=requirement,
                    from_state=str(from_state),
                )

        # Check subsystem requirements
        required_subsystems = get_required_subsystems(to_state, to_substate)
        for subsystem in required_subsystems:
            if subsystem not in self._active_subsystems:
                failed_preconditions.append(f"subsystem_{subsystem}")
                logger.warning(
                    "Required subsystem not active",
                    subsystem=subsystem,
                    to_state=str(to_state),
                )

        # Check calibration requirement
        if to_metadata.requires_calibration and not self._calibration_complete:
            failed_preconditions.append("calibration_required")
            logger.warning(
                "Calibration required but not complete", to_state=str(to_state)
            )

        # Check mission-specific requirements
        if to_state == SystemState.AUTONOMOUS and to_substate:
            mission_failed = self._check_mission_requirements(
                to_substate, failed_preconditions
            )
            if mission_failed:
                failed_preconditions.extend(mission_failed)

        # Determine result
        if failed_preconditions:
            message = (
                f"Transition validation failed. "
                f"Failed preconditions: {', '.join(failed_preconditions)}"
            )
            return False, message, failed_preconditions

        logger.info(
            "Transition validated successfully",
            from_state=str(from_state),
            to_state=str(to_state),
        )
        return True, "Transition valid", []

    def _check_requirement(self, requirement: str) -> bool:
        """
        Check if a specific requirement is met.

        Args:
            requirement: Requirement string to check

        Returns:
            True if requirement is met
        """
        requirement_checks = {
            "boot_complete": lambda: self._boot_complete,
            "calibration_complete": lambda: self._calibration_complete,
            "communication_ok": lambda: self._communication_ok,
            "safety_cleared": lambda: self._safety_cleared,
            "manual_verification": lambda: self._manual_verification,
        }

        check_func = requirement_checks.get(requirement)
        if check_func:
            return check_func()

        # If requirement not recognized, log warning and assume not met
        logger.warning("Unknown requirement", requirement=requirement)
        return False

    def _check_mission_requirements(
        self, substate: AutonomousSubstate, failed: List[str]
    ) -> List[str]:
        """
        Check mission-specific requirements for autonomous substates.

        Args:
            substate: Autonomous substate to check
            failed: List to append failures to

        Returns:
            List of failed requirements
        """
        mission_failed = []

        # AUTONOMOUS_NAVIGATION requires GNSS
        if (
            substate == AutonomousSubstate.AUTONOMOUS_NAVIGATION
            and not self._gnss_available
        ):
            mission_failed.append("gnss_required")
            logger.warning(
                "GNSS required for autonomous navigation but not available"
            )
            
        # FOLLOW_ME requires ArUco detection
        if (
            substate == AutonomousSubstate.FOLLOW_ME
            and not self._aruco_detection_available
        ):
            mission_failed.append("aruco_detection_required")
            logger.warning(
                "ArUco detection required for follow me mode but not available"
            )

        return mission_failed

    def get_precondition_status(self) -> Dict[str, bool]:
        """
        Get current status of all preconditions.

        Returns:
            Dictionary mapping precondition names to their status
        """
        return {
            "boot_complete": self._boot_complete,
            "calibration_complete": self._calibration_complete,
            "communication_ok": self._communication_ok,
            "safety_cleared": self._safety_cleared,
            "manual_verification": self._manual_verification,
            "gnss_available": self._gnss_available,
            "aruco_detection_available": self._aruco_detection_available,
            "active_subsystems": list(self._active_subsystems),
        }

