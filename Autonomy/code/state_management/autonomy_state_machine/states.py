"""
State definitions and metadata for the rover state machine.

Defines the hierarchical state structure with top-level states, substates,
and sub-substates, along with metadata for each state.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class SystemState(Enum):
    """Simplified state machine - user-friendly and maintainable"""

    BOOT = "BOOT"  # System startup and validation
    IDLE = "IDLE"  # Ready for commands
    TELEOPERATION = "TELEOPERATION"  # Manual control
    SAFESTOP = "SAFESTOP"  # Context-aware pause (freeze arm, decelerate motion)
    AUTONOMOUS = "AUTONOMOUS"  # Mission execution with sub-modes
    ESTOP = "ESTOP"  # Emergency stop
    SHUTDOWN = "SHUTDOWN"  # System shutdown

    def __str__(self) -> str:
        return self.value


class AutonomousMode(Enum):
    """Autonomous operation modes - context determines behavior"""

    NAVIGATION = "NAVIGATION"  # GPS waypoint navigation
    ARM_CONTROL = "ARM_CONTROL"  # Robotic arm manipulation
    SCIENCE = "SCIENCE"  # Sample collection/analysis
    EQUIPMENT = "EQUIPMENT"  # Equipment servicing
    FOLLOW_ME = "FOLLOW_ME"  # Person following

    def __str__(self) -> str:
        return self.value


class EquipmentServicingSubstate(Enum):
    """Sub-substates for equipment servicing mission."""

    NONE = "NONE"  # Not in equipment servicing
    TRAVELING = "TRAVELING"  # Traveling to lander
    SAMPLE_DELIVERY = "SAMPLE_DELIVERY"  # Delivering sample cache
    PANEL_OPERATIONS = "PANEL_OPERATIONS"  # Opening panels/drawers
    AUTONOMOUS_TYPING = "AUTONOMOUS_TYPING"  # Typing launch code
    USB_CONNECTION = "USB_CONNECTION"  # USB connection and data read
    FUEL_CONNECTION = "FUEL_CONNECTION"  # Connecting fuel hose
    BUTTON_OPERATIONS = "BUTTON_OPERATIONS"  # Buttons, switches, knobs
    COMPLETE = "COMPLETE"  # Mission complete

    def __str__(self) -> str:
        return self.value


class CalibrationSubstate(Enum):
    """
    Substates for detailed calibration procedures.

    URC 2026 Competition Requirements:
    - Camera intrinsic calibration (required for accurate computer vision)
    - Camera extrinsic calibration (hand-eye calibration for manipulation)
    - System validation and integration testing
    """

    NONE = "NONE"  # Not in calibration
    SETUP = "SETUP"  # Environment setup and target preparation
    INTRINSIC_CAPTURE = "INTRINSIC_CAPTURE"  # Capture images for intrinsic calibration
    INTRINSIC_CALIBRATION = "INTRINSIC_CALIBRATION"  # Compute intrinsic parameters
    INTRINSIC_VALIDATION = "INTRINSIC_VALIDATION"  # Validate intrinsic calibration quality
    EXTRINSIC_SETUP = "EXTRINSIC_SETUP"  # Prepare for hand-eye calibration
    EXTRINSIC_CAPTURE = "EXTRINSIC_CAPTURE"  # Capture robot-camera pose pairs
    EXTRINSIC_CALIBRATION = "EXTRINSIC_CALIBRATION"  # Compute hand-eye transformation
    EXTRINSIC_VALIDATION = "EXTRINSIC_VALIDATION"  # Validate extrinsic calibration
    INTEGRATION_TEST = "INTEGRATION_TEST"  # Test full vision-manipulation pipeline
    COMPLETE = "COMPLETE"  # Calibration complete, ready for autonomous ops

    def __str__(self) -> str:
        return self.value


@dataclass
class StateMetadata:
    """
    Metadata for a specific state.

    Attributes:
        state: The state this metadata applies to
        allowed_transitions: States that can be transitioned to from this state
        entry_requirements: List of requirements that must be met to enter state
        exit_requirements: List of requirements that must be met to exit state
        timeout_seconds: Maximum time allowed in state (0.0 = no timeout)
        requires_calibration: Whether calibration must be complete
        requires_subsystems: List of subsystems that must be active
        description: Human-readable description of the state
        auto_transition: If set, automatically transition to this state after timeout
    """

    state: SystemState
    allowed_transitions: List[SystemState] = field(default_factory=list)
    entry_requirements: List[str] = field(default_factory=list)
    exit_requirements: List[str] = field(default_factory=list)
    timeout_seconds: float = 0.0
    requires_calibration: bool = False
    requires_subsystems: List[str] = field(default_factory=list)
    description: str = ""
    auto_transition: Optional[SystemState] = None


# State metadata registry
STATE_METADATA: Dict[SystemState, StateMetadata] = {
    SystemState.BOOT: StateMetadata(
        state=SystemState.BOOT,
        allowed_transitions=[
            SystemState.IDLE,
            SystemState.SAFESTOP,
            SystemState.SHUTDOWN,
        ],
        entry_requirements=[],
        timeout_seconds=30.0,
        auto_transition=SystemState.IDLE,
        description="Initial system boot and validation",
    ),
    SystemState.IDLE: StateMetadata(
        state=SystemState.IDLE,
        allowed_transitions=[
            SystemState.TELEOPERATION,
            SystemState.AUTONOMOUS,
            SystemState.SAFESTOP,
            SystemState.SHUTDOWN,
        ],
        entry_requirements=["boot_complete"],
        description="Ready state, awaiting commands",
    ),
    SystemState.TELEOPERATION: StateMetadata(
        state=SystemState.TELEOPERATION,
        allowed_transitions=[
            SystemState.IDLE,
            SystemState.AUTONOMOUS,
            SystemState.SAFESTOP,
            SystemState.SHUTDOWN,
        ],
        entry_requirements=["boot_complete", "communication_ok"],
        requires_calibration=False,  # Teleoperation doesn't require calibration
        requires_subsystems=["navigation"],
        description="Manual remote control mode",
    ),
    SystemState.AUTONOMOUS: StateMetadata(
        state=SystemState.AUTONOMOUS,
        allowed_transitions=[
            SystemState.IDLE,
            SystemState.TELEOPERATION,
            SystemState.SAFESTOP,
            SystemState.SHUTDOWN,
        ],
        entry_requirements=[
            "boot_complete",
            "calibration_complete",
            "communication_ok",
        ],
        requires_calibration=True,
        requires_subsystems=["navigation", "computer_vision", "slam"],
        description="Autonomous operation with mission substates",
    ),
    SystemState.ESTOP: StateMetadata(
        state=SystemState.ESTOP,
        allowed_transitions=[
            SystemState.IDLE,
            SystemState.SHUTDOWN,
        ],
        entry_requirements=[],  # Can enter from any state
        exit_requirements=["estop_reset", "power_verified", "manual_verification"],
        timeout_seconds=0.0,  # No automatic timeout for emergency stop
        description="Hardware emergency stop - power disconnect requiring manual reset",
    ),
    SystemState.SAFESTOP: StateMetadata(
        state=SystemState.SAFESTOP,
        allowed_transitions=[
            SystemState.IDLE,
            SystemState.TELEOPERATION,
            SystemState.AUTONOMOUS,
            SystemState.ESTOP,  # Can escalate to emergency stop
        ],
        entry_requirements=[],  # Can enter from any state
        exit_requirements=["operator_resume", "area_clear"],
        timeout_seconds=300.0,  # 5 minutes max in safestop before requiring attention
        auto_transition=SystemState.IDLE,  # Auto-resume if operator doesn't respond
        description="Software graceful pause - toggle-able by operator",
    ),
    SystemState.SHUTDOWN: StateMetadata(
        state=SystemState.SHUTDOWN,
        allowed_transitions=[],  # Terminal state
        entry_requirements=[],
        timeout_seconds=10.0,
        description="Graceful shutdown sequence",
    ),
}


# Autonomous mode metadata
AUTONOMOUS_MODE_METADATA: Dict[AutonomousMode, Dict[str, Any]] = {
    AutonomousMode.NAVIGATION: {
        "description": "GPS waypoint navigation to specified locations",
        "requires_subsystems": ["navigation", "localization"],
        "context": "motion_active",
        "max_duration_seconds": 3600,
        "safety_behavior": "decelerate_motion",
    },
    AutonomousMode.ARM_CONTROL: {
        "description": "Robotic arm manipulation for sample handling",
        "requires_subsystems": ["arm_controller", "computer_vision"],
        "context": "arm_active",
        "max_duration_seconds": 1800,
        "safety_behavior": "freeze_arm",
    },
    AutonomousMode.SCIENCE: {
        "description": "Sample collection and onboard analysis",
        "requires_subsystems": [
            "science_instruments",
            "arm_controller",
            "computer_vision",
        ],
        "context": "science_active",
        "max_duration_seconds": 1800,
        "safety_behavior": "pause_analysis",
    },
    AutonomousMode.EQUIPMENT: {
        "description": "Equipment servicing and autonomous typing",
        "requires_subsystems": ["arm_controller", "computer_vision", "typing_system"],
        "context": "equipment_active",
        "max_duration_seconds": 1800,
        "safety_behavior": "freeze_arm",
    },
    AutonomousMode.FOLLOW_ME: {
        "description": "Follow person with ArUco tag at safe distance",
        "requires_subsystems": ["computer_vision", "aruco_detection", "navigation"],
        "context": "follow_active",
        "max_duration_seconds": 3600,
        "safety_behavior": "stop_following",
    },
}


def get_state_metadata(state: SystemState) -> StateMetadata:
    """Get metadata for a given state."""
    return STATE_METADATA.get(state, StateMetadata(state=state))


def is_valid_transition(from_state: SystemState, to_state: SystemState) -> bool:
    """
    Check if a state transition is valid based on state metadata.

    Args:
        from_state: Current state
        to_state: Desired state

    Returns:
        True if transition is allowed
    """
    metadata = get_state_metadata(from_state)
    return to_state in metadata.allowed_transitions


def get_required_subsystems(state: SystemState, substate: Optional[AutonomousMode] = None) -> List[str]:
    """
    Get list of required subsystems for a state/substate combination.

    Args:
        state: System state
        substate: Optional autonomous substate

    Returns:
        List of required subsystem names
    """
    subsystems = STATE_METADATA[state].requires_subsystems.copy()

    if state == SystemState.AUTONOMOUS and substate:
        substate_info = AUTONOMOUS_MODE_METADATA.get(substate, {})
        subsystems.extend(substate_info.get("requires_subsystems", []))

    return list(set(subsystems))  # Remove duplicates
