"""
State definitions and metadata for the rover state machine.

Defines the hierarchical state structure with top-level states, substates,
and sub-substates, along with metadata for each state.
"""

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any


class SystemState(Enum):
    """Top-level system states for the rover."""

    BOOT = "BOOT"  # Initial startup and system initialization
    CALIBRATION = "CALIBRATION"  # Sensor and system calibration
    IDLE = "IDLE"  # Ready but not operating
    TELEOPERATION = "TELEOPERATION"  # Manual remote control
    AUTONOMOUS = "AUTONOMOUS"  # Autonomous operation with substates
    ESTOP = "ESTOP"  # Hardware emergency stop (power cut)
    SAFESTOP = "SAFESTOP"  # Software graceful pause (toggle-able)
    SAFETY = "SAFETY"  # Legacy safety state (deprecated, use ESTOP/SAFESTOP)
    SHUTDOWN = "SHUTDOWN"  # Graceful shutdown sequence

    def __str__(self) -> str:
        return self.value


class AutonomousSubstate(Enum):
    """Substates for autonomous operations (competition missions)."""

    NONE = "NONE"  # No specific mission active
    SCIENCE = "SCIENCE"  # Science mission
    DELIVERY = "DELIVERY"  # Delivery mission
    EQUIPMENT_SERVICING = "EQUIPMENT_SERVICING"  # Equipment servicing mission
    AUTONOMOUS_NAVIGATION = "AUTONOMOUS_NAVIGATION"  # Autonomous navigation mission
    FOLLOW_ME = "FOLLOW_ME"  # Follow me mode using ArUco tag detection

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
            SystemState.CALIBRATION,
            SystemState.IDLE,
            SystemState.SAFETY,
            SystemState.SHUTDOWN,
        ],
        entry_requirements=[],
        timeout_seconds=30.0,
        auto_transition=SystemState.IDLE,
        description="Initial system boot and initialization",
    ),
    SystemState.CALIBRATION: StateMetadata(
        state=SystemState.CALIBRATION,
        allowed_transitions=[
            SystemState.IDLE,
            SystemState.SAFETY,
            SystemState.SHUTDOWN,
        ],
        entry_requirements=["boot_complete"],
        requires_subsystems=["camera", "navigation"],
        timeout_seconds=300.0,  # 5 minutes max for calibration
        description="Sensor and system calibration",
    ),
    SystemState.IDLE: StateMetadata(
        state=SystemState.IDLE,
        allowed_transitions=[
            SystemState.CALIBRATION,
            SystemState.TELEOPERATION,
            SystemState.AUTONOMOUS,
            SystemState.SAFETY,
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
            SystemState.SAFETY,
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
            SystemState.SAFETY,
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
    SystemState.SAFETY: StateMetadata(
        state=SystemState.SAFETY,
        allowed_transitions=[
            SystemState.IDLE,
            SystemState.TELEOPERATION,
            SystemState.SHUTDOWN,
        ],
        entry_requirements=[],  # Can enter from any state
        exit_requirements=["safety_cleared", "manual_verification"],
        description="Legacy safety state - deprecated, use ESTOP/SAFESTOP",
    ),
    SystemState.SHUTDOWN: StateMetadata(
        state=SystemState.SHUTDOWN,
        allowed_transitions=[],  # Terminal state
        entry_requirements=[],
        timeout_seconds=10.0,
        description="Graceful shutdown sequence",
    ),
}


# Autonomous substate metadata
AUTONOMOUS_SUBSTATE_METADATA: Dict[AutonomousSubstate, Dict[str, Any]] = {
    AutonomousSubstate.NONE: {
        "description": "No mission active",
        "requires_subsystems": [],
    },
    AutonomousSubstate.SCIENCE: {
        "description": "Science mission - sample collection and analysis",
        "requires_subsystems": ["computer_vision", "navigation", "science_instruments"],
        "max_duration_seconds": 1800,  # 30 minutes
    },
    AutonomousSubstate.DELIVERY: {
        "description": "Delivery mission - object pickup and delivery",
        "requires_subsystems": ["computer_vision", "navigation", "manipulation"],
        "max_duration_seconds": 3600,  # 60 minutes
    },
    AutonomousSubstate.EQUIPMENT_SERVICING: {
        "description": "Equipment servicing mission",
        "requires_subsystems": [
            "computer_vision",
            "navigation",
            "manipulation",
            "autonomous_typing",
        ],
        "max_duration_seconds": 1800,  # 30 minutes
    },
    AutonomousSubstate.AUTONOMOUS_NAVIGATION: {
        "description": "Autonomous navigation mission",
        "requires_subsystems": ["computer_vision", "navigation", "slam"],
        "requires_gnss": True,
        "max_duration_seconds": 1800,  # 30 minutes
    },
    AutonomousSubstate.FOLLOW_ME: {
        "description": "Follow me mode - follows person with ArUco tag at safe distance",
        "requires_subsystems": ["computer_vision", "navigation", "aruco_detection"],
        "requires_aruco_detection": True,
        "max_duration_seconds": 3600,  # 60 minutes (longer for follow mode)
        "safety_distance_meters": 2.0,  # Safe following distance
        "max_speed_ms": 1.0,  # Maximum following speed
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


def get_required_subsystems(
    state: SystemState, substate: Optional[AutonomousSubstate] = None
) -> List[str]:
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
        substate_info = AUTONOMOUS_SUBSTATE_METADATA.get(substate, {})
        subsystems.extend(substate_info.get("requires_subsystems", []))

    return list(set(subsystems))  # Remove duplicates

