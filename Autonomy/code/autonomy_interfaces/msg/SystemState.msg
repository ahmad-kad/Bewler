# System State Message - URC 2026 Mars Rover Autonomy
#
# This message provides comprehensive state information for the rover's hierarchical
# state machine. It tracks the current operational state, subsystem status, and
# mission context to enable coordinated autonomous operations.
#
# The state machine uses a three-level hierarchy:
# - current_state: Top-level operational mode (BOOT, CALIBRATION, IDLE, etc.)
# - substate: Mission-specific context (SCIENCE, DELIVERY, etc.)
# - sub_substate: Detailed operational state (SAMPLE_DELIVERY, etc.)
#
# This design allows for flexible mission execution while maintaining clear
# operational boundaries and safety constraints.

std_msgs/Header header

# Primary state information
string current_state              # Top-level state (BOOT, CALIBRATION, IDLE, NAVIGATION, EXECUTION, RECOVERY, SHUTDOWN)
string substate                   # Mission-specific substate (SCIENCE, DELIVERY, MAINTENANCE, etc.)
string sub_substate              # Detailed substate (SAMPLE_COLLECTION, DELIVERY_SETUP, DIAGNOSTIC_CHECK, etc.)

# State metadata
float64 time_in_state            # Seconds spent in current state (resets on transition)
float64 state_timeout            # Maximum allowed time in state (0.0 = no timeout, seconds)
string previous_state            # Last state before current transition
builtin_interfaces/Time transition_timestamp  # UTC timestamp when current state was entered

# System status flags
bool is_transitioning            # True during state transitions (prevents conflicting operations)
bool preconditions_met           # True when all autonomous operation prerequisites are satisfied
string[] active_subsystems       # List of currently active subsystems (navigation, vision, slam, etc.)
string[] failed_subsystems       # List of subsystems currently in error state

# Mission and operational context
string mission_phase             # Competition mission context (setup, terrain, equipment_service, science, etc.)
string operator_id               # Identifier of who/what initiated current state (auto, manual, safety_system)
string state_reason              # Human-readable explanation for current state and any restrictions
