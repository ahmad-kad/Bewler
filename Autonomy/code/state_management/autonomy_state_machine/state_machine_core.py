"""
State Machine Core using Transitions Library.

Provides robust state machine implementation with automatic invalid transition
prevention, guards, callbacks, and visualization capabilities.
"""

from typing import Optional, Dict, List, Callable
from transitions import Machine
from transitions.core import EventData

try:
    # Try relative import first (when used as part of package)
    from .states import (
        SystemState,
        AutonomousSubstate,
        CalibrationSubstate,
        get_state_metadata,
        is_valid_transition,
        get_required_subsystems,
    )
except ImportError:
    # Fall back to absolute import (when testing standalone)
    from states import (
        SystemState,
        AutonomousSubstate,
        CalibrationSubstate,
        get_state_metadata,
        is_valid_transition,
        get_required_subsystems,
    )


class RoverStateMachine:
    """
    State machine using transitions library for robustness.

    Provides:
    - Automatic invalid transition prevention
    - State machine visualization (graphviz)
    - Guards and callbacks for complex logic
    - Event-driven architecture
    - State persistence and restoration
    """

    def __init__(self, node_logger=None, enable_visualization: bool = False):
        """
        Initialize the rover state machine.

        Args:
            node_logger: ROS2 logger instance
            enable_visualization: Enable graphviz visualization
        """
        self.logger = node_logger
        self.enable_visualization = enable_visualization

        # Define all possible states
        self.states_list = [state.value for state in SystemState]

        # Initialize transitions machine
        self.machine = Machine(
            model=self,
            states=self.states_list,
            initial=SystemState.BOOT.value,
            auto_transitions=False,  # We'll define explicit transitions
            ignore_invalid_triggers=True,  # Log invalid transitions instead of crashing
        )

        # Add callbacks
        self.machine.before_state_change.append(self._on_before_transition)
        self.machine.after_state_change.append(self._on_after_transition)

        # Add all valid transitions from states.py metadata
        self._add_transitions()

        # Store substate information (transitions library doesn't handle nested states well)
        self._current_autonomous_substate = AutonomousSubstate.NONE
        self._current_calibration_substate = CalibrationSubstate.NONE

        if self.logger:
            self.logger.info("RoverStateMachine initialized with transitions library")

    def _add_transitions(self) -> None:
        """Add all valid state transitions from the metadata."""

        # BOOT transitions
        self.machine.add_transition('boot_to_calibration', SystemState.BOOT.value, SystemState.CALIBRATION.value)
        self.machine.add_transition('boot_to_idle', SystemState.BOOT.value, SystemState.IDLE.value)
        self.machine.add_transition('boot_to_safety', SystemState.BOOT.value, SystemState.SAFETY.value)
        self.machine.add_transition('boot_to_shutdown', SystemState.BOOT.value, SystemState.SHUTDOWN.value)

        # CALIBRATION transitions
        self.machine.add_transition('calibration_to_idle', SystemState.CALIBRATION.value, SystemState.IDLE.value)
        self.machine.add_transition('calibration_to_safety', SystemState.CALIBRATION.value, SystemState.SAFETY.value)
        self.machine.add_transition('calibration_to_shutdown', SystemState.CALIBRATION.value, SystemState.SHUTDOWN.value)

        # IDLE transitions
        self.machine.add_transition('idle_to_calibration', SystemState.IDLE.value, SystemState.CALIBRATION.value)
        self.machine.add_transition('idle_to_teleop', SystemState.IDLE.value, SystemState.TELEOPERATION.value)
        self.machine.add_transition('idle_to_autonomous', SystemState.IDLE.value, SystemState.AUTONOMOUS.value)
        self.machine.add_transition('idle_to_safety', SystemState.IDLE.value, SystemState.SAFETY.value)
        self.machine.add_transition('idle_to_shutdown', SystemState.IDLE.value, SystemState.SHUTDOWN.value)

        # TELEOPERATION transitions
        self.machine.add_transition('teleop_to_idle', SystemState.TELEOPERATION.value, SystemState.IDLE.value)
        self.machine.add_transition('teleop_to_autonomous', SystemState.TELEOPERATION.value, SystemState.AUTONOMOUS.value)
        self.machine.add_transition('teleop_to_safety', SystemState.TELEOPERATION.value, SystemState.SAFETY.value)
        self.machine.add_transition('teleop_to_shutdown', SystemState.TELEOPERATION.value, SystemState.SHUTDOWN.value)

        # AUTONOMOUS transitions
        self.machine.add_transition('autonomous_to_idle', SystemState.AUTONOMOUS.value, SystemState.IDLE.value)
        self.machine.add_transition('autonomous_to_teleop', SystemState.AUTONOMOUS.value, SystemState.TELEOPERATION.value)
        self.machine.add_transition('autonomous_to_safety', SystemState.AUTONOMOUS.value, SystemState.SAFETY.value)
        self.machine.add_transition('autonomous_to_shutdown', SystemState.AUTONOMOUS.value, SystemState.SHUTDOWN.value)

        # ESTOP transitions (hardware emergency stop)
        self.machine.add_transition('any_to_estop', '*', SystemState.ESTOP.value)  # Can enter from any state
        self.machine.add_transition('estop_to_idle', SystemState.ESTOP.value, SystemState.IDLE.value)
        self.machine.add_transition('estop_to_shutdown', SystemState.ESTOP.value, SystemState.SHUTDOWN.value)

        # SAFESTOP transitions (software graceful pause)
        self.machine.add_transition('any_to_safestop', '*', SystemState.SAFESTOP.value)  # Can enter from any state
        self.machine.add_transition('safestop_to_idle', SystemState.SAFESTOP.value, SystemState.IDLE.value)
        self.machine.add_transition('safestop_to_teleop', SystemState.SAFESTOP.value, SystemState.TELEOPERATION.value)
        self.machine.add_transition('safestop_to_autonomous', SystemState.SAFESTOP.value, SystemState.AUTONOMOUS.value)
        self.machine.add_transition('safestop_to_estop', SystemState.SAFESTOP.value, SystemState.ESTOP.value)  # Can escalate

        # SAFETY transitions (legacy - deprecated)
        self.machine.add_transition('safety_to_idle', SystemState.SAFETY.value, SystemState.IDLE.value)
        self.machine.add_transition('safety_to_teleop', SystemState.SAFETY.value, SystemState.TELEOPERATION.value)
        self.machine.add_transition('safety_to_shutdown', SystemState.SAFETY.value, SystemState.SHUTDOWN.value)

        # SHUTDOWN is terminal - no outgoing transitions

    def _on_before_transition(self, *args, **kwargs) -> None:
        """Called before any state transition."""
        # The transitions library passes event_data as the first arg
        if args:
            event = args[0]
            from_state = getattr(event, 'source', None)
            to_state = getattr(event, 'dest', None)

            if from_state and to_state:
                # Handle state objects
                from_name = from_state.name if hasattr(from_state, 'name') else str(from_state)
                to_name = to_state.name if hasattr(to_state, 'name') else str(to_state)

                if self.logger:
                    self.logger.info(f"Preparing transition: {from_name} -> {to_name}")

                # Validate transition using our metadata (extra safety check)
                try:
                    from_state_enum = SystemState(from_name)
                    to_state_enum = SystemState(to_name)

                    if not is_valid_transition(from_state_enum, to_state_enum):
                        if self.logger:
                            self.logger.error(f"Invalid transition blocked: {from_name} -> {to_name}")
                        raise ValueError(f"Invalid transition: {from_name} -> {to_name}")
                except ValueError as e:
                    if self.logger:
                        self.logger.error(f"State validation error: {e}")
                    raise

    def _on_after_transition(self, *args, **kwargs) -> None:
        """Called after any state transition."""
        if args:
            event = args[0]
            from_state = getattr(event, 'source', None)
            to_state = getattr(event, 'dest', None)

            if from_state and to_state:
                from_name = from_state.name if hasattr(from_state, 'name') else str(from_state)
                to_name = to_state.name if hasattr(to_state, 'name') else str(to_state)

                if self.logger:
                    self.logger.info(f"Transition completed: {from_name} -> {to_name}")

    def get_current_state(self) -> SystemState:
        """Get current system state."""
        return SystemState(self.state)

    def get_current_autonomous_substate(self) -> AutonomousSubstate:
        """Get current autonomous substate."""
        return self._current_autonomous_substate

    def get_current_calibration_substate(self) -> CalibrationSubstate:
        """Get current calibration substate."""
        return self._current_calibration_substate

    def set_autonomous_substate(self, substate: AutonomousSubstate) -> None:
        """Set autonomous substate (for when in AUTONOMOUS state)."""
        if self.get_current_state() == SystemState.AUTONOMOUS:
            self._current_autonomous_substate = substate
            if self.logger:
                self.logger.info(f"Autonomous substate set to: {substate.value}")
        else:
            if self.logger:
                self.logger.warning(f"Cannot set autonomous substate when not in AUTONOMOUS state (current: {self.state})")

    def set_calibration_substate(self, substate: CalibrationSubstate) -> None:
        """Set calibration substate (for when in CALIBRATION state)."""
        if self.get_current_state() == SystemState.CALIBRATION:
            self._current_calibration_substate = substate
            if self.logger:
                self.logger.info(f"Calibration substate set to: {substate.value}")
        else:
            if self.logger:
                self.logger.warning(f"Cannot set calibration substate when not in CALIBRATION state (current: {self.state})")

    def can_transition_to(self, target_state: SystemState) -> bool:
        """Check if transition to target state is possible."""
        try:
            # Map state names to trigger abbreviations used in _add_transitions
            state_abbrevs = {
                'TELEOPERATION': 'teleop',
                'AUTONOMOUS': 'autonomous',
                'CALIBRATION': 'calibration',
            }
            from_abbrev = state_abbrevs.get(self.state, self.state.lower())
            to_abbrev = state_abbrevs.get(target_state.value, target_state.value.lower())
            trigger_name = f"{from_abbrev}_to_{to_abbrev}"
            return hasattr(self, trigger_name)
        except Exception:
            return False

    def transition_to(self, target_state: SystemState) -> bool:
        """
        Attempt transition to target state.

        Returns:
            True if transition successful, False if invalid or blocked
        """
        if self.state == target_state.value:
            # Already in target state
            return True

        try:
            # Use same trigger name generation as can_transition_to
            state_abbrevs = {
                'TELEOPERATION': 'teleop',
                'AUTONOMOUS': 'autonomous',
                'CALIBRATION': 'calibration',
            }
            from_abbrev = state_abbrevs.get(self.state, self.state.lower())
            to_abbrev = state_abbrevs.get(target_state.value, target_state.value.lower())
            trigger_name = f"{from_abbrev}_to_{to_abbrev}"

            if hasattr(self, trigger_name):
                trigger_method = getattr(self, trigger_name)
                trigger_method()
                return True
            else:
                if self.logger:
                    self.logger.warning(f"No trigger {trigger_name} for transition from {self.state} to {target_state.value}")
                return False
        except Exception as e:
            if self.logger:
                self.logger.error(f"Transition failed: {str(e)}")
            return False

    def get_available_transitions(self) -> List[str]:
        """Get list of available transition triggers."""
        transitions = []
        for attr_name in dir(self):
            if attr_name.endswith('_to_') and callable(getattr(self, attr_name)):
                transitions.append(attr_name)
        return transitions

    def get_state_info(self) -> Dict:
        """Get comprehensive state information."""
        current_state = self.get_current_state()
        metadata = get_state_metadata(current_state)

        return {
            'current_state': current_state.value,
            'autonomous_substate': self._current_autonomous_substate.value,
            'calibration_substate': self._current_calibration_substate.value,
            'available_transitions': self.get_available_transitions(),
            'state_metadata': {
                'description': metadata.description,
                'requires_calibration': metadata.requires_calibration,
                'required_subsystems': get_required_subsystems(current_state, self._current_autonomous_substate),
            }
        }

    def visualize(self, filename: str = "state_machine") -> None:
        """Generate state machine visualization (requires graphviz)."""
        if not self.enable_visualization:
            if self.logger:
                self.logger.warning("Visualization not enabled")
            return

        try:
            from transitions.extensions import GraphMachine
            if isinstance(self.machine, GraphMachine):
                self.machine.get_graph().draw(f"{filename}.png", prog='dot')
                if self.logger:
                    self.logger.info(f"State machine visualization saved to {filename}.png")
            else:
                if self.logger:
                    self.logger.warning("GraphMachine not available for visualization")
        except ImportError:
            if self.logger:
                self.logger.warning("graphviz not installed, cannot generate visualization")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Visualization failed: {str(e)}")

    def reset_to_boot(self) -> None:
        """Reset state machine to BOOT state (for testing/emergency)."""
        self._current_autonomous_substate = AutonomousSubstate.NONE
        self._current_calibration_substate = CalibrationSubstate.NONE
        self.to_boot()  # transitions library method
        if self.logger:
            self.logger.warning("State machine reset to BOOT state")
