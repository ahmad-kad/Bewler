#!/usr/bin/env python3
"""
State Machine Transition Simulator.

Simulates realistic state transitions for the URC 2026 rover state machine,
demonstrating hierarchical state management with proper logging.
"""

import time
import logging
from datetime import datetime
from typing import Optional, Dict, List
from enum import Enum

# Import core state management components
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'autonomy_state_machine'))

from states import (
    SystemState,
    AutonomousSubstate,
    EquipmentServicingSubstate,
    is_valid_transition,
    get_state_metadata,
    get_required_subsystems,
)


class StateTransitionSimulator:
    """
    Simulates state machine transitions with realistic timing and logging.

    Demonstrates the hierarchical state management system in action.
    """

    def __init__(self):
        """Initialize the state transition simulator."""
        self.current_state = SystemState.BOOT
        self.current_substate = AutonomousSubstate.NONE
        self.current_sub_substate = EquipmentServicingSubstate.NONE
        self.previous_state = None
        self.state_entry_time = datetime.now()
        self.transition_count = 0

        # Set up logging
        self.setup_logging()

        self.logger.info(f"[INIT] State Machine Simulator Initialized - Starting in {self.current_state.value} state")

    def setup_logging(self):
        """Configure structured logging for state transitions."""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s [%(levelname)s] %(message)s',
            datefmt='%H:%M:%S'
        )

        # Create a custom logger that formats state transition events nicely
        self.logger = logging.getLogger('StateMachine')
        self.logger.setLevel(logging.INFO)

    def log_state_transition(self, from_state: SystemState, to_state: SystemState,
                           reason: str = "", initiated_by: str = "simulator"):
        """Log a state transition with detailed information."""
        self.transition_count += 1

        # Get state metadata
        from_meta = get_state_metadata(from_state)
        to_meta = get_state_metadata(to_state)

        self.logger.info(f"[TRANSITION #{self.transition_count}] {from_state.value} -> {to_state.value} | Reason: {reason} | By: {initiated_by}")
        self.logger.info(f"   From: {from_meta.description}")
        self.logger.info(f"   To: {to_meta.description}")

        # Log subsystem requirements
        from_subsystems = get_required_subsystems(from_state)
        to_subsystems = get_required_subsystems(to_state)

        if from_subsystems:
            self.logger.info(f"[EXIT] Deactivating subsystems for {from_state.value}: {', '.join(from_subsystems)}")

        if to_subsystems:
            self.logger.info(f"[ENTER] Activating subsystems for {to_state.value}: {', '.join(to_subsystems)}")

    def execute_transition(self, to_state: SystemState,
                          to_substate: Optional[AutonomousSubstate] = None,
                          reason: str = "", initiated_by: str = "simulator") -> bool:
        """
        Execute a state transition with validation and logging.

        Args:
            to_state: Target state
            to_substate: Target substate (if applicable)
            reason: Reason for transition
            initiated_by: Who/what initiated the transition

        Returns:
            True if transition successful
        """
        from_state = self.current_state

        # Validate transition
        if not is_valid_transition(from_state, to_state):
            self.logger.error(f"[BLOCKED] INVALID TRANSITION: {from_state.value} -> {to_state.value} (Invalid transition attempt)")
            return False

        # Log the transition
        self.log_state_transition(from_state, to_state, reason, initiated_by)

        # Execute transition
        self.previous_state = from_state
        self.current_state = to_state
        self.state_entry_time = datetime.now()

        # Update substate if applicable
        if to_substate:
            self.current_substate = to_substate
            self.logger.info(f"📍 Substate updated to {to_substate.value}")
        elif to_state != SystemState.AUTONOMOUS:
            self.current_substate = AutonomousSubstate.NONE
            self.current_sub_substate = EquipmentServicingSubstate.NONE

        # Simulate transition timing
        time.sleep(0.5)  # Simulate transition time

        self.logger.info(f"[COMPLETE] Transition completed: Now in {to_state.value} state (substate: {self.current_substate.value})")

        return True

    def simulate_mission_sequence(self):
        """Simulate a complete mission sequence with realistic state transitions."""
        self.logger.info("[MISSION] Starting Mission Sequence Simulation")
        self.logger.info("=" * 60)

        # Mission sequence: Boot → Calibration → Idle → Autonomous (Science) → Idle → Shutdown

        # 1. Boot to Calibration (system startup)
        time.sleep(1)
        self.execute_transition(
            SystemState.CALIBRATION,
            reason="System boot complete, starting calibration",
            initiated_by="system_boot"
        )

        # 2. Calibration to Idle (calibration complete)
        time.sleep(2)
        self.execute_transition(
            SystemState.IDLE,
            reason="Calibration completed successfully",
            initiated_by="calibration_service"
        )

        # 3. Idle to Autonomous (Science mission)
        time.sleep(1)
        self.execute_transition(
            SystemState.AUTONOMOUS,
            to_substate=AutonomousSubstate.SCIENCE,
            reason="Operator initiated science mission",
            initiated_by="operator_command"
        )

        # Simulate science mission activities
        self.logger.info("[SCIENCE] Science mission active - collecting samples...")
        time.sleep(3)

        # 4. Autonomous to Idle (mission complete)
        self.execute_transition(
            SystemState.IDLE,
            reason="Science mission completed successfully",
            initiated_by="mission_controller"
        )

        # 5. Idle to Autonomous (Equipment Servicing mission)
        time.sleep(1)
        self.execute_transition(
            SystemState.AUTONOMOUS,
            to_substate=AutonomousSubstate.EQUIPMENT_SERVICING,
            reason="Operator initiated equipment servicing mission",
            initiated_by="operator_command"
        )

        # Simulate equipment servicing mission
        self.logger.info("[SERVICING] Equipment servicing mission active...")
        time.sleep(2)

        # 6. Autonomous to Idle (equipment servicing complete)
        self.execute_transition(
            SystemState.IDLE,
            reason="Equipment servicing mission completed",
            initiated_by="mission_controller"
        )

        # 7. Idle to Teleoperation (operator takeover)
        time.sleep(1)
        self.execute_transition(
            SystemState.TELEOPERATION,
            reason="Operator taking manual control",
            initiated_by="operator_command"
        )

        # Simulate teleoperation
        self.logger.info("[TELEOP] Teleoperation mode active...")
        time.sleep(2)

        # 8. Teleoperation to Safety (emergency condition)
        self.execute_transition(
            SystemState.SAFETY,
            reason="Obstacle detected in path - safety trigger",
            initiated_by="safety_monitor"
        )

        # 9. Safety to Idle (safety cleared)
        time.sleep(1)
        self.execute_transition(
            SystemState.IDLE,
            reason="Safety condition cleared by operator",
            initiated_by="operator_command"
        )

        # 10. Idle to Shutdown (mission complete)
        time.sleep(1)
        self.execute_transition(
            SystemState.SHUTDOWN,
            reason="Mission complete, initiating graceful shutdown",
            initiated_by="operator_command"
        )

        self.logger.info("=" * 60)
        self.logger.info("[MISSION] Mission Sequence Simulation Complete")
        self.logger.info(f"[STATS] Total transitions executed: {self.transition_count}")

    def simulate_error_conditions(self):
        """Simulate error conditions and invalid transitions."""
        self.logger.info("[ERROR_TEST] Testing Error Conditions and Invalid Transitions")
        self.logger.info("-" * 50)

        # Reset to a known state
        self.current_state = SystemState.IDLE
        self.logger.info("Reset to IDLE state for error testing")

        # Try invalid transitions
        invalid_transitions = [
            (SystemState.IDLE, SystemState.AUTONOMOUS, "Direct to autonomous (missing calibration)"),
            (SystemState.SHUTDOWN, SystemState.IDLE, "Back from shutdown (terminal state)"),
            (SystemState.SAFETY, SystemState.AUTONOMOUS, "Direct from safety to autonomous"),
            (SystemState.CALIBRATION, SystemState.TELEOPERATION, "Skip idle state"),
        ]

        for from_state, to_state, description in invalid_transitions:
            self.current_state = from_state
            self.logger.info(f"Testing invalid transition: {description}")
            success = self.execute_transition(to_state, reason="Testing invalid transition")
            if not success:
                self.logger.info("[VALIDATED] Invalid transition correctly blocked")
            time.sleep(0.5)

        # Test valid safety transitions
        self.current_state = SystemState.AUTONOMOUS
        self.logger.info("Testing emergency safety transitions")
        self.execute_transition(
            SystemState.SAFETY,
            reason="Emergency stop triggered",
            initiated_by="e_stop_button"
        )

        self.execute_transition(
            SystemState.IDLE,
            reason="Safety condition resolved",
            initiated_by="operator_clearance"
        )

        self.logger.info("-" * 50)
        self.logger.info("[ERROR_TEST] Error Condition Testing Complete")

    def show_state_hierarchy_demo(self):
        """Demonstrate the hierarchical state structure."""
        self.logger.info("[HIERARCHY] Demonstrating Hierarchical State Structure")
        self.logger.info("-" * 50)

        # Show state hierarchy
        states = [
            (SystemState.BOOT, None, None),
            (SystemState.CALIBRATION, None, None),
            (SystemState.IDLE, None, None),
            (SystemState.TELEOPERATION, None, None),
            (SystemState.AUTONOMOUS, AutonomousSubstate.SCIENCE, None),
            (SystemState.AUTONOMOUS, AutonomousSubstate.EQUIPMENT_SERVICING, EquipmentServicingSubstate.TRAVELING),
            (SystemState.AUTONOMOUS, AutonomousSubstate.FOLLOW_ME, None),
            (SystemState.SAFETY, None, None),
            (SystemState.SHUTDOWN, None, None),
        ]

        for state, substate, sub_substate in states:
            hierarchy = [state.value]
            if substate:
                hierarchy.append(substate.value)
            if sub_substate:
                hierarchy.append(sub_substate.value)

            hierarchy_str = " -> ".join(hierarchy)
            self.logger.info(f"[STATE] State Hierarchy: {hierarchy_str}")

            # Show required subsystems
            subsystems = get_required_subsystems(state, substate)
            if subsystems:
                self.logger.info(f"   [SUBSYSTEMS] Required subsystems: {', '.join(subsystems)}")
            else:
                self.logger.info("   [SUBSYSTEMS] No specific subsystems required")

        self.logger.info("-" * 50)
        self.logger.info("[HIERARCHY] Hierarchy Demonstration Complete")


def main():
    """Run the state transition simulation."""
    print("[SIMULATOR] URC 2026 State Machine Transition Simulator")
    print("=" * 60)

    simulator = StateTransitionSimulator()

    # Demonstrate state hierarchy
    simulator.show_state_hierarchy_demo()
    print()

    # Simulate error conditions
    simulator.simulate_error_conditions()
    print()

    # Run full mission sequence
    simulator.simulate_mission_sequence()
    print()

    print("[SIMULATOR] Simulation complete! Check the logs above for detailed state transitions.")


if __name__ == "__main__":
    main()
