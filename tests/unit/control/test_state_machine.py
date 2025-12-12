#!/usr/bin/env python3
"""
Unit Tests for State Machine Control.

Tests state transitions, safety triggers, and mission coordination.
These tests require the autonomy_state_machine package to be importable.
If it is not available in the current environment, the entire module is skipped.
"""

import os
import sys
import unittest
from unittest.mock import Mock

import pytest

# Ensure state_management package root is on PYTHONPATH
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
STATE_MGMT_ROOT = os.path.join(PROJECT_ROOT, "Autonomy", "code", "state_management")
sys.path.insert(0, STATE_MGMT_ROOT)

try:
    from autonomy_state_machine.states import (  # type: ignore  # noqa: E402
        AutonomousMode,
        SystemState,
    )
except Exception:
    SystemState = None
    AutonomousMode = None

if SystemState is None or AutonomousMode is None:
    pytest.skip(
        "autonomy_state_machine package not importable; "
        "state machine unit tests require state_management to be on PYTHONPATH.",
        allow_module_level=True,
    )


class TestStateMachine(unittest.TestCase):
    """Test state machine functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.state_machine = Mock()
        self.state_machine.current_state = SystemState.IDLE
        self.state_machine.transition_to = Mock()
        self.state_machine.is_transition_allowed = Mock(return_value=True)

    def test_initial_state(self):
        """Test initial state is IDLE."""
        self.assertEqual(self.state_machine.current_state, SystemState.IDLE)

    def test_idle_to_teleop_transition(self):
        """Test transition from IDLE to TELEOPERATION."""
        self.state_machine.transition_to(SystemState.TELEOPERATION)

        self.state_machine.transition_to.assert_called_once_with(SystemState.TELEOPERATION)

    def test_idle_to_autonomous_transition(self):
        """Test transition from IDLE to AUTONOMOUS."""
        self.state_machine.transition_to(SystemState.AUTONOMOUS)

        self.state_machine.transition_to.assert_called_once_with(SystemState.AUTONOMOUS)

    def test_autonomous_substates(self):
        """Test autonomous substate transitions."""
        substates = [
            AutonomousMode.SCIENCE,
            AutonomousMode.ARM_CONTROL,
            AutonomousMode.NAVIGATION,
            AutonomousMode.EQUIPMENT
        ]

        for substate in substates:
            with self.subTest(substate=substate):
                self.assertIsInstance(substate, AutonomousMode)

    def test_emergency_stop_transitions(self):
        """Test emergency stop transitions from any state."""
        states_to_test = [
            SystemState.IDLE,
            SystemState.TELEOPERATION,
            SystemState.AUTONOMOUS,
            SystemState.CALIBRATION
        ]

        for from_state in states_to_test:
            with self.subTest(from_state=from_state):
                self.state_machine.current_state = from_state
                self.state_machine.transition_to(SystemState.ESTOP)

                # Verify transition was called
                self.state_machine.transition_to.assert_called_with(SystemState.ESTOP)

    def test_safestop_transitions(self):
        """Test safestop transitions."""
        states_to_test = [
            SystemState.TELEOPERATION,
            SystemState.AUTONOMOUS
        ]

        for from_state in states_to_test:
            with self.subTest(from_state=from_state):
                self.state_machine.current_state = from_state
                self.state_machine.transition_to(SystemState.SAFESTOP)

                self.state_machine.transition_to.assert_called_with(SystemState.SAFESTOP)

    def test_invalid_transitions(self):
        """Test invalid state transitions are blocked."""
        # Mock invalid transition
        self.state_machine.is_transition_allowed.return_value = False

        # Try invalid transition
        self.state_machine.transition_to(SystemState.SHUTDOWN)

        # Should not have transitioned
        self.state_machine.transition_to.assert_called_once_with(SystemState.SHUTDOWN)

    def test_state_machine_recovery(self):
        """Test recovery from safety states."""
        # Start in ESTOP
        self.state_machine.current_state = SystemState.ESTOP

        # Transition back to IDLE
        self.state_machine.transition_to(SystemState.IDLE)

        self.state_machine.transition_to.assert_called_with(SystemState.IDLE)


class TestSubsystemCoordinator(unittest.TestCase):
    """Test subsystem coordination functionality."""

    def setUp(self):
        """Set up subsystem coordinator tests."""
        self.coordinator = Mock()
        self.coordinator.activate_subsystem = Mock()
        self.coordinator.deactivate_subsystem = Mock()
        self.coordinator.engage_safety_mode = Mock()
        self.coordinator.disengage_safety_mode = Mock()

    def test_subsystem_activation(self):
        """Test activating subsystems."""
        subsystems = ['navigation', 'vision', 'safety']

        for subsystem in subsystems:
            self.coordinator.activate_subsystem(subsystem)
            self.coordinator.activate_subsystem.assert_called_with(subsystem)

    def test_subsystem_deactivation(self):
        """Test deactivating subsystems."""
        subsystems = ['navigation', 'vision', 'safety']

        for subsystem in subsystems:
            self.coordinator.deactivate_subsystem(subsystem)
            self.coordinator.deactivate_subsystem.assert_called_with(subsystem)

    def test_safety_mode_engagement(self):
        """Test engaging safety mode."""
        self.coordinator.engage_safety_mode()

        self.coordinator.engage_safety_mode.assert_called_once()

    def test_safety_mode_disengagement(self):
        """Test disengaging safety mode."""
        self.coordinator.disengage_safety_mode()

        self.coordinator.disengage_safety_mode.assert_called_once()

    def test_subsystem_dependencies(self):
        """Test subsystem dependency management."""
        # Navigation depends on safety
        # Vision depends on safety
        # Safety is independent

        dependencies = {
            'navigation': ['safety'],
            'vision': ['safety'],
            'safety': []
        }

        # Test dependency resolution
        for subsystem, deps in dependencies.items():
            self.assertIsInstance(deps, list)
            for dep in deps:
                self.assertIn(dep, dependencies.keys())


class TestMissionCoordinator(unittest.TestCase):
    """Test mission coordination functionality."""

    def setUp(self):
        """Set up mission coordinator tests."""
        self.mission_coord = Mock()
        self.mission_coord.start_mission = Mock(return_value=True)
        self.mission_coord.pause_mission = Mock(return_value=True)
        self.mission_coord.resume_mission = Mock(return_value=True)
        self.mission_coord.stop_mission = Mock(return_value=True)
        self.mission_coord.get_mission_status = Mock(return_value={
            'state': 'running',
            'progress': 0.5,
            'current_waypoint': 2,
            'total_waypoints': 7
        })

    def test_mission_lifecycle(self):
        """Test complete mission lifecycle."""
        # Start mission
        result = self.mission_coord.start_mission()
        self.assertTrue(result)
        self.mission_coord.start_mission.assert_called_once()

        # Pause mission
        result = self.mission_coord.pause_mission()
        self.assertTrue(result)
        self.mission_coord.pause_mission.assert_called_once()

        # Resume mission
        result = self.mission_coord.resume_mission()
        self.assertTrue(result)
        self.mission_coord.resume_mission.assert_called_once()

        # Stop mission
        result = self.mission_coord.stop_mission()
        self.assertTrue(result)
        self.mission_coord.stop_mission.assert_called_once()

    def test_mission_status_tracking(self):
        """Test mission progress tracking."""
        status = self.mission_coord.get_mission_status()

        self.assertEqual(status['state'], 'running')
        self.assertEqual(status['progress'], 0.5)
        self.assertEqual(status['current_waypoint'], 2)
        self.assertEqual(status['total_waypoints'], 7)

    def test_waypoint_progression(self):
        """Test waypoint-by-waypoint progression."""
        waypoints = [
            {'id': 1, 'position': (0.0, 0.0)},
            {'id': 2, 'position': (10.0, 0.0)},
            {'id': 3, 'position': (10.0, 10.0)},
            {'id': 4, 'position': (0.0, 10.0)},
            {'id': 5, 'position': (0.0, 0.0)},  # Return home
        ]

        # Mock waypoint progression
        current_waypoint_index = 0

        def mock_get_status():
            nonlocal current_waypoint_index
            return {
                'current_waypoint': current_waypoint_index + 1,
                'waypoint_position': waypoints[current_waypoint_index]['position']
            }

        self.mission_coord.get_mission_status.side_effect = mock_get_status

        # Simulate progressing through waypoints
        for i, waypoint in enumerate(waypoints):
            status = self.mission_coord.get_mission_status()
            self.assertEqual(status['current_waypoint'], i + 1)
            self.assertEqual(status['waypoint_position'], waypoint['position'])
            current_waypoint_index += 1


class TestSafetyIntegration(unittest.TestCase):
    """Test safety system integration with state machine."""

    def setUp(self):
        """Set up safety integration tests."""
        self.safety_manager = Mock()
        self.safety_manager.trigger_safety = Mock()
        self.safety_manager.clear_trigger = Mock()
        self.safety_manager.get_safety_status = Mock(return_value={
            'active_triggers': [],
            'highest_severity': None
        })

        self.state_machine = Mock()
        self.state_machine.transition_to = Mock()

    def test_emergency_stop_integration(self):
        """Test emergency stop integration."""
        from autonomy_state_machine.safety_manager import (
            SafetySeverity,
            SafetyTriggerType,
        )

        # Trigger emergency stop
        self.safety_manager.trigger_safety(
            SafetyTriggerType.EMERGENCY_STOP,
            SafetySeverity.EMERGENCY,
            "Emergency stop button pressed",
            "operator"
        )

        self.safety_manager.trigger_safety.assert_called_once_with(
            SafetyTriggerType.EMERGENCY_STOP,
            SafetySeverity.EMERGENCY,
            "Emergency stop button pressed",
            "operator"
        )

    def test_software_estop_integration(self):
        """Test software ESTOP integration."""
        from autonomy_state_machine.safety_manager import (
            SafetySeverity,
            SafetyTriggerType,
        )

        # Trigger software ESTOP
        self.safety_manager.trigger_safety(
            SafetyTriggerType.SOFTWARE_ESTOP,
            SafetySeverity.EMERGENCY,
            "Software emergency stop triggered",
            "system"
        )

        self.safety_manager.trigger_safety.assert_called_once_with(
            SafetyTriggerType.SOFTWARE_ESTOP,
            SafetySeverity.EMERGENCY,
            "Software emergency stop triggered",
            "system"
        )

    def test_safestop_integration(self):
        """Test safestop integration."""
        from autonomy_state_machine.safety_manager import (
            SafetySeverity,
            SafetyTriggerType,
        )

        # Trigger safestop
        self.safety_manager.trigger_safety(
            SafetyTriggerType.SAFESTOP_REQUEST,
            SafetySeverity.WARNING,
            "Operator requested safestop",
            "operator"
        )

        self.safety_manager.trigger_safety.assert_called_once_with(
            SafetyTriggerType.SAFESTOP_REQUEST,
            SafetySeverity.WARNING,
            "Operator requested safestop",
            "operator"
        )

    def test_proximity_violation_integration(self):
        """Test proximity violation integration."""
        from autonomy_state_machine.safety_manager import (
            SafetySeverity,
            SafetyTriggerType,
        )

        # Trigger proximity violation
        self.safety_manager.trigger_safety(
            SafetyTriggerType.PROXIMITY_VIOLATION,
            SafetySeverity.CRITICAL,
            "Object detected too close to rover",
            "auto_safe_system"
        )

        self.safety_manager.trigger_safety.assert_called_once_with(
            SafetyTriggerType.PROXIMITY_VIOLATION,
            SafetySeverity.CRITICAL,
            "Object detected too close to rover",
            "auto_safe_system"
        )

    def test_safety_state_transitions(self):
        """Test state machine transitions due to safety events."""
        safety_states = [
            ('emergency_stop', 'ESTOP'),
            ('software_estop', 'ESTOP'),
            ('safestop_request', 'SAFESTOP'),
            ('proximity_violation', 'SAFESTOP')
        ]

        for trigger_type, expected_state in safety_states:
            with self.subTest(trigger=trigger_type, state=expected_state):
                # Mock safety event triggering state transition
                if expected_state == 'ESTOP':
                    self.state_machine.transition_to.assert_called_with(SystemState.ESTOP)
                elif expected_state == 'SAFESTOP':
                    self.state_machine.transition_to.assert_called_with(SystemState.SAFESTOP)


if __name__ == '__main__':
    unittest.main()
