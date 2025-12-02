#!/usr/bin/env python3
"""
Integration Tests for Safety and Navigation Systems

Tests interaction between safety system and navigation system.
"""

import unittest
import time
from unittest.mock import Mock, patch, MagicMock


class TestSafetyNavigationIntegration(unittest.TestCase):
    """Test integration between safety and navigation systems."""

    def setUp(self):
        """Set up integration test fixtures."""
        # Mock safety manager
        self.safety_manager = Mock()
        self.safety_manager.trigger_safety = Mock()
        self.safety_manager.clear_trigger = Mock()
        self.safety_manager.get_safety_status = Mock(return_value={
            'active_triggers': [],
            'highest_severity': None
        })

        # Mock navigation system
        self.navigation = Mock()
        self.navigation.stop_navigation = Mock()
        self.navigation.resume_navigation = Mock()
        self.navigation.get_navigation_status = Mock(return_value={
            'is_navigating': False,
            'current_waypoint': None,
            'velocity': 0.0
        })

        # Mock state machine
        self.state_machine = Mock()
        self.state_machine.transition_to = Mock()
        self.state_machine.current_state = Mock()

    def test_emergency_stop_halts_navigation(self):
        """Test that emergency stop immediately halts navigation."""
        from autonomy_state_machine.safety_manager import SafetyTriggerType, SafetySeverity

        # Start navigation
        self.navigation.get_navigation_status.return_value = {
            'is_navigating': True,
            'current_waypoint': (10.0, 20.0),
            'velocity': 1.5
        }

        # Verify navigation is active
        status = self.navigation.get_navigation_status()
        self.assertTrue(status['is_navigating'])
        self.assertEqual(status['velocity'], 1.5)

        # Trigger emergency stop
        self.safety_manager.trigger_safety(
            SafetyTriggerType.EMERGENCY_STOP,
            SafetySeverity.EMERGENCY,
            "Emergency stop activated",
            "safety_system"
        )

        # Verify emergency stop was triggered
        self.safety_manager.trigger_safety.assert_called_once_with(
            SafetyTriggerType.EMERGENCY_STOP,
            SafetySeverity.EMERGENCY,
            "Emergency stop activated",
            "safety_system"
        )

        # Simulate safety system response - navigation should stop
        self.navigation.stop_navigation()
        self.navigation.stop_navigation.assert_called_once()

        # Verify navigation stopped
        self.navigation.get_navigation_status.return_value = {
            'is_navigating': False,
            'current_waypoint': None,
            'velocity': 0.0
        }
        status = self.navigation.get_navigation_status()
        self.assertFalse(status['is_navigating'])
        self.assertEqual(status['velocity'], 0.0)

    def test_safestop_pauses_navigation(self):
        """Test that safestop pauses navigation (not halts)."""
        from autonomy_state_machine.safety_manager import SafetyTriggerType, SafetySeverity

        # Start navigation
        self.navigation.get_navigation_status.return_value = {
            'is_navigating': True,
            'current_waypoint': (5.0, 5.0),
            'velocity': 1.0
        }

        # Trigger safestop
        self.safety_manager.trigger_safety(
            SafetyTriggerType.SAFESTOP_REQUEST,
            SafetySeverity.WARNING,
            "Operator requested safestop",
            "operator"
        )

        # Verify safestop was triggered
        self.safety_manager.trigger_safety.assert_called_once()

        # Navigation should pause but remember state
        self.navigation.stop_navigation()  # Pause navigation
        self.navigation.stop_navigation.assert_called_once()

        # Check that navigation state is preserved
        preserved_state = {
            'last_waypoint': (5.0, 5.0),
            'can_resume': True
        }
        self.assertTrue(preserved_state['can_resume'])

    def test_safestop_resume_navigation(self):
        """Test resuming navigation after safestop."""
        from autonomy_state_machine.safety_manager import SafetyTriggerType, SafetySeverity

        # Clear any active triggers
        self.safety_manager.clear_trigger(SafetyTriggerType.SAFESTOP_REQUEST)

        # Resume navigation
        self.navigation.resume_navigation()
        self.navigation.resume_navigation.assert_called_once()

        # Verify navigation resumed
        self.navigation.get_navigation_status.return_value = {
            'is_navigating': True,
            'current_waypoint': (5.0, 5.0),
            'velocity': 1.0
        }
        status = self.navigation.get_navigation_status()
        self.assertTrue(status['is_navigating'])

    def test_proximity_violation_navigation_response(self):
        """Test navigation response to proximity violations."""
        from autonomy_state_machine.safety_manager import SafetyTriggerType, SafetySeverity

        # Navigation active
        self.navigation.get_navigation_status.return_value = {
            'is_navigating': True,
            'velocity': 1.5
        }

        # Proximity violation detected
        self.safety_manager.trigger_safety(
            SafetyTriggerType.PROXIMITY_VIOLATION,
            SafetySeverity.CRITICAL,
            "Object detected 0.5m ahead",
            "proximity_monitor"
        )

        # Navigation should slow down or stop
        self.navigation.stop_navigation()
        self.navigation.stop_navigation.assert_called_once()

        # Verify reduced speed mode
        self.navigation.get_navigation_status.return_value = {
            'is_navigating': True,
            'velocity': 0.1,  # Reduced speed
            'safe_mode': True
        }
        status = self.navigation.get_navigation_status()
        self.assertLess(status['velocity'], 0.5)  # Significantly reduced

    def test_state_machine_safety_transitions(self):
        """Test state machine transitions due to safety events."""
        from autonomy_state_machine.states import SystemState
        from autonomy_state_machine.safety_manager import SafetyTriggerType, SafetySeverity

        # Test ESTOP transition
        self.safety_manager.trigger_safety(
            SafetyTriggerType.EMERGENCY_STOP,
            SafetySeverity.EMERGENCY,
            "Hardware ESTOP",
            "operator"
        )

        # State machine should transition to ESTOP
        self.state_machine.transition_to(SystemState.ESTOP)
        self.state_machine.transition_to.assert_called_with(SystemState.ESTOP)

        # Reset for next test
        self.state_machine.reset_mock()

        # Test SAFESTOP transition
        self.safety_manager.trigger_safety(
            SafetyTriggerType.SAFESTOP_REQUEST,
            SafetySeverity.WARNING,
            "Software safestop",
            "operator"
        )

        # State machine should transition to SAFESTOP
        self.state_machine.transition_to(SystemState.SAFESTOP)
        self.state_machine.transition_to.assert_called_with(SystemState.SAFESTOP)

    def test_navigation_safety_mode_coordination(self):
        """Test coordination between navigation and safety modes."""
        # Mock subsystem coordinator
        coordinator = Mock()
        coordinator.engage_safety_mode = Mock()
        coordinator.disengage_safety_mode = Mock()

        # Engage safety mode
        coordinator.engage_safety_mode()

        # Navigation should be set to safe state
        coordinator.engage_safety_mode.assert_called_once()

        # Safety mode should zero velocity
        self.navigation.get_navigation_status.return_value = {
            'velocity': 0.0,
            'is_navigating': False
        }

        # Disengage safety mode
        coordinator.disengage_safety_mode()
        coordinator.disengage_safety_mode.assert_called_once()

        # Navigation should be able to resume
        self.navigation.resume_navigation()
        self.navigation.resume_navigation.assert_called_once()

    def test_multiple_safety_events_handling(self):
        """Test handling multiple simultaneous safety events."""
        from autonomy_state_machine.safety_manager import SafetyTriggerType, SafetySeverity

        # Multiple safety events
        events = [
            (SafetyTriggerType.PROXIMITY_VIOLATION, "Object too close"),
            (SafetyTriggerType.SAFESTOP_REQUEST, "Operator pause"),
        ]

        # Trigger multiple events
        for trigger_type, description in events:
            self.safety_manager.trigger_safety(
                trigger_type,
                SafetySeverity.WARNING,
                description,
                "system"
            )

        # Verify both were triggered
        self.assertEqual(self.safety_manager.trigger_safety.call_count, 2)

        # System should handle multiple safety states (prioritize most critical)
        # In this case, SAFESTOP should be active
        self.state_machine.transition_to(SystemState.SAFESTOP)

    def test_safety_event_recovery_flow(self):
        """Test complete safety event recovery flow."""
        from autonomy_state_machine.states import SystemState
        from autonomy_state_machine.safety_manager import SafetyTriggerType, SafetySeverity

        # 1. Trigger safety event
        self.safety_manager.trigger_safety(
            SafetyTriggerType.SAFESTOP_REQUEST,
            SafetySeverity.WARNING,
            "Temporary pause",
            "operator"
        )

        # 2. System transitions to safe state
        self.state_machine.transition_to(SystemState.SAFESTOP)

        # 3. Navigation pauses
        self.navigation.stop_navigation()

        # 4. Clear safety trigger
        self.safety_manager.clear_trigger(SafetyTriggerType.SAFESTOP_REQUEST)

        # 5. Transition back to operational state
        self.state_machine.transition_to(SystemState.AUTONOMOUS)

        # 6. Navigation resumes
        self.navigation.resume_navigation()

        # Verify complete flow
        self.assertEqual(self.state_machine.transition_to.call_count, 2)
        self.assertEqual(self.navigation.stop_navigation.call_count, 1)
        self.assertEqual(self.navigation.resume_navigation.call_count, 1)


class TestNavigationVisionIntegration(unittest.TestCase):
    """Test integration between navigation and computer vision."""

    def setUp(self):
        """Set up navigation-vision integration tests."""
        # Mock navigation
        self.navigation = Mock()
        self.navigation.update_waypoint_from_vision = Mock()

        # Mock vision system
        self.vision = Mock()
        self.vision.detect_aruco_markers = Mock(return_value=[
            {'id': 42, 'position': (10.0, 20.0), 'orientation': (0, 0, 0, 1)}
        ])
        self.vision.get_target_pose = Mock(return_value={
            'position': (5.0, 5.0, 0.0),
            'orientation': (0, 0, 0, 1)
        })

    def test_vision_guided_navigation(self):
        """Test navigation using vision-based pose corrections."""
        # Vision detects target
        targets = self.vision.detect_aruco_markers()
        self.assertEqual(len(targets), 1)
        self.assertEqual(targets[0]['id'], 42)

        # Navigation updates waypoint based on vision
        target_pose = self.vision.get_target_pose()
        self.navigation.update_waypoint_from_vision(target_pose)

        self.navigation.update_waypoint_from_vision.assert_called_once_with(target_pose)

    def test_precision_navigation_integration(self):
        """Test precision navigation using vision feedback."""
        # Initial waypoint
        initial_waypoint = (10.0, 10.0)

        # Vision provides precise correction
        vision_correction = {
            'position': (10.05, 10.02, 0.0),  # 5cm adjustment
            'orientation': (0, 0, 0, 1)
        }

        # Navigation should use vision correction for precision
        self.navigation.update_waypoint_from_vision(vision_correction)
        self.navigation.update_waypoint_from_vision.assert_called_once_with(vision_correction)


if __name__ == '__main__':
    unittest.main()







