#!/usr/bin/env python3
"""
Unit Tests for Safety Manager.

Tests the safety manager's trigger logic, recovery behavior, and state management.
These tests require the autonomy_state_machine package to be importable.
If it is not available in the current environment, the entire module is skipped.
"""

import os
import sys
import unittest
from unittest.mock import Mock, patch

import pytest

# Ensure state_management package root is on PYTHONPATH
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
STATE_MGMT_ROOT = os.path.join(PROJECT_ROOT, "Autonomy", "code", "state_management")
sys.path.insert(0, STATE_MGMT_ROOT)

try:
    from autonomy_state_machine.safety_manager import (  # type: ignore  # noqa: E402
        RecoveryBehavior,
        SafetyManager,
        SafetySeverity,
        SafetyTriggerType,
    )
except Exception:
    SafetyManager = None

if SafetyManager is None:
    pytest.skip(
        "autonomy_state_machine.safety_manager not importable; "
        "safety unit tests require state_management to be on PYTHONPATH.",
        allow_module_level=True,
    )


class TestSafetyManager(unittest.TestCase):
    """Test safety manager functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.safety_manager = SafetyManager()
        self.mock_logger = Mock()
        self.safety_manager._logger = self.mock_logger

    def test_initialization(self):
        """Test safety manager initialization."""
        self.assertIsInstance(self.safety_manager, SafetyManager)
        self.assertEqual(len(self.safety_manager._active_triggers), 0)
        self.assertEqual(len(self.safety_manager._trigger_history), 0)

    def test_emergency_stop_recovery(self):
        """Test emergency stop recovery behavior."""
        behavior = self.safety_manager.determine_recovery(
            SafetyTriggerType.EMERGENCY_STOP,
            None  # current_state not needed for this test
        )

        self.assertIsInstance(behavior, RecoveryBehavior)
        self.assertTrue(behavior.requires_manual_intervention)
        self.assertFalse(behavior.can_auto_recover)
        self.assertIn("Reset emergency stop button", behavior.recovery_steps)
        self.assertEqual(behavior.estimated_recovery_time, 60.0)

    def test_software_estop_recovery(self):
        """Test software ESTOP recovery behavior."""
        behavior = self.safety_manager.determine_recovery(
            SafetyTriggerType.SOFTWARE_ESTOP,
            None
        )

        self.assertIsInstance(behavior, RecoveryBehavior)
        self.assertTrue(behavior.requires_manual_intervention)
        self.assertFalse(behavior.can_auto_recover)
        self.assertIn("Verify software emergency stop trigger", behavior.recovery_steps)
        self.assertEqual(behavior.estimated_recovery_time, 60.0)

    def test_safestop_recovery(self):
        """Test safestop recovery behavior."""
        behavior = self.safety_manager.determine_recovery(
            SafetyTriggerType.SAFESTOP_REQUEST,
            None
        )

        self.assertIsInstance(behavior, RecoveryBehavior)
        self.assertFalse(behavior.requires_manual_intervention)
        self.assertTrue(behavior.can_auto_recover)
        self.assertIn("Operator sends resume signal", behavior.recovery_steps)
        self.assertEqual(behavior.estimated_recovery_time, 5.0)

    def test_proximity_violation_recovery(self):
        """Test proximity violation recovery behavior."""
        behavior = self.safety_manager.determine_recovery(
            SafetyTriggerType.PROXIMITY_VIOLATION,
            None
        )

        self.assertIsInstance(behavior, RecoveryBehavior)
        self.assertFalse(behavior.requires_manual_intervention)
        self.assertTrue(behavior.can_auto_recover)
        self.assertIn("Clear area around rover", behavior.recovery_steps)
        self.assertIsNotNone(behavior.safe_mode_config)
        self.assertEqual(behavior.estimated_recovery_time, 30.0)

    @patch('time.time')
    def test_trigger_safety_event(self, mock_time):
        """Test triggering a safety event."""
        mock_time.return_value = 1000.0

        # Trigger a safety event
        self.safety_manager.trigger_safety(
            SafetyTriggerType.EMERGENCY_STOP,
            SafetySeverity.EMERGENCY,
            "Test emergency stop",
            "test_operator"
        )

        # Verify the trigger was added
        self.assertIn(SafetyTriggerType.EMERGENCY_STOP, self.safety_manager._active_triggers)
        self.assertEqual(len(self.safety_manager._trigger_history), 1)

        # Verify history entry
        history_entry = self.safety_manager._trigger_history[0]
        self.assertEqual(history_entry['type'], SafetyTriggerType.EMERGENCY_STOP)
        self.assertEqual(history_entry['severity'], SafetySeverity.EMERGENCY)
        self.assertEqual(history_entry['description'], "Test emergency stop")
        self.assertEqual(history_entry['source'], "test_operator")

    def test_clear_trigger(self):
        """Test clearing a specific trigger."""
        # Add a trigger
        self.safety_manager.trigger_safety(
            SafetyTriggerType.EMERGENCY_STOP,
            SafetySeverity.EMERGENCY,
            "Test",
            "test"
        )

        # Verify it's active
        self.assertIn(SafetyTriggerType.EMERGENCY_STOP, self.safety_manager._active_triggers)

        # Clear the trigger
        self.safety_manager.clear_trigger(SafetyTriggerType.EMERGENCY_STOP)

        # Verify it's cleared
        self.assertNotIn(SafetyTriggerType.EMERGENCY_STOP, self.safety_manager._active_triggers)

    def test_clear_all_triggers(self):
        """Test clearing all triggers."""
        # Add multiple triggers
        self.safety_manager.trigger_safety(SafetyTriggerType.EMERGENCY_STOP, SafetySeverity.EMERGENCY, "Test1", "test")
        self.safety_manager.trigger_safety(SafetyTriggerType.SAFESTOP_REQUEST, SafetySeverity.WARNING, "Test2", "test")

        # Verify they're active
        self.assertEqual(len(self.safety_manager._active_triggers), 2)

        # Clear all
        self.safety_manager.clear_all_triggers()

        # Verify all cleared
        self.assertEqual(len(self.safety_manager._active_triggers), 0)

    def test_get_safety_status(self):
        """Test getting safety status."""
        # Initially should have no active triggers
        status = self.safety_manager.get_safety_status()
        self.assertEqual(len(status['active_triggers']), 0)
        self.assertIsNone(status['highest_severity'])

        # Add a trigger
        self.safety_manager.trigger_safety(
            SafetyTriggerType.EMERGENCY_STOP,
            SafetySeverity.EMERGENCY,
            "Test emergency",
            "test"
        )

        # Check status again
        status = self.safety_manager.get_safety_status()
        self.assertEqual(len(status['active_triggers']), 1)
        self.assertEqual(status['highest_severity'], SafetySeverity.EMERGENCY)
        self.assertIn(SafetyTriggerType.EMERGENCY_STOP, status['active_triggers'])


if __name__ == '__main__':
    unittest.main()
