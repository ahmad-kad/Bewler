"""
Unit tests for state machine components.

Tests state definitions, transition validation, safety handling,
and state machine logic.
"""

import pytest
from autonomy_state_machine.safety_manager import (
    SafetyManager,
    SafetySeverity,
    SafetyTriggerType,
)
from autonomy_state_machine.states import (
    AutonomousSubstate,
    SystemState,
    get_required_subsystems,
    get_state_metadata,
    is_valid_transition,
)
from autonomy_state_machine.transition_validator import TransitionValidator


class TestStates:
    """Test state definitions and metadata."""

    def test_system_states_exist(self):
        """Test that all system states are defined."""
        expected_states = [
            "BOOT",
            "CALIBRATION",
            "IDLE",
            "TELEOPERATION",
            "AUTONOMOUS",
            "SAFETY",
            "SHUTDOWN",
        ]

        for state_name in expected_states:
            state = SystemState(state_name)
            assert state is not None
            assert str(state) == state_name

    def test_autonomous_substates_exist(self):
        """Test that autonomous substates are defined."""
        expected_substates = [
            "NONE",
            "SCIENCE",
            "DELIVERY",
            "EQUIPMENT_SERVICING",
            "AUTONOMOUS_NAVIGATION",
        ]

        for substate_name in expected_substates:
            substate = AutonomousSubstate(substate_name)
            assert substate is not None
            assert str(substate) == substate_name

    def test_state_metadata_exists(self):
        """Test that metadata exists for all states."""
        for state in SystemState:
            metadata = get_state_metadata(state)
            assert metadata is not None
            assert metadata.state == state

    def test_boot_to_idle_transition_valid(self):
        """Test that BOOT -> IDLE transition is valid."""
        assert is_valid_transition(SystemState.BOOT, SystemState.IDLE) is True

    def test_idle_to_boot_transition_invalid(self):
        """Test that IDLE -> BOOT transition is invalid."""
        assert is_valid_transition(SystemState.IDLE, SystemState.BOOT) is False

    def test_any_state_to_safety_valid(self):
        """Test that any state can transition to SAFETY."""
        for state in SystemState:
            if state != SystemState.SHUTDOWN:  # SHUTDOWN is terminal
                metadata = get_state_metadata(state)
                # SAFETY should be in allowed transitions for most states
                # (except SHUTDOWN which is terminal)
                if state != SystemState.SHUTDOWN:
                    assert SystemState.SAFETY in metadata.allowed_transitions

    def test_required_subsystems_autonomous(self):
        """Test required subsystems for autonomous state."""
        subsystems = get_required_subsystems(SystemState.AUTONOMOUS)
        assert "navigation" in subsystems
        assert "computer_vision" in subsystems
        assert "slam" in subsystems

    def test_required_subsystems_autonomous_navigation(self):
        """Test required subsystems for autonomous navigation mission."""
        subsystems = get_required_subsystems(SystemState.AUTONOMOUS, AutonomousSubstate.AUTONOMOUS_NAVIGATION)
        assert "navigation" in subsystems
        assert "computer_vision" in subsystems


class TestTransitionValidator:
    """Test transition validation logic."""

    def test_validator_initialization(self):
        """Test validator initializes correctly."""
        validator = TransitionValidator()
        assert validator is not None

    def test_boot_to_idle_requires_boot_complete(self):
        """Test BOOT -> IDLE requires boot complete."""
        validator = TransitionValidator()

        # Without boot complete
        is_valid, message, failed = validator.validate_transition(SystemState.BOOT, SystemState.IDLE)
        assert is_valid is False
        assert "boot_complete" in failed

        # With boot complete
        validator.set_boot_complete(True)
        is_valid, message, failed = validator.validate_transition(SystemState.BOOT, SystemState.IDLE)
        assert is_valid is True
        assert len(failed) == 0

    def test_idle_to_autonomous_requires_calibration(self):
        """Test IDLE -> AUTONOMOUS requires calibration."""
        validator = TransitionValidator()
        validator.set_boot_complete(True)
        validator.set_communication_status(True)
        validator.update_active_subsystems(["navigation", "computer_vision", "slam"])

        # Without calibration
        is_valid, message, failed = validator.validate_transition(SystemState.IDLE, SystemState.AUTONOMOUS)
        assert is_valid is False
        assert "calibration_required" in failed or "calibration_complete" in failed

        # With calibration
        validator.set_calibration_complete(True)
        is_valid, message, failed = validator.validate_transition(SystemState.IDLE, SystemState.AUTONOMOUS)
        assert is_valid is True

    def test_force_transition_skips_validation(self):
        """Test that forced transitions skip precondition checks."""
        validator = TransitionValidator()

        # Force transition without meeting requirements
        is_valid, message, failed = validator.validate_transition(SystemState.BOOT, SystemState.AUTONOMOUS, force=True)
        assert is_valid is True
        assert len(failed) == 0

    def test_autonomous_navigation_requires_gnss(self):
        """Test autonomous navigation requires GNSS."""
        validator = TransitionValidator()
        validator.set_boot_complete(True)
        validator.set_calibration_complete(True)
        validator.set_communication_status(True)
        validator.update_active_subsystems(["navigation", "computer_vision", "slam"])

        # Without GNSS
        is_valid, message, failed = validator.validate_transition(
            SystemState.IDLE,
            SystemState.AUTONOMOUS,
            AutonomousSubstate.AUTONOMOUS_NAVIGATION,
        )
        assert is_valid is False
        assert "gnss_required" in failed

        # With GNSS
        validator.set_gnss_available(True)
        is_valid, message, failed = validator.validate_transition(
            SystemState.IDLE,
            SystemState.AUTONOMOUS,
            AutonomousSubstate.AUTONOMOUS_NAVIGATION,
        )
        assert is_valid is True

    def test_precondition_status(self):
        """Test getting precondition status."""
        validator = TransitionValidator()
        validator.set_boot_complete(True)
        validator.set_calibration_complete(False)

        status = validator.get_precondition_status()
        assert status["boot_complete"] is True
        assert status["calibration_complete"] is False


class TestSafetyManager:
    """Test safety management logic."""

    def test_safety_manager_initialization(self):
        """Test safety manager initializes correctly."""
        manager = SafetyManager()
        assert manager is not None
        assert manager.get_active_triggers() == []

    def test_trigger_safety(self):
        """Test triggering safety event."""
        manager = SafetyManager()

        manager.trigger_safety(
            SafetyTriggerType.EMERGENCY_STOP,
            SafetySeverity.EMERGENCY,
            "E-stop pressed",
            "physical_button",
        )

        triggers = manager.get_active_triggers()
        assert SafetyTriggerType.EMERGENCY_STOP in triggers

    def test_clear_trigger(self):
        """Test clearing safety trigger."""
        manager = SafetyManager()

        manager.trigger_safety(
            SafetyTriggerType.THERMAL_WARNING,
            SafetySeverity.WARNING,
            "Temperature high",
        )
        assert len(manager.get_active_triggers()) == 1

        manager.clear_trigger(SafetyTriggerType.THERMAL_WARNING)
        assert len(manager.get_active_triggers()) == 0

    def test_highest_severity(self):
        """Test getting highest severity level."""
        manager = SafetyManager()

        manager.trigger_safety(SafetyTriggerType.THERMAL_WARNING, SafetySeverity.WARNING, "Temp high")
        manager.trigger_safety(SafetyTriggerType.BATTERY_CRITICAL, SafetySeverity.EMERGENCY, "Battery low")

        highest = manager.get_highest_severity()
        assert highest == SafetySeverity.EMERGENCY

    def test_emergency_stop_recovery(self):
        """Test recovery behavior for emergency stop."""
        manager = SafetyManager()

        recovery = manager.determine_recovery(
            SafetyTriggerType.EMERGENCY_STOP,
            SystemState.TELEOPERATION,
        )

        assert recovery.requires_manual_intervention is True
        assert recovery.can_auto_recover is False
        assert len(recovery.recovery_steps) > 0

    def test_communication_loss_teleoperation(self):
        """Test communication loss in teleoperation mode."""
        manager = SafetyManager()

        recovery = manager.determine_recovery(
            SafetyTriggerType.COMMUNICATION_LOSS,
            SystemState.TELEOPERATION,
        )

        assert recovery.requires_manual_intervention is True
        assert recovery.can_auto_recover is False

    def test_communication_loss_autonomous(self):
        """Test communication loss in autonomous mode."""
        manager = SafetyManager()

        recovery = manager.determine_recovery(
            SafetyTriggerType.COMMUNICATION_LOSS,
            SystemState.AUTONOMOUS,
        )

        # In autonomous, can attempt auto-recovery
        assert recovery.can_auto_recover is True

    def test_thermal_warning_auto_recovery(self):
        """Test thermal warning allows auto-recovery."""
        manager = SafetyManager()

        recovery = manager.determine_recovery(
            SafetyTriggerType.THERMAL_WARNING,
            SystemState.AUTONOMOUS,
        )

        assert recovery.can_auto_recover is True
        assert recovery.requires_manual_intervention is False

    def test_battery_critical_auto_trigger(self):
        """Test battery critical auto-triggers safety."""
        manager = SafetyManager()

        # Update with critical battery level
        manager.update_system_health(battery_level=5.0)

        triggers = manager.get_active_triggers()
        assert SafetyTriggerType.BATTERY_CRITICAL in triggers

    def test_temperature_warning_auto_trigger(self):
        """Test temperature warning auto-triggers safety."""
        manager = SafetyManager()

        # Update with high temperature
        manager.update_system_health(temperature=75.0)

        triggers = manager.get_active_triggers()
        assert SafetyTriggerType.THERMAL_WARNING in triggers

    def test_safety_status(self):
        """Test getting safety status."""
        manager = SafetyManager()

        status = manager.get_safety_status()
        assert status["is_safe"] is True

        manager.trigger_safety(SafetyTriggerType.SENSOR_FAILURE, SafetySeverity.CRITICAL, "Sensor failed")

        status = manager.get_safety_status()
        assert status["is_safe"] is False
        assert len(status["active_triggers"]) > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
