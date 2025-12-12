"""
Context-aware safety management and recovery coordination.

Manages safety triggers, determines appropriate responses based on current
system state and mission context, and coordinates recovery procedures.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

from .states import AutonomousMode, SystemState


class SafetyTriggerType(Enum):
    """Types of safety triggers."""

    EMERGENCY_STOP = "EMERGENCY_STOP"  # Physical E-stop pressed (hardware power cut)
    SOFTWARE_ESTOP = "SOFTWARE_ESTOP"  # Software-triggered emergency stop (full shutdown)
    SAFESTOP_REQUEST = "SAFESTOP_REQUEST"  # Operator-requested safestop (toggle-able)
    PROXIMITY_VIOLATION = "PROXIMITY_VIOLATION"  # Auto-safe: object too close
    COMMUNICATION_LOSS = "COMMUNICATION_LOSS"  # Lost communication link
    BATTERY_CRITICAL = "BATTERY_CRITICAL"  # Battery critically low
    THERMAL_WARNING = "THERMAL_WARNING"  # System overheating
    SENSOR_FAILURE = "SENSOR_FAILURE"  # Critical sensor failure
    OBSTACLE_CRITICAL = "OBSTACLE_CRITICAL"  # Imminent collision
    NAV_INTEGRITY_LOSS = "NAV_INTEGRITY_LOSS"  # Navigation integrity compromised
    TIMING_VIOLATION = "TIMING_VIOLATION"  # Timing safety constraint violated
    SYSTEM_FAULT = "SYSTEM_FAULT"  # General system fault
    MANUAL_INTERVENTION = "MANUAL_INTERVENTION"  # Manual safety trigger

    def __str__(self) -> str:
        return self.value


class SafetySeverity(Enum):
    """Severity levels for safety triggers."""

    INFO = "INFO"  # Informational, no action needed
    WARNING = "WARNING"  # Warning, monitor but continue
    CRITICAL = "CRITICAL"  # Critical, degrade operations
    EMERGENCY = "EMERGENCY"  # Emergency, immediate halt

    def __str__(self) -> str:
        return self.value


@dataclass
class RecoveryBehavior:
    """
    Defines recovery behavior for a safety trigger.

    Attributes:
        requires_manual_intervention: Whether manual intervention is required
        can_auto_recover: Whether automatic recovery is possible
        recovery_steps: List of steps needed for recovery
        safe_mode_config: Configuration for safe mode if applicable
        estimated_recovery_time: Estimated time to recover (seconds)
    """

    requires_manual_intervention: bool
    can_auto_recover: bool
    recovery_steps: List[str]
    safe_mode_config: Optional[Dict[str, any]] = None
    estimated_recovery_time: float = 0.0


class SafetyManager:
    """
    Context-aware safety management with adaptive recovery procedures.

    Monitors safety triggers and determines appropriate responses based on
    current system state, mission phase, and operational context.
    """

    def __init__(self, logger=None) -> None:
        """Initialize safety manager with monitoring state."""
        self._logger = logger  # Set by state machine director
        self._active_triggers: Dict[SafetyTriggerType, SafetySeverity] = {}
        self._trigger_history: List[Dict] = []
        self._battery_level: float = 100.0
        self._temperature: float = 25.0
        self._communication_ok: bool = True
        self._sensors_ok: bool = True

    def trigger_safety(
        self,
        trigger_type: SafetyTriggerType,
        severity: SafetySeverity,
        description: str,
        source: str = "unknown",
    ) -> None:
        """
        Trigger a safety event.

        Args:
            trigger_type: Type of safety trigger
            severity: Severity level
            description: Human-readable description
            source: Source subsystem that triggered safety
        """
        self._active_triggers[trigger_type] = severity

        trigger_info = {
            "type": trigger_type,
            "severity": severity,
            "description": description,
            "source": source,
        }
        self._trigger_history.append(trigger_info)

        if self._logger:
            self._logger.warning(
                f"Safety triggered: {trigger_type.value} ({severity.value}) - {description} " f"(source: {source})"
            )

    def clear_trigger(self, trigger_type: SafetyTriggerType) -> None:
        """Clear a specific safety trigger."""
        if trigger_type in self._active_triggers:
            del self._active_triggers[trigger_type]
            if self._logger:
                self._logger.info(f"Safety trigger cleared: {trigger_type.value}")

    def clear_all_triggers(self) -> None:
        """Clear all active safety triggers."""
        self._active_triggers.clear()
        if self._logger:
            self._logger.info("All safety triggers cleared")

    def get_active_triggers(self) -> List[SafetyTriggerType]:
        """Get list of currently active safety triggers."""
        return list(self._active_triggers.keys())

    def get_highest_severity(self) -> Optional[SafetySeverity]:
        """Get the highest severity of active triggers."""
        if not self._active_triggers:
            return None

        severities = list(self._active_triggers.values())
        # Order: EMERGENCY > CRITICAL > WARNING > INFO
        severity_order = [
            SafetySeverity.EMERGENCY,
            SafetySeverity.CRITICAL,
            SafetySeverity.WARNING,
            SafetySeverity.INFO,
        ]

        for severity in severity_order:
            if severity in severities:
                return severity

        return None

    def determine_recovery(
        self,
        trigger_type: SafetyTriggerType,
        current_state: SystemState,
        mission_phase: Optional[AutonomousMode] = None,
    ) -> RecoveryBehavior:
        """
        Determine appropriate recovery behavior based on context.

        Args:
            trigger_type: Type of safety trigger
            current_state: Current system state
            mission_phase: Current mission phase if in autonomous state

        Returns:
            RecoveryBehavior object describing recovery procedure
        """
        if self._logger:
            state_str = current_state.value if current_state else "unknown"
            self._logger.info(f"Determining recovery behavior for {trigger_type.value} in {state_str} state")

        # Emergency stop (ESTOP) - hardware power cut, requires full recovery
        if trigger_type == SafetyTriggerType.EMERGENCY_STOP:
            return RecoveryBehavior(
                requires_manual_intervention=True,
                can_auto_recover=False,
                recovery_steps=[
                    "Reset emergency stop button",
                    "Verify rover systems",
                    "Manually clear safety state",
                    "Check power systems",
                ],
                estimated_recovery_time=60.0,
            )

        # Software emergency stop - behaves exactly like hardware ESTOP
        if trigger_type == SafetyTriggerType.SOFTWARE_ESTOP:
            return RecoveryBehavior(
                requires_manual_intervention=True,
                can_auto_recover=False,
                recovery_steps=[
                    "Verify software emergency stop trigger",
                    "Reset software emergency stop",
                    "Verify rover systems",
                    "Manually clear safety state",
                    "Check system integrity",
                ],
                estimated_recovery_time=60.0,
            )

        # Safestop request - operator-initiated, toggle-able
        if trigger_type == SafetyTriggerType.SAFESTOP_REQUEST:
            return RecoveryBehavior(
                requires_manual_intervention=False,
                can_auto_recover=True,
                recovery_steps=[
                    "Operator sends resume signal",
                    "Verify area is clear",
                    "Resume normal operation",
                ],
                estimated_recovery_time=5.0,  # Quick recovery
            )

        # Proximity violation - auto-safe triggered
        if trigger_type == SafetyTriggerType.PROXIMITY_VIOLATION:
            return RecoveryBehavior(
                requires_manual_intervention=False,
                can_auto_recover=True,
                recovery_steps=[
                    "Clear area around rover",
                    "Verify proximity sensors",
                    "Resume when safe distance maintained",
                ],
                safe_mode_config={
                    "max_velocity": 0.1,
                    "require_sensor_clearance": True,
                },
                estimated_recovery_time=30.0,
            )

        # Navigation integrity loss
        if trigger_type == SafetyTriggerType.NAV_INTEGRITY_LOSS:
            if current_state == SystemState.AUTONOMOUS:
                return RecoveryBehavior(
                    requires_manual_intervention=True,
                    can_auto_recover=False,
                    recovery_steps=[
                        "Switch to teleoperation mode",
                        "Recalibrate navigation sensors",
                        "Verify GPS and localization",
                        "Manually resume autonomous operation",
                    ],
                    estimated_recovery_time=300.0,
                )
            else:
                return RecoveryBehavior(
                    requires_manual_intervention=False,
                    can_auto_recover=True,
                    recovery_steps=[
                        "Attempt sensor recalibration",
                        "Use backup navigation sources",
                        "Continue with degraded navigation",
                    ],
                    safe_mode_config={"use_backup_nav": True, "reduced_speed": True},
                    estimated_recovery_time=60.0,
                )

        # Timing violation
        if trigger_type == SafetyTriggerType.TIMING_VIOLATION:
            return RecoveryBehavior(
                requires_manual_intervention=False,
                can_auto_recover=True,
                recovery_steps=[
                    "Check system timing constraints",
                    "Adjust task priorities",
                    "Monitor for timing violations",
                    "Resume normal timing",
                ],
                safe_mode_config={"reduced_task_load": True, "timing_monitoring": True},
                estimated_recovery_time=10.0,
            )

        # Communication loss - different behavior for teleoperation vs autonomous
        if trigger_type == SafetyTriggerType.COMMUNICATION_LOSS:
            if current_state == SystemState.TELEOPERATION:
                # In teleoperation, stop immediately
                return RecoveryBehavior(
                    requires_manual_intervention=True,
                    can_auto_recover=False,
                    recovery_steps=[
                        "Restore communication link",
                        "Verify rover position",
                        "Resume teleoperation",
                    ],
                    estimated_recovery_time=120.0,
                )
            else:
                # In autonomous, can attempt auto-recovery
                return RecoveryBehavior(
                    requires_manual_intervention=False,
                    can_auto_recover=True,
                    recovery_steps=[
                        "Switch to backup communication",
                        "Return to last known good position",
                        "Wait for communication restoration",
                    ],
                    safe_mode_config={"max_velocity": 0.5, "use_backup_comms": True},
                    estimated_recovery_time=180.0,
                )

        # Battery critical - graceful shutdown
        if trigger_type == SafetyTriggerType.BATTERY_CRITICAL:
            return RecoveryBehavior(
                requires_manual_intervention=True,
                can_auto_recover=False,
                recovery_steps=[
                    "Navigate to safe position if possible",
                    "Shutdown non-critical systems",
                    "Save state and prepare for shutdown",
                    "Replace/recharge battery",
                ],
                safe_mode_config={
                    "disable_non_essential": True,
                    "emergency_navigation": True,
                },
                estimated_recovery_time=300.0,
            )

        # Thermal warning - throttle operations
        if trigger_type == SafetyTriggerType.THERMAL_WARNING:
            return RecoveryBehavior(
                requires_manual_intervention=False,
                can_auto_recover=True,
                recovery_steps=[
                    "Reduce computational load",
                    "Throttle motor operations",
                    "Monitor temperature",
                    "Resume when temperature normal",
                ],
                safe_mode_config={
                    "max_velocity": 0.3,
                    "reduce_vision_fps": True,
                    "disable_intensive_tasks": True,
                },
                estimated_recovery_time=300.0,
            )

        # Sensor failure - degrade to safe operating mode
        if trigger_type == SafetyTriggerType.SENSOR_FAILURE:
            # More critical during autonomous navigation
            if current_state == SystemState.AUTONOMOUS:
                return RecoveryBehavior(
                    requires_manual_intervention=True,
                    can_auto_recover=False,
                    recovery_steps=[
                        "Stop autonomous operations",
                        "Switch to teleoperation",
                        "Diagnose sensor issue",
                        "Replace or recalibrate sensor",
                    ],
                    estimated_recovery_time=600.0,
                )
            else:
                return RecoveryBehavior(
                    requires_manual_intervention=False,
                    can_auto_recover=True,
                    recovery_steps=[
                        "Switch to backup sensor",
                        "Continue with degraded capability",
                        "Log sensor failure for maintenance",
                    ],
                    safe_mode_config={"use_backup_sensors": True},
                    estimated_recovery_time=30.0,
                )

        # Obstacle critical - immediate stop
        if trigger_type == SafetyTriggerType.OBSTACLE_CRITICAL:
            return RecoveryBehavior(
                requires_manual_intervention=False,
                can_auto_recover=True,
                recovery_steps=[
                    "Emergency stop",
                    "Assess obstacle",
                    "Plan alternative path",
                    "Resume navigation",
                ],
                safe_mode_config={
                    "max_velocity": 0.1,
                    "increase_obstacle_margin": True,
                },
                estimated_recovery_time=60.0,
            )

        # Default system fault behavior
        return RecoveryBehavior(
            requires_manual_intervention=True,
            can_auto_recover=False,
            recovery_steps=[
                "Stop all operations",
                "Diagnose fault",
                "Apply fix",
                "Verify systems",
                "Manually clear safety",
            ],
            estimated_recovery_time=300.0,
        )

    def update_system_health(
        self,
        battery_level: Optional[float] = None,
        temperature: Optional[float] = None,
        communication_ok: Optional[bool] = None,
        sensors_ok: Optional[bool] = None,
    ) -> None:
        """
        Update system health parameters.

        Args:
            battery_level: Battery percentage (0-100)
            temperature: System temperature in Celsius
            communication_ok: Communication link status
            sensors_ok: Sensor system status
        """
        if battery_level is not None:
            self._battery_level = battery_level
            # Auto-trigger battery critical if below threshold
            if battery_level < 10.0:
                self.trigger_safety(
                    SafetyTriggerType.BATTERY_CRITICAL,
                    SafetySeverity.EMERGENCY,
                    f"Battery critically low: {battery_level}%",
                    "power_management",
                )

        if temperature is not None:
            self._temperature = temperature
            # Auto-trigger thermal warning if too hot
            if temperature > 70.0:
                self.trigger_safety(
                    SafetyTriggerType.THERMAL_WARNING,
                    SafetySeverity.CRITICAL,
                    f"System temperature high: {temperature}°C",
                    "thermal_management",
                )

        if communication_ok is not None:
            prev_status = self._communication_ok
            self._communication_ok = communication_ok
            # Trigger if communication lost
            if prev_status and not communication_ok:
                self.trigger_safety(
                    SafetyTriggerType.COMMUNICATION_LOSS,
                    SafetySeverity.EMERGENCY,
                    "Communication link lost",
                    "communication_system",
                )
            # Clear trigger if communication restored
            elif not prev_status and communication_ok:
                self.clear_trigger(SafetyTriggerType.COMMUNICATION_LOSS)

        if sensors_ok is not None:
            prev_status = self._sensors_ok
            self._sensors_ok = sensors_ok
            # Trigger if sensors fail
            if prev_status and not sensors_ok:
                self.trigger_safety(
                    SafetyTriggerType.SENSOR_FAILURE,
                    SafetySeverity.CRITICAL,
                    "Critical sensor failure detected",
                    "sensor_system",
                )
            # Clear trigger if sensors recover
            elif not prev_status and sensors_ok:
                self.clear_trigger(SafetyTriggerType.SENSOR_FAILURE)

    def get_safety_status(self) -> Dict:
        """
        Get comprehensive safety status.

        Returns:
            Dictionary with safety status information
        """
        return {
            "is_safe": len(self._active_triggers) == 0,
            "active_triggers": list(self._active_triggers.keys()),
            "highest_severity": (self.get_highest_severity() if self.get_highest_severity() else None),
            "battery_level": self._battery_level,
            "temperature": self._temperature,
            "communication_ok": self._communication_ok,
            "sensors_ok": self._sensors_ok,
        }
