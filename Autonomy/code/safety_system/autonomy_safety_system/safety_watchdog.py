"""
Independent safety monitoring system.

Provides multi-level watchdog monitoring with automatic safety violation
detection and emergency response coordination.
"""

import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

import rclpy
from autonomy_interfaces.msg import SafetyStatus
from autonomy_interfaces.msg import SystemState as SystemStateMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String


class WatchdogLevel(Enum):
    """Watchdog monitoring levels."""

    HEARTBEAT = "HEARTBEAT"  # Basic system heartbeat monitoring
    STATE_TRANSITIONS = "STATE_TRANSITIONS"  # State machine transition monitoring
    SUBSYSTEM_HEALTH = "SUBSYSTEM_HEALTH"  # Individual subsystem health monitoring
    SENSOR_INTEGRITY = "SENSOR_INTEGRITY"  # Sensor data integrity monitoring
    SAFETY_VIOLATIONS = "SAFETY_VIOLATIONS"  # Active safety violation monitoring


class SafetySeverity(Enum):
    """Safety violation severity levels."""

    MONITOR = "MONITOR"  # Monitor but allow operation
    WARNING = "WARNING"  # Warning, prepare for intervention
    CRITICAL = "CRITICAL"  # Critical, immediate intervention required
    EMERGENCY = "EMERGENCY"  # Emergency, immediate halt required


@dataclass
class WatchdogConfiguration:
    """Configuration for watchdog monitoring."""

    heartbeat_timeout: float = 5.0  # seconds
    state_transition_timeout: float = 30.0  # seconds
    subsystem_health_timeout: float = 10.0  # seconds
    sensor_integrity_timeout: float = 2.0  # seconds
    battery_critical_threshold: float = 10.0  # percentage
    temperature_warning_threshold: float = 70.0  # celsius
    temperature_critical_threshold: float = 85.0  # celsius
    enable_emergency_stop: bool = True
    enable_automatic_recovery: bool = False
    watchdog_levels: List[WatchdogLevel] = field(
        default_factory=lambda: [
            WatchdogLevel.HEARTBEAT,
            WatchdogLevel.STATE_TRANSITIONS,
            WatchdogLevel.SUBSYSTEM_HEALTH,
        ]
    )


class SafetyWatchdog(Node):
    """
    Independent safety monitoring with automatic violation detection.

    Monitors system health across multiple levels and coordinates
    emergency responses when safety thresholds are exceeded.
    """

    def __init__(self, config: Optional[WatchdogConfiguration] = None) -> None:
        """Initialize watchdog with monitoring configuration."""
        super().__init__("safety_watchdog")

        self.config = config or WatchdogConfiguration()
        self._logger = self.get_logger()

        # Watchdog state tracking
        self.last_heartbeat_time = time.time()
        self.last_state_change_time = time.time()
        self.last_subsystem_update_time = time.time()
        self.last_sensor_update_time = time.time()
        self.current_system_state = None
        self.active_violations: Dict[str, Dict[str, Any]] = {}
        self.violation_history: List[Dict[str, Any]] = []

        # QoS profiles for reliability
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        self.qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=20,
        )

        # Callback groups for thread safety
        self.monitoring_group = MutuallyExclusiveCallbackGroup()
        self.emergency_group = MutuallyExclusiveCallbackGroup()

        # Setup publishers
        self._setup_publishers()

        # Setup subscribers
        self._setup_subscribers()

        # Setup timers
        self._setup_timers()

        self.logger.info(
            "Safety Watchdog initialized with monitoring levels: "
            f"{[level.value for level in self.config.watchdog_levels]}"
        )

    def _setup_publishers(self):
        """Setup ROS2 publishers for safety monitoring."""
        self.emergency_stop_pub = self.create_publisher(
            Bool, "/safety/emergency_stop", 10, callback_group=self.emergency_group
        )

        self.safety_violations_pub = self.create_publisher(
            SafetyStatus, "/safety/violations", 10, callback_group=self.monitoring_group
        )

        self.watchdog_status_pub = self.create_publisher(
            String, "/safety/watchdog_status", 1, callback_group=self.monitoring_group
        )

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            "/safety/diagnostics",
            10,
            callback_group=self.monitoring_group,
        )

    def _setup_subscribers(self):
        """Setup ROS2 subscribers for system monitoring."""
        # Heartbeat monitoring
        if WatchdogLevel.HEARTBEAT in self.config.watchdog_levels:
            self.create_subscription(
                String,
                "/state_machine/heartbeat",
                self._heartbeat_callback,
                1,
                callback_group=self.monitoring_group,
            )

        # State transition monitoring
        if WatchdogLevel.STATE_TRANSITIONS in self.config.watchdog_levels:
            self.create_subscription(
                SystemStateMsg,
                "/state_machine/current_state",
                self._state_callback,
                1,
                callback_group=self.monitoring_group,
            )

        # Subsystem health monitoring
        if WatchdogLevel.SUBSYSTEM_HEALTH in self.config.watchdog_levels:
            self.create_subscription(
                String,
                "/state_machine/subsystem_status",
                self._subsystem_callback,
                10,
                callback_group=self.monitoring_group,
            )

        # Sensor integrity monitoring
        if WatchdogLevel.SENSOR_INTEGRITY in self.config.watchdog_levels:
            self.create_subscription(
                BatteryState,
                "/battery/status",
                self._battery_callback,
                10,
                qos_profile=self.qos_best_effort,
                callback_group=self.monitoring_group,
            )

    def _setup_timers(self):
        """Setup watchdog monitoring timers."""
        # Heartbeat timeout check
        if WatchdogLevel.HEARTBEAT in self.config.watchdog_levels:
            self.heartbeat_timer = self.create_timer(
                self.config.heartbeat_timeout,
                self._check_heartbeat_timeout,
                callback_group=self.monitoring_group,
            )

        # State transition timeout check
        if WatchdogLevel.STATE_TRANSITIONS in self.config.watchdog_levels:
            self.state_timer = self.create_timer(
                self.config.state_transition_timeout,
                self._check_state_timeout,
                callback_group=self.monitoring_group,
            )

        # Subsystem health check
        if WatchdogLevel.SUBSYSTEM_HEALTH in self.config.watchdog_levels:
            self.subsystem_timer = self.create_timer(
                self.config.subsystem_health_timeout,
                self._check_subsystem_health,
                callback_group=self.monitoring_group,
            )

        # Sensor integrity check
        if WatchdogLevel.SENSOR_INTEGRITY in self.config.watchdog_levels:
            self.sensor_timer = self.create_timer(
                self.config.sensor_integrity_timeout,
                self._check_sensor_integrity,
                callback_group=self.monitoring_group,
            )

        # Status publishing timer
        self.status_timer = self.create_timer(1.0, self._publish_watchdog_status, callback_group=self.monitoring_group)

        # Diagnostics publishing timer
        self.diagnostics_timer = self.create_timer(5.0, self._publish_diagnostics, callback_group=self.monitoring_group)

    def _heartbeat_callback(self, msg: String):
        """Handle heartbeat messages from state machine."""
        try:
            heartbeat_data = json.loads(msg.data)
            self.last_heartbeat_time = time.time()

            # Clear any heartbeat violations
            if "heartbeat_timeout" in self.active_violations:
                del self.active_violations["heartbeat_timeout"]
                self.logger.info("Heartbeat restored")

        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid heartbeat message: {e}")

    def _state_callback(self, msg: SystemStateMsg):
        """Handle state change messages."""
        self.last_state_change_time = time.time()
        self.current_system_state = msg.current_state

        # Clear any state transition violations
        if "state_transition_timeout" in self.active_violations:
            del self.active_violations["state_transition_timeout"]

        self.logger.debug(f"State changed to: {msg.current_state}")

    def _subsystem_callback(self, msg: String):
        """Handle subsystem status updates."""
        try:
            subsystem_data = json.loads(msg.data)
            self.last_subsystem_update_time = time.time()

            # Check for subsystem failures
            failed_subsystems = [name for name, status in subsystem_data.items() if not status.get("healthy", True)]

            if failed_subsystems:
                self._handle_subsystem_failure(failed_subsystems, subsystem_data)

        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid subsystem status message: {e}")

    def _battery_callback(self, msg: BatteryState):
        """Handle battery status for safety monitoring."""
        self.last_sensor_update_time = time.time()

        # Check battery safety thresholds
        if msg.percentage <= self.config.battery_critical_threshold:
            self._trigger_safety_violation(
                "battery_critical",
                SafetySeverity.EMERGENCY,
                f"Battery critically low: {msg.percentage:.1f}%",
                {"battery_level": msg.percentage},
            )
        elif msg.percentage <= 20.0:
            self._trigger_safety_violation(
                "battery_low",
                SafetySeverity.WARNING,
                f"Battery low: {msg.percentage:.1f}%",
                {"battery_level": msg.percentage},
            )

    def _check_heartbeat_timeout(self):
        """Check for heartbeat timeout violations."""
        time_since_heartbeat = time.time() - self.last_heartbeat_time

        if time_since_heartbeat > self.config.heartbeat_timeout:
            self._trigger_safety_violation(
                "heartbeat_timeout",
                SafetySeverity.CRITICAL,
                f"Heartbeat timeout: {time_since_heartbeat:.1f}s since last heartbeat",
                {"time_since_heartbeat": time_since_heartbeat},
            )

    def _check_state_timeout(self):
        """Check for state transition timeout violations."""
        time_since_state_change = time.time() - self.last_state_change_time

        if time_since_state_change > self.config.state_transition_timeout:
            self._trigger_safety_violation(
                "state_transition_timeout",
                SafetySeverity.WARNING,
                f"No state change for {time_since_state_change:.1f}s",
                {"time_since_state_change": time_since_state_change},
            )

    def _check_subsystem_health(self):
        """Check subsystem health status."""
        time_since_subsystem_update = time.time() - self.last_subsystem_update_time

        if time_since_subsystem_update > self.config.subsystem_health_timeout:
            self._trigger_safety_violation(
                "subsystem_health_timeout",
                SafetySeverity.WARNING,
                f"No subsystem health update for {time_since_subsystem_update:.1f}s",
                {"time_since_update": time_since_subsystem_update},
            )

    def _check_sensor_integrity(self):
        """Check sensor data integrity."""
        time_since_sensor_update = time.time() - self.last_sensor_update_time

        if time_since_sensor_update > self.config.sensor_integrity_timeout:
            self._trigger_safety_violation(
                "sensor_integrity_timeout",
                SafetySeverity.WARNING,
                f"No sensor data for {time_since_sensor_update:.1f}s",
                {"time_since_sensor_update": time_since_sensor_update},
            )

    def _handle_subsystem_failure(self, failed_subsystems: List[str], subsystem_data: Dict):
        """Handle subsystem failure detection."""
        for subsystem in failed_subsystems:
            failure_info = subsystem_data[subsystem]
            severity = SafetySeverity.CRITICAL if failure_info.get("critical", False) else SafetySeverity.WARNING

            self._trigger_safety_violation(
                f"subsystem_failure_{subsystem}",
                severity,
                f'Subsystem {subsystem} failed: {failure_info.get("error", "Unknown error")}',
                {"subsystem": subsystem, "failure_info": failure_info},
            )

    def _trigger_safety_violation(
        self,
        violation_id: str,
        severity: SafetySeverity,
        description: str,
        context: Dict[str, Any],
    ):
        """Trigger a safety violation and take appropriate action."""
        if violation_id in self.active_violations:
            # Update existing violation
            self.active_violations[violation_id].update(
                {
                    "last_triggered": time.time(),
                    "trigger_count": self.active_violations[violation_id]["trigger_count"] + 1,
                    "context": context,
                }
            )
        else:
            # New violation
            self.active_violations[violation_id] = {
                "id": violation_id,
                "severity": severity,
                "description": description,
                "first_triggered": time.time(),
                "last_triggered": time.time(),
                "trigger_count": 1,
                "context": context,
            }

            # Log violation
            self.logger.warning(f"Safety violation triggered: {violation_id} - {description}")

            # Record in history
            self.violation_history.append(
                {
                    "timestamp": time.time(),
                    "violation_id": violation_id,
                    "severity": severity.value,
                    "description": description,
                    "context": context,
                }
            )

            # Keep only recent history (last 100 violations)
            if len(self.violation_history) > 100:
                self.violation_history = self.violation_history[-100:]

        # Take action based on severity - unified safety stop
        if severity in [SafetySeverity.CRITICAL, SafetySeverity.EMERGENCY]:
            if self.config.enable_emergency_stop:
                self._execute_safety_stop(violation_id, severity, description)

    def _execute_safety_stop(self, violation_id: str, severity: SafetySeverity, description: str):
        """Execute unified safety stop procedure (sets all systems to safe values)."""
        self.logger.error(f"SAFETY STOP triggered by {violation_id}: {description}")

        # Determine safety type based on violation severity
        is_emergency = severity == SafetySeverity.EMERGENCY
        safety_level = "EMERGENCY" if is_emergency else "CRITICAL"

        # Publish safety stop signal
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)

        # Publish unified safety status
        safety_msg = SafetyStatus()
        safety_msg.header.stamp = self.get_clock().now().to_msg()
        safety_msg.header.frame_id = "safety_watchdog"
        safety_msg.is_safe = False
        safety_msg.safety_level = safety_level
        safety_msg.active_triggers = [violation_id]
        safety_msg.trigger_type = violation_id
        safety_msg.trigger_source = "safety_watchdog"
        safety_msg.trigger_time = self.get_clock().now().to_msg()
        safety_msg.trigger_description = description
        safety_msg.requires_manual_intervention = is_emergency
        safety_msg.can_auto_recover = not is_emergency
        safety_msg.recovery_steps = [
            "Investigate safety violation",
            "Clear safety stop",
            "Verify system health",
        ]
        safety_msg.estimated_recovery_time = 300.0 if is_emergency else 60.0
        safety_msg.context_state = self.current_system_state or "unknown"
        safety_msg.mission_phase = "unknown"
        safety_msg.safe_to_retry = not is_emergency

        self.safety_violations_pub.publish(safety_msg)

    def _publish_watchdog_status(self):
        """Publish current watchdog status."""
        status_data = {
            "timestamp": time.time(),
            "active_violations_count": len(self.active_violations),
            "active_violations": list(self.active_violations.keys()),
            "current_system_state": self.current_system_state,
            "last_heartbeat_age": time.time() - self.last_heartbeat_time,
            "last_state_change_age": time.time() - self.last_state_change_time,
            "last_subsystem_update_age": time.time() - self.last_subsystem_update_time,
            "last_sensor_update_age": time.time() - self.last_sensor_update_time,
            "total_violations_in_history": len(self.violation_history),
        }

        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.watchdog_status_pub.publish(status_msg)

    def _publish_diagnostics(self):
        """Publish diagnostic information."""
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.header.frame_id = "safety_watchdog"

        # Watchdog health diagnostic
        watchdog_status = DiagnosticStatus()
        watchdog_status.name = "Safety Watchdog"
        watchdog_status.hardware_id = "safety_watchdog_v1.0"

        if len(self.active_violations) == 0:
            watchdog_status.level = DiagnosticStatus.OK
            watchdog_status.message = "All systems monitored and healthy"
        elif any(
            v["severity"] in [SafetySeverity.CRITICAL, SafetySeverity.EMERGENCY]
            for v in self.active_violations.values()
        ):
            watchdog_status.level = DiagnosticStatus.ERROR
            watchdog_status.message = f"Critical safety violations: {len(self.active_violations)}"
        else:
            watchdog_status.level = DiagnosticStatus.WARN
            watchdog_status.message = f"Warning safety violations: {len(self.active_violations)}"

        # Add violation details
        for violation_id, violation_data in self.active_violations.items():
            watchdog_status.values.append(
                {
                    "key": f"violation_{violation_id}",
                    "value": violation_data["description"],
                }
            )

        diagnostics.status = [watchdog_status]
        self.diagnostics_pub.publish(diagnostics)

    def get_violation_history(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Get recent violation history."""
        return self.violation_history[-limit:]

    def clear_violation(self, violation_id: str) -> bool:
        """Clear a specific safety violation."""
        if violation_id in self.active_violations:
            del self.active_violations[violation_id]
            self.logger.info(f"Cleared safety violation: {violation_id}")
            return True
        return False

    def get_active_violations(self) -> Dict[str, Dict[str, Any]]:
        """Get all currently active violations."""
        return self.active_violations.copy()


def main(args=None):
    """Main entry point for safety watchdog node."""
    rclpy.init(args=args)

    # Create watchdog with default configuration
    watchdog = SafetyWatchdog()

    try:
        rclpy.spin(watchdog)
    except KeyboardInterrupt:
        watchdog.logger.info("Safety Watchdog shutting down")
    finally:
        watchdog.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
