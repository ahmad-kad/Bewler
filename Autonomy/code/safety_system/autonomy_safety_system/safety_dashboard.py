"""
Safety Monitoring Dashboard.

Aggregates and visualizes safety data from all safety systems.
Provides real-time monitoring and alerting for safety violations.
"""

import json
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

import rclpy
from autonomy_interfaces.msg import SafetyStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Temperature
from std_msgs.msg import String


class SafetyAlertLevel(Enum):
    """Safety alert severity levels."""

    INFO = "INFO"  # Informational
    WARNING = "WARNING"  # Warning, monitor closely
    CRITICAL = "CRITICAL"  # Critical, immediate attention required
    EMERGENCY = "EMERGENCY"  # Emergency, immediate action required


class SafetySystemStatus(Enum):
    """Status of individual safety systems."""

    HEALTHY = "HEALTHY"  # System operating normally
    DEGRADED = "DEGRADED"  # System operating with reduced capability
    FAILED = "FAILED"  # System has failed
    UNKNOWN = "UNKNOWN"  # System status unknown


@dataclass
class SafetyAlert:
    """Safety alert information."""

    alert_id: str
    timestamp: float
    level: SafetyAlertLevel
    source: str
    message: str
    context: Dict[str, Any] = field(default_factory=dict)
    acknowledged: bool = False
    acknowledged_by: Optional[str] = None
    acknowledged_time: Optional[float] = None
    resolved: bool = False
    resolved_time: Optional[float] = None


@dataclass
class SystemHealthStatus:
    """Aggregated system health status."""

    system_name: str
    status: SafetySystemStatus
    last_update: float
    health_score: float  # 0.0 to 1.0
    active_issues: List[str] = field(default_factory=list)
    metrics: Dict[str, Any] = field(default_factory=dict)


class SafetyDashboard(Node):
    """
    Safety Monitoring Dashboard.

    Aggregates safety data from all safety systems and provides
    comprehensive monitoring, alerting, and visualization.
    """

    def __init__(self):
        super().__init__("safety_dashboard")

        self.logger = self.get_logger()

        # Dashboard state
        self.active_alerts: Dict[str, SafetyAlert] = {}
        self.system_health: Dict[str, SystemHealthStatus] = {}
        self.alert_history: deque = deque(maxlen=500)  # Keep last 500 alerts

        # Data buffers for trending
        self.safety_violations_buffer: deque = deque(maxlen=100)
        self.system_status_buffer: deque = deque(maxlen=50)

        # Safety thresholds
        self.alert_thresholds = {
            "max_active_alerts": 10,
            "health_score_threshold": 0.7,
            "communication_timeout": 30.0,  # seconds
            "sensor_timeout": 10.0,  # seconds
        }

        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # Callback groups
        self.monitoring_group = ReentrantCallbackGroup()
        self.alerting_group = MutuallyExclusiveCallbackGroup()
        self.aggregation_group = MutuallyExclusiveCallbackGroup()

        # Setup publishers and subscribers
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_timers()

        # Initialize system health tracking
        self._initialize_system_health_tracking()

        self.logger.info("Safety Dashboard initialized")

    def _initialize_system_health_tracking(self):
        """Initialize tracking for all safety-related systems."""
        safety_systems = [
            "state_machine",
            "safety_watchdog",
            "redundant_safety_monitor",
            "emergency_response_coordinator",
            "sensor_bridge",
            "navigation",
            "computer_vision",
            "slam",
            "battery_monitor",
            "temperature_monitor",
        ]

        for system in safety_systems:
            self.system_health[system] = SystemHealthStatus(
                system_name=system,
                status=SafetySystemStatus.UNKNOWN,
                last_update=0.0,
                health_score=0.5,  # Neutral starting score
            )

    def _setup_publishers(self):
        """Setup ROS2 publishers for dashboard data."""
        self.dashboard_status_pub = self.create_publisher(
            String, "/safety/dashboard_status", 10, callback_group=self.monitoring_group
        )

        self.active_alerts_pub = self.create_publisher(
            String, "/safety/active_alerts", 10, callback_group=self.alerting_group
        )

        self.system_health_pub = self.create_publisher(
            String, "/safety/system_health", 10, callback_group=self.aggregation_group
        )

        self.dashboard_diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            "/safety/dashboard_diagnostics",
            10,
            callback_group=self.monitoring_group,
        )

    def _setup_subscribers(self):
        """Setup ROS2 subscribers for safety data aggregation."""
        # Core safety systems
        self.create_subscription(
            SafetyStatus,
            "/state_machine/safety_status",
            self._safety_status_callback,
            10,
            callback_group=self.monitoring_group,
        )

        self.create_subscription(
            SafetyStatus,
            "/safety/violations",
            self._safety_violations_callback,
            10,
            callback_group=self.monitoring_group,
        )

        self.create_subscription(
            SafetyStatus,
            "/safety/redundant_status",
            self._redundant_safety_callback,
            10,
            callback_group=self.monitoring_group,
        )

        # System monitoring
        self.create_subscription(
            String,
            "/safety/watchdog_status",
            self._watchdog_status_callback,
            10,
            callback_group=self.aggregation_group,
        )

        self.create_subscription(
            String,
            "/safety/sensor_health",
            self._sensor_health_callback,
            10,
            callback_group=self.aggregation_group,
        )

        self.create_subscription(
            String,
            "/safety/emergency_status",
            self._emergency_status_callback,
            10,
            callback_group=self.monitoring_group,
        )

        # Diagnostics aggregation
        self.create_subscription(
            DiagnosticArray,
            "/safety/diagnostics",
            self._diagnostics_callback,
            10,
            callback_group=self.aggregation_group,
        )

        self.create_subscription(
            DiagnosticArray,
            "/safety/redundant_diagnostics",
            self._diagnostics_callback,
            10,
            callback_group=self.aggregation_group,
        )

        # Sensor data for health monitoring
        self.create_subscription(
            BatteryState,
            "/battery/status",
            self._battery_status_callback,
            10,
            qos_profile=self.qos_reliable,
            callback_group=self.aggregation_group,
        )

        self.create_subscription(
            Temperature,
            "/temperature/data",
            self._temperature_status_callback,
            10,
            qos_profile=self.qos_reliable,
            callback_group=self.aggregation_group,
        )

    def _setup_timers(self):
        """Setup dashboard monitoring and publishing timers."""
        # Main dashboard status publishing
        self.dashboard_timer = self.create_timer(
            1.0, self._publish_dashboard_status, callback_group=self.monitoring_group
        )

        # Active alerts publishing
        self.alerts_timer = self.create_timer(2.0, self._publish_active_alerts, callback_group=self.alerting_group)

        # System health aggregation
        self.health_timer = self.create_timer(5.0, self._publish_system_health, callback_group=self.aggregation_group)

        # Alert evaluation and cleanup
        self.alert_evaluation_timer = self.create_timer(10.0, self._evaluate_alerts, callback_group=self.alerting_group)

        # Diagnostics publishing
        self.diagnostics_timer = self.create_timer(
            15.0,
            self._publish_dashboard_diagnostics,
            callback_group=self.monitoring_group,
        )

    def _safety_status_callback(self, msg: SafetyStatus):
        """Handle primary safety system status."""
        self._update_system_health("state_machine", SafetySystemStatus.HEALTHY, time.time())

        # Check for safety violations
        if not msg.is_safe or msg.safety_level in ["WARNING", "CRITICAL", "EMERGENCY"]:
            alert_level = self._map_safety_level_to_alert(msg.safety_level)

            self._create_alert(
                alert_id=f"safety_{msg.trigger_type}_{int(time.time())}",
                level=alert_level,
                source="state_machine",
                message=f"Safety violation: {msg.trigger_description}",
                context={
                    "safety_level": msg.safety_level,
                    "trigger_type": msg.trigger_type,
                    "trigger_source": msg.trigger_source,
                    "active_triggers": msg.active_triggers,
                },
            )

        self.safety_violations_buffer.append(
            {
                "timestamp": time.time(),
                "is_safe": msg.is_safe,
                "safety_level": msg.safety_level,
                "active_triggers": len(msg.active_triggers),
            }
        )

    def _safety_violations_callback(self, msg: SafetyStatus):
        """Handle watchdog safety violations."""
        self._update_system_health("safety_watchdog", SafetySystemStatus.HEALTHY, time.time())

        if not msg.is_safe:
            alert_level = SafetyAlertLevel.CRITICAL

            self._create_alert(
                alert_id=f"watchdog_{msg.trigger_type}_{int(time.time())}",
                level=alert_level,
                source="safety_watchdog",
                message=f"Watchdog violation: {msg.trigger_description}",
                context={
                    "trigger_type": msg.trigger_type,
                    "trigger_source": msg.trigger_source,
                },
            )

    def _redundant_safety_callback(self, msg: SafetyStatus):
        """Handle redundant safety system status."""
        self._update_system_health("redundant_safety_monitor", SafetySystemStatus.HEALTHY, time.time())

        if not msg.is_safe:
            alert_level = SafetyAlertLevel.WARNING  # Redundant system warnings

            self._create_alert(
                alert_id=f"redundant_{msg.trigger_type}_{int(time.time())}",
                level=alert_level,
                source="redundant_safety_monitor",
                message=f"Redundant safety check: {msg.trigger_description}",
                context={
                    "trigger_type": msg.trigger_type,
                    "trigger_source": msg.trigger_source,
                },
            )

    def _watchdog_status_callback(self, msg: String):
        """Handle watchdog status updates."""
        try:
            status_data = json.loads(msg.data)

            # Update system health based on watchdog data
            active_violations = status_data.get("active_violations_count", 0)
            health_score = max(0.0, 1.0 - (active_violations * 0.1))

            status = SafetySystemStatus.DEGRADED if active_violations > 0 else SafetySystemStatus.HEALTHY

            self._update_system_health("safety_watchdog", status, time.time(), health_score)

        except json.JSONDecodeError:
            self._update_system_health("safety_watchdog", SafetySystemStatus.FAILED, time.time(), 0.0)

    def _sensor_health_callback(self, msg: String):
        """Handle sensor health updates."""
        try:
            health_data = json.loads(msg.data)

            # Update sensor system health
            sensors = health_data.get("sensors", {})
            healthy_sensors = sum(1 for s in sensors.values() if s.get("healthy", False))
            total_sensors = len(sensors)

            if total_sensors > 0:
                health_score = healthy_sensors / total_sensors
                status = (
                    SafetySystemStatus.HEALTHY
                    if health_score >= 0.8
                    else (SafetySystemStatus.DEGRADED if health_score >= 0.5 else SafetySystemStatus.FAILED)
                )

                self._update_system_health("sensor_health", status, time.time(), health_score)

        except json.JSONDecodeError:
            pass

    def _emergency_status_callback(self, msg: String):
        """Handle emergency status updates."""
        try:
            emergency_data = json.loads(msg.data)

            if emergency_data.get("emergency_active", False):
                self._create_alert(
                    alert_id=f"emergency_active_{int(time.time())}",
                    level=SafetyAlertLevel.EMERGENCY,
                    source="emergency_response_coordinator",
                    message="Emergency stop active",
                    context=emergency_data,
                )

                self._update_system_health(
                    "emergency_response_coordinator",
                    SafetySystemStatus.DEGRADED,
                    time.time(),
                    0.3,
                )
            else:
                self._update_system_health(
                    "emergency_response_coordinator",
                    SafetySystemStatus.HEALTHY,
                    time.time(),
                    1.0,
                )

        except json.JSONDecodeError:
            pass

    def _diagnostics_callback(self, msg: DiagnosticArray):
        """Aggregate diagnostic information."""
        for status in msg.status:
            system_name = status.name.lower().replace(" ", "_")

            # Map diagnostic level to system status
            if status.level == DiagnosticStatus.OK:
                system_status = SafetySystemStatus.HEALTHY
                health_score = 0.9
            elif status.level == DiagnosticStatus.WARN:
                system_status = SafetySystemStatus.DEGRADED
                health_score = 0.6
            else:  # ERROR
                system_status = SafetySystemStatus.FAILED
                health_score = 0.1

            self._update_system_health(system_name, system_status, time.time(), health_score)

    def _battery_status_callback(self, msg: BatteryState):
        """Monitor battery status for dashboard."""
        self._update_system_health("battery_monitor", SafetySystemStatus.HEALTHY, time.time())

        # Check battery health
        if msg.percentage <= 20.0:
            health_score = msg.percentage / 100.0
            status = SafetySystemStatus.DEGRADED if msg.percentage > 10.0 else SafetySystemStatus.FAILED
            self._update_system_health("battery_monitor", status, time.time(), health_score)

    def _temperature_status_callback(self, msg: Temperature):
        """Monitor temperature status for dashboard."""
        self._update_system_health("temperature_monitor", SafetySystemStatus.HEALTHY, time.time())

        # Check temperature health
        if msg.temperature >= 70.0:
            health_score = max(0.1, 1.0 - ((msg.temperature - 70.0) / 30.0))
            status = SafetySystemStatus.DEGRADED if msg.temperature < 85.0 else SafetySystemStatus.FAILED
            self._update_system_health("temperature_monitor", status, time.time(), health_score)

    def _map_safety_level_to_alert(self, safety_level: str) -> SafetyAlertLevel:
        """Map safety status level to alert level."""
        mapping = {
            "NORMAL": SafetyAlertLevel.INFO,
            "WARNING": SafetyAlertLevel.WARNING,
            "CRITICAL": SafetyAlertLevel.CRITICAL,
            "EMERGENCY": SafetyAlertLevel.EMERGENCY,
        }
        return mapping.get(safety_level, SafetyAlertLevel.WARNING)

    def _create_alert(
        self,
        alert_id: str,
        level: SafetyAlertLevel,
        source: str,
        message: str,
        context: Dict[str, Any] = None,
    ):
        """Create a new safety alert."""
        if alert_id in self.active_alerts:
            # Update existing alert
            self.active_alerts[alert_id].context.update(context or {})
            return

        alert = SafetyAlert(
            alert_id=alert_id,
            timestamp=time.time(),
            level=level,
            source=source,
            message=message,
            context=context or {},
        )

        self.active_alerts[alert_id] = alert
        self.alert_history.append(alert)

        self.logger.warning(f"ALERT CREATED: [{level.value}] {source}: {message}")

    def _update_system_health(
        self,
        system_name: str,
        status: SafetySystemStatus,
        update_time: float,
        health_score: float = None,
        active_issues: List[str] = None,
        metrics: Dict[str, Any] = None,
    ):
        """Update system health information."""
        if system_name not in self.system_health:
            self.system_health[system_name] = SystemHealthStatus(
                system_name=system_name,
                status=status,
                last_update=update_time,
                health_score=health_score or 0.5,
            )
        else:
            health_info = self.system_health[system_name]
            health_info.status = status
            health_info.last_update = update_time
            if health_score is not None:
                health_info.health_score = health_score
            if active_issues is not None:
                health_info.active_issues = active_issues
            if metrics is not None:
                health_info.metrics.update(metrics)

    def _evaluate_alerts(self):
        """Evaluate active alerts and check for escalation or resolution."""
        current_time = time.time()

        alerts_to_remove = []

        for alert_id, alert in self.active_alerts.items():
            # Check for alert timeout (auto-resolve after 5 minutes)
            if current_time - alert.timestamp > 300.0 and not alert.resolved:
                alert.resolved = True
                alert.resolved_time = current_time
                self.logger.info(f"Alert auto-resolved: {alert_id}")

            # Check for escalation based on alert patterns
            if not alert.resolved and self._should_escalate_alert(alert):
                alert.level = SafetyAlertLevel.EMERGENCY
                self.logger.error(f"ALERT ESCALATED TO EMERGENCY: {alert_id}")

            # Remove resolved alerts after 30 seconds
            if alert.resolved and alert.resolved_time and current_time - alert.resolved_time > 30.0:
                alerts_to_remove.append(alert_id)

        # Remove old resolved alerts
        for alert_id in alerts_to_remove:
            del self.active_alerts[alert_id]

    def _should_escalate_alert(self, alert: SafetyAlert) -> bool:
        """Determine if an alert should be escalated."""
        # Escalate if multiple similar alerts exist
        similar_alerts = sum(
            1
            for a in self.active_alerts.values()
            if a.source == alert.source and a.level == alert.level and not a.resolved
        )

        return similar_alerts >= 3  # Escalate if 3+ similar active alerts

    def _publish_dashboard_status(self):
        """Publish comprehensive dashboard status."""
        current_time = time.time()

        # Calculate overall system health
        active_alerts_count = len(self.active_alerts)
        critical_alerts = sum(
            1 for a in self.active_alerts.values() if a.level in [SafetyAlertLevel.CRITICAL, SafetyAlertLevel.EMERGENCY]
        )

        # System health scores
        system_scores = [h.health_score for h in self.system_health.values()]
        average_health = sum(system_scores) / len(system_scores) if system_scores else 0.5

        # Overall status determination
        if critical_alerts > 0 or average_health < 0.3:
            overall_status = "CRITICAL"
        elif active_alerts_count > 5 or average_health < 0.7:
            overall_status = "WARNING"
        else:
            overall_status = "NORMAL"

        dashboard_data = {
            "timestamp": current_time,
            "overall_status": overall_status,
            "active_alerts_count": active_alerts_count,
            "critical_alerts_count": critical_alerts,
            "average_system_health": average_health,
            "systems_monitored": len(self.system_health),
            "systems_healthy": sum(1 for h in self.system_health.values() if h.status == SafetySystemStatus.HEALTHY),
            "systems_degraded": sum(1 for h in self.system_health.values() if h.status == SafetySystemStatus.DEGRADED),
            "systems_failed": sum(1 for h in self.system_health.values() if h.status == SafetySystemStatus.FAILED),
        }

        dashboard_msg = String()
        dashboard_msg.data = json.dumps(dashboard_data, indent=2)
        self.dashboard_status_pub.publish(dashboard_msg)

    def _publish_active_alerts(self):
        """Publish currently active alerts."""
        alerts_data = {
            "timestamp": time.time(),
            "active_alerts": [
                {
                    "alert_id": alert.alert_id,
                    "level": alert.level.value,
                    "source": alert.source,
                    "message": alert.message,
                    "timestamp": alert.timestamp,
                    "age_seconds": time.time() - alert.timestamp,
                    "acknowledged": alert.acknowledged,
                    "context": alert.context,
                }
                for alert in self.active_alerts.values()
            ],
        }

        alerts_msg = String()
        alerts_msg.data = json.dumps(alerts_data, indent=2)
        self.active_alerts_pub.publish(alerts_msg)

    def _publish_system_health(self):
        """Publish system health status."""
        health_data = {
            "timestamp": time.time(),
            "systems": {
                name: {
                    "status": health.status.value,
                    "last_update": health.last_update,
                    "age_seconds": time.time() - health.last_update,
                    "health_score": health.health_score,
                    "active_issues": health.active_issues,
                    "metrics": health.metrics,
                }
                for name, health in self.system_health.items()
            },
        }

        health_msg = String()
        health_msg.data = json.dumps(health_data, indent=2)
        self.system_health_pub.publish(health_msg)

    def _publish_dashboard_diagnostics(self):
        """Publish dashboard diagnostics."""
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.header.frame_id = "safety_dashboard"

        # Dashboard diagnostic
        dashboard_status = DiagnosticStatus()
        dashboard_status.name = "Safety Dashboard"
        dashboard_status.hardware_id = "safety_dashboard_v1.0"

        active_alerts = len(self.active_alerts)
        if active_alerts == 0:
            dashboard_status.level = DiagnosticStatus.OK
            dashboard_status.message = "Safety dashboard monitoring normal"
        elif active_alerts <= 5:
            dashboard_status.level = DiagnosticStatus.WARN
            dashboard_status.message = f"Safety dashboard: {active_alerts} active alerts"
        else:
            dashboard_status.level = DiagnosticStatus.ERROR
            dashboard_status.message = f"Safety dashboard: {active_alerts} active alerts (high)"

        # Add system health information
        for system_name, health in self.system_health.items():
            dashboard_status.values.append(
                {
                    "key": f"{system_name}_health",
                    "value": f"{health.status.value} ({health.health_score:.2f})",
                }
            )

        diagnostics.status = [dashboard_status]
        self.dashboard_diagnostics_pub.publish(diagnostics)

    def acknowledge_alert(self, alert_id: str, operator_id: str = "dashboard") -> bool:
        """Acknowledge a safety alert."""
        if alert_id in self.active_alerts:
            self.active_alerts[alert_id].acknowledged = True
            self.active_alerts[alert_id].acknowledged_by = operator_id
            self.active_alerts[alert_id].acknowledged_time = time.time()
            self.logger.info(f"Alert acknowledged: {alert_id} by {operator_id}")
            return True
        return False

    def resolve_alert(self, alert_id: str, operator_id: str = "dashboard") -> bool:
        """Resolve a safety alert."""
        if alert_id in self.active_alerts:
            self.active_alerts[alert_id].resolved = True
            self.active_alerts[alert_id].resolved_time = time.time()
            self.logger.info(f"Alert resolved: {alert_id} by {operator_id}")
            return True
        return False

    def get_alert_history(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Get recent alert history."""
        history = []
        for alert in list(self.alert_history)[-limit:]:
            history.append(
                {
                    "alert_id": alert.alert_id,
                    "timestamp": alert.timestamp,
                    "level": alert.level.value,
                    "source": alert.source,
                    "message": alert.message,
                    "acknowledged": alert.acknowledged,
                    "resolved": alert.resolved,
                    "context": alert.context,
                }
            )
        return history


def main(args=None):
    """Main entry point for safety dashboard node."""
    rclpy.init(args=args)

    # Create safety dashboard
    dashboard = SafetyDashboard()

    try:
        rclpy.spin(dashboard)
    except KeyboardInterrupt:
        dashboard.logger.info("Safety Dashboard shutting down")
    finally:
        dashboard.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
