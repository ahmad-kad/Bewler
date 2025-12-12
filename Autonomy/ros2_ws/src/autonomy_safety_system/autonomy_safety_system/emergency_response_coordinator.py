"""
Emergency response coordination across all rover subsystems.

Provides centralized safety event management with differentiated
handling for ESTOP (hard stop) and SAFESTOP (graceful pause).
"""

import asyncio
import json
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Set

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from autonomy_interfaces.msg import SafetyStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


class SafetyStopType(Enum):
    """Safety stop types with different behaviors."""

    ESTOP = "ESTOP"  # Hardware power cut via custom PCB (hard stop)
    SAFESTOP = "SAFESTOP"  # Software graceful pause (toggle-able)


class EmergencyResponsePhase(Enum):
    """Phases of emergency response."""

    DETECTION = "DETECTION"  # Emergency detected
    COORDINATION = "COORDINATION"  # Coordinating subsystem responses
    EXECUTION = "EXECUTION"  # Executing emergency actions
    VERIFICATION = "VERIFICATION"  # Verifying emergency actions completed
    STABILIZATION = "STABILIZATION"  # System stabilization
    RECOVERY_READY = "RECOVERY_READY"  # Ready for recovery procedures


@dataclass
class SubsystemEmergencyClient:
    """Emergency client for a specific subsystem."""

    name: str
    emergency_client: Optional[Client] = None
    status_client: Optional[Client] = None
    last_response_time: float = 0.0
    response_count: int = 0
    error_count: int = 0
    acknowledged_stop: bool = False
    recovery_time: float = 0.0


@dataclass
class SafetyEvent:
    """Safety event tracking for both ESTOP and SAFESTOP."""

    event_id: str
    trigger_time: float
    safety_type: SafetyStopType
    trigger_source: str
    description: str
    affected_subsystems: Set[str] = field(default_factory=set)
    response_status: Dict[str, str] = field(default_factory=dict)  # subsystem -> status
    phase: EmergencyResponsePhase = EmergencyResponsePhase.DETECTION
    recovery_available: bool = False
    manual_intervention_required: bool = False
    resolved_time: Optional[float] = None
    notes: List[str] = field(default_factory=list)
    can_toggle: bool = False  # True for SAFESTOP, False for ESTOP


class EmergencyResponseCoordinator(Node):
    """
    Coordinates safety responses across all rover subsystems.

    Manages both ESTOP (hard power disconnect) and SAFESTOP (graceful pause)
    with appropriate recovery procedures for each type.
    """

    def __init__(self) -> None:
        """Initialize emergency response coordinator."""
        super().__init__("emergency_response_coordinator")

        self._logger = self.get_logger()

        # Safety state tracking
        self.active_safety_event: Optional[SafetyEvent] = None
        self.safety_history: List[SafetyEvent] = []
        self.safety_stop_active = False

        # Subsystem emergency clients
        self.subsystem_clients: Dict[str, SubsystemEmergencyClient] = {}
        self._initialize_subsystem_clients()

        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10
        )

        # Thread pool for concurrent operations
        self.executor = ThreadPoolExecutor(max_workers=4)

        # Callback groups
        self.emergency_group = MutuallyExclusiveCallbackGroup()
        self.coordination_group = ReentrantCallbackGroup()
        self.monitoring_group = MutuallyExclusiveCallbackGroup()

        # Setup publishers and subscribers
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_timers()

        self.logger.info("Emergency Response Coordinator initialized")

    def _initialize_subsystem_clients(self):
        """Initialize emergency clients for all subsystems."""
        subsystems = [
            "navigation",
            "computer_vision",
            "slam",
            "manipulation",
            "state_machine",
            "led_status",
            "sensor_bridge",
        ]

        for subsystem in subsystems:
            self.subsystem_clients[subsystem] = SubsystemEmergencyClient(
                name=subsystem,
                emergency_client=self.create_client(Empty, f"/{subsystem}/emergency_stop"),
                status_client=self.create_client(Empty, f"/{subsystem}/status"),
            )

    def _setup_publishers(self):
        """Setup ROS2 publishers for emergency coordination."""
        self.emergency_status_pub = self.create_publisher(
            String, "/safety/emergency_status", 10, callback_group=self.emergency_group
        )

        self.emergency_coordination_pub = self.create_publisher(
            String, "/safety/emergency_coordination", 10, callback_group=self.coordination_group
        )

        self.recovery_coordination_pub = self.create_publisher(
            String, "/safety/recovery_coordination", 10, callback_group=self.monitoring_group
        )

        self.emergency_diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/safety/emergency_diagnostics", 10, callback_group=self.monitoring_group
        )

    def _setup_subscribers(self):
        """Setup ROS2 subscribers for emergency monitoring."""
        # Emergency stop signals from multiple sources
        self.create_subscription(
            Bool, "/safety/emergency_stop", self._emergency_stop_callback, 10, callback_group=self.emergency_group
        )

        self.create_subscription(
            Bool,
            "/safety/watchdog/emergency_stop",
            self._emergency_stop_callback,
            10,
            callback_group=self.emergency_group,
        )

        # Safety status monitoring
        self.create_subscription(
            SafetyStatus,
            "/state_machine/safety_status",
            self._safety_status_callback,
            10,
            callback_group=self.monitoring_group,
        )

        self.create_subscription(
            SafetyStatus,
            "/safety/redundant_status",
            self._safety_status_callback,
            10,
            callback_group=self.monitoring_group,
        )

    def _setup_timers(self):
        """Setup coordination and monitoring timers."""
        # Emergency status publishing timer
        self.status_timer = self.create_timer(1.0, self._publish_emergency_status, callback_group=self.monitoring_group)

        # Coordination monitoring timer
        self.coordination_timer = self.create_timer(
            2.0, self._monitor_emergency_coordination, callback_group=self.coordination_group
        )

        # Recovery readiness check timer
        self.recovery_timer = self.create_timer(
            5.0, self._check_recovery_readiness, callback_group=self.monitoring_group
        )

        # Diagnostics publishing timer
        self.diagnostics_timer = self.create_timer(
            10.0, self._publish_emergency_diagnostics, callback_group=self.monitoring_group
        )

    def _emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop signal."""
        if msg.data and not self.safety_stop_active:
            # Default to ESTOP for legacy emergency_stop_topic
            self._initiate_safety_response(
                SafetyStopType.ESTOP, "emergency_stop_topic", "Emergency stop signal received"
            )

    def _safety_status_callback(self, msg: SafetyStatus):
        """Monitor safety status for safety conditions."""
        if msg.safety_level in ["CRITICAL", "EMERGENCY"] and not self.safety_stop_active:
            # Determine safety type based on severity and trigger type
            if (
                msg.safety_level == "EMERGENCY"
                or "EMERGENCY_STOP" in str(msg.trigger_type)
                or "SOFTWARE_ESTOP" in str(msg.trigger_type)
            ):
                safety_type = SafetyStopType.ESTOP
            else:
                safety_type = SafetyStopType.SAFESTOP

            self._initiate_safety_response(
                safety_type, msg.trigger_source, f"Safety system triggered: {msg.trigger_description}"
            )

    def _initiate_safety_response(self, safety_type: SafetyStopType, source: str, description: str):
        """Initiate coordinated safety response."""
        if self.safety_stop_active:
            self.logger.warning("Safety response already active, ignoring new trigger")
            return

        self.safety_stop_active = True
        event_id = f"safety_{int(time.time())}_{safety_type.value}"

        self.active_safety_event = SafetyEvent(
            event_id=event_id,
            trigger_time=time.time(),
            safety_type=safety_type,
            trigger_source=source,
            description=description,
            affected_subsystems=set(self.subsystem_clients.keys()),
            phase=EmergencyResponsePhase.COORDINATION,
            can_toggle=(safety_type == SafetyStopType.SAFESTOP),
        )

        self.logger.error(f"SAFETY RESPONSE INITIATED: {safety_type.value} - {description}")

        # Start coordinated safety stop
        asyncio.create_task(self._coordinate_safety_stop())

    async def _coordinate_safety_stop(self):
        """Coordinate safety stop across all subsystems."""
        if not self.active_safety_event:
            return

        self.active_safety_event.phase = EmergencyResponsePhase.EXECUTION
        self.logger.info("Starting coordinated safety stop across all subsystems")

        # Prepare coordination message
        coordination_msg = String()
        coordination_data = {
            "event_id": self.active_safety_event.event_id,
            "phase": "EXECUTION",
            "safety_type": self.active_safety_event.safety_type.value,
            "trigger_time": self.active_safety_event.trigger_time,
            "affected_subsystems": list(self.active_safety_event.affected_subsystems),
            "coordination_start": time.time(),
            "can_toggle": self.active_safety_event.can_toggle,
        }

        # Send unified safety command to all subsystems
        # The actual stopping behavior is the same - set to safe values
        safety_tasks = []
        for subsystem_name, client in self.subsystem_clients.items():
            task = asyncio.create_task(self._send_unified_safety_command(subsystem_name, client))
            safety_tasks.append(task)

        # Wait for all subsystems to respond (with timeout)
        try:
            results = await asyncio.gather(*safety_tasks, return_exceptions=True)
            coordination_data["execution_results"] = [
                {"subsystem": name, "success": not isinstance(result, Exception)}
                for name, result in zip(self.subsystem_clients.keys(), results)
            ]
        except asyncio.TimeoutError:
            coordination_data["execution_results"] = "TIMEOUT"
            self.logger.error("Safety stop coordination timed out")

        # Update coordination message
        coordination_data["execution_complete"] = time.time()
        coordination_msg.data = json.dumps(coordination_data, indent=2)
        self.emergency_coordination_pub.publish(coordination_msg)

        # Move to verification phase
        self.active_safety_event.phase = EmergencyResponsePhase.VERIFICATION

    async def _send_unified_safety_command(self, subsystem_name: str, client: SubsystemEmergencyClient) -> bool:
        """Send unified safety command to set subsystem to safe values."""
        try:
            if not client.emergency_client.service_is_ready():
                self.logger.warning(f"Safety command service not ready for {subsystem_name}")
                client.error_count += 1
                return False

            # Send unified safety command (sets subsystem to known safe values)
            request = Empty.Request()  # TODO: Replace with dedicated safety service
            future = client.emergency_client.call_async(request)

            # Wait for acknowledgment with timeout
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 2.0:
                await asyncio.sleep(0.1)

            if future.done():
                try:
                    response = future.result()
                    client.last_response_time = time.time()
                    client.response_count += 1
                    client.acknowledged_stop = True

                    # Update safety event status
                    if self.active_safety_event:
                        self.active_safety_event.response_status[subsystem_name] = "SUCCESS"

                    self.logger.info(f"Safety command acknowledged by {subsystem_name}")
                    return True

                except Exception as e:
                    self.logger.error(f"Safety command failed for {subsystem_name}: {e}")
                    client.error_count += 1
                    if self.active_safety_event:
                        self.active_safety_event.response_status[subsystem_name] = f"ERROR: {str(e)}"
                    return False
            else:
                self.logger.error(f"Safety command timeout for {subsystem_name}")
                client.error_count += 1
                if self.active_safety_event:
                    self.active_safety_event.response_status[subsystem_name] = "TIMEOUT"
                return False

        except Exception as e:
            self.logger.error(f"Failed to send safety command to {subsystem_name}: {e}")
            client.error_count += 1
            if self.active_safety_event:
                self.active_safety_event.response_status[subsystem_name] = f"EXCEPTION: {str(e)}"
            return False

    def initiate_recovery(self, recovery_method: str = "AUTO", operator_id: str = "coordinator") -> bool:
        """Initiate recovery from safety state."""
        if not self.active_safety_event or not self.safety_stop_active:
            self.logger.warning("No active safety event to recover from")
            return False

        if self.active_safety_event.phase != EmergencyResponsePhase.VERIFICATION:
            self.logger.warning(f"Cannot recover during {self.active_safety_event.phase.value} phase")
            return False

        # SAFESTOP can be toggled, ESTOP requires full recovery
        if self.active_safety_event.safety_type == SafetyStopType.SAFESTOP:
            return self._toggle_safestop(operator_id)

        self.logger.info(f"Initiating recovery: {recovery_method} by {operator_id}")

        # Update safety event
        self.active_safety_event.phase = EmergencyResponsePhase.STABILIZATION
        self.active_safety_event.notes.append(f"Recovery initiated: {recovery_method} by {operator_id}")

        # For now, implement basic recovery
        # TODO: Implement full recovery coordination
        self._complete_safety_recovery()

        return True

    def _toggle_safestop(self, operator_id: str) -> bool:
        """Toggle SAFESTOP on/off."""
        if not self.active_safety_event or self.active_safety_event.safety_type != SafetyStopType.SAFESTOP:
            return False

        self.logger.info(f"SAFESTOP toggled by {operator_id}")

        # For SAFESTOP, simply clear the event and resume
        self.active_safety_event.phase = EmergencyResponsePhase.RECOVERY_READY
        self.active_safety_event.resolved_time = time.time()
        self.active_safety_event.notes.append(f"SAFESTOP toggled off by {operator_id}")

        # Move to history and clear active event
        self.safety_history.append(self.active_safety_event)
        self.active_safety_event = None
        self.safety_stop_active = False

        # Publish recovery coordination
        recovery_msg = String()
        recovery_data = {
            "recovery_complete": True,
            "timestamp": time.time(),
            "message": "SAFESTOP disengaged, system resuming normal operation",
            "safety_type": "SAFESTOP",
            "operator_id": operator_id,
        }
        recovery_msg.data = json.dumps(recovery_data, indent=2)
        self.recovery_coordination_pub.publish(recovery_msg)

        self.logger.info("SAFESTOP disengaged, system ready for normal operation")
        return True

    def _complete_safety_recovery(self):
        """Complete safety recovery process."""
        if not self.active_safety_event:
            return

        self.active_safety_event.phase = EmergencyResponsePhase.RECOVERY_READY
        self.active_safety_event.resolved_time = time.time()
        self.active_safety_event.recovery_available = True

        # Move safety event to history
        self.safety_history.append(self.active_safety_event)
        self.active_safety_event = None
        self.safety_stop_active = False

        # Publish recovery coordination
        recovery_msg = String()
        recovery_data = {
            "recovery_complete": True,
            "timestamp": time.time(),
            "message": "Safety recovery complete, system ready for normal operation",
        }
        recovery_msg.data = json.dumps(recovery_data, indent=2)
        self.recovery_coordination_pub.publish(recovery_msg)

        self.logger.info("Safety recovery completed, system ready for normal operation")

    def _monitor_emergency_coordination(self):
        """Monitor ongoing safety coordination."""
        if not self.active_safety_event:
            return

        # Check if all subsystems have responded
        total_subsystems = len(self.subsystem_clients)
        responded_subsystems = len(self.active_safety_event.response_status)
        successful_responses = sum(
            1 for status in self.active_safety_event.response_status.values() if status == "SUCCESS"
        )

        if responded_subsystems >= total_subsystems:
            if self.active_safety_event.phase == EmergencyResponsePhase.EXECUTION:
                self.active_safety_event.phase = EmergencyResponsePhase.VERIFICATION
                self.logger.info(f"Safety coordination complete: {successful_responses}/{total_subsystems} successful")

    def _check_recovery_readiness(self):
        """Check if system is ready for recovery."""
        if not self.active_safety_event or self.active_safety_event.phase != EmergencyResponsePhase.STABILIZATION:
            return

        # Check subsystem status
        # TODO: Implement subsystem health checks for recovery readiness
        stabilization_time = time.time() - (
            self.active_safety_event.resolved_time or self.active_safety_event.trigger_time
        )

        if stabilization_time > 10.0:  # 10 second stabilization period
            self._complete_safety_recovery()

    def _publish_emergency_status(self):
        """Publish current safety status."""
        status_data = {"safety_active": self.safety_stop_active, "timestamp": time.time()}

        if self.active_safety_event:
            status_data.update(
                {
                    "event_id": self.active_safety_event.event_id,
                    "safety_type": self.active_safety_event.safety_type.value,
                    "phase": self.active_safety_event.phase.value,
                    "trigger_source": self.active_safety_event.trigger_source,
                    "description": self.active_safety_event.description,
                    "affected_subsystems": list(self.active_safety_event.affected_subsystems),
                    "response_status": self.active_safety_event.response_status,
                    "trigger_age": time.time() - self.active_safety_event.trigger_time,
                    "can_toggle": self.active_safety_event.can_toggle,
                }
            )

        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.emergency_status_pub.publish(status_msg)

    def _publish_emergency_diagnostics(self):
        """Publish safety system diagnostics."""
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.header.frame_id = "emergency_response_coordinator"

        # Safety coordinator diagnostic
        safety_status = DiagnosticStatus()
        safety_status.name = "Safety Response Coordinator"
        safety_status.hardware_id = "safety_coordinator_v1.0"

        if not self.safety_stop_active:
            safety_status.level = DiagnosticStatus.OK
            safety_status.message = "Safety coordinator ready"
        else:
            safety_status.level = DiagnosticStatus.ERROR
            safety_status.message = f'Safety active: {self.active_safety_event.safety_type.value if self.active_safety_event else "unknown"}'

        # Add subsystem response information
        for subsystem_name, client in self.subsystem_clients.items():
            response_status = "UNKNOWN"
            if self.active_safety_event and subsystem_name in self.active_safety_event.response_status:
                response_status = self.active_safety_event.response_status[subsystem_name]

            safety_status.values.append(
                {"key": f"{subsystem_name}_response", "value": f"{response_status} (errors: {client.error_count})"}
            )

        diagnostics.status = [safety_status]
        self.emergency_diagnostics_pub.publish(diagnostics)

    def get_emergency_history(self, limit: int = 10) -> List[Dict[str, Any]]:
        """Get recent safety event history."""
        history = []
        for event in self.safety_history[-limit:]:
            history.append(
                {
                    "event_id": event.event_id,
                    "trigger_time": event.trigger_time,
                    "safety_type": event.safety_type.value,
                    "trigger_source": event.trigger_source,
                    "description": event.description,
                    "phase": event.phase.value,
                    "resolved_time": event.resolved_time,
                    "can_toggle": event.can_toggle,
                    "duration": (event.resolved_time - event.trigger_time) if event.resolved_time else None,
                }
            )
        return history

    def get_subsystem_status(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all emergency clients."""
        return {
            name: {
                "last_response_time": client.last_response_time,
                "response_count": client.response_count,
                "error_count": client.error_count,
                "acknowledged_stop": client.acknowledged_stop,
                "service_ready": client.emergency_client.service_is_ready() if client.emergency_client else False,
            }
            for name, client in self.subsystem_clients.items()
        }


def main(args=None):
    """Main entry point for emergency response coordinator node."""
    rclpy.init(args=args)

    # Create emergency response coordinator
    coordinator = EmergencyResponseCoordinator()

    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        coordinator.logger.info("Emergency Response Coordinator shutting down")
    finally:
        coordinator.executor.shutdown(wait=True)
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
