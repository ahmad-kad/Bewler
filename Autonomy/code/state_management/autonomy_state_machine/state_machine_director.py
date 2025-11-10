"""
State Machine Director Node.

Central coordinator for the rover state machine providing hierarchical,
event-driven state management with frontend integration and subsystem coordination.
"""

import time
from typing import Optional, Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from autonomy_interfaces.msg import (
    SafetyStatus as SafetyStatusMsg,
    StateTransition as StateTransitionMsg,
    SystemState as SystemStateMsg,
)
from autonomy_interfaces.srv import (
    ChangeState,
    GetSystemState as GetSystemStateSrv,
    RecoverFromSafety,
    SafestopControl,
    SoftwareEstop,
)
from std_msgs.msg import String

from .follow_me_behavior import FollowMeBehavior
from .led_state_publisher import LEDStatePublisher
from .safety_manager import SafetyManager, SafetySeverity, SafetyTriggerType
from .states import (
    AutonomousSubstate,
    CalibrationSubstate,
    EquipmentServicingSubstate,
    SystemState,
    get_state_metadata,
)
from .subsystem_coordinator import SubsystemCoordinator
from .transition_validator import TransitionValidator


class StateMachineDirector(Node):
    """
    Central coordinator for rover state management.

    Manages hierarchical state machine with automatic safety monitoring,
    subsystem coordination, and frontend integration.
    """

    def __init__(self) -> None:
        """Initialize state machine director with all subsystems."""
        super().__init__("state_machine_director")


        self.get_logger().info("Initializing State Machine Director")

        # State variables
        self._current_state = SystemState.BOOT
        self._current_substate = AutonomousSubstate.NONE
        self._current_sub_substate = EquipmentServicingSubstate.NONE
        self._current_calibration_substate = CalibrationSubstate.NONE
        self._previous_state = SystemState.BOOT
        self._state_entry_time = self.get_clock().now()
        self._is_transitioning = False

        # Transition history
        self._transition_history: List[Dict] = []
        self._max_history_length = 100

        # Metadata
        self._state_reason = "System initialization"
        self._operator_id = "system"

        # Force transition audit log
        self._force_transition_log: List[Dict] = []

        # Initialize components
        self.safety_manager = SafetyManager(logger=self.get_logger())
        self.transition_validator = TransitionValidator(logger=self.get_logger(), state_machine_ref=self)
        self.subsystem_coordinator = SubsystemCoordinator(self)
        self.led_publisher = LEDStatePublisher(self)
        self.follow_me_behavior = FollowMeBehavior(self)

        # QoS profiles
        self.qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.qos_event = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # Callback groups for thread safety
        self.state_transition_group = MutuallyExclusiveCallbackGroup()
        self.query_group = ReentrantCallbackGroup()

        # Publishers
        self.state_publisher = self.create_publisher(
            SystemStateMsg, "/state_machine/current_state", self.qos_state
        )
        self.transition_publisher = self.create_publisher(
            StateTransitionMsg, "/state_machine/transitions", self.qos_event
        )
        self.safety_publisher = self.create_publisher(
            SafetyStatusMsg, "/state_machine/safety_status", self.qos_state
        )

        # Service servers
        self.change_state_service = self.create_service(
            ChangeState, "/state_machine/change_state", self._handle_change_state,
            callback_group=self.state_transition_group  # Ensures serialized transitions
        )
        self.get_state_service = self.create_service(
            GetSystemStateSrv,
            "/state_machine/get_system_state",
            self._handle_get_system_state,
            callback_group=self.query_group  # Allows concurrent queries
        )
        self.recover_safety_service = self.create_service(
            RecoverFromSafety,
            "/state_machine/recover_from_safety",
            self._handle_recover_from_safety,
            callback_group=self.state_transition_group  # Safety operations are critical
        )

        self.safestop_control_service = self.create_service(
            SafestopControl,
            "/state_machine/safestop_control",
            self._handle_safestop_control,
            callback_group=self.state_transition_group
        )

        self.software_estop_service = self.create_service(
            SoftwareEstop,
            "/state_machine/software_estop",
            self._handle_software_estop,
            callback_group=self.state_transition_group
        )

        # Timers
        self.update_rate = 10.0  # Hz
        self.state_publish_timer = self.create_timer(
            1.0 / self.update_rate, self._publish_state_update
        )
        self.safety_check_timer = self.create_timer(1.0, self._check_safety)
        self.timeout_check_timer = self.create_timer(1.0, self._check_state_timeout)

        # Parameters
        self.declare_parameter("auto_transition_boot", True)
        self.declare_parameter("boot_timeout", 30.0)
        self.declare_parameter("enable_safety_checks", True)

        # Complete boot after initialization
        self.create_timer(2.0, self._complete_boot, one_shot=True)

        self.get_logger().info(
            f"State Machine Director initialized successfully - starting in {self._current_state.value} state"
        )

    def _complete_boot(self) -> None:
        """Complete boot sequence and transition to IDLE."""
        self.get_logger().info("Boot sequence complete")
        self.transition_validator.set_boot_complete(True)

        # Auto-transition to IDLE if configured
        if self.get_parameter("auto_transition_boot").value:
            self._execute_transition(
                SystemState.IDLE, reason="Boot sequence completed", initiated_by="system"
            )

    def _handle_change_state(
        self, request: ChangeState.Request, response: ChangeState.Response
    ) -> ChangeState.Response:
        """
        Handle state change service request.

        Args:
            request: ChangeState request
            response: ChangeState response

        Returns:
            Populated response object
        """
        self.get_logger().info(
            f"State change requested: {request.desired_state} by {request.operator_id} "
            f"(from {self._current_state.value}, reason: {request.reason})"
        )

        try:
            # Parse desired state
            desired_state = SystemState(request.desired_state)
            desired_substate = (
                AutonomousSubstate(request.desired_substate)
                if request.desired_substate
                else None
            )
            desired_calibration_substate = (
                CalibrationSubstate(request.desired_calibration_substate)
                if hasattr(request, 'desired_calibration_substate') and request.desired_calibration_substate
                else None
            )

            # Validate transition
            is_valid, message, failed_preconditions = (
                self.transition_validator.validate_transition(
                    self._current_state,
                    desired_state,
                    desired_substate,
                    force=request.force,
                )
            )

            # Note: Calibration substate validation would be added here when validator supports it

            if not is_valid:
                if request.force:
                    # Log force transition with high visibility
                    self.get_logger().warn(
                        f"FORCE TRANSITION REQUESTED by {request.operator_id}: "
                        f"{self._current_state.value} -> {request.desired_state}. "
                        f"Reason: {request.reason}"
                    )
                    # Add to separate audit log
                    self._audit_force_transition(request)
                else:
                    response.success = False
                    response.message = message
                    response.actual_state = str(self._current_state)
                    response.actual_substate = str(self._current_substate)
                    if hasattr(response, 'actual_calibration_substate'):
                        response.actual_calibration_substate = str(self._current_calibration_substate)
                    response.preconditions_met = False
                    response.failed_preconditions = failed_preconditions
                    self.get_logger().warning(f"State change rejected: {message}")
                    return response

            # Execute transition
            transition_start = time.time()
            success = self._execute_transition(
                desired_state,
                desired_substate,
                desired_calibration_substate,
                request.reason,
                request.operator_id,
            )
            transition_time = time.time() - transition_start

            response.success = success
            response.actual_state = str(self._current_state)
            response.actual_substate = str(self._current_substate)
            if hasattr(response, 'actual_calibration_substate'):
                response.actual_calibration_substate = str(self._current_calibration_substate)
            response.transition_time = transition_time
            response.preconditions_met = is_valid
            response.failed_preconditions = []

            if success:
                response.message = f"Transitioned to {desired_state}"
                self.get_logger().info(
                    f"State changed successfully to {desired_state} in {transition_time:.3f}s"
                )
            else:
                response.message = "Transition execution failed"
                self.get_logger().error("State transition execution failed")

        except ValueError as e:
            response.success = False
            response.message = f"Invalid state: {str(e)}"
            response.actual_state = str(self._current_state)
            response.actual_substate = str(self._current_substate)
            if hasattr(response, 'actual_calibration_substate'):
                response.actual_calibration_substate = str(self._current_calibration_substate)
            self.get_logger().error(f"Invalid state requested: {str(e)}")

        return response

    def _execute_transition(
        self,
        to_state: SystemState,
        to_substate: Optional[AutonomousSubstate] = None,
        to_calibration_substate: Optional[CalibrationSubstate] = None,
        reason: str = "",
        initiated_by: str = "unknown",
    ) -> bool:
        """
        Execute a state transition.

        Args:
            to_state: Target state
            to_substate: Target substate (if applicable)
            reason: Reason for transition
            initiated_by: Who/what initiated the transition

        Returns:
            True if transition successful
        """
        # SAVE COMPLETE STATE SNAPSHOT FOR ROLLBACK
        saved_state = self._current_state
        saved_substate = self._current_substate
        saved_sub_substate = self._current_sub_substate
        saved_calibration_substate = self._current_calibration_substate
        saved_previous_state = self._previous_state
        saved_state_entry_time = self._state_entry_time
        saved_state_reason = self._state_reason
        saved_operator_id = self._operator_id
        saved_is_transitioning = self._is_transitioning

        self._is_transitioning = True
        transition_start_time = self.get_clock().now()

        try:
            from_state = self._current_state

            # Execute exit actions for current state
            exit_actions = self._exit_state(from_state)

            # Update state
            self._previous_state = from_state
            self._current_state = to_state
            self._state_entry_time = self.get_clock().now()
            self._state_reason = reason
            self._operator_id = initiated_by

            # Update substates
            if to_substate:
                self._current_substate = to_substate
            elif to_state != SystemState.AUTONOMOUS:
                self._current_substate = AutonomousSubstate.NONE
                self._current_sub_substate = EquipmentServicingSubstate.NONE

            if to_calibration_substate:
                self._current_calibration_substate = to_calibration_substate
            elif to_state != SystemState.CALIBRATION:
                self._current_calibration_substate = CalibrationSubstate.NONE

            # Execute entry actions for new state
            entry_actions = self._enter_state(to_state, to_substate)

            # Publish transition event
            self._publish_transition_event(
                from_state,
                to_state,
                transition_start_time,
                self.get_clock().now(),
                True,
                reason,
                initiated_by,
                exit_actions,
                entry_actions,
            )

            # Record in history
            self._add_to_history(from_state, to_state, reason, initiated_by)

            self.get_logger().info(
                f"Transition executed: {from_state.value} -> {to_state.value} "
                f"(reason: {reason}, by: {initiated_by})"
            )

            return True

        except Exception as e:
            # ROLLBACK: Restore complete state snapshot
            self.get_logger().error(
                f"Transition failed, rolling back from {to_state.value} to {saved_state.value}: {str(e)}"
            )

            self._current_state = saved_state
            self._current_substate = saved_substate
            self._current_sub_substate = saved_sub_substate
            self._current_calibration_substate = saved_calibration_substate
            self._previous_state = saved_previous_state
            self._state_entry_time = saved_state_entry_time
            self._state_reason = saved_state_reason
            self._operator_id = saved_operator_id

            # Publish failure event
            self._publish_transition_event(
                saved_state,
                to_state,
                transition_start_time,
                self.get_clock().now(),
                False,
                f"FAILED: {str(e)}",
                initiated_by,
                [],  # No exit actions on failure
                [],  # No entry actions on failure
            )

            # Re-raise for caller to handle
            raise

        finally:
            # Always reset transitioning flag to prevent system lockup
            self._is_transitioning = False

    def _audit_force_transition(self, request) -> None:
        """Audit force transitions for security review."""
        audit_entry = {
            'timestamp': self.get_clock().now().to_msg(),
            'operator_id': request.operator_id,
            'from_state': str(self._current_state),
            'to_state': request.desired_state,
            'reason': request.reason,
            'force_used': True
        }
        # Store in separate force_transition history
        self._force_transition_log.append(audit_entry)

        # Keep only last 50 force transitions to prevent unbounded growth
        if len(self._force_transition_log) > 50:
            self._force_transition_log.pop(0)

        self.get_logger().info(f"Force transition audited: {len(self._force_transition_log)} total force transitions logged")

    def _enter_state(
        self, state: SystemState, substate: Optional[AutonomousSubstate] = None
    ) -> List[str]:
        """
        Execute entry actions for a state.

        Args:
            state: State being entered
            substate: Substate being entered (if applicable)

        Returns:
            List of actions executed
        """
        actions = []

        # State-specific entry actions
        if state == SystemState.BOOT:
            actions.append("initialize_systems")
            self.subsystem_coordinator.initialize_subsystems()

        elif state == SystemState.CALIBRATION:
            actions.append("start_calibration")
            # Initialize calibration substate if not set
            if self._current_calibration_substate == CalibrationSubstate.NONE:
                self._current_calibration_substate = CalibrationSubstate.SETUP
            self.get_logger().info(
                f"Entering CALIBRATION state, substate: {self._current_calibration_substate}"
            )
            self.subsystem_coordinator.start_calibration()

        elif state == SystemState.IDLE:
            actions.append("set_idle_mode")
            self.led_publisher.update_state(state, substate)

        elif state == SystemState.TELEOPERATION:
            actions.append("enable_teleoperation")
            actions.append("activate_blue_led")
            self.subsystem_coordinator.enable_teleoperation()
            self.led_publisher.update_state(state, substate)

        elif state == SystemState.AUTONOMOUS:
            actions.append("enable_autonomous_mode")
            actions.append("activate_red_led")
            self.subsystem_coordinator.enable_autonomous(substate)
            self.led_publisher.update_state(state, substate)
            
            # Handle follow me substate
            if substate == AutonomousSubstate.FOLLOW_ME:
                actions.append("enable_follow_me_mode")
                # Follow me behavior will be started via service call

        elif state in [SystemState.ESTOP, SystemState.SAFESTOP, SystemState.SAFETY]:
            # Unified safety implementation - set everything to safe values
            # Differences between ESTOP/SAFESTOP are in recovery, not stopping
            actions.append("engage_safety_mode")
            actions.append("set_all_systems_to_safe_values")
            self.subsystem_coordinator.engage_safety_mode()
            self.led_publisher.update_state(state, substate)

        elif state == SystemState.SHUTDOWN:
            actions.append("begin_shutdown_sequence")
            self.subsystem_coordinator.shutdown_subsystems()

        return actions

    def _exit_state(self, state: SystemState) -> List[str]:
        """
        Execute exit actions for a state.

        Args:
            state: State being exited

        Returns:
            List of actions executed
        """
        actions = []

        # State-specific exit actions
        if state == SystemState.CALIBRATION:
            actions.append("finalize_calibration")
            # Keep calibration substate for resume capability
            self.get_logger().info(
                f"Exiting CALIBRATION state, was at substate: {self._current_calibration_substate}"
            )
            self.transition_validator.set_calibration_complete(True)

        elif state == SystemState.AUTONOMOUS:
            actions.append("disable_autonomous_mode")
            # Stop follow me behavior if active
            if self._current_substate == AutonomousSubstate.FOLLOW_ME:
                actions.append("disable_follow_me_mode")
                self.follow_me_behavior.stop_following()

        elif state in [SystemState.ESTOP, SystemState.SAFESTOP, SystemState.SAFETY]:
            # Unified safety disengagement - resume normal operation
            actions.append("disengage_safety_mode")
            actions.append("resume_normal_operation")
            self.subsystem_coordinator.disengage_safety_mode()

            # Clear triggers based on safety state type
            if state == SystemState.ESTOP:
                # Clear both hardware and software ESTOP triggers
                self.safety_manager.clear_trigger(SafetyTriggerType.EMERGENCY_STOP)
                self.safety_manager.clear_trigger(SafetyTriggerType.SOFTWARE_ESTOP)
                actions.append("verify_power_restoration")
            elif state == SystemState.SAFESTOP:
                self.safety_manager.clear_trigger(SafetyTriggerType.SAFESTOP_REQUEST)
                self.safety_manager.clear_trigger(SafetyTriggerType.PROXIMITY_VIOLATION)
            else:  # Legacy SAFETY
                self.safety_manager.clear_all_triggers()

        return actions

    def _publish_state_update(self) -> None:
        """Publish current state update (called at update_rate Hz)."""
        msg = SystemStateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "state_machine"

        msg.current_state = str(self._current_state)
        msg.substate = str(self._current_substate)
        msg.sub_substate = str(self._current_sub_substate)

        time_in_state = (
            self.get_clock().now() - self._state_entry_time
        ).nanoseconds / 1e9
        msg.time_in_state = time_in_state

        metadata = get_state_metadata(self._current_state)
        msg.state_timeout = metadata.timeout_seconds

        msg.previous_state = str(self._previous_state)
        msg.transition_timestamp = self._state_entry_time.to_msg()

        msg.is_transitioning = self._is_transitioning
        preconditions = self.transition_validator.get_precondition_status()
        msg.preconditions_met = preconditions.get("calibration_complete", False)

        msg.active_subsystems = self.subsystem_coordinator.get_active_subsystems()
        msg.failed_subsystems = self.subsystem_coordinator.get_failed_subsystems()

        msg.state_reason = self._state_reason
        msg.operator_id = self._operator_id

        self.state_publisher.publish(msg)

    def _publish_transition_event(
        self,
        from_state: SystemState,
        to_state: SystemState,
        start_time,
        end_time,
        success: bool,
        reason: str,
        initiated_by: str,
        exit_actions: List[str],
        entry_actions: List[str],
    ) -> None:
        """Publish state transition event."""
        msg = StateTransitionMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "state_machine"

        msg.from_state = str(from_state)
        msg.to_state = str(to_state)
        msg.start_time = start_time.to_msg()
        msg.end_time = end_time.to_msg()
        msg.transition_duration = (end_time - start_time).nanoseconds / 1e9

        msg.success = success
        msg.reason = reason
        msg.initiated_by = initiated_by
        msg.failure_reason = "" if success else "Transition failed"

        msg.exit_actions_executed = exit_actions
        msg.entry_actions_executed = entry_actions

        self.transition_publisher.publish(msg)

    def _check_safety(self) -> None:
        """Check safety status and handle safety triggers."""
        if not self.get_parameter("enable_safety_checks").value:
            return

        safety_status = self.safety_manager.get_safety_status()

        # Publish safety status
        self._publish_safety_status()

        # If safety triggered and not in safety state, transition
        if not safety_status["is_safe"] and self._current_state not in [SystemState.SAFETY, SystemState.ESTOP, SystemState.SAFESTOP]:
            highest_severity = self.safety_manager.get_highest_severity()
            active_triggers = self.safety_manager.get_active_triggers()

            # Determine appropriate safety state based on trigger type
            target_state = self._determine_safety_state(active_triggers, highest_severity)

            if target_state:
                self.get_logger().warning(
                    f"Safety trigger detected, transitioning to {target_state.value} state"
                )
                self._execute_transition(
                    target_state,
                    reason="Safety triggered",
                    initiated_by="safety_manager",
                )

    def _determine_safety_state(self, active_triggers: List[SafetyTriggerType],
                               highest_severity: Optional[SafetySeverity]) -> Optional[SystemState]:
        """Determine which safety state to transition to based on active triggers."""
        from .safety_manager import SafetyTriggerType

        # Emergency stops take highest priority - both hardware and software
        if (SafetyTriggerType.EMERGENCY_STOP in active_triggers or
            SafetyTriggerType.SOFTWARE_ESTOP in active_triggers):
            return SystemState.ESTOP

        # Proximity violation or operator safestop request
        if (SafetyTriggerType.PROXIMITY_VIOLATION in active_triggers or
            SafetyTriggerType.SAFESTOP_REQUEST in active_triggers):
            return SystemState.SAFESTOP

        # Other critical/emergency triggers default to legacy safety state
        if highest_severity in [SafetySeverity.CRITICAL, SafetySeverity.EMERGENCY]:
            return SystemState.SAFETY

        return None

    def _publish_safety_status(self) -> None:
        """Publish safety status message."""
        status = self.safety_manager.get_safety_status()

        msg = SafetyStatusMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "state_machine"

        msg.is_safe = status["is_safe"]
        msg.safety_level = status["highest_severity"]
        msg.active_triggers = status["active_triggers"]

        msg.battery_level = status["battery_level"]
        msg.temperature = status["temperature"]
        msg.communication_ok = status["communication_ok"]
        msg.sensors_ok = status["sensors_ok"]

        self.safety_publisher.publish(msg)

    def _check_state_timeout(self) -> None:
        """Check if current state has exceeded timeout."""
        metadata = get_state_metadata(self._current_state)
        if metadata.timeout_seconds <= 0.0:
            return  # No timeout configured

        time_in_state = (
            self.get_clock().now() - self._state_entry_time
        ).nanoseconds / 1e9

        if time_in_state > metadata.timeout_seconds:
            self.get_logger().warning(
                f"State timeout exceeded: {self._current_state.value} "
                f"(time: {time_in_state:.1f}s, timeout: {metadata.timeout_seconds}s)"
            )

            # Auto-transition if configured
            if metadata.auto_transition:
                self.get_logger().info(
                    f"Auto-transitioning to {metadata.auto_transition}"
                )
                self._execute_transition(
                    metadata.auto_transition,
                    reason="State timeout",
                    initiated_by="timeout_handler",
                )

    def _handle_get_system_state(
        self,
        request: GetSystemStateSrv.Request,
        response: GetSystemStateSrv.Response,
    ) -> GetSystemStateSrv.Response:
        """Handle get system state service request."""
        response.success = True
        response.message = "State retrieved successfully"

        response.current_state = str(self._current_state)
        response.substate = str(self._current_substate)
        response.sub_substate = str(self._current_sub_substate)

        time_in_state = (
            self.get_clock().now() - self._state_entry_time
        ).nanoseconds / 1e9
        response.time_in_state = time_in_state
        response.state_entered = self._state_entry_time.to_msg()

        # Include history if requested
        if request.include_history:
            limit = request.history_limit if request.history_limit > 0 else 10
            history = self._transition_history[-limit:]
            response.recent_states = [h["to_state"] for h in history]
            response.transition_reasons = [h["reason"] for h in history]

        # Include subsystem info if requested
        if request.include_subsystems:
            response.active_subsystems = (
                self.subsystem_coordinator.get_active_subsystems()
            )
            response.failed_subsystems = (
                self.subsystem_coordinator.get_failed_subsystems()
            )

        return response

    def _handle_recover_from_safety(
        self,
        request: RecoverFromSafety.Request,
        response: RecoverFromSafety.Response,
    ) -> RecoverFromSafety.Response:
        """Handle safety recovery service request."""
        self.get_logger().info(
            f"Safety recovery requested: {request.recovery_method} by {request.operator_id}"
        )

        # Check if in safety state
        if self._current_state != SystemState.SAFETY:
            response.success = False
            response.message = "Not in safety state"
            response.recovery_state = "NOT_APPLICABLE"
            return response

        # Verify operator acknowledged risks
        if not request.acknowledge_risks:
            response.success = False
            response.message = "Must acknowledge risks before recovery"
            response.recovery_state = "REQUIRES_ACTION"
            return response

        # Execute recovery based on method
        if request.recovery_method == "AUTO":
            # Attempt automatic recovery
            self.safety_manager.clear_all_triggers()
            response.success = True
            response.message = "Automatic recovery initiated"
            response.recovery_state = "COMPLETED"
            response.is_safe_to_proceed = True
            response.recommended_next_state = str(SystemState.IDLE)

            # Transition back to IDLE
            self._execute_transition(
                SystemState.IDLE,
                reason="Safety recovery completed",
                initiated_by=request.operator_id,
            )

        elif request.recovery_method == "MANUAL_GUIDED":
            # Manual guided recovery
            response.success = True
            response.message = "Manual recovery in progress"
            response.recovery_state = "IN_PROGRESS"
            response.remaining_steps = ["Verify systems", "Clear safety state manually"]

        elif request.recovery_method == "FULL_RESET":
            # Full system reset
            self.safety_manager.clear_all_triggers()
            response.success = True
            response.message = "Full reset completed"
            response.recovery_state = "COMPLETED"
            response.recommended_next_state = str(SystemState.BOOT)

            # Transition to boot
            self._execute_transition(
                SystemState.BOOT,
                reason="Full system reset",
                initiated_by=request.operator_id,
            )

        else:
            response.success = False
            response.message = f"Unknown recovery method: {request.recovery_method}"
            response.recovery_state = "FAILED"

        return response

    def _handle_safestop_control(
        self,
        request: SafestopControl.Request,
        response: SafestopControl.Response,
    ) -> SafestopControl.Response:
        """Handle safestop control service request."""
        self.get_logger().info(
            f"Safestop control requested: {request.command} by {request.operator_id}"
        )

        # Validate command
        valid_commands = ["ENGAGE", "DISENGAGE", "TOGGLE"]
        if request.command not in valid_commands:
            response.success = False
            response.message = f"Invalid command: {request.command}. Valid: {valid_commands}"
            response.current_state = "FAILED"
            return response

        # Determine target state based on command
        current_is_safestop = self._current_state == SystemState.SAFESTOP

        if request.command == "ENGAGE":
            target_state = SystemState.SAFESTOP
            target_is_safestop = True
        elif request.command == "DISENGAGE":
            target_state = SystemState.IDLE  # Default resume state
            target_is_safestop = False
        elif request.command == "TOGGLE":
            target_is_safestop = not current_is_safestop
            target_state = SystemState.SAFESTOP if target_is_safestop else SystemState.IDLE

        # Check if already in desired state
        if current_is_safestop == target_is_safestop:
            response.success = True
            response.message = f"Already {'in' if current_is_safestop else 'not in'} safestop"
            response.current_state = "ENGAGED" if current_is_safestop else "DISENGAGED"
            response.affected_subsystems = []
            response.timestamp = self.get_clock().now().nanoseconds / 1e9
            response.operator_id = request.operator_id
            return response

        # Execute state transition
        try:
            self._execute_transition(
                target_state,
                reason=f"Safestop {request.command.lower()} by operator",
                initiated_by=request.operator_id,
            )

            response.success = True
            response.message = f"Safestop {request.command.lower()} successful"
            response.current_state = "ENGAGED" if target_is_safestop else "DISENGAGED"
            response.affected_subsystems = ["all"]  # All subsystems affected
            response.timestamp = self.get_clock().now().nanoseconds / 1e9
            response.operator_id = request.operator_id

        except Exception as e:
            self.get_logger().error(f"Safestop control failed: {e}")
            response.success = False
            response.message = f"Safestop control failed: {str(e)}"
            response.current_state = "FAILED"

        return response

    def _handle_software_estop(
        self,
        request: SoftwareEstop.Request,
        response: SoftwareEstop.Response,
    ) -> SoftwareEstop.Response:
        """Handle software emergency stop service request."""
        self.get_logger().critical(
            f"SOFTWARE ESTOP TRIGGERED by {request.operator_id}: {request.reason}"
        )

        # Validate request
        if not request.acknowledge_criticality:
            response.success = False
            response.message = "Must acknowledge criticality of software ESTOP"
            response.triggered_by = request.operator_id
            response.timestamp = self.get_clock().now().nanoseconds / 1e9
            return response

        # Generate unique ESTOP ID
        estop_id = f"software_estop_{int(self.get_clock().now().nanoseconds / 1e9)}_{request.operator_id}"

        try:
            # Trigger software ESTOP safety event
            self.safety_manager.trigger_safety(
                SafetyTriggerType.SOFTWARE_ESTOP,
                SafetySeverity.EMERGENCY,
                f"Software ESTOP triggered by {request.operator_id}: {request.reason}",
                request.operator_id
            )

            # If force_immediate is True, immediately publish emergency stop signal
            if request.force_immediate:
                # Publish emergency stop signal to trigger immediate coordination
                from std_msgs.msg import Bool
                emergency_msg = Bool()
                emergency_msg.data = True

                # This will trigger the emergency response coordinator
                # In a real implementation, you'd have a dedicated publisher for this
                self.get_logger().critical("FORCE IMMEDIATE: Broadcasting emergency stop signal")

            response.success = True
            response.message = f"Software ESTOP triggered successfully: {estop_id}"
            response.estop_id = estop_id
            response.triggered_by = request.operator_id
            response.timestamp = self.get_clock().now().nanoseconds / 1e9
            response.coordination_started = True

            self.get_logger().critical(f"Software ESTOP {estop_id} activated - full emergency shutdown initiated")

        except Exception as e:
            self.get_logger().error(f"Failed to trigger software ESTOP: {e}")
            response.success = False
            response.message = f"Failed to trigger software ESTOP: {str(e)}"
            response.triggered_by = request.operator_id
            response.timestamp = self.get_clock().now().nanoseconds / 1e9
            response.coordination_started = False

        return response

    def _add_to_history(
        self, from_state: SystemState, to_state: SystemState, reason: str, initiated_by: str
    ) -> None:
        """Add transition to history."""
        self._transition_history.append(
            {
                "from_state": str(from_state),
                "to_state": str(to_state),
                "reason": reason,
                "initiated_by": initiated_by,
                "timestamp": self.get_clock().now(),
            }
        )

        # Prune old entries if exceeding max length (FIFO - remove oldest)
        if len(self._transition_history) > self._max_history_length:
            self._transition_history.pop(0)  # Remove oldest entry

        self.get_logger().debug(f"History size: {len(self._transition_history)}/{self._max_history_length}")


def main(args=None):
    """Main entry point for state machine director node."""
    rclpy.init(args=args)
    node = StateMachineDirector()

    # Use MultiThreadedExecutor for proper callback group support
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        node.get_logger().info("Starting state machine director with MultiThreadedExecutor (4 threads)")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

