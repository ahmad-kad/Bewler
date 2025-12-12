"""
Frontend interface module for state machine.

Provides helper utilities for frontend integration with the state machine,
including service clients, state update subscribers, and acknowledgment mechanism.
"""

from typing import Callable, Dict, Optional

import structlog
from autonomy_interfaces.msg import SystemState as SystemStateMsg
from autonomy_interfaces.srv import ChangeState, GetSystemState, RecoverFromSafety
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String

logger = structlog.get_logger(__name__)


class FrontendInterface:
    """
    Frontend interface for state machine interaction.

    Provides a clean interface for frontend applications to:
    - Request state changes
    - Subscribe to state updates
    - Send acknowledgments
    - Monitor connection status
    """

    def __init__(self, node: Node):
        """
        Initialize frontend interface.

        Args:
            node: ROS2 node instance
        """
        self.node = node
        self._state_callbacks: list[Callable] = []
        self._connection_ok = False
        self._last_state_update: Optional[SystemStateMsg] = None

        # Service clients
        self.change_state_client: Client = node.create_client(
            ChangeState, "/state_machine/change_state"
        )
        self.get_state_client: Client = node.create_client(
            GetSystemState, "/state_machine/get_system_state"
        )
        self.recover_safety_client: Client = node.create_client(
            RecoverFromSafety, "/state_machine/recover_from_safety"
        )

        # Subscribers
        self.state_subscriber = node.create_subscription(
            SystemStateMsg,
            "/state_machine/current_state",
            self._state_callback,
            10,
        )

        # Publisher for acknowledgments
        self.ack_publisher = node.create_publisher(
            String, "/state_machine/frontend_ack", 10
        )

        # Connection monitoring timer
        self.connection_timer = node.create_timer(1.0, self._check_connection)

        logger.info("Frontend interface initialized")

    def request_state_change(
        self,
        desired_state: str,
        desired_substate: str = "",
        reason: str = "",
        operator_id: str = "frontend",
        force: bool = False,
        callback: Optional[Callable] = None,
    ) -> Future:
        """
        Request a state change.

        Args:
            desired_state: Target state name
            desired_substate: Optional target substate
            reason: Human-readable reason
            operator_id: Operator identifier
            force: Force transition (skip validation)
            callback: Optional callback for async response

        Returns:
            Future object for async request
        """
        # Wait for service to be available
        if not self.change_state_client.wait_for_service(timeout_sec=5.0):
            logger.error("Change state service not available")
            raise RuntimeError("State machine service not available")

        # Create request
        request = ChangeState.Request()
        request.desired_state = desired_state
        request.desired_substate = desired_substate
        request.reason = reason
        request.operator_id = operator_id
        request.force = force

        # Send request
        future = self.change_state_client.call_async(request)

        if callback:
            future.add_done_callback(lambda f: callback(f.result()))

        logger.info(
            "State change requested",
            desired_state=desired_state,
            reason=reason,
        )

        return future

    def get_current_state(
        self,
        include_history: bool = False,
        include_subsystems: bool = False,
        callback: Optional[Callable] = None,
    ) -> Future:
        """
        Get current system state.

        Args:
            include_history: Include transition history
            include_subsystems: Include subsystem status
            callback: Optional callback for async response

        Returns:
            Future object for async request
        """
        if not self.get_state_client.wait_for_service(timeout_sec=5.0):
            logger.error("Get state service not available")
            raise RuntimeError("State machine service not available")

        request = GetSystemState.Request()
        request.include_history = include_history
        request.include_subsystems = include_subsystems
        request.history_limit = 10

        future = self.get_state_client.call_async(request)

        if callback:
            future.add_done_callback(lambda f: callback(f.result()))

        logger.debug("State query requested")

        return future

    def request_safety_recovery(
        self,
        recovery_method: str,
        operator_id: str = "frontend",
        acknowledge_risks: bool = True,
        notes: str = "",
        callback: Optional[Callable] = None,
    ) -> Future:
        """
        Request safety recovery.

        Args:
            recovery_method: "AUTO", "MANUAL_GUIDED", or "FULL_RESET"
            operator_id: Operator identifier
            acknowledge_risks: Acknowledge recovery risks
            notes: Additional notes
            callback: Optional callback for async response

        Returns:
            Future object for async request
        """
        if not self.recover_safety_client.wait_for_service(timeout_sec=5.0):
            logger.error("Recover safety service not available")
            raise RuntimeError("State machine service not available")

        request = RecoverFromSafety.Request()
        request.recovery_method = recovery_method
        request.operator_id = operator_id
        request.acknowledge_risks = acknowledge_risks
        request.notes = notes

        future = self.recover_safety_client.call_async(request)

        if callback:
            future.add_done_callback(lambda f: callback(f.result()))

        logger.info("Safety recovery requested", method=recovery_method)

        return future

    def register_state_callback(self, callback: Callable) -> None:
        """
        Register a callback for state updates.

        Args:
            callback: Function to call on state update (receives SystemStateMsg)
        """
        self._state_callbacks.append(callback)
        logger.info("State callback registered")

    def _state_callback(self, msg: SystemStateMsg) -> None:
        """
        Internal callback for state updates.

        Args:
            msg: State update message
        """
        self._last_state_update = msg
        self._connection_ok = True

        # Call all registered callbacks
        for callback in self._state_callbacks:
            try:
                callback(msg)
            except Exception as e:
                logger.error(
                    "Error in state callback",
                    error=str(e),
                    callback=callback.__name__,
                )

        # Send acknowledgment
        self._send_acknowledgment(msg)

    def _send_acknowledgment(self, msg: SystemStateMsg) -> None:
        """
        Send acknowledgment of state update.

        Args:
            msg: State message being acknowledged
        """
        ack_msg = String()
        ack_msg.data = (
            f"ACK:{msg.current_state}:{msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )
        self.ack_publisher.publish(ack_msg)

        logger.debug("State update acknowledged", state=msg.current_state)

    def _check_connection(self) -> None:
        """Check connection status to state machine."""
        # Simple connection check - could be more sophisticated
        if self._last_state_update is None:
            self._connection_ok = False
            logger.warning("No state updates received yet")
        else:
            # Check if last update was recent
            now = self.node.get_clock().now()
            last_update_time = (
                self._last_state_update.header.stamp.sec
                + self._last_state_update.header.stamp.nanosec * 1e-9
            )
            current_time = (
                now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
            )

            # If no update in last 2 seconds, consider connection lost
            if current_time - last_update_time > 2.0:
                if self._connection_ok:
                    logger.warning("Connection to state machine appears lost")
                    self._connection_ok = False

    def is_connected(self) -> bool:
        """
        Check if connected to state machine.

        Returns:
            True if connection is healthy
        """
        return self._connection_ok

    def get_last_state(self) -> Optional[Dict]:
        """
        Get last received state.

        Returns:
            Dictionary with last state information, or None if no state received
        """
        if self._last_state_update is None:
            return None

        return {
            "current_state": self._last_state_update.current_state,
            "substate": self._last_state_update.substate,
            "sub_substate": self._last_state_update.sub_substate,
            "time_in_state": self._last_state_update.time_in_state,
            "is_transitioning": self._last_state_update.is_transitioning,
            "active_subsystems": self._last_state_update.active_subsystems,
            "failed_subsystems": self._last_state_update.failed_subsystems,
            "state_reason": self._last_state_update.state_reason,
        }


# Example usage for frontend developers:
"""
Example frontend integration:

```python
import rclpy
from rclpy.node import Node
from autonomy_state_machine.frontend_interface import FrontendInterface

class FrontendNode(Node):
    def __init__(self):
        super().__init__('frontend_node')
        self.interface = FrontendInterface(self)

        # Register callback for state updates
        self.interface.register_state_callback(self.on_state_update)

    def on_state_update(self, msg):
        # Update UI based on state
        print(f"State: {msg.current_state}, Substate: {msg.substate}")
        self.update_ui(msg.current_state, msg.substate)

    def request_autonomous_mode(self):
        # Request state change to autonomous
        future = self.interface.request_state_change(
            desired_state="AUTONOMOUS",
            desired_substate="AUTONOMOUS_NAVIGATION",
            reason="Starting autonomous navigation mission",
            operator_id="operator_1"
        )

        # Can wait for result or use callback
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.success:
            print("Successfully transitioned to autonomous mode")
        else:
            print(f"Failed: {response.message}")

    def update_ui(self, state, substate):
        # Update your UI here
        pass

def main():
    rclpy.init()
    node = FrontendNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
"""
