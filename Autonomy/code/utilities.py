#!/usr/bin/env python3
"""
URC 2026 Autonomy System - Core Utilities

Type-safe utility classes and functions for improved code quality,
debugging, and maintainability across the autonomy system.

Author: URC 2026 Autonomy Team
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ===== ERROR HANDLING =====

Result = Tuple[Optional[Any], Optional[str]]
"""Type alias for (result, error) tuples. None result indicates success with no return value."""


def safe_execute(func: Callable, *args, **kwargs) -> Result:
    """
    Execute function with consistent error handling.

    Args:
        func: Function to execute
        *args: Positional arguments for function
        **kwargs: Keyword arguments for function

    Returns:
        Tuple of (result, error_message). If error_message is None, execution succeeded.
    """
    try:
        result = func(*args, **kwargs)
        return result, None
    except Exception as e:
        return None, str(e)


# ===== RESULT TYPES =====

from typing import Generic, TypeVar, Union

T = TypeVar('T')
E = TypeVar('E')

@dataclass(frozen=True)
class Success(Generic[T]):
    """Successful operation result."""
    value: T
    operation: str = ""

@dataclass(frozen=True)
class Failure(Generic[E]):
    """Failed operation result."""
    error: E
    operation: str = ""
    details: Dict[str, Any] = field(default_factory=dict)

OperationResult = Union[Success[T], Failure[E]]

# Common error types
@dataclass
class ValidationError:
    """Validation error with context."""
    field: str
    value: Any
    reason: str

@dataclass
class ProcessingError:
    """Processing error with context."""
    operation: str
    reason: str
    context: Dict[str, Any] = field(default_factory=dict)

# Helper functions for result types
def success(value: T, operation: str = "") -> Success[T]:
    """Create success result."""
    return Success(value, operation)

def failure(error: E, operation: str = "", **details) -> Failure[E]:
    """Create failure result."""
    return Failure(error, operation, details)


# ===== LOGGING =====

@dataclass
class LogContext:
    """Structured logging context for consistent formatting."""
    component: str
    operation: Optional[str] = None
    correlation_id: Optional[str] = None
    extra_fields: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert context to dictionary for logging."""
        result = {"component": self.component}
        if self.operation:
            result["operation"] = self.operation
        if self.correlation_id:
            result["correlation_id"] = self.correlation_id
        result.update(self.extra_fields)
        return result


class NodeLogger:
    """
    Type-safe logging helper for ROS2 nodes.

    Provides consistent logging format and context across the autonomy system.
    """

    def __init__(self, node: Node, component: str):
        """
        Initialize logger for a specific node and component.

        Args:
            node: ROS2 node instance
            component: Component name for log context
        """
        self._logger = node.get_logger()
        self._component = component

    def _format_message(self, message: str, context: LogContext) -> str:
        """Format log message with context."""
        ctx_dict = context.to_dict()
        if ctx_dict:
            return f"{message} | {ctx_dict}"
        return message

    def debug(self, message: str, operation: Optional[str] = None, **context) -> None:
        """Log debug message with optional context."""
        ctx = LogContext(self._component, operation, extra_fields=context)
        self._logger.debug(self._format_message(message, ctx))

    def info(self, message: str, operation: Optional[str] = None, **context) -> None:
        """Log info message with optional context."""
        ctx = LogContext(self._component, operation, extra_fields=context)
        self._logger.info(self._format_message(message, ctx))

    def warn(self, message: str, operation: Optional[str] = None, **context) -> None:
        """Log warning message with optional context."""
        ctx = LogContext(self._component, operation, extra_fields=context)
        self._logger.warn(self._format_message(message, ctx))

    def error(self, message: str, operation: Optional[str] = None, error: Optional[Exception] = None, **context) -> None:
        """Log error message with optional exception and context."""
        ctx_fields = dict(context)
        if error:
            ctx_fields.update({
                "error_type": type(error).__name__,
                "error_message": str(error)
            })
        ctx = LogContext(self._component, operation, extra_fields=ctx_fields)
        self._logger.error(self._format_message(message, ctx))

    def critical(self, message: str, operation: Optional[str] = None, **context) -> None:
        """Log critical message with optional context."""
        ctx = LogContext(self._component, operation, extra_fields=context)
        self._logger.fatal(self._format_message(message, ctx))


# ===== PARAMETER MANAGEMENT =====

ValidatorFunc = Callable[[Any], bool]
"""Type alias for parameter validation functions."""

@dataclass
class NodeParameters:
    """Type-safe parameter configuration for any node."""

    # Common parameters
    update_rate: float = 10.0
    timeout: float = 5.0
    debug_mode: bool = False
    max_retries: int = 3

    # Node-specific parameters (override in subclasses)
    node_specific_params: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def for_navigation(cls) -> 'NodeParameters':
        """Navigation-specific parameters."""
        return cls(
            update_rate=20.0,  # Higher for navigation
            timeout=30.0,      # Longer timeouts
            node_specific_params={
                "waypoint_tolerance": 2.0,
                "obstacle_distance": 5.0,
                "max_linear_velocity": 1.0,
                "max_angular_velocity": 1.0,
            }
        )

    @classmethod
    def for_vision(cls) -> 'NodeParameters':
        """Vision-specific parameters."""
        return cls(
            update_rate=15.0,  # Vision processing rate
            timeout=10.0,      # Shorter timeouts
            node_specific_params={
                "camera_timeout": 2.0,
                "detection_confidence": 0.7,
                "max_detections": 10,
            }
        )

    @classmethod
    def for_led(cls) -> 'NodeParameters':
        """LED-specific parameters."""
        return cls(
            update_rate=5.0,   # LED update rate
            timeout=1.0,       # Quick timeouts
            node_specific_params={
                "flash_on_duration": 0.5,
                "flash_off_duration": 0.5,
                "enable_hardware": True,
            }
        )

    def declare_all(self, node: Node) -> None:
        """Declare all parameters on the node."""
        for name, value in self.__dict__.items():
            if name != 'node_specific_params':
                node.declare_parameter(name, value)

        for name, value in self.node_specific_params.items():
            node.declare_parameter(name, value)

    def load_all(self, node: Node, logger: NodeLogger) -> None:
        """Load and validate all parameters."""
        for name in self.__dict__.keys():
            if name != 'node_specific_params':
                setattr(self, name, get_validated_parameter(
                    node, name, getattr(self, name), logger=logger
                ))

        for name, default in self.node_specific_params.items():
            self.node_specific_params[name] = get_validated_parameter(
                node, name, default, logger=logger
            )


class ParameterValidator:
    """
    Type-safe parameter validation and loading for ROS2 nodes.
    """

    @staticmethod
    def validate_positive(value: Union[int, float]) -> bool:
        """Validate that value is positive."""
        return isinstance(value, (int, float)) and value > 0

    @staticmethod
    def validate_range(value: Union[int, float], min_val: Union[int, float], max_val: Union[int, float]) -> bool:
        """Validate that value is within range [min_val, max_val]."""
        return isinstance(value, (int, float)) and min_val <= value <= max_val

    @staticmethod
    def validate_choice(value: Any, choices: List[Any]) -> bool:
        """Validate that value is in the list of valid choices."""
        return value in choices

    @staticmethod
    def validate_string_length(value: str, min_len: int = 0, max_len: Optional[int] = None) -> bool:
        """Validate string length constraints."""
        if not isinstance(value, str):
            return False
        if len(value) < min_len:
            return False
        if max_len is not None and len(value) > max_len:
            return False
        return True


def get_validated_parameter(
    node: Node,
    name: str,
    default_value: Any,
    validator: Optional[ValidatorFunc] = None,
    error_msg: Optional[str] = None,
    logger: Optional[NodeLogger] = None
) -> Any:
    """
    Get ROS2 parameter with validation and consistent error handling.

    Args:
        node: ROS2 node instance
        name: Parameter name
        default_value: Default value if parameter is invalid
        validator: Optional validation function that returns True for valid values
        error_msg: Optional custom error message
        logger: Optional logger for error reporting

    Returns:
        Validated parameter value or default_value if invalid
    """
    try:
        value = node.get_parameter(name).value

        # Apply validation if provided
        if validator is not None:
            if not validator(value):
                error = error_msg or f"Parameter '{name}' failed validation, using default: {default_value}"
                if logger:
                    logger.warn(error, parameter=name, value=value, default=default_value)
                else:
                    node.get_logger().warn(error)
                return default_value

        return value

    except Exception as e:
        error = f"Failed to get parameter '{name}': {e}, using default: {default_value}"
        if logger:
            logger.error(error, parameter=name, default=default_value)
        else:
            node.get_logger().error(error)
        return default_value


# ===== STATE MACHINE =====

class StateMachineNode(Node):
    """
    Base class for ROS2 nodes with simple state machine functionality.

    Provides consistent state tracking, transitions, and debugging support.
    """

    def __init__(self, node_name: str, initial_state: str = "idle"):
        """
        Initialize state machine node.

        Args:
            node_name: ROS2 node name
            initial_state: Initial state name
        """
        super().__init__(node_name)

        # State management
        self._current_state = initial_state
        self._state_entry_time = self.get_clock().now()
        self._state_history: List[Dict[str, Any]] = []
        self._max_history_size = 20

        # Initialize logger
        self.logger = NodeLogger(self, node_name)

        # Log initial state
        self.logger.info("State machine initialized", initial_state=initial_state)

    def transition_to(self, new_state: str, reason: str = "") -> None:
        """
        Transition to a new state with logging.

        Args:
            new_state: Name of the new state
            reason: Optional reason for the transition
        """
        old_state = self._current_state
        if old_state == new_state:
            self.logger.debug("State transition requested but already in state", state=new_state)
            return

        # Record transition
        transition = {
            "from": old_state,
            "to": new_state,
            "reason": reason,
            "timestamp": self.get_clock().now().to_msg(),
            "time_in_previous_state": self.time_in_state()
        }
        self._state_history.append(transition)

        # Maintain history size
        if len(self._state_history) > self._max_history_size:
            self._state_history = self._state_history[-self._max_history_size//2:]

        # Update state
        self._current_state = new_state
        self._state_entry_time = self.get_clock().now()

        # Log transition
        self.logger.info(
            f"State transition: {old_state} → {new_state}",
            operation="state_transition",
            old_state=old_state,
            new_state=new_state,
            reason=reason
        )

    @property
    def current_state(self) -> str:
        """Get current state name."""
        return self._current_state

    def time_in_state(self) -> float:
        """Get time spent in current state in seconds."""
        return (self.get_clock().now() - self._state_entry_time).nanoseconds / 1e9

    def get_state_history(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Get recent state transition history.

        Args:
            limit: Maximum number of transitions to return (None for all)

        Returns:
            List of transition dictionaries
        """
        history = self._state_history
        if limit:
            history = history[-limit:]
        return history


class AutonomyNode(StateMachineNode):
    """
    Base class for all autonomy nodes with common functionality.

    Provides:
    - Parameter management
    - Interface factory
    - Timer management
    - Structured logging
    - Debug utilities
    - Data tracing
    """

    def __init__(self, node_name: str, node_params: Optional[NodeParameters] = None):
        """
        Initialize autonomy node with common infrastructure.

        Args:
            node_name: ROS2 node name
            node_params: Node-specific parameters (uses defaults if None)
        """
        super().__init__(node_name)

        # Initialize utilities
        self.params = node_params or NodeParameters()
        self.interfaces = ROS2InterfaceRegistry()
        self.interface_factory = ROS2InterfaceFactory(self, self.interfaces)
        self.timers = TimerManager(self)

        # Declare and load parameters
        self.params.declare_all(self)
        self.params.load_all(self, self.logger)

        # Common publishers and timers
        self.status_pub = self.interface_factory.create_publisher(String, f"{node_name}/status")
        self.status_timer = self.interface_factory.create_timer(1.0, self._publish_status, "status")

        # Data tracing
        self.trace_enabled = self.params.debug_mode
        self.trace_buffer: List[Dict[str, Any]] = []
        self.max_trace_entries = 100

        self.logger.info("Autonomy node initialized", node_name=node_name)

    def _publish_status(self) -> None:
        """Publish node status for monitoring."""
        try:
            status_data = {
                "state": self.current_state,
                "time_in_state": self.time_in_state(),
                "active_interfaces": self.interfaces.get_active_interfaces(),
                "active_timers": self.timers.get_active_timers(),
                "timestamp": self.get_clock().now().nanoseconds / 1e9,
            }

            status_msg = String()
            import json
            status_msg.data = json.dumps(status_data)
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.logger.error("Failed to publish status", error=e)

    def trace_data(self, operation: str, data: Any, **context) -> None:
        """Trace data flow for debugging."""
        if not self.trace_enabled:
            return

        trace_entry = {
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "operation": operation,
            "data_type": type(data).__name__,
            "data_size": len(str(data)) if hasattr(data, '__len__') else 0,
            "context": context,
            "state": self.current_state,
        }

        self.trace_buffer.append(trace_entry)
        if len(self.trace_buffer) > self.max_trace_entries:
            self.trace_buffer.pop(0)

    def get_trace_log(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """Get data trace log for debugging."""
        if limit:
            return self.trace_buffer[-limit:]
        return self.trace_buffer.copy()

    def debug_node_state(self) -> Dict[str, Any]:
        """Get comprehensive node debug information."""
        return {
            "state_machine": self.debug_state_info(),
            "interfaces": self.interfaces.get_active_interfaces(),
            "timers": self.timers.get_timer_info(),
            "parameters": self.params.__dict__,
            "trace_entries": len(self.trace_buffer),
            "trace_enabled": self.trace_enabled,
        }

    def validate_operation(self, operation_name: str, **requirements) -> OperationResult[bool, ValidationError]:
        """Validate operation requirements before execution."""
        try:
            # Check state requirements
            if "required_state" in requirements:
                if self.current_state != requirements["required_state"]:
                    return failure(
                        ValidationError("state", self.current_state, f"must be {requirements['required_state']}"),
                        operation_name
                    )

            # Check interface requirements
            if "required_interfaces" in requirements:
                active = self.interfaces.get_active_interfaces()
                for interface_type, required in requirements["required_interfaces"].items():
                    if len(active.get(interface_type, [])) < required:
                        return failure(
                            ValidationError("interfaces", active[interface_type], f"need {required} {interface_type}"),
                            operation_name
                        )

            # Check parameter requirements
            if "parameter_checks" in requirements:
                for param_check in requirements["parameter_checks"]:
                    param_name = param_check["name"]
                    validator = param_check["validator"]
                    value = getattr(self.params, param_name, None)
                    if not validator(value):
                        return failure(
                            ValidationError(param_name, value, "validation failed"),
                            operation_name
                        )

            return success(True, operation_name)

        except Exception as e:
            return failure(
                ProcessingError(operation_name, f"validation_error: {str(e)}"),
                operation_name
            )


class MessagePipeline:
    """Composable message processing pipeline with error handling and tracing."""

    def __init__(self, logger: NodeLogger, tracer: Optional[Callable] = None):
        """
        Initialize message processing pipeline.

        Args:
            logger: Logger for pipeline operations
            tracer: Optional tracing function for data flow
        """
        self.logger = logger
        self.tracer = tracer
        self.steps: List[Tuple[Callable, str]] = []

    def add_step(self, step_func: Callable, step_name: str) -> 'MessagePipeline':
        """Add processing step to pipeline."""
        self.steps.append((step_func, step_name))
        return self

    def process(self, initial_data: Any) -> OperationResult[Any, ProcessingError]:
        """Process data through all pipeline steps."""
        current_data = initial_data
        step_results = []

        for step_func, step_name in self.steps:
            try:
                # Trace input data
                if self.tracer:
                    self.tracer(f"pipeline_input_{step_name}", current_data, step=step_name)

                # Execute step
                current_data = step_func(current_data)

                # Trace output data
                if self.tracer:
                    self.tracer(f"pipeline_output_{step_name}", current_data, step=step_name)

                step_results.append({"step": step_name, "status": "success"})

            except Exception as e:
                error = ProcessingError(
                    "pipeline_processing",
                    f"step_failed: {step_name}",
                    {"step": step_name, "exception": str(e), "completed_steps": step_results}
                )
                self.logger.error(f"Pipeline step failed: {step_name}", error=error)
                return Failure(error)

        return Success(current_data, "pipeline_processing")

    def debug_state_info(self) -> Dict[str, Any]:
        """Get current state debugging information."""
        return {
            "current_state": self.current_state,
            "time_in_state": self.time_in_state(),
            "total_transitions": len(self._state_history),
            "recent_transitions": self.get_state_history(5)
        }


# ===== ROS2 INTERFACE REGISTRY =====

class ROS2InterfaceRegistry:
    """
    Registry for tracking ROS2 interfaces (publishers, subscribers, services, actions).

    Helps with debugging and ensures proper cleanup.
    """

    def __init__(self):
        self._publishers: Dict[str, Any] = {}
        self._subscribers: Dict[str, Any] = {}
        self._services: Dict[str, Any] = {}
        self._actions: Dict[str, Any] = {}
        self._timers: Dict[str, Any] = {}

    def get_timer(self, name: str) -> Optional[Any]:
        """Get registered timer by name."""
        return self._timers.get(name)


class ROS2InterfaceFactory:
    """Factory for creating and tracking ROS2 interfaces with automatic registration."""

    def __init__(self, node: Node, registry: ROS2InterfaceRegistry):
        """
        Initialize interface factory.

        Args:
            node: ROS2 node instance
            registry: Interface registry for tracking
        """
        self.node = node
        self.registry = registry

    def create_publisher(self, msg_type: Any, topic: str, qos_profile=None, **kwargs):
        """Create publisher with automatic registration."""
        publisher = self.node.create_publisher(msg_type, topic, qos_profile or 10, **kwargs)
        self.registry.add_publisher(topic, publisher)
        return publisher

    def create_subscriber(self, msg_type: Any, topic: str, callback, qos_profile=None, **kwargs):
        """Create subscriber with automatic registration."""
        subscriber = self.node.create_subscription(msg_type, topic, callback, qos_profile or 10, **kwargs)
        self.registry.add_subscriber(topic, subscriber)
        return subscriber

    def create_service(self, srv_type: Any, service_name: str, callback, **kwargs):
        """Create service with automatic registration."""
        service = self.node.create_service(srv_type, service_name, callback, **kwargs)
        self.registry.add_service(service_name, service)
        return service

    def create_action_server(self, action_type: Any, action_name: str, execute_callback, **kwargs):
        """Create action server with automatic registration."""
        from rclpy.action import ActionServer
        action_server = ActionServer(self.node, action_type, action_name, execute_callback, **kwargs)
        self.registry.add_action(action_name, action_server)
        return action_server

    def create_timer(self, period: float, callback, name: str = None, **kwargs):
        """Create timer with automatic registration."""
        timer = self.node.create_timer(period, callback, **kwargs)
        timer_name = name or f"timer_{id(timer)}"
        self.registry.add_timer(timer_name, timer)
        return timer

    def add_publisher(self, name: str, publisher: Any) -> None:
        """Register a publisher."""
        self._publishers[name] = publisher

    def add_subscriber(self, name: str, subscriber: Any) -> None:
        """Register a subscriber."""
        self._subscribers[name] = subscriber

    def add_service(self, name: str, service: Any) -> None:
        """Register a service."""
        self._services[name] = service

    def add_action(self, name: str, action: Any) -> None:
        """Register an action."""
        self._actions[name] = action

    def add_timer(self, name: str, timer: Any) -> None:
        """Register a timer."""
        self._timers[name] = timer

    def get_active_interfaces(self) -> Dict[str, List[str]]:
        """Get summary of active interfaces for debugging."""
        return {
            "publishers": list(self._publishers.keys()),
            "subscribers": list(self._subscribers.keys()),
            "services": list(self._services.keys()),
            "actions": list(self._actions.keys()),
            "timers": list(self._timers.keys())
        }

    def get_publisher(self, name: str) -> Optional[Any]:
        """Get registered publisher by name."""
        return self._publishers.get(name)

    def get_subscriber(self, name: str) -> Optional[Any]:
        """Get registered subscriber by name."""
        return self._subscribers.get(name)

    def get_publisher(self, name: str) -> Optional[Any]:
        """Get registered publisher by name."""
        return self._publishers.get(name)

    def get_subscriber(self, name: str) -> Optional[Any]:
        """Get registered subscriber by name."""
        return self._subscribers.get(name)


# ===== TIMER MANAGEMENT =====

class TimerManager:
    """
    Simple timer lifecycle management for ROS2 nodes.

    Prevents resource leaks and provides debugging capabilities.
    """

    def __init__(self, node: Node):
        """
        Initialize timer manager.

        Args:
            node: ROS2 node instance
        """
        self.node = node
        self.timers: Dict[str, Any] = {}

    def create_named_timer(
        self,
        name: str,
        period: float,
        callback: Callable,
        **kwargs
    ) -> Any:
        """
        Create a named timer with automatic tracking.

        Args:
            name: Unique timer name for tracking
            period: Timer period in seconds
            callback: Timer callback function
            **kwargs: Additional arguments for create_timer

        Returns:
            Created timer object
        """
        if name in self.timers:
            self.node.get_logger().warn(f"Timer '{name}' already exists, replacing")

        timer = self.node.create_timer(period, callback, **kwargs)
        self.timers[name] = timer

        return timer

    def cancel_timer(self, name: str) -> bool:
        """
        Cancel a specific timer by name.

        Args:
            name: Timer name to cancel

        Returns:
            True if timer was found and cancelled, False otherwise
        """
        if name in self.timers:
            self.timers[name].cancel()
            del self.timers[name]
            return True
        return False

    def cancel_all(self) -> int:
        """
        Cancel all managed timers.

        Returns:
            Number of timers cancelled
        """
        count = len(self.timers)
        for timer in self.timers.values():
            timer.cancel()
        self.timers.clear()
        return count

    def get_active_timers(self) -> List[str]:
        """Get list of active timer names for debugging."""
        return list(self.timers.keys())

    def get_timer_info(self) -> Dict[str, Any]:
        """Get detailed timer information for debugging."""
        return {
            "active_timers": self.get_active_timers(),
            "total_count": len(self.timers)
        }


# ===== CONFIGURATION MANAGEMENT =====

def load_config_file(
    filename: str,
    required_keys: Optional[List[str]] = None,
    logger: Optional[NodeLogger] = None
) -> Optional[Dict[str, Any]]:
    """
    Load YAML configuration file with basic validation.

    Args:
        filename: Path to YAML configuration file
        required_keys: List of required configuration keys
        logger: Optional logger for error reporting

    Returns:
        Configuration dictionary or None if loading failed
    """
    try:
        import yaml

        with open(filename, 'r') as f:
            config = yaml.safe_load(f) or {}

        # Validate required keys
        if required_keys:
            missing = [k for k in required_keys if k not in config]
            if missing:
                error_msg = f"Missing required config keys in {filename}: {missing}"
                if logger:
                    logger.error(error_msg, file=filename, missing_keys=missing)
                else:
                    print(f"ERROR: {error_msg}")
                return None

        if logger:
            logger.info("Configuration loaded successfully", file=filename, keys=list(config.keys()))
        return config

    except Exception as e:
        error_msg = f"Failed to load configuration file {filename}: {e}"
        if logger:
            logger.error(error_msg, file=filename, error=str(e))
        else:
            print(f"ERROR: {error_msg}")
        return None
