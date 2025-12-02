# URC 2026 Autonomy Utilities

Type-safe utility classes and functions for improved code quality, debugging, and maintainability across the autonomy system.

## Overview

These utilities provide consistent patterns for:
- Error handling with type safety
- Structured logging with context
- Parameter validation
- State machine management
- Interface tracking
- Timer lifecycle management
- Configuration loading

## Quick Start

```python
from autonomy.utilities import (
    NodeLogger, safe_execute, get_validated_parameter,
    StateMachineNode, ROS2InterfaceRegistry, TimerManager
)

class MyNode(StateMachineNode):
    def __init__(self):
        super().__init__("my_node")

        # Type-safe logging
        self.logger = NodeLogger(self, "my_component")

        # Interface registry
        self.interfaces = ROS2InterfaceRegistry()

        # Timer management
        self.timers = TimerManager(self)

        # Validated parameters
        self.timeout = get_validated_parameter(
            self, "timeout", 5.0,
            validator=lambda x: x > 0,
            logger=self.logger
        )
    }
```

## Core Components

### Error Handling

```python
from autonomy.utilities import safe_execute

# BEFORE: Manual try/except everywhere
try:
    result = risky_operation()
    return result, None
except Exception as e:
    return None, str(e)

# AFTER: Consistent error handling
result, error = safe_execute(risky_operation)
if error:
    handle_error(error)
else:
    process_result(result)
```

### Logging

```python
from autonomy.utilities import NodeLogger

# Initialize in your node
self.logger = NodeLogger(self, "component_name")

# Usage with context
self.logger.info("Processing started", item_id=item.id, operation="validation")
self.logger.error("Processing failed", error=e, item_id=item.id)

# Debug mode control
if self.config["debug_mode"]:
    self.logger.debug("Detailed debug info", **debug_context)
```

### Parameter Validation

```python
from autonomy.utilities import get_validated_parameter, ParameterValidator

# Validated parameters with automatic fallback
self.timeout = get_validated_parameter(
    self, "timeout", 5.0,
    validator=ParameterValidator.validate_positive,
    error_msg="Timeout must be positive",
    logger=self.logger
)

# Custom validators
self.max_retries = get_validated_parameter(
    self, "max_retries", 3,
    validator=lambda x: 0 <= x <= 10,
    logger=self.logger
)
```

### State Machine

```python
from autonomy.utilities import StateMachineNode

class MyNode(StateMachineNode):
    def __init__(self):
        super().__init__("my_node", "idle")

    def start_process(self):
        self.transition_to("processing", "Starting data processing")

    def finish_process(self):
        self.transition_to("idle", "Processing complete")

    def debug_state(self):
        state_info = self.debug_state_info()
        self.logger.info("Current state", **state_info)
```

### Interface Registry

```python
from autonomy.utilities import ROS2InterfaceRegistry

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.interfaces = ROS2InterfaceRegistry()

        # Register interfaces
        self.interfaces.add_publisher("status", self.create_publisher(...))
        self.interfaces.add_subscriber("commands", self.create_subscription(...))

        # Debug interface status
        active = self.interfaces.get_active_interfaces()
        self.logger.info("Active interfaces", **active)
```

### Timer Management

```python
from autonomy.utilities import TimerManager

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.timers = TimerManager(self)

        # Named timers with automatic tracking
        self.timers.create_named_timer("control", 0.1, self.control_loop)
        self.timers.create_named_timer("health", 5.0, self.health_check)

    def destroy_node(self):
        # Automatic cleanup
        cancelled = self.timers.cancel_all()
        self.logger.info("Shutdown complete", timers_cancelled=cancelled)
        super().destroy_node()
```

### Configuration Loading

```python
from autonomy.utilities import load_config_file

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")

        # Load and validate config
        self.config = load_config_file(
            "config/my_node.yaml",
            required_keys=["update_rate", "timeout"],
            logger=self.logger
        )

        if not self.config:
            raise RuntimeError("Failed to load configuration")
```

## Migration Guide

### Existing Code → New Patterns

```python
# OLD: Inconsistent error handling
def process_data(self, data):
    try:
        result = self.validate(data)
        return result
    except ValueError as e:
        self.get_logger().error(f"Validation: {e}")
        return None

# NEW: Type-safe error handling
def process_data(self, data) -> Result[Dict, str]:
    result, error = safe_execute(self.validate, data)
    if error:
        self.logger.error("Validation failed", error=error)
        return None, error
    return result, None
```

```python
# OLD: Manual parameter handling
timeout = self.get_parameter("timeout").value
if timeout <= 0:
    self.get_logger().error("Invalid timeout")
    timeout = 5.0

# NEW: Validated parameters
self.timeout = get_validated_parameter(
    self, "timeout", 5.0,
    validator=lambda x: x > 0,
    logger=self.logger
)
```

## Benefits

- **Type Safety**: Compile-time error detection with mypy
- **Debuggability**: Structured logging with correlation IDs
- **Consistency**: Uniform patterns across the codebase
- **Maintainability**: Centralized error handling and validation
- **Reliability**: Automatic resource cleanup and state tracking
- **Developer Experience**: Better IDE support and error messages

## Best Practices

1. **Always use NodeLogger** instead of `get_logger()` for consistent formatting
2. **Use safe_execute** for any operation that might fail
3. **Validate parameters** at startup, not runtime
4. **Register interfaces** for debugging visibility
5. **Use named timers** for automatic cleanup
6. **Extend StateMachineNode** for nodes with state transitions

## Testing

```python
# Test utilities work without ROS2
import pytest
from autonomy.utilities import safe_execute, ParameterValidator

def test_safe_execute():
    result, error = safe_execute(lambda: 42)
    assert result == 42
    assert error is None

def test_safe_execute_error():
    result, error = safe_execute(lambda: 1/0)
    assert result is None
    assert "division by zero" in error

def test_parameter_validator():
    assert ParameterValidator.validate_positive(5.0)
    assert not ParameterValidator.validate_positive(-1.0)
```
