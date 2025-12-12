#!/usr/bin/env python3
"""
Unit tests for streamlined autonomy utilities.

Tests the core utility functions and classes without ROS2 dependencies.
"""

import os

# Import utilities directly since ROS2 packages aren't available in test environment
import sys
from typing import Any, Dict

import pytest
from utilities import (
    Failure,
    MessagePipeline,
    NodeParameters,
    ProcessingError,
    Success,
    ValidationError,
    failure,
    safe_execute,
    success,
)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'Autonomy', 'code'))


class TestCoreUtilities:
    """Test core utility functions."""

    def test_safe_execute_success(self):
        """Test safe_execute with successful operation."""
        result, error = safe_execute(lambda: 42)
        assert result == 42
        assert error is None

    def test_safe_execute_failure(self):
        """Test safe_execute with failed operation."""
        result, error = safe_execute(lambda: 1 / 0)
        assert result is None
        assert "division by zero" in error

    def test_result_types(self):
        """Test result type creation and pattern matching."""
        # Test success
        ok_result = success(42, "test_operation")
        assert ok_result.value == 42
        assert ok_result.operation == "test_operation"

        # Test failure
        err_result = failure(ValidationError("test", "value", "reason"), "test_operation")
        assert isinstance(err_result.error, ValidationError)
        assert err_result.operation == "test_operation"

        # Test pattern matching
        def handle_result(result):
            if isinstance(result, Success):
                return f"Success: {result.value}"
            elif isinstance(result, Failure):
                return f"Error: {result.error}"
            else:
                return "Unknown result type"

        assert handle_result(ok_result) == "Success: 42"
        assert "Error:" in handle_result(err_result)  # Check that it contains error info


class TestNodeParameters:
    """Test parameter management utilities."""

    def test_navigation_parameters(self):
        """Test navigation-specific parameter creation."""
        params = NodeParameters.for_navigation()

        assert params.update_rate == 20.0  # Navigation-specific
        assert "waypoint_tolerance" in params.node_specific_params
        assert params.node_specific_params["waypoint_tolerance"] == 2.0

    def test_led_parameters(self):
        """Test LED-specific parameter creation."""
        params = NodeParameters.for_led()

        assert params.update_rate == 5.0  # LED-specific
        assert "flash_on_duration" in params.node_specific_params
        assert params.node_specific_params["flash_on_duration"] == 0.5

    def test_vision_parameters(self):
        """Test vision-specific parameter creation."""
        params = NodeParameters.for_vision()

        assert params.update_rate == 15.0  # Vision-specific
        assert "camera_timeout" in params.node_specific_params
        assert params.node_specific_params["camera_timeout"] == 2.0


class TestMessagePipeline:
    """Test message processing pipeline."""

    def test_pipeline_success(self):
        """Test successful pipeline execution."""
        # Mock logger
        class MockLogger:
            def debug(self, msg, **kwargs): pass
            def error(self, msg, **kwargs): pass

        logger = MockLogger()

        def step1(data):
            return data + 1

        def step2(data):
            return data * 2

        pipeline = (
            MessagePipeline(logger)
            .add_step(step1, "add_one")
            .add_step(step2, "multiply_two")
        )

        result = pipeline.process(5)

        assert isinstance(result, Success)
        assert result.value == 12  # (5 + 1) * 2

    def test_pipeline_failure(self):
        """Test pipeline failure handling."""
        class MockLogger:
            def debug(self, msg, **kwargs): pass
            def error(self, msg, **kwargs): pass

        logger = MockLogger()

        def failing_step(data):
            raise ValueError("Step failed")

        pipeline = MessagePipeline(logger).add_step(failing_step, "fail_step")

        result = pipeline.process(5)

        assert isinstance(result, Failure)
        assert isinstance(result.error, ProcessingError)
        assert "fail_step" in result.error.reason

    def test_pipeline_with_tracing(self):
        """Test pipeline with data tracing."""
        traces = []

        def tracer(operation, data, **context):
            traces.append({"operation": operation, "data": data, "context": context})

        class MockLogger:
            def debug(self, msg, **kwargs): pass
            def error(self, msg, **kwargs): pass

        logger = MockLogger()

        def step1(data):
            return data + 1

        pipeline = MessagePipeline(logger, tracer).add_step(step1, "add_one")

        result = pipeline.process(5)

        assert isinstance(result, Success)
        assert len(traces) == 2  # input and output traces
        assert traces[0]["operation"] == "pipeline_input_add_one"
        assert traces[1]["operation"] == "pipeline_output_add_one"


class TestValidationErrors:
    """Test error type definitions."""

    def test_validation_error(self):
        """Test validation error creation."""
        error = ValidationError("latitude", 100.0, "must be between -90 and 90")
        assert error.field == "latitude"
        assert error.value == 100.0
        assert error.reason == "must be between -90 and 90"

        # Test string representation
        assert "latitude" in str(error)
        assert "100.0" in str(error)

    def test_processing_error(self):
        """Test processing error creation."""
        error = ProcessingError("navigation", "path planning failed", {"goal": [1, 2]})
        assert error.operation == "navigation"
        assert error.reason == "path planning failed"
        assert error.context["goal"] == [1, 2]

        # Test string representation
        assert "navigation" in str(error)
        assert "path planning failed" in str(error)


class TestParameterValidation:
    """Test parameter validation logic."""

    def test_parameter_declaration(self):
        """Test parameter declaration and loading."""
        params = NodeParameters()

        # Create a mock node that tracks declared parameters
        class TrackingMockNode:
            def __init__(self):
                self.declared_params = {}

            def declare_parameter(self, name, default_value):
                self.declared_params[name] = default_value

            def get_parameter(self, name):
                class MockParameter:
                    def __init__(self, value):
                        self.value = value
                return MockParameter(self.declared_params.get(name))

        # Test parameter declaration
        mock_node = TrackingMockNode()
        params.declare_all(mock_node)

        # Check that common parameters are declared
        assert "update_rate" in mock_node.declared_params
        assert "timeout" in mock_node.declared_params
        assert "debug_mode" in mock_node.declared_params

        # Test loading parameters
        params.load_all(mock_node, MockLogger())

        assert params.update_rate == 10.0  # Default value
        assert params.timeout == 5.0
        assert params.debug_mode is False


# Mock classes for testing without ROS2
class MockNode:
    """Mock ROS2 node for testing."""

    def __init__(self, param_values=None):
        self.param_values = param_values or {}

    def declare_parameter(self, name, default_value):
        self.param_values[name] = default_value

    def get_parameter(self, name):
        class MockParameter:
            def __init__(self, value):
                self.value = value
        return MockParameter(self.param_values.get(name, None))


class MockLogger:
    """Mock logger for testing."""

    def info(self, msg, **kwargs): pass
    def error(self, msg, **kwargs): pass
    def warn(self, msg, **kwargs): pass
    def debug(self, msg, **kwargs): pass


class TestIntegrationPatterns:
    """Test how utilities work together."""

    def test_complete_parameter_workflow(self):
        """Test complete parameter workflow."""
        # Create navigation parameters
        params = NodeParameters.for_navigation()

        # Declare parameters
        mock_node = MockNode()
        params.declare_all(mock_node)

        # Simulate ROS2 providing different values
        mock_node.param_values.update({
            "update_rate": 30.0,  # Override default
            "waypoint_tolerance": 3.0,  # Override navigation default
        })

        # Load parameters with validation
        mock_logger = MockLogger()
        params.load_all(mock_node, mock_logger)

        # Verify final values
        assert params.update_rate == 30.0
        assert params.node_specific_params["waypoint_tolerance"] == 3.0
        assert params.node_specific_params["obstacle_distance"] == 5.0  # Default

    def test_pipeline_with_error_handling(self):
        """Test pipeline with comprehensive error handling."""
        class TestLogger:
            def __init__(self):
                self.errors = []
                self.debugs = []

            def debug(self, msg, **kwargs):
                self.debugs.append((msg, kwargs))

            def error(self, msg, **kwargs):
                self.errors.append((msg, kwargs))

        logger = TestLogger()

        def validate_data(data: Dict[str, Any]) -> Dict[str, Any]:
            if not isinstance(data, dict):
                raise ValueError("Data must be a dictionary")
            if "required_field" not in data:
                raise ValueError("Missing required field")
            return data

        def process_data(data: Dict[str, Any]) -> Dict[str, Any]:
            data["processed"] = True
            data["result"] = data["value"] * 2
            return data

        pipeline = (
            MessagePipeline(logger)
            .add_step(validate_data, "validation")
            .add_step(process_data, "processing")
        )

        # Test successful processing
        result = pipeline.process({"value": 5, "required_field": True})
        assert isinstance(result, Success)
        assert result.value["processed"] is True
        assert result.value["result"] == 10

        # Test validation failure
        result = pipeline.process({"value": 5})  # Missing required_field
        assert isinstance(result, Failure)
        assert len(logger.errors) == 1
        assert "Pipeline step failed" in logger.errors[0][0]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
