#!/usr/bin/env python3
"""
Integration tests for streamlined autonomy utilities and nodes.

Tests the new base classes, utilities, and refactored nodes to ensure
they work correctly together and provide the expected functionality.
"""

import os
import sys

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

# Ensure Autonomy/code is on path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "Autonomy", "code"))

from utilities import (  # type: ignore  # noqa: E402
    AutonomyNode,
    MessagePipeline,
    NodeLogger,
    NodeParameters,
    ProcessingError,
    ROS2InterfaceFactory,
    ROS2InterfaceRegistry,
    TimerManager,
    ValidationError,
    failure,
    safe_execute,
    success,
)

try:
    # This import requires autonomy_interfaces.action.NavigateToPose to be available
    from navigation.autonomy_navigation.navigation_node import (  # type: ignore  # noqa: E402
        NavigationNode,
        Waypoint,
    )
except Exception:
    NavigationNode = None
    Waypoint = None

if NavigationNode is None:
    pytest.skip(
        "NavigationNode/autonomy_interfaces not available; "
        "streamlined autonomy integration tests require full ROS2 build.",
        allow_module_level=True,
    )


class TestUtilities:
    """Test utility functions and classes."""

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
            match result:
                case success(value, _):
                    return f"Success: {value}"
                case failure(error, _):
                    return f"Error: {error}"

        assert handle_result(ok_result) == "Success: 42"
        assert handle_result(err_result) == "Error: test validation failed"


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


class TestROS2Interfaces:
    """Test ROS2 interface management."""

    @pytest.fixture
    def ros_context(self):
        """Setup ROS2 context for testing."""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_interface_registry(self, ros_context):
        """Test interface registry functionality."""
        node = Node("test_node")
        registry = ROS2InterfaceRegistry()

        # Test registration
        publisher = node.create_publisher(String, "test_topic", 10)
        registry.add_publisher("test_topic", publisher)

        subscriber = node.create_subscription(String, "test_topic", lambda msg: None, 10)
        registry.add_subscriber("test_topic", subscriber)

        # Test retrieval
        active = registry.get_active_interfaces()
        assert "test_topic" in active["publishers"]
        assert "test_topic" in active["subscribers"]

        assert registry.get_publisher("test_topic") == publisher
        assert registry.get_subscriber("test_topic") == subscriber

    def test_interface_factory(self, ros_context):
        """Test interface factory functionality."""
        node = Node("test_node")
        registry = ROS2InterfaceRegistry()
        factory = ROS2InterfaceFactory(node, registry)

        # Create interfaces
        publisher = factory.create_publisher(String, "test_pub")
        subscriber = factory.create_subscriber(String, "test_sub", lambda msg: None)
        timer = factory.create_timer(1.0, lambda: None, "test_timer")

        # Verify registration
        active = registry.get_active_interfaces()
        assert "test_pub" in active["publishers"]
        assert "test_sub" in active["subscribers"]
        assert "test_timer" in active["timers"]


class TestTimerManager:
    """Test timer lifecycle management."""

    @pytest.fixture
    def ros_context(self):
        """Setup ROS2 context for testing."""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_timer_management(self, ros_context):
        """Test timer creation and cleanup."""
        node = Node("test_node")
        timer_manager = TimerManager(node)

        # Create named timer
        timer = timer_manager.create_named_timer("test_timer", 1.0, lambda: None)

        # Verify tracking
        assert "test_timer" in timer_manager.get_active_timers()
        assert timer_manager.get_timer("test_timer") == timer

        # Test cleanup
        cancelled = timer_manager.cancel_all()
        assert cancelled == 1
        assert len(timer_manager.get_active_timers()) == 0


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

        assert isinstance(result, success().__class__)
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

        assert isinstance(result, failure().__class__)
        assert isinstance(result.error, ProcessingError)
        assert "fail_step" in result.error.reason


class TestAutonomyNode:
    """Test the streamlined AutonomyNode base class."""

    @pytest.fixture
    def ros_context(self):
        """Setup ROS2 context for testing."""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_autonomy_node_initialization(self, ros_context):
        """Test AutonomyNode initialization with utilities."""
        params = NodeParameters.for_navigation()
        node = AutonomyNode("test_autonomy_node", params)

        # Verify utilities are initialized
        assert hasattr(node, 'params')
        assert hasattr(node, 'interfaces')
        assert hasattr(node, 'interface_factory')
        assert hasattr(node, 'timers')
        assert hasattr(node, 'logger')

        # Verify parameters are loaded
        assert node.params.update_rate == 20.0
        assert "waypoint_tolerance" in node.params.node_specific_params

        # Verify interfaces are registered
        active = node.interfaces.get_active_interfaces()
        assert "test_autonomy_node/status" in active["publishers"]

    def test_data_tracing(self, ros_context):
        """Test data tracing functionality."""
        node = AutonomyNode("test_node")

        # Enable tracing
        node.trace_enabled = True

        # Trace some data
        node.trace_data("test_operation", {"key": "value"}, context="test")

        # Verify tracing
        traces = node.get_trace_log()
        assert len(traces) == 1
        assert traces[0]["operation"] == "test_operation"
        assert traces[0]["context"] == "test"

    def test_operation_validation(self, ros_context):
        """Test operation validation functionality."""
        node = AutonomyNode("test_node")

        # Test successful validation
        result = node.validate_operation("test_op", required_state="idle")
        assert isinstance(result, success().__class__)

        # Test failed validation
        node.transition_to("busy", "testing")
        result = node.validate_operation("test_op", required_state="idle")
        assert isinstance(result, failure().__class__)
        assert isinstance(result.error, ValidationError)


class TestNavigationNodeIntegration:
    """Integration tests for refactored NavigationNode."""

    @pytest.fixture
    def ros_context(self):
        """Setup ROS2 context for testing."""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_navigation_node_initialization(self, ros_context):
        """Test NavigationNode initializes correctly with new base class."""
        node = NavigationNode()

        # Verify inheritance
        assert isinstance(node, AutonomyNode)

        # Verify navigation-specific initialization
        assert hasattr(node, 'gnss_processor')
        assert hasattr(node, 'path_planner')
        assert hasattr(node, 'motion_controller')
        assert hasattr(node, 'terrain_classifier')

        # Verify navigation state
        assert node.current_waypoint is None
        assert node.current_path == []
        assert not node.is_navigating

    def test_waypoint_validation(self, ros_context):
        """Test waypoint validation functionality."""
        node = NavigationNode()

        # Valid waypoint
        valid_waypoint = Waypoint(latitude=45.0, longitude=-122.0)
        result = node._validate_waypoint(valid_waypoint)
        assert isinstance(result, success().__class__)

        # Invalid latitude
        invalid_waypoint = Waypoint(latitude=100.0, longitude=-122.0)
        result = node._validate_waypoint(invalid_waypoint)
        assert isinstance(result, failure().__class__)
        assert isinstance(result.error, ValidationError)
        assert result.error.field == "latitude"

    def test_navigation_state_machine(self, ros_context):
        """Test navigation state machine integration."""
        node = NavigationNode()

        # Initial state
        assert node.current_state == "idle"

        # Test state transition
        node.transition_to("navigating", "test navigation")
        assert node.current_state == "navigating"

        # Test time in state
        assert node.time_in_state() >= 0.0

    def test_interface_registration(self, ros_context):
        """Test that navigation interfaces are properly registered."""
        node = NavigationNode()

        active = node.interfaces.get_active_interfaces()

        # Should have navigation-specific interfaces
        assert len(active["publishers"]) > 0  # Status publisher + navigation publishers
        assert len(active["subscribers"]) > 0  # GNSS, IMU subscribers
        assert len(active["timers"]) > 0  # Control timer

        # Should have navigation-specific publishers
        publisher_names = active["publishers"]
        assert any("cmd_vel" in name for name in publisher_names)
        assert any("current_waypoint" in name for name in publisher_names)


class TestEndToEndIntegration:
    """End-to-end integration tests."""

    @pytest.fixture
    def ros_context(self):
        """Setup ROS2 context for testing."""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_full_node_lifecycle(self, ros_context):
        """Test complete node lifecycle with all utilities."""
        # Create navigation node
        node = NavigationNode()

        # Test parameter loading
        assert node.params.update_rate == 20.0
        assert node.params.node_specific_params["waypoint_tolerance"] == 2.0

        # Test interface setup
        active_interfaces = node.interfaces.get_active_interfaces()
        assert len(active_interfaces["publishers"]) > 0
        assert len(active_interfaces["subscribers"]) > 0

        # Test tracing
        node.trace_enabled = True
        node.trace_data("test_operation", {"test": "data"})

        traces = node.get_trace_log()
        assert len(traces) == 1

        # Test debug state
        debug_state = node.debug_node_state()
        assert "state_machine" in debug_state
        assert "interfaces" in debug_state
        assert "timers" in debug_state
        assert "parameters" in debug_state

        # Test state machine
        node.transition_to("testing", "integration test")
        assert node.current_state == "testing"

        # Test validation
        result = node.validate_operation("test", required_state="testing")
        assert isinstance(result, success().__class__)

        # Verify logging works
        node.logger.info("Integration test completed", test_status="success")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
