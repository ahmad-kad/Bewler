#!/usr/bin/env python3
"""
Pytest Configuration and Shared Fixtures

Provides shared test fixtures and configuration for the autonomy test suite.
"""

import time
from unittest.mock import Mock

import numpy as np
import pytest

# Import mock sensors
from tests.fixtures.mock_sensors import (
    MockActuator,
    MockBattery,
    MockCamera,
    MockEncoder,
    MockGPS,
    MockIMU,
    MockLiDAR,
    SensorFusionMock,
    create_mock_sensor_suite,
)


@pytest.fixture(scope="session", autouse=True)
def set_random_seed():
    """Set random seed for reproducible tests."""
    np.random.seed(42)


@pytest.fixture
def mock_gps():
    """Provide mock GPS sensor."""
    return MockGPS()


@pytest.fixture
def mock_imu():
    """Provide mock IMU sensor."""
    return MockIMU()


@pytest.fixture
def mock_camera():
    """Provide mock camera sensor."""
    return MockCamera()


@pytest.fixture
def mock_lidar():
    """Provide mock LiDAR sensor."""
    return MockLiDAR()


@pytest.fixture
def mock_encoders():
    """Provide mock wheel encoders (4 wheels)."""
    return [MockEncoder() for _ in range(4)]


@pytest.fixture
def mock_actuators():
    """Provide mock actuators."""
    return [MockActuator() for _ in range(6)]  # 6-DOF arm


@pytest.fixture
def mock_battery():
    """Provide mock battery sensor."""
    return MockBattery()


@pytest.fixture
def sensor_fusion_mock():
    """Provide complete sensor fusion mock."""
    return SensorFusionMock()


@pytest.fixture
def mock_sensor_suite():
    """Provide complete mock sensor suite."""
    return create_mock_sensor_suite()


@pytest.fixture
def mock_safety_manager():
    """Provide mock safety manager."""
    safety_manager = Mock()
    safety_manager.trigger_safety = Mock()
    safety_manager.clear_trigger = Mock()
    safety_manager.get_safety_status = Mock(return_value={
        'active_triggers': [],
        'highest_severity': None,
        'system_safe': True
    })
    return safety_manager


@pytest.fixture
def mock_navigation():
    """Provide mock navigation system."""
    navigation = Mock()
    navigation.navigate_to_waypoint = Mock()
    navigation.get_navigation_status = Mock(return_value={
        'is_navigating': False,
        'current_position': (0.0, 0.0),
        'target_waypoint': None,
        'distance_to_target': 0.0,
        'velocity': 0.0
    })
    navigation.stop_navigation = Mock()
    navigation.resume_navigation = Mock()
    return navigation


@pytest.fixture
def mock_vision():
    """Provide mock computer vision system."""
    vision = Mock()
    vision.detect_objects = Mock(return_value=[])
    vision.detect_aruco_markers = Mock(return_value=[])
    vision.get_camera_intrinsics = Mock(return_value={
        'fx': 600.0, 'fy': 600.0, 'cx': 320.0, 'cy': 240.0
    })
    vision.get_health_status = Mock(return_value={
        'healthy': True,
        'fps': 30.0,
        'resolution': (640, 480)
    })
    return vision


@pytest.fixture
def mock_control():
    """Provide mock control system."""
    control = Mock()
    control.set_velocity = Mock()
    control.stop_all = Mock()
    control.get_control_status = Mock(return_value={
        'active': True,
        'velocity': 0.0,
        'effort': 0.0,
        'temperature': 25.0
    })
    return control


@pytest.fixture
def mock_state_machine():
    """Provide mock state machine."""
    state_machine = Mock()
    state_machine.current_state = Mock()
    state_machine.transition_to = Mock()
    state_machine.start_mission = Mock()
    state_machine.stop_mission = Mock()
    return state_machine


@pytest.fixture
def integration_test_setup(mock_safety_manager, mock_navigation,
                          mock_vision, mock_control, mock_state_machine):
    """Provide complete integration test setup."""
    return {
        'safety': mock_safety_manager,
        'navigation': mock_navigation,
        'vision': mock_vision,
        'control': mock_control,
        'state_machine': mock_state_machine
    }


# Test data fixtures
@pytest.fixture
def test_waypoints():
    """Provide test mission waypoints."""
    return [
        {'id': 1, 'position': (0.0, 0.0), 'type': 'gnss_waypoint'},
        {'id': 2, 'position': (10.0, 0.0), 'type': 'aruco_marker'},
        {'id': 3, 'position': (10.0, 10.0), 'type': 'ground_object'},
        {'id': 4, 'position': (0.0, 10.0), 'type': 'gnss_waypoint'},
    ]


@pytest.fixture
def test_trajectory():
    """Provide test trajectory data."""
    trajectory = []
    for i in range(50):
        x = i * 0.2  # 20cm steps
        y = np.sin(i * 0.1) * 1.0  # Sine wave
        z = 0.0
        trajectory.append((x, y, z))
    return trajectory


@pytest.fixture
def performance_monitor():
    """Provide performance monitoring fixture."""
    class PerformanceMonitor:
        def __init__(self):
            self.metrics = []
            self.start_time = None

        def start(self):
            self.start_time = time.time()

        def record_metric(self, name, value):
            self.metrics.append({
                'timestamp': time.time(),
                'name': name,
                'value': value
            })

        def get_average(self, name):
            values = [m['value'] for m in self.metrics if m['name'] == name]
            return sum(values) / len(values) if values else 0

        def get_elapsed_time(self):
            return time.time() - self.start_time if self.start_time else 0

    return PerformanceMonitor()


# Custom pytest marks
def pytest_configure(config):
    """Configure custom pytest marks."""
    config.addinivalue_line("markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')")
    config.addinivalue_line("markers", "integration: marks tests as integration tests")
    config.addinivalue_line("markers", "performance: marks tests as performance tests")
    config.addinivalue_line("markers", "safety: marks tests as safety-critical")


# Test utilities
def assert_pose_equal(pose1, pose2, tolerance=1e-6):
    """Assert that two poses are equal within tolerance."""
    assert len(pose1) == len(pose2), f"Pose dimensions don't match: {len(pose1)} vs {len(pose2)}"

    for i, (p1, p2) in enumerate(zip(pose1, pose2)):
        assert abs(p1 - p2) < tolerance, f"Pose component {i} differs: {p1} vs {p2}"


def assert_transforms_equal(transform1, transform2, tolerance=1e-6):
    """Assert that two transforms are equal within tolerance."""
    assert 'position' in transform1 and 'position' in transform2
    assert 'orientation' in transform1 and 'orientation' in transform2

    assert_pose_equal(transform1['position'], transform2['position'], tolerance)
    assert_pose_equal(transform1['orientation'], transform2['orientation'], tolerance)


def wait_for_condition(condition_func, timeout=5.0, check_interval=0.1):
    """Wait for a condition to become true."""
    start_time = time.time()

    while time.time() - start_time < timeout:
        if condition_func():
            return True
        time.sleep(check_interval)

    return False


def simulate_time_passing(seconds):
    """Context manager to simulate time passing for testing."""
    class TimeSimulator:
        def __enter__(self):
            self.start_time = time.time()
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            # Time has "passed" - just return
            pass

        def elapsed(self):
            return time.time() - self.start_time

    return TimeSimulator()
