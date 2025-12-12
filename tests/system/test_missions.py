#!/usr/bin/env python3
"""
Mission Testing Framework - Comprehensive testing for all mission types

Tests mission execution with both mock and real hardware interfaces.
Validates mission logic, error handling, and integration.

Author: URC 2026 Autonomy Team
"""

import os
import sys
from unittest.mock import Mock

import pytest
import rclpy

# Ensure project root is importable as a package root
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)

try:
    # Import mission components (requires missions to be a proper package)
    from missions.hardware_abstraction import (  # type: ignore  # noqa: E402
        HardwareInterface,
        MockActuatorInterface,
        MockSensorInterface,
    )
    from missions.mission_behaviors import (  # type: ignore  # noqa: E402
        DeliveryMission,
        FollowMeMission,
        ObjectDetectionMission,
        WaypointNavigation,
    )
    from missions.mission_executor import (  # type: ignore  # noqa: E402
        MissionExecutor,
        MissionState,
    )
except Exception:
    MissionExecutor = None

if MissionExecutor is None:
    pytest.skip(
        "missions package not importable; system mission tests "
        "require missions to be installed/packaged.",
        allow_module_level=True,
    )


class TestMissionExecutor:
    """Test the mission executor core functionality"""

    @pytest.fixture
    def ros_context(self):
        """Setup ROS2 context for testing"""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def mock_hardware(self, ros_context):
        """Create mock hardware interface"""
        node = rclpy.create_node('test_node')
        hardware = HardwareInterface(node, use_mock=True)
        yield hardware
        node.destroy_node()

    @pytest.fixture
    def mission_executor(self, mock_hardware):
        """Create mission executor with mock hardware"""
        executor = MissionExecutor()
        executor.hardware = mock_hardware
        yield executor

    def test_mission_configuration(self, mission_executor):
        """Test mission configuration loading and validation"""
        config = {
            'name': 'Test Mission',
            'type': 'waypoint_navigation',
            'waypoints': [
                {'x': 10.0, 'y': 0.0, 'heading': 0.0},
                {'x': 20.0, 'y': 10.0, 'heading': 90.0}
            ]
        }

        success = mission_executor.configure_mission(config)
        assert success
        assert mission_executor.current_mission == config

    def test_invalid_mission_config(self, mission_executor):
        """Test handling of invalid mission configurations"""
        # Missing waypoints for navigation mission
        config = {
            'name': 'Invalid Mission',
            'type': 'waypoint_navigation'
            # Missing waypoints
        }

        with pytest.raises(ValueError):
            mission_executor.configure_mission(config)

    def test_mission_state_transitions(self, mission_executor):
        """Test mission state machine transitions"""
        # Start with IDLE state
        assert mission_executor.mission_state == MissionState.IDLE

        # Configure mission
        config = {'name': 'Test', 'type': 'waypoint_navigation', 'waypoints': []}
        mission_executor.configure_mission(config)

        # Start mission
        mission_executor.start_mission_execution()
        assert mission_executor.mission_state == MissionState.EXECUTING

        # Pause mission
        mission_executor.pause_mission_callback(Mock(), Mock())
        assert mission_executor.mission_state == MissionState.PAUSED

        # Resume mission
        mission_executor.resume_mission_callback(Mock(), Mock())
        assert mission_executor.mission_state == MissionState.EXECUTING

        # Stop mission
        mission_executor.stop_mission_callback(Mock(), Mock())
        assert mission_executor.mission_state == MissionState.IDLE


class TestWaypointNavigation:
    """Test waypoint navigation behavior"""

    @pytest.fixture
    def navigation_behavior(self):
        """Create waypoint navigation instance"""
        node = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value = Mock()
        navigation = WaypointNavigation(node)
        yield navigation

    @pytest.fixture
    def mock_sensors(self):
        """Create mock sensor interface"""
        sensors = Mock()

        # Mock position that moves toward waypoints
        sensors.get_current_position = Mock(return_value=(0.0, 0.0, 0.0))

        # Mock velocity commands
        sensors.send_velocity_command = Mock()

        yield sensors

    def test_waypoint_reach_calculation(self, navigation_behavior):
        """Test distance and heading calculations"""
        current_pos = (0.0, 0.0, 0.0)
        waypoint = {'x': 3.0, 'y': 4.0, 'heading': 0.0}

        distance = navigation_behavior._calculate_distance(current_pos, waypoint)
        assert abs(distance - 5.0) < 0.01  # 3-4-5 triangle

        heading_error = navigation_behavior._calculate_heading_error(current_pos, waypoint)
        expected_heading = 90.0  # atan2(4, 3) * 180/π
        assert abs(heading_error - expected_heading) < 1.0

    def test_velocity_command_generation(self, navigation_behavior):
        """Test velocity command computation"""
        # Test approaching waypoint
        vx, vtheta = navigation_behavior._compute_velocity_commands(2.0, 10.0)
        assert vx > 0  # Should move forward
        assert abs(vtheta) > 0  # Should turn toward waypoint

        # Test at waypoint
        vx, vtheta = navigation_behavior._compute_velocity_commands(0.1, 1.0)
        assert vx > 0  # Should still move slowly
        assert abs(vtheta) < 0.1  # Should not turn much

    def test_complete_navigation_mission(self, navigation_behavior, mock_sensors):
        """Test complete waypoint navigation mission"""
        waypoints = [
            {'x': 1.0, 'y': 0.0, 'heading': 0.0},
            {'x': 1.0, 'y': 1.0, 'heading': 90.0}
        ]

        # Mock sensor returning waypoint positions (simulate reaching waypoints)
        position_sequence = [
            (0.0, 0.0, 0.0),    # Start
            (0.9, 0.0, 0.0),    # Near first waypoint
            (1.0, 0.0, 0.0),    # At first waypoint
            (1.0, 0.9, 90.0),   # Near second waypoint
            (1.0, 1.0, 90.0)    # At second waypoint
        ]

        mock_sensors.get_current_position.side_effect = position_sequence

        result = navigation_behavior.execute(waypoints, mock_sensors)

        assert result['success']
        assert result['completed_waypoints'] == 2
        assert mock_sensors.send_velocity_command.call_count > 0


class TestObjectDetectionMission:
    """Test object detection and approach behavior"""

    @pytest.fixture
    def detection_mission(self):
        """Create object detection mission instance"""
        node = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value = Mock()
        mission = ObjectDetectionMission(node)
        yield mission

    def test_target_object_recognition(self, detection_mission):
        """Test target object identification"""

        # Create mock detection
        detection = Mock()
        detection.label = "sample rock"
        detection.bbox = Mock()
        detection.bbox.width = 100
        detection.bbox.height = 80

        assert detection_mission._is_target_object(detection)

        # Test non-target object
        detection.label = "sky"
        assert not detection_mission._is_target_object(detection)

    def test_distance_estimation(self, detection_mission):
        """Test distance estimation from detection size"""
        detection = Mock()
        detection.bbox = Mock()
        detection.bbox.width = 200
        detection.bbox.height = 150

        distance = detection_mission._calculate_detection_distance(detection)
        assert distance < 5.0  # Large bbox should indicate close object

    def test_approach_velocity_calculation(self, detection_mission):
        """Test approach velocity computation"""
        detection = Mock()
        detection.bbox = Mock()
        detection.bbox.center = Mock()
        detection.bbox.center.x = 340  # Slightly right of center (640x480 image)

        vx, vtheta = detection_mission._compute_approach_velocity(detection, 2.0)

        assert vx > 0  # Should move forward
        assert vtheta > 0  # Should turn right toward center


class TestFollowMeMission:
    """Test follow-me behavior"""

    @pytest.fixture
    def follow_mission(self):
        """Create follow-me mission instance"""
        node = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value = Mock()
        mission = FollowMeMission(node)
        yield mission

    def test_follow_velocity_computation(self, follow_mission):
        """Test follow velocity calculation"""

        # Create mock tag pose
        tag_pose = Mock()
        tag_pose.pose = Mock()
        tag_pose.pose.position = Mock()
        tag_pose.pose.position.x = 1.0   # 1m forward
        tag_pose.pose.position.y = 0.5   # 0.5m right
        tag_pose.pose.position.z = 0.0

        vx, vtheta = follow_mission._compute_follow_velocity(tag_pose)

        assert vx > 0  # Should move forward
        assert vtheta > 0  # Should turn toward tag

    def test_distance_control(self, follow_mission):
        """Test distance-based velocity control"""
        # Test too far from tag
        far_pose = Mock()
        far_pose.pose.position.x = 5.0  # 5m away (beyond desired 2m)
        far_pose.pose.position.y = 0.0

        vx, _ = follow_mission._compute_follow_velocity(far_pose)
        assert vx > 0  # Should speed up

        # Test too close to tag
        close_pose = Mock()
        close_pose.pose.position.x = 0.5  # 0.5m away (closer than desired 2m)
        close_pose.pose.position.y = 0.0

        vx, _ = follow_mission._compute_follow_velocity(close_pose)
        assert vx < 0  # Should slow down or reverse


class TestDeliveryMission:
    """Test delivery mission behavior"""

    @pytest.fixture
    def delivery_mission(self):
        """Create delivery mission instance"""
        node = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value = Mock()
        mission = DeliveryMission(node)
        yield mission

    def test_delivery_workflow(self, delivery_mission):
        """Test complete delivery workflow"""
        config = {
            'pickup_location': {'x': 5.0, 'y': 0.0},
            'delivery_location': {'x': 10.0, 'y': 5.0}
        }

        # Mock sensors
        sensors = Mock()
        sensors.navigate_to_position = Mock(return_value=True)
        sensors.send_velocity_command = Mock()

        result = delivery_mission.execute(config, sensors)

        assert result['success']
        assert sensors.navigate_to_position.call_count == 2  # Pickup + delivery
        assert delivery_mission.has_object == False  # Object delivered


class TestHardwareAbstraction:
    """Test hardware abstraction layer"""

    def test_mock_sensor_interface(self):
        """Test mock sensor interface functionality"""
        node = Mock()
        sensors = MockSensorInterface(node)

        # Test position access
        position = sensors.get_current_position()
        assert position is not None
        assert len(position) == 3  # x, y, heading

    def test_mock_actuator_interface(self):
        """Test mock actuator interface functionality"""
        node = Mock()
        actuators = MockActuatorInterface(node)

        # Test velocity command
        actuators.send_velocity_command(1.0, 0.5)

        # Verify command was sent (would check publisher in real test)
        assert actuators.cmd_vel_pub is not None

    def test_hardware_interface_creation(self):
        """Test hardware interface factory"""
        node = Mock()

        # Test mock interface
        hardware = HardwareInterface(node, use_mock=True)
        assert isinstance(hardware.sensors, MockSensorInterface)
        assert isinstance(hardware.actuators, MockActuatorInterface)

        # Test real interface (should fail with NotImplementedError)
        with pytest.raises(NotImplementedError):
            HardwareInterface(node, use_mock=False)


class TestIntegration:
    """Integration tests combining multiple components"""

    def test_complete_mission_workflow(self):
        """Test complete mission execution workflow"""
        # This would integrate MissionExecutor with HardwareInterface
        # and test end-to-end mission execution

        # For now, test the integration points
        node = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value = Mock()

        # Test that components can be instantiated together

        # Mock the ROS2 node creation
        with pytest.raises(Exception):  # Would need proper ROS2 context
            pass

    def test_configuration_validation(self):
        """Test mission configuration validation"""
        from hardware_abstraction import validate_hardware_config

        # Valid config
        valid_config = {
            'use_mock': True,
            'sensor_sources': {
                'websocket': {'url': 'ws://localhost:8080'},
                'can_bus': {'interface': 'can0'},
                'network_cameras': {'endpoints': []}
            }
        }

        errors = validate_hardware_config(valid_config)
        assert len(errors) == 0

        # Invalid config
        invalid_config = {'use_mock': True}  # Missing sensor_sources
        errors = validate_hardware_config(invalid_config)
        assert len(errors) > 0


if __name__ == '__main__':
    pytest.main([__file__])
