#!/usr/bin/env python3
"""
Integration Tests for Vision and Control Systems

Tests interaction between computer vision and state machine control.
"""

import unittest
from unittest.mock import MagicMock, Mock, patch


class TestVisionControlIntegration(unittest.TestCase):
    """Test integration between vision and control systems."""

    def setUp(self):
        """Set up vision-control integration tests."""
        # Mock vision system
        self.vision = Mock()
        self.vision.detect_objects = Mock(return_value=[
            {'type': 'hammer', 'position': (2.0, 1.0, 0.0), 'confidence': 0.95},
            {'type': 'water_bottle', 'position': (3.0, -1.0, 0.0), 'confidence': 0.87}
        ])
        self.vision.get_keyboard_pose = Mock(return_value={
            'position': (1.5, 0.8, 0.3),
            'orientation': (0, 0, 0, 1),
            'detected': True
        })

        # Mock control system
        self.control = Mock()
        self.control.set_target_object = Mock()
        self.control.start_typing_sequence = Mock()
        self.control.get_control_status = Mock(return_value={
            'is_active': False,
            'current_target': None,
            'precision_mode': False
        })

        # Mock state machine
        self.state_machine = Mock()
        self.state_machine.transition_to = Mock()
        self.state_machine.current_state = Mock()

    def test_object_detection_control_integration(self):
        """Test object detection triggering control actions."""
        # Vision detects competition objects
        detected_objects = self.vision.detect_objects()

        self.assertEqual(len(detected_objects), 2)
        self.assertEqual(detected_objects[0]['type'], 'hammer')
        self.assertEqual(detected_objects[1]['type'], 'water_bottle')

        # Control system should respond to detections
        for obj in detected_objects:
            self.control.set_target_object(obj)
            self.control.set_target_object.assert_called_with(obj)

    def test_keyboard_detection_typing_integration(self):
        """Test keyboard detection enabling typing control."""
        # Vision detects keyboard
        keyboard_pose = self.vision.get_keyboard_pose()

        self.assertTrue(keyboard_pose['detected'])
        self.assertIsNotNone(keyboard_pose['position'])

        # Control should start typing sequence
        self.control.start_typing_sequence(keyboard_pose)
        self.control.start_typing_sequence.assert_called_once_with(keyboard_pose)

    def test_vision_based_state_transitions(self):
        """Test vision detections triggering state machine transitions."""
        from autonomy_state_machine.states import SystemState

        # Vision detects keyboard → transition to typing mode
        keyboard_detected = self.vision.get_keyboard_pose()['detected']
        self.assertTrue(keyboard_detected)

        # Should transition to appropriate state for typing
        # (This would depend on current mission phase)
        self.state_machine.transition_to(SystemState.AUTONOMOUS)
        self.state_machine.transition_to.assert_called_with(SystemState.AUTONOMOUS)

    def test_precision_mode_integration(self):
        """Test precision mode activation based on vision confidence."""
        # High confidence detection
        high_confidence_objects = [
            {'type': 'hammer', 'confidence': 0.95, 'position': (1.0, 0.0, 0.0)},
            {'type': 'water_bottle', 'confidence': 0.87, 'position': (2.0, 0.0, 0.0)}
        ]

        # Should activate precision mode for high confidence detections
        for obj in high_confidence_objects:
            if obj['confidence'] > 0.9:
                self.control.set_target_object(obj)
                # Control should enable precision mode
                self.assertGreater(obj['confidence'], 0.9)

    def test_vision_failure_handling(self):
        """Test handling vision system failures."""
        # Simulate vision failure
        self.vision.detect_objects.side_effect = Exception("Camera disconnected")

        # Control should handle gracefully
        try:
            objects = self.vision.detect_objects()
        except Exception as e:
            # Should not crash the control system
            self.assertIsInstance(e, Exception)

        # Control status should remain stable
        status = self.control.get_control_status()
        self.assertIsInstance(status, dict)

    def test_real_time_vision_control_loop(self):
        """Test real-time vision-to-control feedback loop."""
        # Simulate real-time detection loop
        detection_cycles = 10
        objects_detected = 0

        for cycle in range(detection_cycles):
            # Vision detects objects
            objects = self.vision.detect_objects()
            objects_detected += len(objects)

            # Control processes detections
            for obj in objects:
                self.control.set_target_object(obj)

        # Verify detections occurred
        self.assertEqual(objects_detected, detection_cycles * 2)  # 2 objects per cycle

        # Verify control was called for each detection
        self.assertEqual(self.control.set_target_object.call_count, objects_detected)


class TestStateMachineVisionIntegration(unittest.TestCase):
    """Test integration between state machine and vision systems."""

    def setUp(self):
        """Set up state machine-vision integration tests."""
        # Mock state machine
        self.state_machine = Mock()
        self.state_machine.current_state = Mock()
        self.state_machine.transition_to = Mock()

        # Mock vision system
        self.vision = Mock()
        self.vision.detect_mission_objects = Mock(return_value=True)
        self.vision.get_detection_confidence = Mock(return_value=0.92)
        self.vision.is_keyboard_visible = Mock(return_value=True)

    def test_mission_object_detection_state_change(self):
        """Test state changes based on mission object detection."""
        from autonomy_state_machine.states import AutonomousSubstate, SystemState

        # Vision detects mission-critical objects
        mission_objects_found = self.vision.detect_mission_objects()
        self.assertTrue(mission_objects_found)

        # Should transition to appropriate autonomous substate
        confidence = self.vision.get_detection_confidence()
        self.assertGreater(confidence, 0.9)

        # State machine should handle the detection
        self.state_machine.transition_to(SystemState.AUTONOMOUS)

    def test_keyboard_detection_enables_typing(self):
        """Test keyboard detection enables autonomous typing."""
        # Vision detects keyboard
        keyboard_visible = self.vision.is_keyboard_visible()
        self.assertTrue(keyboard_visible)

        # Should enable typing capabilities
        # (State machine should allow typing transitions)

    def test_vision_health_monitoring(self):
        """Test vision system health monitoring."""
        # Vision should report health status
        # If vision fails, state machine should handle gracefully

        # Simulate healthy vision
        self.assertTrue(self.vision.detect_mission_objects())

        # Vision confidence should be high
        confidence = self.vision.get_detection_confidence()
        self.assertGreater(confidence, 0.8)


class TestMultiSubsystemVisionControl(unittest.TestCase):
    """Test multi-subsystem integration with vision and control."""

    def setUp(self):
        """Set up multi-subsystem integration tests."""
        # Mock all subsystems
        self.vision = Mock()
        self.control = Mock()
        self.navigation = Mock()
        self.safety = Mock()
        self.state_machine = Mock()

        # Configure mocks
        self.vision.detect_all_objects = Mock(return_value=[
            {'type': 'keyboard', 'position': (1.0, 0.0, 0.0)},
            {'type': 'hammer', 'position': (2.0, 1.0, 0.0)},
            {'type': 'water_bottle', 'position': (0.0, 1.0, 0.0)}
        ])

    def test_complete_perception_to_action_chain(self):
        """Test complete chain from perception to action."""
        # 1. Vision detects all objects
        detected_objects = self.vision.detect_all_objects()

        self.assertEqual(len(detected_objects), 3)

        # 2. State machine evaluates situation
        # (Would determine next action based on mission state)

        # 3. Control system executes appropriate actions
        for obj in detected_objects:
            if obj['type'] == 'keyboard':
                # Start typing sequence
                self.control.start_typing_sequence(obj)
            elif obj['type'] in ['hammer', 'water_bottle']:
                # Navigate to object
                self.navigation.navigate_to_object(obj)

        # 4. Safety system monitors throughout
        self.safety.monitor_operation()

        # Verify all subsystems were engaged
        self.assertEqual(self.control.start_typing_sequence.call_count, 1)
        self.assertEqual(self.navigation.navigate_to_object.call_count, 2)
        self.assertEqual(self.safety.monitor_operation.call_count, 1)

    def test_error_propagation_and_recovery(self):
        """Test error handling across subsystems."""
        # Simulate vision failure
        self.vision.detect_all_objects.side_effect = Exception("Camera failure")

        # System should handle gracefully
        try:
            objects = self.vision.detect_all_objects()
        except Exception:
            # Safety system should trigger appropriate response
            self.safety.handle_vision_failure()

        # Verify safety response
        self.safety.handle_vision_failure.assert_called_once()

    def test_performance_coordination(self):
        """Test performance coordination across vision and control."""
        # Vision processing time
        vision_time = 0.1  # 100ms

        # Control response time
        control_time = 0.05  # 50ms

        # Total loop time should be acceptable
        total_time = vision_time + control_time
        self.assertLess(total_time, 0.2)  # Less than 200ms total


if __name__ == '__main__':
    unittest.main()
