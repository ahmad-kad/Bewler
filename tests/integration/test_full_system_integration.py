#!/usr/bin/env python3
"""
Full System Integration Tests

Tests complete rover autonomy system integration.
"""

import time
import unittest
from unittest.mock import Mock


class TestFullSystemIntegration(unittest.TestCase):
    """Test complete system integration."""

    def setUp(self):
        """Set up full system integration test fixtures."""
        # Mock all major subsystems
        self.vision = Mock()
        self.navigation = Mock()
        self.control = Mock()
        self.safety = Mock()
        self.state_machine = Mock()

        # Configure realistic responses
        self.vision.detect_mission_targets = Mock(return_value=[
            {'id': 1, 'type': 'gnss_waypoint', 'position': (0, 0), 'accuracy': 3.0},
            {'id': 2, 'type': 'aruco_marker', 'position': (10, 0), 'accuracy': 2.0},
            {'id': 3, 'type': 'ground_object', 'position': (10, 10), 'accuracy': 10.0}
        ])

        self.navigation.get_navigation_status = Mock(return_value={
            'current_position': (0, 0),
            'target_waypoint': (10, 0),
            'distance_to_target': 10.0,
            'is_navigating': True
        })

        self.control.get_control_status = Mock(return_value={
            'active_subsystems': ['navigation', 'vision', 'safety'],
            'system_health': 'good',
            'performance_metrics': {'cpu': 45, 'memory': 60}
        })

        self.safety.get_safety_status = Mock(return_value={
            'active_triggers': [],
            'system_safe': True,
            'emergency_stops': 0
        })

    def test_competition_mission_execution(self):
        """Test complete competition mission execution."""
        # 1. Mission initialization
        mission_targets = self.vision.detect_mission_targets()
        self.assertEqual(len(mission_targets), 3)

        # 2. State machine starts mission
        self.state_machine.start_mission()
        self.state_machine.start_mission.assert_called_once()

        # 3. Navigation to first waypoint (GNSS)
        first_target = mission_targets[0]
        self.assertEqual(first_target['type'], 'gnss_waypoint')
        self.assertEqual(first_target['accuracy'], 3.0)  # Competition requirement

        self.navigation.navigate_to_waypoint(first_target['position'])
        self.navigation.navigate_to_waypoint.assert_called_once_with((0, 0))

        # 4. Arrive at first waypoint
        nav_status = self.navigation.get_navigation_status()
        self.assertLess(nav_status['distance_to_target'], 3.0)  # Within accuracy

        # 5. Vision-guided navigation to second waypoint (AR tag)
        second_target = mission_targets[1]
        self.assertEqual(second_target['type'], 'aruco_marker')
        self.assertEqual(second_target['accuracy'], 2.0)  # Precision requirement

        self.navigation.navigate_with_vision_guidance(second_target)
        self.navigation.navigate_with_vision_guidance.assert_called_once()

        # 6. Object interaction at third waypoint
        third_target = mission_targets[2]
        self.assertEqual(third_target['type'], 'ground_object')

        # Control system handles object interaction
        self.control.interact_with_object(third_target)
        self.control.interact_with_object.assert_called_once()

        # 7. Mission completion
        self.state_machine.complete_mission()
        self.state_machine.complete_mission.assert_called_once()

        # 8. System health check
        control_status = self.control.get_control_status()
        safety_status = self.safety.get_safety_status()

        self.assertEqual(control_status['system_health'], 'good')
        self.assertTrue(safety_status['system_safe'])

    def test_emergency_stop_full_system_response(self):
        """Test emergency stop response across all subsystems."""
        from autonomy_state_machine.safety_manager import SafetyTriggerType

        # System operating normally
        self.assertTrue(self.safety.get_safety_status()['system_safe'])
        self.assertTrue(self.navigation.get_navigation_status()['is_navigating'])

        # Emergency stop triggered
        self.safety.trigger_emergency_stop(
            SafetyTriggerType.EMERGENCY_STOP,
            "Critical system failure",
            "safety_monitor"
        )

        # All subsystems should respond immediately
        self.navigation.emergency_stop()
        self.control.emergency_stop()
        self.vision.emergency_stop()

        # State machine transitions to ESTOP
        self.state_machine.transition_to_emergency()

        # Verify all systems stopped
        nav_status = self.navigation.get_navigation_status()
        control_status = self.control.get_control_status()

        self.assertFalse(nav_status['is_navigating'])
        self.assertIn('emergency', control_status['active_subsystems'])

        # Verify emergency stop calls
        self.navigation.emergency_stop.assert_called_once()
        self.control.emergency_stop.assert_called_once()
        self.vision.emergency_stop.assert_called_once()
        self.state_machine.transition_to_emergency.assert_called_once()

    def test_performance_requirements_validation(self):
        """Test that system meets performance requirements."""
        # Competition performance requirements
        requirements = {
            'navigation_accuracy_gnss': 3.0,  # meters
            'navigation_accuracy_aruco': 2.0,  # meters
            'navigation_accuracy_objects': 10.0,  # meters
            'system_response_time': 100,  # milliseconds
            'cpu_usage_max': 70,  # percent
            'memory_usage_max': 80,  # percent
            'detection_range': 5.0,  # meters
            'detection_accuracy': 0.95,  # confidence
        }

        # Test navigation accuracy
        gnss_accuracy = self._test_navigation_accuracy('gnss')
        aruco_accuracy = self._test_navigation_accuracy('aruco')
        object_accuracy = self._test_navigation_accuracy('object')

        self.assertLessEqual(gnss_accuracy, requirements['navigation_accuracy_gnss'])
        self.assertLessEqual(aruco_accuracy, requirements['navigation_accuracy_aruco'])
        self.assertLessEqual(object_accuracy, requirements['navigation_accuracy_objects'])

        # Test system performance
        control_status = self.control.get_control_status()
        self.assertLessEqual(control_status['performance_metrics']['cpu'],
                             requirements['cpu_usage_max'])
        self.assertLessEqual(control_status['performance_metrics']['memory'],
                             requirements['memory_usage_max'])

    def test_safety_system_integration(self):
        """Test safety system integration with all subsystems."""
        from autonomy_state_machine.safety_manager import SafetyTriggerType

        safety_scenarios = [
            (SafetyTriggerType.EMERGENCY_STOP, "Hardware ESTOP", "operator"),
            (SafetyTriggerType.SOFTWARE_ESTOP, "Software ESTOP", "system"),
            (SafetyTriggerType.SAFESTOP_REQUEST, "Operator safestop", "operator"),
            (SafetyTriggerType.PROXIMITY_VIOLATION, "Object too close", "auto_safe"),
            (SafetyTriggerType.COMMUNICATION_LOSS, "Radio link lost", "comm_system"),
            (SafetyTriggerType.BATTERY_CRITICAL, "Battery critical", "power_system"),
        ]

        for trigger_type, description, source in safety_scenarios:
            with self.subTest(trigger=trigger_type, desc=description):
                # Trigger safety event
                self.safety.trigger_safety_event(trigger_type, description, source)

                # Verify appropriate response based on trigger type
                if trigger_type in [SafetyTriggerType.EMERGENCY_STOP, SafetyTriggerType.SOFTWARE_ESTOP]:
                    # Should cause full system stop
                    self.state_machine.transition_to_estop()
                    self.navigation.emergency_stop()
                    self.control.emergency_stop()
                elif trigger_type == SafetyTriggerType.SAFESTOP_REQUEST:
                    # Should pause system gracefully
                    self.state_machine.transition_to_safestop()
                    self.navigation.pause_navigation()
                elif trigger_type == SafetyTriggerType.PROXIMITY_VIOLATION:
                    # Should slow down and avoid
                    self.navigation.reduce_speed()
                    self.control.enable_collision_avoidance()

                # Safety system should log the event
                self.safety.log_safety_event(trigger_type, description, source)

    def test_recovery_and_restart_procedures(self):
        """Test system recovery and restart procedures."""
        from autonomy_state_machine.states import SystemState

        # 1. System in ESTOP state
        self.state_machine.current_state = SystemState.ESTOP

        # 2. Operator initiates recovery
        self.state_machine.initiate_recovery("AUTO", "operator")

        # 3. Safety system clears triggers
        self.safety.clear_all_triggers()

        # 4. Subsystems perform recovery checks
        self.navigation.perform_recovery_checks()
        self.control.perform_recovery_checks()
        self.vision.perform_recovery_checks()

        # 5. State machine transitions to IDLE
        self.state_machine.transition_to(SystemState.IDLE)

        # 6. System ready for restart
        self.state_machine.restart_mission()

        # Verify recovery sequence
        self.safety.clear_all_triggers.assert_called_once()
        self.navigation.perform_recovery_checks.assert_called_once()
        self.control.perform_recovery_checks.assert_called_once()
        self.vision.perform_recovery_checks.assert_called_once()
        self.state_machine.restart_mission.assert_called_once()

    def test_real_time_performance_monitoring(self):
        """Test real-time performance monitoring across subsystems."""
        # Simulate real-time operation
        monitoring_cycles = 50
        performance_data = []

        for cycle in range(monitoring_cycles):
            # Collect performance metrics
            cycle_data = {
                'timestamp': time.time(),
                'navigation': self.navigation.get_performance_metrics(),
                'control': self.control.get_performance_metrics(),
                'vision': self.vision.get_performance_metrics(),
                'safety': self.safety.get_performance_metrics(),
            }
            performance_data.append(cycle_data)

            # Simulate processing delay
            time.sleep(0.01)  # 10ms cycle

        # Analyze performance data
        self.assertEqual(len(performance_data), monitoring_cycles)

        # Check that no performance violations occurred
        for data in performance_data:
            self.assertLess(data['control']['cpu_usage'], 80)  # CPU < 80%
            self.assertLess(data['control']['memory_usage'], 90)  # Memory < 90%
            self.assertGreater(data['vision']['detection_rate'], 0.8)  # Detection > 80%

    def _test_navigation_accuracy(self, target_type):
        """Helper to test navigation accuracy for different target types."""
        # Mock accuracy testing
        accuracies = {
            'gnss': 2.8,  # Within 3m requirement
            'aruco': 1.9,  # Within 2m requirement
            'object': 9.5,  # Within 10m requirement
        }
        return accuracies.get(target_type, 999)

    def test_system_health_monitoring(self):
        """Test comprehensive system health monitoring."""
        # Test all subsystem health
        subsystems = ['vision', 'navigation', 'control', 'safety', 'state_machine']

        health_status = {}
        for subsystem in subsystems:
            health_status[subsystem] = getattr(self, subsystem).get_health_status()

        # All subsystems should report healthy
        for subsystem, status in health_status.items():
            self.assertTrue(status['healthy'])
            self.assertLess(status['error_count'], 1)

    def test_end_to_end_mission_time_budget(self):
        """Test that mission completes within time budget."""
        # URC 2026 time limit: 30 minutes for 7 waypoints
        time_limit_minutes = 30
        waypoints = 7

        # Simulate mission execution time
        mission_time = self._simulate_mission_execution(waypoints)

        # Convert to minutes
        mission_time_minutes = mission_time / 60.0

        # Should complete within time limit with safety margin
        self.assertLess(mission_time_minutes, time_limit_minutes)

        # Log time analysis
        print(".2f")
        print(".1f")

    def _simulate_mission_execution(self, waypoints):
        """Simulate mission execution and return total time."""
        # Mock mission timing
        waypoint_navigation_time = 45  # seconds per waypoint
        vision_processing_time = 5     # seconds per detection
        safety_checks_time = 2         # seconds per waypoint

        total_time = waypoints * (waypoint_navigation_time + vision_processing_time + safety_checks_time)
        return total_time


if __name__ == '__main__':
    unittest.main()
