#!/usr/bin/env python3
"""
Tests for CAN Mock Simulator and Priority Message Router
"""

import unittest

from bridges.can_mock_simulator import CANBusMockSimulator
from bridges.priority_message_router import MessagePriority, PriorityMessageRouter


class TestCANMockSimulator(unittest.TestCase):
    """Test CAN bus mock simulator functionality"""

    def setUp(self):
        self.simulator = CANBusMockSimulator()

    def test_get_mock_reading_known_sensor(self):
        """Test getting mock data for known sensor"""
        reading = self.simulator.get_mock_reading('imu')

        self.assertIn('sensor', reading)
        self.assertEqual(reading['sensor'], 'imu')
        self.assertIn('data', reading)
        self.assertTrue(reading['mock'])
        self.assertIn('timestamp', reading)
        self.assertIn('warning', reading)

    def test_get_mock_reading_unknown_sensor(self):
        """Test getting mock data for unknown sensor"""
        reading = self.simulator.get_mock_reading('unknown_sensor')

        self.assertEqual(reading['sensor'], 'unknown_sensor')
        self.assertIn('error', reading)
        self.assertTrue(reading['mock'])

    def test_mock_data_realistic_ranges(self):
        """Test that mock data has realistic value ranges"""
        imu_data = self.simulator.get_mock_reading('imu')['data']

        # Check accelerometer values are reasonable (±20 m/s²)
        self.assertGreaterEqual(imu_data['accel_z'], -20)
        self.assertLessEqual(imu_data['accel_z'], 20)

        # Check temperature is reasonable (0-50°C)
        self.assertGreaterEqual(imu_data['temp'], 0)
        self.assertLessEqual(imu_data['temp'], 50)

    def test_motor_command_simulation(self):
        """Test motor command simulation"""
        initial_velocity = self.simulator.mock_data['motor_left']['velocity']

        self.simulator.set_motor_command('motor_left', 2.5)

        # Velocity should be updated
        self.assertEqual(self.simulator.mock_data['motor_left']['velocity'], 2.5)


class TestPriorityMessageRouter(unittest.TestCase):
    """Test priority-based message routing"""

    def setUp(self):
        self.router = PriorityMessageRouter(max_queue_size=10)

    def test_priority_assignment(self):
        """Test that messages get correct priority assignments"""
        # Safety message should be critical
        safety_msg = {'type': 'safety_trigger'}
        self.assertEqual(self.router.determine_priority(safety_msg), MessagePriority.CRITICAL)

        # Calibration message should be high
        calib_msg = {'type': 'calibration_command'}
        self.assertEqual(self.router.determine_priority(calib_msg), MessagePriority.HIGH)

        # Telemetry should be low
        telemetry_msg = {'type': 'telemetry'}
        self.assertEqual(self.router.determine_priority(telemetry_msg), MessagePriority.LOW)

    def test_message_enqueue_dequeue(self):
        """Test enqueuing and dequeuing messages"""
        message = {'type': 'test_message', 'data': 'test'}

        # Enqueue message
        result = self.router.enqueue_message(message, 'test_source')
        self.assertTrue(result)

        # Dequeue message
        dequeued = self.router.dequeue_message()
        self.assertIsNotNone(dequeued)
        self.assertEqual(dequeued['type'], 'test_message')

    def test_priority_ordering(self):
        """Test that higher priority messages are processed first"""
        # Add messages in reverse priority order
        low_msg = {'type': 'telemetry', 'priority': 'low'}
        high_msg = {'type': 'calibration_command', 'priority': 'high'}
        critical_msg = {'type': 'safety_trigger', 'priority': 'critical'}

        self.router.enqueue_message(low_msg, 'test')
        self.router.enqueue_message(high_msg, 'test')
        self.router.enqueue_message(critical_msg, 'test')

        # Should dequeue in priority order: critical, high, low
        first = self.router.dequeue_message()
        self.assertEqual(first['type'], 'safety_trigger')

        second = self.router.dequeue_message()
        self.assertEqual(second['type'], 'calibration_command')

        third = self.router.dequeue_message()
        self.assertEqual(third['type'], 'telemetry')

    def test_queue_size_limits(self):
        """Test queue size limits and overflow handling"""
        # Fill queue beyond limit
        for i in range(15):  # Max size is 10
            message = {'type': 'test', 'id': i}
            self.router.enqueue_message(message, 'test')

        # Should only keep 10 messages
        status = self.router.get_queue_status()
        self.assertEqual(status['queue_size'], 10)
        self.assertGreater(status['stats']['messages_dropped'], 0)

    def test_queue_status_tracking(self):
        """Test queue status and statistics tracking"""
        # Add some messages
        messages = [
            {'type': 'safety_trigger'},
            {'type': 'calibration_command'},
            {'type': 'telemetry'},
            {'type': 'sensor_data'}
        ]

        for msg in messages:
            self.router.enqueue_message(msg, 'test')

        status = self.router.get_queue_status()

        # Check basic stats
        self.assertEqual(status['queue_size'], 4)
        self.assertIn('priority_breakdown', status)
        self.assertIn('stats', status)

        # Process one message
        self.router.dequeue_message()

        # Check stats updated
        status_after = self.router.get_queue_status()
        self.assertEqual(status_after['queue_size'], 3)
        self.assertGreater(status_after['stats']['messages_processed'], 0)


if __name__ == '__main__':
    unittest.main()
