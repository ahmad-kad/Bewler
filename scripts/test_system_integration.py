#!/usr/bin/env python3
"""
URC 2026 System Integration Test - Software Level
Tests ROS2 node communication and system-wide functionality
"""

import asyncio
import signal
import sys
import threading
import time
from typing import Any, Dict, List

import rclpy
from autonomy_interfaces.msg import SafetyStatus, SystemState, VisionDetection
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class SystemIntegrationTester(Node):
    """Comprehensive system integration test node."""

    def __init__(self):
        super().__init__('system_integration_tester')
        self.logger = self.get_logger()
        self.start_time = time.time()

        # Test results tracking
        self.test_results = {
            'nodes_started': [],
            'topics_detected': [],
            'services_available': [],
            'message_flow_tests': [],
            'safety_tests': [],
            'errors': [],
            'warnings': []
        }

        # Message tracking
        self.received_messages = {
            '/state_machine/current_state': [],
            '/safety/emergency_stop': [],
            'vision/detections': [],
            '/safety/violations': []
        }

        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self._setup_test_subscriptions()
        self._setup_test_publishers()

        self.logger.info("System Integration Tester initialized")

    def _setup_test_subscriptions(self):
        """Set up subscriptions to monitor system topics."""
        self.state_sub = self.create_subscription(
            SystemState, '/state_machine/current_state',
            self._state_callback, self.qos_reliable
        )

        self.emergency_sub = self.create_subscription(
            Bool, '/safety/emergency_stop',
            self._emergency_callback, self.qos_reliable
        )

        self.vision_sub = self.create_subscription(
            VisionDetection, 'vision/detections',
            self._vision_callback, 10  # BEST_EFFORT
        )

        self.safety_sub = self.create_subscription(
            SafetyStatus, '/safety/violations',
            self._safety_callback, self.qos_reliable
        )

    def _setup_test_publishers(self):
        """Set up test publishers for system stimulation."""
        self.test_command_pub = self.create_publisher(
            String, '/mission/commands', self.qos_reliable
        )

        self.test_state_pub = self.create_publisher(
            String, '/state_machine/led_info', self.qos_reliable
        )

    def _state_callback(self, msg):
        """Handle state machine state updates."""
        self.received_messages['/state_machine/current_state'].append(msg)
        self.logger.info(f" State update: {msg.current_state}")
        self.test_results['topics_detected'].append('/state_machine/current_state')

    def _emergency_callback(self, msg):
        """Handle emergency stop signals."""
        self.received_messages['/safety/emergency_stop'].append(msg)
        if msg.data:
            self.logger.warn(" EMERGENCY STOP DETECTED!")
        else:
            self.logger.info(" Emergency stop cleared")
        self.test_results['topics_detected'].append('/safety/emergency_stop')

    def _vision_callback(self, msg):
        """Handle vision detection messages."""
        self.received_messages['vision/detections'].append(msg)
        self.logger.info(f" Vision detection: {msg.target_type}")
        self.test_results['topics_detected'].append('vision/detections')

    def _safety_callback(self, msg):
        """Handle safety violation messages."""
        self.received_messages['/safety/violations'].append(msg)
        if not msg.is_safe:
            self.logger.warn(f" Safety violation: {msg.trigger_description}")
        self.test_results['topics_detected'].append('/safety/violations')

    def send_test_commands(self):
        """Send test commands to stimulate the system."""
        try:
            # Send mission command
            mission_cmd = String()
            mission_cmd.data = "TEST_MISSION_START"
            self.test_command_pub.publish(mission_cmd)
            self.logger.info(" Sent test mission command")

            # Send LED info
            led_cmd = String()
            led_cmd.data = "TEST_LED_COMMAND"
            self.test_state_pub.publish(led_cmd)
            self.logger.info(" Sent test LED command")

        except Exception as e:
            self.logger.error(f"Failed to send test commands: {e}")

    def check_topic_discovery(self):
        """Check if expected topics are discoverable."""
        import subprocess
        try:
            result = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True, timeout=5)
            discovered_topics = set(result.stdout.strip().split('\n'))

            expected_topics = {
                '/state_machine/current_state',
                '/state_machine/transitions',
                '/safety/emergency_stop',
                '/safety/violations',
                'vision/detections',
                '/cmd_vel',
                '/odom',
                '/imu',
                '/mission/commands',
                '/mission/progress'
            }

            found_topics = expected_topics.intersection(discovered_topics)
            missing_topics = expected_topics - discovered_topics

            self.test_results['topic_discovery'] = {
                'expected': len(expected_topics),
                'found': len(found_topics),
                'missing': list(missing_topics)
            }

            if missing_topics:
                self.test_results['warnings'].append(f"Missing topics: {missing_topics}")
                self.logger.warn(f"Missing {len(missing_topics)} expected topics")
            else:
                self.logger.info(" All expected topics discovered")

        except Exception as e:
            self.test_results['errors'].append(f"Topic discovery failed: {e}")

    def check_service_discovery(self):
        """Check if expected services are available."""
        import subprocess
        try:
            result = subprocess.run(['ros2', 'service', 'list'],
                                  capture_output=True, text=True, timeout=5)
            discovered_services = set(result.stdout.strip().split('\n'))

            expected_services = {
                '/state_machine/change_state',
                '/state_machine/software_estop',
                '/state_machine/safestop_control'
            }

            found_services = expected_services.intersection(discovered_services)
            missing_services = expected_services - discovered_services

            self.test_results['service_discovery'] = {
                'expected': len(expected_services),
                'found': len(found_services),
                'missing': list(missing_services)
            }

            if missing_services:
                self.test_results['warnings'].append(f"Missing services: {missing_services}")
            else:
                self.logger.info(" Key services available")

        except Exception as e:
            self.test_results['errors'].append(f"Service discovery failed: {e}")

    def test_message_flow(self):
        """Test message flow between components."""
        # Send test messages and check if they're processed
        self.send_test_commands()

        # Check if we've received responses
        time.sleep(2)  # Wait for message processing

        flow_tests = []

        # Test 1: Mission command processing
        if self.received_messages['/state_machine/current_state']:
            flow_tests.append(" Mission commands processed")
        else:
            flow_tests.append(" Mission commands not processed")

        # Test 2: Safety system monitoring
        if self.received_messages['/safety/violations']:
            flow_tests.append(" Safety system active")
        else:
            flow_tests.append(" Safety system not yet triggered")

        self.test_results['message_flow_tests'] = flow_tests
        for test in flow_tests:
            self.logger.info(f"Message Flow: {test}")

    def run_safety_tests(self):
        """Run safety system integration tests."""
        safety_tests = []

        # Test emergency stop signaling
        emergency_received = len(self.received_messages['/safety/emergency_stop']) > 0
        if emergency_received:
            safety_tests.append(" Emergency stop topic active")
        else:
            safety_tests.append("ℹ Emergency stop not triggered (expected)")

        # Test safety violations monitoring
        violations_received = len(self.received_messages['/safety/violations']) > 0
        if violations_received:
            safety_tests.append(" Safety violations monitoring active")
        else:
            safety_tests.append("ℹ No safety violations detected (expected)")

        self.test_results['safety_tests'] = safety_tests
        for test in safety_tests:
            self.logger.info(f"Safety Test: {test}")

    def generate_report(self):
        """Generate comprehensive test report."""
        runtime = time.time() - self.start_time

        report = f"""
{'='*60}
URC 2026 AUTONOMY SYSTEM INTEGRATION TEST REPORT
{'='*60}

Test Duration: {runtime:.1f} seconds

 TOPIC DISCOVERY:
{self.test_results.get('topic_discovery', {})}

 SERVICE DISCOVERY:
{self.test_results.get('service_discovery', {})}

 MESSAGE FLOW TESTS:
"""

        for test in self.test_results.get('message_flow_tests', []):
            report += f"  {test}\n"

        report += "\n SAFETY SYSTEM TESTS:\n"
        for test in self.test_results.get('safety_tests', []):
            report += f"  {test}\n"

        if self.test_results['errors']:
            report += "\n ERRORS:\n"
            for error in self.test_results['errors']:
                report += f"  {error}\n"

        if self.test_results['warnings']:
            report += "\n WARNINGS:\n"
            for warning in self.test_results['warnings']:
                report += f"  {warning}\n"

        # Overall assessment
        error_count = len(self.test_results['errors'])
        warning_count = len(self.test_results['warnings'])

        if error_count == 0 and warning_count == 0:
            assessment = " PERFECT - All systems operational"
        elif error_count == 0:
            assessment = " GOOD - Minor warnings, systems functional"
        else:
            assessment = " NEEDS ATTENTION - Some errors detected"

        report += f"\n OVERALL ASSESSMENT: {assessment}\n"
        report += '='*60

        print(report)
        return report

def main():
    """Main test function."""
    print(" Starting URC 2026 System Integration Test...")

    rclpy.init()

    tester = SystemIntegrationTester()

    # Run test sequence
    try:
        # Phase 1: Discovery tests
        tester.check_topic_discovery()
        tester.check_service_discovery()

        # Phase 2: Message flow tests
        tester.test_message_flow()

        # Phase 3: Safety tests
        tester.run_safety_tests()

        # Run for a few seconds to collect data
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 10:  # 10 second test
            rclpy.spin_once(tester, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        tester.generate_report()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
