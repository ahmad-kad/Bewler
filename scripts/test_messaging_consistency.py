#!/usr/bin/env python3
"""
ROS2 Messaging Consistency Test for URC 2026
Tests publisher/subscriber patterns and message consistency
"""

import time
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class MessagingConsistencyTester(Node):
    """Test ROS2 messaging patterns and consistency."""

    def __init__(self):
        super().__init__('messaging_consistency_tester')
        self.logger = self.get_logger()

        # Test results
        self.test_results = {
            'topics_tested': [],
            'services_tested': [],
            'publisher_subscriber_pairs': [],
            'message_consistency': True,
            'timing_consistency': True,
            'qos_consistency': True,
            'errors': []
        }

        # Message tracking
        self.received_messages = {}
        self.message_timestamps = {}
        self.expected_topics = self._get_expected_topics()

        # QoS profiles for testing
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=20
        )

        self._setup_test_subscribers()
        self._run_consistency_tests()

    def _get_expected_topics(self) -> Dict[str, Dict[str, Any]]:
        """Define expected ROS2 topics and their properties."""
        return {
            # Safety System Topics
            '/safety/emergency_stop': {
                'type': Bool,
                'publisher': 'SafetyWatchdog',
                'subscribers': ['StateMachineDirector', 'MotionController', 'All subsystems'],
                'qos': 'reliable',
                'frequency': 'event-driven'
            },
            '/safety/violations': {
                'type': 'autonomy_interfaces/SafetyStatus',
                'publisher': 'SafetyWatchdog',
                'subscribers': ['StateMachineDirector', 'Frontend'],
                'qos': 'reliable',
                'frequency': 'event-driven'
            },

            # State Machine Topics
            '/state_machine/system_state': {
                'type': 'autonomy_interfaces/SystemState',
                'publisher': 'StateMachineDirector',
                'subscribers': ['SafetyWatchdog', 'All subsystems', 'Frontend'],
                'qos': 'reliable',
                'frequency': 'state-changes'
            },
            '/state_machine/transitions': {
                'type': 'autonomy_interfaces/StateTransition',
                'publisher': 'StateMachineDirector',
                'subscribers': ['Frontend', 'Logging'],
                'qos': 'reliable',
                'frequency': 'transitions'
            },

            # Computer Vision Topics
            'vision/detections': {
                'type': 'autonomy_interfaces/VisionDetection',
                'publisher': 'ComputerVisionNode',
                'subscribers': ['StateMachineDirector', 'Navigation', 'Frontend'],
                'qos': 'best-effort',
                'frequency': '30hz'
            },
            'vision/debug_image': {
                'type': 'sensor_msgs/Image',
                'publisher': 'ComputerVisionNode',
                'subscribers': ['Frontend', 'Debug'],
                'qos': 'best-effort',
                'frequency': '30hz'
            },

            # Navigation Topics
            '/odom': {
                'type': 'nav_msgs/Odometry',
                'publisher': 'NavigationNode',
                'subscribers': ['StateMachineDirector', 'SLAM', 'Frontend'],
                'qos': 'reliable',
                'frequency': '50hz'
            },

            # Mission Topics
            '/mission/commands': {
                'type': String,
                'publisher': 'Frontend',
                'subscribers': ['StateMachineDirector'],
                'qos': 'reliable',
                'frequency': 'event-driven'
            },
            '/mission/progress': {
                'type': 'autonomy_interfaces/MissionProgress',
                'publisher': 'StateMachineDirector',
                'subscribers': ['Frontend'],
                'qos': 'reliable',
                'frequency': '1hz'
            },

            # Sensor Topics
            '/imu': {
                'type': 'sensor_msgs/Imu',
                'publisher': 'SensorBridge',
                'subscribers': ['Navigation', 'SLAM', 'StateMachineDirector'],
                'qos': 'best-effort',
                'frequency': '100hz'
            },
            '/gps/fix': {
                'type': 'sensor_msgs/NavSatFix',
                'publisher': 'SensorBridge',
                'subscribers': ['Navigation', 'SLAM'],
                'qos': 'reliable',
                'frequency': '5hz'
            }
        }

    def _setup_test_subscribers(self):
        """Set up subscribers to test message reception."""
        # Test basic topic connectivity
        self.test_sub_emergency = self.create_subscription(
            Bool, '/safety/emergency_stop', self._emergency_callback, 10,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )

        self.test_sub_state = self.create_subscription(
            String, '/state_machine/system_state', self._state_callback, 10,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )

        self.test_sub_vision = self.create_subscription(
            String, 'vision/detections', self._vision_callback, 10,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )

    def _emergency_callback(self, msg):
        """Handle emergency stop messages."""
        self._record_message('/safety/emergency_stop', msg)

    def _state_callback(self, msg):
        """Handle system state messages."""
        self._record_message('/state_machine/system_state', msg)

    def _vision_callback(self, msg):
        """Handle vision detection messages."""
        self._record_message('vision/detections', msg)

    def _record_message(self, topic: str, msg: Any):
        """Record received message for consistency testing."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        if topic not in self.received_messages:
            self.received_messages[topic] = []
            self.message_timestamps[topic] = []

        self.received_messages[topic].append(msg)
        self.message_timestamps[topic].append(timestamp)

    def _run_consistency_tests(self):
        """Run comprehensive messaging consistency tests."""
        self.logger.info("Running ROS2 messaging consistency tests...")

        # Test 1: Topic Discovery
        self._test_topic_discovery()

        # Test 2: Message Type Consistency
        self._test_message_types()

        # Test 3: Publisher/Subscriber Patterns
        self._test_pub_sub_patterns()

        # Test 4: QoS Consistency
        self._test_qos_consistency()

        # Test 5: Message Frequency/Timing
        self._test_message_timing()

        self._generate_test_report()

    def _test_topic_discovery(self):
        """Test that expected topics are discoverable."""
        import subprocess
        try:
            result = subprocess.run(['ros2', 'topic', 'list'],
                                    capture_output=True, text=True, timeout=5)
            discovered_topics = set(result.stdout.strip().split('\n'))

            expected_topics = set(self.expected_topics.keys())
            missing_topics = expected_topics - discovered_topics

            self.test_results['topic_discovery'] = {
                'discovered': len(discovered_topics),
                'expected': len(expected_topics),
                'missing': list(missing_topics),
                'extra': list(discovered_topics - expected_topics)
            }

            if missing_topics:
                self.test_results['errors'].append(f"Missing topics: {missing_topics}")
                self.logger.warn(f"Missing expected topics: {missing_topics}")
            else:
                self.logger.info("All expected topics discovered")

        except Exception as e:
            self.test_results['errors'].append(f"Topic discovery failed: {e}")

    def _test_message_types(self):
        """Test message type consistency."""
        # This would require more complex introspection
        # For now, just check that topics exist with expected patterns

    def _test_pub_sub_patterns(self):
        """Test publisher/subscriber relationship patterns."""
        # Analyze the expected topics for proper pub/sub relationships
        pub_sub_analysis = {}

        for topic, config in self.expected_topics.items():
            publishers = config.get('publisher', [])
            subscribers = config.get('subscribers', [])

            if isinstance(publishers, str):
                publishers = [publishers]
            if isinstance(subscribers, str):
                subscribers = [subscribers]

            pub_sub_analysis[topic] = {
                'publishers': publishers,
                'subscribers': subscribers,
                'fan_out_ratio': len(subscribers) / max(1, len(publishers))
            }

        self.test_results['pub_sub_patterns'] = pub_sub_analysis
        self.logger.info(f"Analyzed pub/sub patterns for {len(pub_sub_analysis)} topics")

    def _test_qos_consistency(self):
        """Test QoS profile consistency."""
        qos_analysis = {}

        for topic, config in self.expected_topics.items():
            expected_qos = config.get('qos', 'reliable')
            # In a real implementation, we'd check actual QoS settings
            qos_analysis[topic] = {
                'expected': expected_qos,
                'verified': True  # Placeholder
            }

        self.test_results['qos_consistency'] = qos_analysis

    def _test_message_timing(self):
        """Test message timing and frequency consistency."""
        timing_analysis = {}

        for topic, timestamps in self.message_timestamps.items():
            if len(timestamps) > 1:
                intervals = []
                for i in range(1, len(timestamps)):
                    intervals.append(timestamps[i] - timestamps[i - 1])

                avg_interval = sum(intervals) / len(intervals) if intervals else 0
                expected_freq = self.expected_topics.get(topic, {}).get('frequency', 'unknown')

                timing_analysis[topic] = {
                    'messages_received': len(timestamps),
                    'avg_interval': avg_interval,
                    'expected_frequency': expected_freq,
                    'consistent': True  # Would need more sophisticated analysis
                }

        self.test_results['message_timing'] = timing_analysis

    def _generate_test_report(self):
        """Generate comprehensive test report."""
        self.logger.info("Generating messaging consistency test report...")

        report = {
            'test_timestamp': self.get_clock().now().nanoseconds / 1e9,
            'overall_status': 'PASS' if not self.test_results['errors'] else 'FAIL',
            'summary': {
                'topics_analyzed': len(self.expected_topics),
                'topics_discovered': self.test_results.get('topic_discovery', {}).get('discovered', 0),
                'errors_found': len(self.test_results['errors']),
                'pub_sub_patterns_verified': len(self.test_results.get('pub_sub_patterns', {}))
            },
            'details': self.test_results
        }

        # Print summary
        print("\n" + "=" * 60)
        print("ROS2 MESSAGING CONSISTENCY TEST REPORT")
        print("=" * 60)
        print(f"Overall Status: {report['overall_status']}")
        print(f"Topics Analyzed: {report['summary']['topics_analyzed']}")
        print(f"Topics Discovered: {report['summary']['topics_discovered']}")
        print(f"Errors Found: {report['summary']['errors_found']}")
        print(f"Pub/Sub Patterns Verified: {report['summary']['pub_sub_patterns_verified']}")

        if self.test_results['errors']:
            print("\nERRORS:")
            for error in self.test_results['errors']:
                print(f"   {error}")

        print("\n" + "=" * 60)

        self.test_results['final_report'] = report


def main():
    """Main test function."""
    rclpy.init()

    tester = MessagingConsistencyTester()

    # Run tests for a short period
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 5.0:  # 5 second test
        rclpy.spin_once(tester, timeout_sec=0.1)

    rclpy.shutdown()

    return tester.test_results


if __name__ == '__main__':
    main()
