#!/usr/bin/env python3
"""
URC 2026 Network Communication Stress Test

Tests ROS2 DDS communication under extreme network conditions that are harsher
than real-world scenarios, including severe congestion, packet loss, and latency.
"""

import asyncio
import random
import statistics
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional

import rclpy

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


class NetworkStressPublisher(Node):
    """Publisher node for network stress testing."""

    def __init__(self, topic_name: str, stress_config: Dict):
        super().__init__(f'network_stress_pub_{topic_name.replace("/", "_")}')

        self.stress_config = stress_config
        self.message_count = 0
        self.dropped_messages = 0
        self.latency_samples = []

        # Configure QoS for stress testing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Stress test with best effort
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=stress_config.get('queue_depth', 100)
        )

        self.publisher = self.create_publisher(String, topic_name, qos_profile)

        # High-frequency timer for stress testing
        self.timer = self.create_timer(
            stress_config.get('publish_interval', 0.01),  # 100Hz default
            self.publish_stress_message
        )

    def publish_stress_message(self):
        """Publish a stress test message with simulated network conditions."""
        # Simulate message drops under stress
        if random.random() < self.stress_config.get('drop_rate', 0.0):
            self.dropped_messages += 1
            return

        # Create message with timestamp and stress data
        timestamp = time.time_ns()
        stress_level = self.stress_config.get('stress_level', 'normal')
        payload_size = self.stress_config.get('payload_size', 1000)

        # Create large payload to stress network
        payload = 'X' * payload_size
        message_data = f"{timestamp}:{stress_level}:{payload}"

        msg = String()
        msg.data = message_data

        self.publisher.publish(msg)
        self.message_count += 1


class NetworkStressSubscriber(Node):
    """Subscriber node for network stress testing."""

    def __init__(self, topic_name: str, stress_config: Dict):
        super().__init__(f'network_stress_sub_{topic_name.replace("/", "_")}')

        self.stress_config = stress_config
        self.received_count = 0
        self.latency_samples = []
        self.out_of_order_count = 0
        self.last_timestamp = 0

        # Configure QoS for stress testing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=stress_config.get('queue_depth', 100)
        )

        self.subscription = self.create_subscription(
            String, topic_name, self.stress_callback, qos_profile
        )

    def stress_callback(self, msg):
        """Handle incoming stress test messages."""
        try:
            parts = msg.data.split(':', 2)
            if len(parts) >= 2:
                timestamp_ns = int(parts[0])
                current_time_ns = time.time_ns()

                # Calculate latency
                latency_ms = (current_time_ns - timestamp_ns) / 1_000_000
                self.latency_samples.append(latency_ms)

                # Check for out-of-order messages
                if timestamp_ns < self.last_timestamp:
                    self.out_of_order_count += 1
                self.last_timestamp = timestamp_ns

                self.received_count += 1

        except (ValueError, IndexError):
            # Corrupted message
            pass

    def get_stats(self) -> Dict:
        """Get stress test statistics."""
        if not self.latency_samples:
            return {
                'received': 0,
                'avg_latency_ms': 0,
                'max_latency_ms': 0,
                'out_of_order': 0,
                'jitter_ms': 0
            }

        return {
            'received': self.received_count,
            'avg_latency_ms': statistics.mean(self.latency_samples),
            'max_latency_ms': max(self.latency_samples),
            'min_latency_ms': min(self.latency_samples),
            'out_of_order': self.out_of_order_count,
            'jitter_ms': statistics.stdev(self.latency_samples) if len(self.latency_samples) > 1 else 0,
            'p95_latency_ms': statistics.quantiles(self.latency_samples, n=20)[18] if len(self.latency_samples) >= 20 else max(self.latency_samples)
        }


class NetworkEmulator:
    """Network emulator to create harsh network conditions."""

    def __init__(self):
        self.active_rules = []

    def apply_network_stress(self, stress_level: str):
        """Apply network stress using tc (traffic control)."""

        # Clear existing rules
        self.clear_network_stress()

        if stress_level == 'extreme':
            # Simulate extreme conditions: 50% packet loss, 500ms latency, 10Mbps bandwidth
            rules = [
                'sudo tc qdisc add dev lo root netem delay 500ms 100ms loss 50% rate 10mbit',
                'sudo tc qdisc add dev lo root tbf rate 10mbit burst 32kbit latency 400ms'
            ]
        elif stress_level == 'severe':
            # Severe conditions: 20% packet loss, 200ms latency, 50Mbps bandwidth
            rules = [
                'sudo tc qdisc add dev lo root netem delay 200ms 50ms loss 20% rate 50mbit'
            ]
        elif stress_level == 'moderate':
            # Moderate conditions: 5% packet loss, 50ms latency
            rules = [
                'sudo tc qdisc add dev lo root netem delay 50ms 10ms loss 5%'
            ]
        else:
            return  # No stress

        # Apply rules
        for rule in rules:
            try:
                subprocess.run(rule.split(), check=True, capture_output=True)
                self.active_rules.append(rule)
            except subprocess.CalledProcessError as e:
                print(f"Warning: Failed to apply network rule: {rule}")
                print(f"Error: {e}")

    def clear_network_stress(self):
        """Clear all network stress rules."""
        try:
            subprocess.run(['sudo', 'tc', 'qdisc', 'del', 'dev', 'lo', 'root'],
                         capture_output=True)
        except subprocess.CalledProcessError:
            pass  # May not have rules to clear

        self.active_rules = []


def run_network_stress_test(stress_level: str = 'extreme', duration: float = 30.0):
    """Run comprehensive network stress test."""

    print(f"🔥 Running {stress_level.upper()} Network Stress Test")
    print("=" * 60)

    # Configure stress parameters based on level
    stress_configs = {
        'extreme': {
            'publish_interval': 0.005,  # 200Hz
            'drop_rate': 0.3,  # 30% artificial drops
            'payload_size': 5000,  # 5KB messages
            'queue_depth': 200
        },
        'severe': {
            'publish_interval': 0.01,  # 100Hz
            'drop_rate': 0.1,  # 10% artificial drops
            'payload_size': 2000,  # 2KB messages
            'queue_depth': 100
        },
        'moderate': {
            'publish_interval': 0.02,  # 50Hz
            'drop_rate': 0.02,  # 2% artificial drops
            'payload_size': 1000,  # 1KB messages
            'queue_depth': 50
        }
    }

    config = stress_configs.get(stress_level, stress_configs['moderate'])

    # Initialize network emulator
    net_emulator = NetworkEmulator()
    net_emulator.apply_network_stress(stress_level)

    try:
        # Initialize ROS2
        rclpy.init()

        # Create test topic
        test_topic = f'/stress_test/network/{stress_level}'

        # Create publisher and subscriber
        publisher = NetworkStressPublisher(test_topic, config)
        subscriber = NetworkStressSubscriber(test_topic, config)

        # Spin nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Run stress test
        print(f"📊 Testing {stress_level} network conditions for {duration}s...")
        print("   - Extreme: 50% packet loss, 500ms latency, 10Mbps bandwidth"
        print("   - Severe: 20% packet loss, 200ms latency, 50Mbps bandwidth"
        print("   - Moderate: 5% packet loss, 50ms latency"        print(f"   - Publishing at {1.0/config['publish_interval']:.0f}Hz with {config['payload_size']}B payloads"

        start_time = time.time()
        last_report = start_time

        while time.time() - start_time < duration:
            current_time = time.time()

            # Periodic status report
            if current_time - last_report >= 5.0:
                stats = subscriber.get_stats()
                print(f"   [{current_time - start_time:.1f}s] "
                      f"Sent: {publisher.message_count}, "
                      f"Received: {stats['received']}, "
                      f"Latency: {stats['avg_latency_ms']:.1f}ms, "
                      f"Dropped: {publisher.dropped_messages}")
                last_report = current_time

            time.sleep(0.1)

        # Final statistics
        final_stats = subscriber.get_stats()

        # Cleanup
        executor.shutdown()
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()

        return {
            'stress_level': stress_level,
            'duration': duration,
            'messages_sent': publisher.message_count,
            'messages_received': final_stats['received'],
            'messages_dropped': publisher.dropped_messages,
            'avg_latency_ms': final_stats['avg_latency_ms'],
            'max_latency_ms': final_stats['max_latency_ms'],
            'p95_latency_ms': final_stats['p95_latency_ms'],
            'jitter_ms': final_stats['jitter_ms'],
            'out_of_order': final_stats['out_of_order'],
            'packet_loss_rate': (publisher.dropped_messages /
                               (publisher.message_count + publisher.dropped_messages)) * 100 if publisher.message_count > 0 else 0
        }

    finally:
        # Always clear network stress
        net_emulator.clear_network_stress()


def run_comprehensive_network_stress_test():
    """Run network stress tests across all severity levels."""

    print("🌐 URC 2026 Network Communication Stress Test Suite")
    print("=" * 65)

    results = {}

    # Test each stress level
    stress_levels = ['moderate', 'severe', 'extreme']

    for level in stress_levels:
        print(f"\n🔥 Testing {level.upper()} network conditions...")
        result = run_network_stress_test(level, duration=15.0)  # Shorter duration for comprehensive test
        results[level] = result

        # Quick summary
        print(f"   ✅ {result['messages_received']}/{result['messages_sent']} messages delivered")
        print(".1f"        print(".1f"        print(".1f"
    # Comparative analysis
    print("
📈 NETWORK STRESS ANALYSIS")
    print("=" * 40)

    for level in stress_levels:
        result = results[level]
        print(f"\n{level.upper()} Conditions:")
        print(f"  Reliability: {'❌ POOR' if result['packet_loss_rate'] > 30 else '⚠️ FAIR' if result['packet_loss_rate'] > 10 else '✅ GOOD'}")
        print(f"  Latency: {'❌ VERY HIGH' if result['avg_latency_ms'] > 100 else '⚠️ HIGH' if result['avg_latency_ms'] > 50 else '✅ ACCEPTABLE'}")
        print(f"  Jitter: {result['jitter_ms']:.1f}ms")

    # Overall assessment
    extreme_result = results['extreme']
    print("
🎯 STRESS TEST ASSESSMENT")

    if extreme_result['messages_received'] > extreme_result['messages_sent'] * 0.5:
        print("✅ System maintains basic functionality under extreme network stress")
    else:
        print("❌ System fails under extreme network conditions")

    if extreme_result['avg_latency_ms'] < 1000:  # Less than 1 second
        print("✅ Communication remains responsive under stress")
    else:
        print("⚠️ High latency may impact real-time performance")

    print("
📋 RECOMMENDATIONS:")
    print("   • Implement message prioritization for critical topics")
    print("   • Add heartbeat monitoring for connection health")
    print("   • Consider redundant communication channels")
    print("   • Implement adaptive QoS based on network conditions")

    return results


if __name__ == '__main__':
    import os

    # Add path for imports
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

    results = run_comprehensive_network_stress_test()
    print("
✨ Network stress testing completed!")
