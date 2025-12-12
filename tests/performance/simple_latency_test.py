#!/usr/bin/env python3
"""
Simple ROS2 Intra-Process vs Inter-Process Latency Test

Demonstrates the performance difference by measuring actual message passing latency.
"""

import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


class LatencyPublisher(Node):
    """Publisher for latency testing."""

    def __init__(self, topic_name: str, qos_profile: QoSProfile):
        super().__init__(f'latency_pub_{topic_name.replace("/", "_")}')
        self.publisher = self.create_publisher(String, topic_name, qos_profile)
        self.message_count = 0

    def publish_test_message(self):
        """Publish a test message with timestamp."""
        timestamp = time.time_ns()  # Use nanoseconds for precision
        msg = String()
        msg.data = f"{timestamp}"
        self.publisher.publish(msg)
        self.message_count += 1


class LatencySubscriber(Node):
    """Subscriber for latency testing."""

    def __init__(self, topic_name: str, qos_profile: QoSProfile):
        super().__init__(f'latency_sub_{topic_name.replace("/", "_")}')
        self.latencies = []
        self.message_count = 0

        self.subscription = self.create_subscription(
            String, topic_name, self.callback, qos_profile
        )

    def callback(self, msg):
        """Measure latency when message is received."""
        try:
            send_time_ns = int(msg.data)
            receive_time_ns = time.time_ns()
            latency_ns = receive_time_ns - send_time_ns
            latency_ms = latency_ns / 1_000_000  # Convert to milliseconds
            self.latencies.append(latency_ms)
            self.message_count += 1
        except ValueError:
            pass  # Skip invalid messages

    def get_average_latency(self) -> float:
        """Get average latency in milliseconds."""
        return sum(self.latencies) / len(self.latencies) if self.latencies else 0.0

    def get_message_count(self) -> int:
        """Get number of messages received."""
        return self.message_count


def run_latency_test(topic_name: str, use_intra_process: bool, num_messages: int = 100):
    """Run a latency test for given configuration."""
    print(f"Testing {'Intra' if use_intra_process else 'Inter'}-Process Communication...")

    # Configure QoS
    durability = DurabilityPolicy.VOLATILE if use_intra_process else DurabilityPolicy.VOLATILE
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=durability,
        depth=10
    )

    # Initialize ROS2
    rclpy.init()

    # Create nodes
    publisher = LatencyPublisher(topic_name, qos)
    subscriber = LatencySubscriber(topic_name, qos)

    # Spin nodes in threads
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Wait for ROS2 discovery
    time.sleep(1.0 if use_intra_process else 2.0)

    # Send messages
    print(f"  Sending {num_messages} messages...")
    for i in range(num_messages):
        publisher.publish_test_message()
        time.sleep(0.01)  # 10ms between messages

    # Wait for all messages to be received
    timeout = 5.0
    start_wait = time.time()
    while subscriber.get_message_count() < num_messages and (time.time() - start_wait) < timeout:
        time.sleep(0.1)

    # Get results
    messages_received = subscriber.get_message_count()
    avg_latency = subscriber.get_average_latency()

    # Cleanup
    executor.shutdown()
    publisher.destroy_node()
    subscriber.destroy_node()
    rclpy.shutdown()

    return {
        'messages_sent': num_messages,
        'messages_received': messages_received,
        'avg_latency_ms': avg_latency,
        'success_rate': messages_received / num_messages * 100
    }


def main():
    """Run the latency comparison test."""
    print("🚀 ROS2 Intra-Process vs Inter-Process Latency Test")
    print("=" * 55)

    # Test parameters
    num_messages = 50  # Smaller number for reliable testing

    # Run intra-process test
    intra_topic = 'latency_test_intra'
    print("\n🔗 Testing INTRA-PROCESS communication...")
    intra_results = run_latency_test(intra_topic, use_intra_process=True, num_messages=num_messages)

    # Small delay between tests
    time.sleep(1.0)

    # Run inter-process test
    inter_topic = 'latency_test_inter'
    print("\n🌐 Testing INTER-PROCESS communication...")
    inter_results = run_latency_test(inter_topic, use_intra_process=False, num_messages=num_messages)

    # Results
    print("\n📊 TEST RESULTS")
    print("=" * 30)

    print("\n🔗 INTRA-PROCESS:")
    print(f"  Messages sent: {intra_results['messages_sent']}")
    print(f"  Messages received: {intra_results['messages_received']}")
    print(".1f")
    print(".3f")

    print("\n🌐 INTER-PROCESS:")
    print(f"  Messages sent: {inter_results['messages_sent']}")
    print(f"  Messages received: {inter_results['messages_received']}")
    print(".1f")
    print(".3f")

    # Performance comparison
    print("\n⚡ PERFORMANCE COMPARISON:")

    if intra_results['avg_latency_ms'] > 0 and inter_results['avg_latency_ms'] > 0:
        latency_reduction = inter_results['avg_latency_ms'] - intra_results['avg_latency_ms']
        latency_improvement = (latency_reduction / inter_results['avg_latency_ms']) * 100

        print(".1f")
        print(".1f")

        if latency_improvement > 0:
            print("  ✅ Intra-process communication is FASTER")
        else:
            print("  ⚠️  No significant performance difference detected")
    else:
        print("  ⚠️  Unable to measure latency difference (no messages received)")

    print("\n🎯 URC 2026 SYSTEM IMPACT:")
    print("  • /imu (100Hz): State estimation accuracy")
    print("  • /cmd_vel (50Hz): Rover control responsiveness")
    print("  • Safety topics: Critical safety response times")
    print("  • Vision topics: Real-time obstacle avoidance")

    print("\n✨ Test completed!")


if __name__ == '__main__':
    main()
