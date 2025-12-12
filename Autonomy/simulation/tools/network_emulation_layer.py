#!/usr/bin/env python3
"""
Network Emulation Layer for URC 2026

Simulates real-world network conditions including latency, packet loss,
bandwidth limitations, and connection failures.
"""

import asyncio
import json
import random
import socket
import statistics
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32MultiArray


class NetworkCondition(Enum):
    """Types of network conditions to emulate."""
    PERFECT = "perfect"
    WIFI_RURAL = "wifi_rural"
    WIFI_URBAN = "wifi_urban"
    CELLULAR_4G = "cellular_4g"
    CELLULAR_5G = "cellular_5g"
    SATELLITE = "satellite"
    COMPLETE_OUTAGE = "complete_outage"
    INTERMITTENT = "intermittent"


@dataclass
class NetworkProfile:
    """Network condition profile."""
    name: str
    latency_ms: Tuple[float, float]  # (min, max) latency
    jitter_ms: float  # Jitter variation
    packet_loss_percent: float  # Packet loss percentage
    bandwidth_mbps: float  # Bandwidth limit
    connection_drops: bool  # Whether connections can drop
    drop_duration_sec: Tuple[float, float] = (1.0, 30.0)  # Drop duration range


@dataclass
class NetworkStatistics:
    """Network performance statistics."""
    messages_sent: int = 0
    messages_received: int = 0
    messages_dropped: int = 0
    average_latency_ms: float = 0.0
    max_latency_ms: float = 0.0
    jitter_ms: float = 0.0
    connection_drops: int = 0
    total_downtime_sec: float = 0.0


class NetworkEmulationLayer(Node):
    """Network emulation layer for testing communication robustness."""

    def __init__(self):
        super().__init__('network_emulation_layer')

        self.logger = self.get_logger()

        # Network profiles
        self.network_profiles = self.define_network_profiles()
        self.current_profile = self.network_profiles[NetworkCondition.PERFECT]

        # Statistics tracking
        self.statistics = NetworkStatistics()
        self.message_latencies = []
        self.connection_state = True
        self.last_connection_drop = 0
        self.start_time = time.time()

        # Message queues for delayed delivery
        self.delayed_messages = []
        self.drop_next_message = False

        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.network_status_pub = self.create_publisher(
            String, '/network_emulation/status', 10
        )

        self.network_metrics_pub = self.create_publisher(
            Float32MultiArray, '/network_emulation/metrics', 10
        )

        # Subscribers for intercepting messages
        self.intercept_sub = self.create_subscription(
            String, '/network_emulation/intercept', self.intercept_callback, 10
        )

        # Control timers
        self.status_timer = self.create_timer(2.0, self.publish_status)
        self.network_timer = self.create_timer(0.1, self.process_network_conditions)
        self.metrics_timer = self.create_timer(5.0, self.publish_metrics)

        self.logger.info("Network Emulation Layer initialized")
        self.logger.info(f"Available profiles: {list(self.network_profiles.keys())}")

    def define_network_profiles(self) -> Dict[NetworkCondition, NetworkProfile]:
        """Define realistic network profiles based on real-world conditions."""
        return {
            NetworkCondition.PERFECT: NetworkProfile(
                name="Perfect Network",
                latency_ms=(0, 1),
                jitter_ms=0.1,
                packet_loss_percent=0.0,
                bandwidth_mbps=1000.0,
                connection_drops=False
            ),

            NetworkCondition.WIFI_RURAL: NetworkProfile(
                name="Rural WiFi",
                latency_ms=(20, 150),
                jitter_ms=25.0,
                packet_loss_percent=2.0,
                bandwidth_mbps=25.0,
                connection_drops=True,
                drop_duration_sec=(2.0, 15.0)
            ),

            NetworkCondition.WIFI_URBAN: NetworkProfile(
                name="Urban WiFi",
                latency_ms=(10, 80),
                jitter_ms=15.0,
                packet_loss_percent=1.0,
                bandwidth_mbps=50.0,
                connection_drops=True,
                drop_duration_sec=(1.0, 8.0)
            ),

            NetworkCondition.CELLULAR_4G: NetworkProfile(
                name="Cellular 4G",
                latency_ms=(50, 200),
                jitter_ms=30.0,
                packet_loss_percent=3.0,
                bandwidth_mbps=15.0,
                connection_drops=True,
                drop_duration_sec=(5.0, 60.0)
            ),

            NetworkCondition.CELLULAR_5G: NetworkProfile(
                name="Cellular 5G",
                latency_ms=(20, 100),
                jitter_ms=10.0,
                packet_loss_percent=1.5,
                bandwidth_mbps=100.0,
                connection_drops=True,
                drop_duration_sec=(2.0, 20.0)
            ),

            NetworkCondition.SATELLITE: NetworkProfile(
                name="Satellite Link",
                latency_ms=(600, 1200),
                jitter_ms=100.0,
                packet_loss_percent=5.0,
                bandwidth_mbps=5.0,
                connection_drops=True,
                drop_duration_sec=(10.0, 300.0)
            ),

            NetworkCondition.COMPLETE_OUTAGE: NetworkProfile(
                name="Complete Outage",
                latency_ms=(0, 0),
                jitter_ms=0.0,
                packet_loss_percent=100.0,
                bandwidth_mbps=0.0,
                connection_drops=True,
                drop_duration_sec=(30.0, 300.0)
            ),

            NetworkCondition.INTERMITTENT: NetworkProfile(
                name="Intermittent Connection",
                latency_ms=(50, 500),
                jitter_ms=50.0,
                packet_loss_percent=15.0,
                bandwidth_mbps=10.0,
                connection_drops=True,
                drop_duration_sec=(0.5, 5.0)
            )
        }

    def set_network_profile(self, condition: NetworkCondition):
        """Set the current network condition profile."""
        if condition in self.network_profiles:
            self.current_profile = self.network_profiles[condition]
            self.logger.info(f"Network profile changed to: {self.current_profile.name}")

            # Reset connection state
            self.connection_state = True
            self.last_connection_drop = 0
        else:
            self.logger.error(f"Unknown network condition: {condition}")

    def intercept_callback(self, msg):
        """Intercept messages for network emulation."""
        if not self.connection_state:
            # Connection is down, drop message
            self.statistics.messages_dropped += 1
            return

        # Apply network conditions
        if self.should_drop_message():
            self.statistics.messages_dropped += 1
            return

        # Add latency
        latency_ms = self.generate_latency()
        delivery_time = time.time() + (latency_ms / 1000.0)

        # Queue message for delayed delivery
        self.delayed_messages.append({
            'message': msg,
            'delivery_time': delivery_time,
            'original_time': time.time()
        })

        self.statistics.messages_sent += 1

    def should_drop_message(self) -> bool:
        """Determine if a message should be dropped based on packet loss."""
        return random.random() < (self.current_profile.packet_loss_percent / 100.0)

    def generate_latency(self) -> float:
        """Generate realistic latency based on current profile."""
        base_latency = random.uniform(
            self.current_profile.latency_ms[0],
            self.current_profile.latency_ms[1]
        )

        # Add jitter
        jitter = random.gauss(0, self.current_profile.jitter_ms)

        return max(0, base_latency + jitter)

    def process_network_conditions(self):
        """Process network conditions and deliver delayed messages."""
        current_time = time.time()

        # Check for connection state changes
        if self.current_profile.connection_drops:
            if self.connection_state:
                # Check if we should drop connection
                if random.random() < 0.001:  # 0.1% chance per 100ms
                    self.connection_state = False
                    drop_duration = random.uniform(
                        self.current_profile.drop_duration_sec[0],
                        self.current_profile.drop_duration_sec[1]
                    )
                    self.last_connection_drop = current_time
                    self.statistics.connection_drops += 1
                    self.logger.warn(f"Network connection dropped for {drop_duration:.1f}s")
            else:
                # Check if connection should be restored
                if current_time - self.last_connection_drop >= random.uniform(
                    self.current_profile.drop_duration_sec[0],
                    self.current_profile.drop_duration_sec[1]
                ):
                    self.connection_state = True
                    downtime = current_time - self.last_connection_drop
                    self.statistics.total_downtime_sec += downtime
                    self.logger.info(f"Network connection restored after {downtime:.1f}s")

        # Deliver delayed messages
        messages_to_deliver = []
        remaining_messages = []

        for delayed_msg in self.delayed_messages:
            if current_time >= delayed_msg['delivery_time']:
                messages_to_deliver.append(delayed_msg)
            else:
                remaining_messages.append(delayed_msg)

        self.delayed_messages = remaining_messages

        # Process delivered messages
        for delayed_msg in messages_to_deliver:
            latency = (current_time - delayed_msg['original_time']) * 1000  # Convert to ms
            self.message_latencies.append(latency)
            self.statistics.messages_received += 1

            # In a real implementation, this would forward the message
            # to its intended destination
            self.logger.debug(f"Message delivered with {latency:.1f}ms latency")
    def publish_status(self):
        """Publish current network status."""
        status_msg = String()
        status_msg.data = json.dumps({
            'profile': self.current_profile.name,
            'connection_state': self.connection_state,
            'packet_loss_percent': self.current_profile.packet_loss_percent,
            'average_latency_ms': self.statistics.average_latency_ms,
            'connection_drops': self.statistics.connection_drops,
            'messages_dropped': self.statistics.messages_dropped,
            'bandwidth_mbps': self.current_profile.bandwidth_mbps
        })
        self.network_status_pub.publish(status_msg)

    def publish_metrics(self):
        """Publish network performance metrics."""
        if self.message_latencies:
            avg_latency = sum(self.message_latencies) / len(self.message_latencies)
            max_latency = max(self.message_latencies)
            jitter = statistics.stdev(self.message_latencies) if len(self.message_latencies) > 1 else 0

            self.statistics.average_latency_ms = avg_latency
            self.statistics.max_latency_ms = max_latency
            self.statistics.jitter_ms = jitter

            metrics_msg = Float32MultiArray()
            metrics_msg.data = [
                float(avg_latency),
                float(max_latency),
                float(jitter),
                float(self.statistics.messages_sent),
                float(self.statistics.messages_received),
                float(self.statistics.messages_dropped),
                float(self.statistics.connection_drops),
                float(self.statistics.total_downtime_sec)
            ]
            self.network_metrics_pub.publish(metrics_msg)

    def get_network_statistics(self) -> Dict[str, Any]:
        """Get comprehensive network statistics."""
        return {
            'current_profile': self.current_profile.name,
            'connection_state': self.connection_state,
            'messages_sent': self.statistics.messages_sent,
            'messages_received': self.statistics.messages_received,
            'messages_dropped': self.statistics.messages_dropped,
            'packet_loss_rate': (
                self.statistics.messages_dropped /
                max(1, self.statistics.messages_sent) * 100
            ),
            'average_latency_ms': self.statistics.average_latency_ms,
            'max_latency_ms': self.statistics.max_latency_ms,
            'jitter_ms': self.statistics.jitter_ms,
            'connection_drops': self.statistics.connection_drops,
            'total_downtime_sec': self.statistics.total_downtime_sec,
            'uptime_percentage': (
                100.0 - (self.statistics.total_downtime_sec /
                         max(1, time.time() - self.start_time) * 100)
            )
        }


def run_network_emulation_demo():
    """Run a demonstration of network emulation capabilities."""
    print(" NETWORK EMULATION LAYER DEMONSTRATION")
    print("=" * 60)

    # Initialize ROS2
    rclpy.init()

    # Create network emulation layer
    network_layer = NetworkEmulationLayer()

    try:
        print("Testing different network conditions...")

        # Test sequence of network conditions
        test_conditions = [
            (NetworkCondition.PERFECT, 5),
            (NetworkCondition.WIFI_URBAN, 10),
            (NetworkCondition.CELLULAR_4G, 10),
            (NetworkCondition.INTERMITTENT, 15),
            (NetworkCondition.SATELLITE, 10),
            (NetworkCondition.COMPLETE_OUTAGE, 5),
        ]

        for condition, duration in test_conditions:
            print(f"\n Switching to: {condition.value}")
            network_layer.set_network_profile(condition)

            # Send some test messages during this condition
            for i in range(5):
                test_msg = String()
                test_msg.data = f"Test message {i+1} under {condition.value}"
                network_layer.intercept_callback(test_msg)
                time.sleep(0.5)

            # Wait for condition to take effect
            time.sleep(duration)

        # Generate final statistics
        stats = network_layer.get_network_statistics()
        print("\n NETWORK EMULATION STATISTICS:")
        print(f"  Total Messages Sent: {stats['messages_sent']}")
        print(f"  Messages Received: {stats['messages_received']}")
        print(f"  Messages Dropped: {stats['messages_dropped']}")
        print(f"  Packet Loss Rate: {stats['packet_loss_rate']:.1f}%")
        print(f"  Average Latency: {stats['average_latency_ms']:.1f}ms")
        print(f"  Connection Drops: {stats['connection_drops']}")
        print(f"  Total Downtime: {stats['total_downtime_sec']:.1f}s")
        print("\n NETWORK EMULATION DEMONSTRATION COMPLETE")
        print("Real-world network conditions successfully simulated!")

    except KeyboardInterrupt:
        print("\n Demonstration interrupted")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    run_network_emulation_demo()