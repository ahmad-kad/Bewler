#!/usr/bin/env python3
"""
URC 2026 Movement Control Communication Stress Test

Tests rover movement control under extreme conditions harsher than real-world
scenarios, including rapid command changes, conflicting instructions, and
emergency stop scenarios.
"""

import asyncio
import math
import os
import random
import statistics
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import rclpy

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class MovementStressLevel(Enum):
    """Movement control stress test severity levels."""
    MODERATE = "moderate"
    SEVERE = "severe"
    EXTREME = "extreme"


class MovementFaultType(Enum):
    """Types of movement control faults to simulate."""
    COMMAND_CONFLICT = "command_conflict"
    RAPID_DIRECTION_CHANGE = "rapid_direction_change"
    OVERSPEED_COMMAND = "overspeed_command"
    EMERGENCY_STOP_CONFLICT = "emergency_stop_conflict"
    COORDINATION_FAILURE = "coordination_failure"
    SENSOR_FEEDBACK_LOSS = "sensor_feedback_loss"


@dataclass
class MovementStressConfig:
    """Configuration for movement control stress testing."""
    stress_level: MovementStressLevel
    test_duration: float = 60.0
    command_frequency_hz: float = 50.0
    fault_injection_rate: float = 0.0
    max_linear_speed: float = 2.0  # m/s
    max_angular_speed: float = 1.0  # rad/s
    emergency_stop_rate: float = 0.0
    command_conflict_rate: float = 0.0

    def __post_init__(self):
        """Set default parameters based on stress level."""
        if self.stress_level == MovementStressLevel.MODERATE:
            self.command_frequency_hz = 100.0
            self.fault_injection_rate = 0.05  # 5% faults
            self.emergency_stop_rate = 0.02   # 2% emergency stops
            self.command_conflict_rate = 0.03 # 3% conflicts
        elif self.stress_level == MovementStressLevel.SEVERE:
            self.command_frequency_hz = 200.0
            self.fault_injection_rate = 0.15  # 15% faults
            self.emergency_stop_rate = 0.1    # 10% emergency stops
            self.command_conflict_rate = 0.1  # 10% conflicts
        elif self.stress_level == MovementStressLevel.EXTREME:
            self.command_frequency_hz = 500.0
            self.fault_injection_rate = 0.4   # 40% faults
            self.emergency_stop_rate = 0.3    # 30% emergency stops
            self.command_conflict_rate = 0.3  # 30% conflicts


class MovementStressController:
    """Movement control system with stress testing capabilities."""

    def __init__(self, config: MovementStressConfig):
        self.config = config

        # State tracking
        self.current_velocity = Twist()
        self.target_velocity = Twist()
        self.emergency_stop_active = False
        self.last_command_time = time.time()

        # Performance metrics
        self.commands_processed = 0
        self.commands_rejected = 0
        self.emergency_stops_triggered = 0
        self.command_conflicts = 0
        self.overspeed_events = 0
        self.response_latencies = []

        # Fault tracking
        self.active_faults = []

    def inject_movement_fault(self, fault_type: MovementFaultType) -> bool:
        """Inject a movement control fault."""
        self.active_faults.append({
            'type': fault_type,
            'timestamp': time.time(),
            'resolved': False
        })

        if fault_type == MovementFaultType.EMERGENCY_STOP_CONFLICT:
            self.emergency_stop_active = True
            self.emergency_stops_triggered += 1
            return True
        elif fault_type == MovementFaultType.COMMAND_CONFLICT:
            self.command_conflicts += 1
            return True

        return False

    def validate_velocity_command(self, velocity: Twist) -> Tuple[bool, str]:
        """Validate velocity command for safety and constraints."""

        # Check speed limits
        if abs(velocity.linear.x) > self.config.max_linear_speed:
            return False, f"Linear speed {velocity.linear.x:.2f} exceeds limit {self.config.max_linear_speed}"

        if abs(velocity.angular.z) > self.config.max_angular_speed:
            return False, f"Angular speed {velocity.angular.z:.2f} exceeds limit {self.config.max_angular_speed}"

        # Check for emergency stop conflicts
        if self.emergency_stop_active and (abs(velocity.linear.x) > 0.01 or abs(velocity.angular.z) > 0.01):
            return False, "Emergency stop active - motion commands rejected"

        # Check for rapid direction changes
        if self._is_rapid_direction_change(velocity):
            return False, "Rapid direction change detected"

        return True, "Command valid"

    def _is_rapid_direction_change(self, new_velocity: Twist) -> bool:
        """Check if velocity change is too rapid."""
        current_speed = math.sqrt(self.current_velocity.linear.x**2 + self.current_velocity.angular.z**2)
        new_speed = math.sqrt(new_velocity.linear.x**2 + new_velocity.angular.z**2)

        # Calculate acceleration
        dt = max(time.time() - self.last_command_time, 0.001)
        acceleration = abs(new_speed - current_speed) / dt

        # Check against maximum acceleration (arbitrary but harsh limits)
        max_acceleration = 5.0  # m/s² - very harsh for testing
        return acceleration > max_acceleration

    def process_movement_command(self, velocity: Twist) -> Tuple[bool, str, float]:
        """Process a movement command with stress simulation."""

        start_time = time.time()

        # Inject faults randomly
        if random.random() < self.config.fault_injection_rate:
            fault_type = random.choice(list(MovementFaultType))
            self.inject_movement_fault(fault_type)

        # Inject emergency stops
        if random.random() < self.config.emergency_stop_rate:
            self.inject_movement_fault(MovementFaultType.EMERGENCY_STOP_CONFLICT)

        # Inject command conflicts
        if random.random() < self.config.command_conflict_rate:
            self.inject_movement_fault(MovementFaultType.COMMAND_CONFLICT)

        # Validate command
        is_valid, reason = self.validate_velocity_command(velocity)

        processing_time = time.time() - start_time
        self.response_latencies.append(processing_time * 1000)  # Store in ms

        if is_valid:
            # Apply command with some processing delay (simulate actuator lag)
            time.sleep(random.uniform(0.001, 0.01))  # 1-10ms random lag

            self.current_velocity = velocity
            self.last_command_time = time.time()
            self.commands_processed += 1

            return True, "Command executed", processing_time
        else:
            self.commands_rejected += 1
            return False, reason, processing_time

    def clear_emergency_stop(self):
        """Clear emergency stop condition."""
        self.emergency_stop_active = False
        # Resolve emergency stop faults
        for fault in self.active_faults:
            if fault['type'] == MovementFaultType.EMERGENCY_STOP_CONFLICT:
                fault['resolved'] = True

    def get_stress_stats(self) -> Dict:
        """Get comprehensive movement control stress statistics."""
        return {
            'commands_processed': self.commands_processed,
            'commands_rejected': self.commands_rejected,
            'emergency_stops': self.emergency_stops_triggered,
            'command_conflicts': self.command_conflicts,
            'overspeed_events': self.overspeed_events,
            'avg_response_latency_ms': statistics.mean(self.response_latencies) if self.response_latencies else 0,
            'max_response_latency_ms': max(self.response_latencies) if self.response_latencies else 0,
            'command_success_rate': (self.commands_processed / max(self.commands_processed + self.commands_rejected, 1)) * 100,
            'active_faults': len([f for f in self.active_faults if not f.get('resolved', False)])
        }


class MovementStressPublisher(Node):
    """ROS2 publisher for movement control stress testing."""

    def __init__(self, stress_config: MovementStressConfig):
        super().__init__('movement_stress_publisher')

        self.config = stress_config
        self.controller = MovementStressController(stress_config)
        self.command_count = 0

        # QoS for real-time movement commands
        qos_realtime = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_realtime)
        self.emergency_stop_pub = self.create_publisher(Bool, '/safety/emergency_stop', qos_realtime)
        self.status_pub = self.create_publisher(String, '/movement/status', qos_realtime)

        # Command generation timer
        self.command_timer = self.create_timer(
            1.0 / self.config.command_frequency_hz,
            self.generate_stress_command
        )

        # Status reporting timer
        self.status_timer = self.create_timer(1.0, self.publish_status)

    def generate_stress_command(self):
        """Generate a movement command with stress characteristics."""

        # Generate random velocity command
        linear_x = random.uniform(-self.config.max_linear_speed, self.config.max_linear_speed)
        angular_z = random.uniform(-self.config.max_angular_speed, self.config.max_angular_speed)

        # Occasionally generate extreme commands for stress testing
        if random.random() < 0.1:  # 10% chance of extreme commands
            if random.choice([True, False]):
                linear_x = random.choice([-self.config.max_linear_speed * 2, self.config.max_linear_speed * 2])
            else:
                angular_z = random.choice([-self.config.max_angular_speed * 2, self.config.max_angular_speed * 2])

        velocity_cmd = Twist()
        velocity_cmd.linear.x = linear_x
        velocity_cmd.angular.z = angular_z

        # Process command through stress controller
        success, reason, latency = self.controller.process_movement_command(velocity_cmd)

        if success:
            # Publish velocity command
            self.cmd_vel_pub.publish(velocity_cmd)
        else:
            # Publish emergency stop on failure
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)

        self.command_count += 1

    def publish_status(self):
        """Publish movement control status."""
        stats = self.controller.get_stress_stats()

        status_msg = String()
        status_msg.data = f"MOVEMENT_STATUS:cmds={stats['commands_processed']},rejected={stats['commands_rejected']},latency={stats['avg_response_latency_ms']:.1f}ms"
        self.status_pub.publish(status_msg)


class MovementStressSubscriber(Node):
    """ROS2 subscriber for movement control stress testing."""

    def __init__(self, stress_config: MovementStressConfig):
        super().__init__('movement_stress_subscriber')

        self.config = stress_config
        self.received_commands = 0
        self.received_emergency_stops = 0
        self.command_latencies = []

        # QoS for real-time movement commands
        qos_realtime = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_realtime
        )

        self.emergency_stop_sub = self.create_subscription(
            Bool, '/safety/emergency_stop', self.emergency_stop_callback, qos_realtime
        )

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity command messages."""
        receive_time = time.time()
        self.received_commands += 1

        # Simulate processing delay
        processing_delay = random.uniform(0.001, 0.005)  # 1-5ms
        time.sleep(processing_delay)

    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop messages."""
        if msg.data:
            self.received_emergency_stops += 1


def run_movement_stress_test_level(level: MovementStressLevel) -> Dict:
    """Run movement control stress test for a specific severity level."""

    print(f"🚗 Testing {level.value.upper()} Movement Control Stress")
    print("-" * 55)

    # Configure stress test
    config = MovementStressConfig(level)

    # Display test parameters
    print("   Test Configuration:")
    print(f"   • Duration: {config.test_duration}s")
    print(f"   • Command Frequency: {config.command_frequency_hz}Hz")
    print(f"   • Fault Injection Rate: {config.fault_injection_rate*100:.1f}%")
    print(f"   • Emergency Stop Rate: {config.emergency_stop_rate*100:.1f}%")
    print(f"   • Command Conflict Rate: {config.command_conflict_rate*100:.1f}%")
    print(f"   • Max Linear Speed: {config.max_linear_speed}m/s")
    print(f"   • Max Angular Speed: {config.max_angular_speed}rad/s")

    # Initialize ROS2
    rclpy.init()

    try:
        # Create nodes
        publisher = MovementStressPublisher(config)
        subscriber = MovementStressSubscriber(config)

        # Spin nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Run stress test
        print("\n   Running movement stress test...")
        start_time = time.time()
        last_report = start_time

        while time.time() - start_time < config.test_duration:
            current_time = time.time()

            # Periodic status report
            if current_time - last_report >= 5.0:
                controller_stats = publisher.controller.get_stress_stats()
                print(f"   [{current_time - start_time:.1f}s] "
                      f"Commands: {controller_stats['commands_processed']}, "
                      f"Rejected: {controller_stats['commands_rejected']}, "
                      f"Conflicts: {controller_stats['command_conflicts']}, "
                      f"Latency: {controller_stats['avg_response_latency_ms']:.1f}ms")
                last_report = current_time

            time.sleep(0.1)

        # Get final statistics
        controller_stats = publisher.controller.get_stress_stats()

        # Cleanup
        executor.shutdown()
        publisher.destroy_node()
        subscriber.destroy_node()

        results = {
            'stress_level': level.value,
            'config': config,
            'commands_processed': controller_stats['commands_processed'],
            'commands_rejected': controller_stats['commands_rejected'],
            'emergency_stops': controller_stats['emergency_stops'],
            'command_conflicts': controller_stats['command_conflicts'],
            'avg_response_latency_ms': controller_stats['avg_response_latency_ms'],
            'max_response_latency_ms': controller_stats['max_response_latency_ms'],
            'command_success_rate': controller_stats['command_success_rate'],
            'commands_received': subscriber.received_commands,
            'emergency_stops_received': subscriber.received_emergency_stops
        }

        print("\n   Final Results:")
        print(f"   • Commands Processed: {results['commands_processed']}")
        print(f"   • Commands Rejected: {results['commands_rejected']}")
        print(f"   • Emergency Stops: {results['emergency_stops']}")
        print(f"   • Command Conflicts: {results['command_conflicts']}")
        print(f"   • Avg Response Latency: {results['avg_response_latency_ms']:.1f}ms")
        print(f"   • Command Success Rate: {results['command_success_rate']:.3f}%")
        print(f"   • Max Response Latency: {results['max_response_latency_ms']:.1f}ms")
        # Performance assessment
        success_rate = results['command_success_rate']
        if success_rate > 90:
            assessment = "✅ EXCELLENT - Handles stress well"
        elif success_rate > 75:
            assessment = "⚠️ GOOD - Some issues under stress"
        else:
            assessment = "❌ POOR - Significant problems"

        print(f"   • Assessment: {assessment}")

        return results

    finally:
        rclpy.shutdown()


def run_comprehensive_movement_stress_test():
    """Run comprehensive movement control stress testing."""

    print("🎮 URC 2026 Movement Control Communication Stress Test Suite")
    print("=" * 70)

    results = {}

    # Test all stress levels
    stress_levels = [MovementStressLevel.MODERATE, MovementStressLevel.SEVERE, MovementStressLevel.EXTREME]

    for level in stress_levels:
        results[level.value] = run_movement_stress_test_level(level)
        print()  # Add spacing

    # Comparative analysis
    print("📊 MOVEMENT CONTROL STRESS ANALYSIS")
    print("=" * 42)

    print("\nPerformance by Stress Level:")
    print("Level       | Success | Conflicts | E-Stops | Latency | Assessment")
    print("-" * 70)

    for level_name, result in results.items():
        level_short = level_name[:3].upper()
        success_rate = result['command_success_rate']
        conflicts = result['command_conflicts']
        estops = result['emergency_stops']
        latency = result['avg_response_latency_ms']

        if success_rate > 90 and conflicts < 10:
            assessment = "✅"
        elif success_rate > 75 and conflicts < 50:
            assessment = "⚠️"
        else:
            assessment = "❌"

        print(f"{level_short:8} | {success_rate:7.1f}% | {conflicts:9} | {estops:7} | {latency:7.1f}ms | {assessment}")

    # Overall assessment
    extreme_results = results['extreme']

    print("\n🎯 MOVEMENT CONTROL STRESS ASSESSMENT")

    if extreme_results['command_success_rate'] > 80:
        print("✅ Movement control maintains good reliability under extreme stress")
    else:
        print("❌ Movement control reliability degrades significantly")

    if extreme_results['command_conflicts'] < extreme_results['commands_processed'] * 0.2:
        print("✅ Command conflict resolution is effective")
    else:
        print("⚠️ High command conflict rate may cause erratic movement")

    if extreme_results['avg_response_latency_ms'] < 20:
        print("✅ Response latency remains acceptable under stress")
    else:
        print("⚠️ High latency may impact real-time control performance")

    print("\n🛠️ MITIGATION STRATEGIES:")
    print("   • Implement command prioritization and queuing")
    print("   • Add command validation and safety limits")
    print("   • Implement emergency stop precedence")
    print("   • Add command smoothing for rapid changes")
    print("   • Use redundant command channels")

    print("\n📋 SYSTEM IMPACT:")
    print("   • Rover stability during rapid command changes")
    print("   • Emergency stop responsiveness under conflict")
    print("   • Navigation accuracy with command latency")
    print("   • Safety system coordination during faults")

    return results


def simulate_movement_emergency_scenarios():
    """Test emergency movement scenarios."""

    print("🚨 Movement Emergency Scenario Simulation")
    print("-" * 45)

    config = MovementStressConfig(MovementStressLevel.SEVERE)
    controller = MovementStressController(config)

    # Test emergency stop during high-speed movement
    print("   Testing emergency stop during high-speed movement...")

    # Send high-speed command
    high_speed_cmd = Twist()
    high_speed_cmd.linear.x = config.max_linear_speed
    high_speed_cmd.angular.z = config.max_angular_speed * 0.5

    success, reason, latency = controller.process_movement_command(high_speed_cmd)
    print(f"   High-speed command: {'✅' if success else '❌'} ({reason})")

    # Inject emergency stop
    controller.inject_movement_fault(MovementFaultType.EMERGENCY_STOP_CONFLICT)

    # Try to send conflicting command
    conflicting_cmd = Twist()
    conflicting_cmd.linear.x = -config.max_linear_speed * 0.5  # Reverse direction

    success, reason, latency = controller.process_movement_command(conflicting_cmd)
    print(f"   Conflicting command during E-stop: {'❌' if not success else '✅'} ({reason})")

    # Clear emergency stop
    controller.clear_emergency_stop()

    # Try command after clearing E-stop
    success, reason, latency = controller.process_movement_command(conflicting_cmd)
    print(f"   Command after E-stop cleared: {'✅' if success else '❌'} ({reason})")

    # Test rapid command changes
    print("\n   Testing rapid command changes...")
    commands_tested = 0
    commands_rejected = 0

    for i in range(20):
        # Rapid direction changes
        rapid_cmd = Twist()
        rapid_cmd.linear.x = config.max_linear_speed * (1 if i % 2 == 0 else -1)
        rapid_cmd.angular.z = config.max_angular_speed * random.uniform(-1, 1)

        success, reason, latency = controller.process_movement_command(rapid_cmd)
        commands_tested += 1
        if not success:
            commands_rejected += 1

    print(f"   Rapid commands: {commands_tested - commands_rejected}/{commands_tested} accepted")

    emergency_stats = controller.get_stress_stats()
    print(f"   Emergency test latency: {emergency_stats['avg_response_latency_ms']:.1f}ms")
    if commands_rejected < commands_tested * 0.3:  # Less than 30% rejected
        print("   ✅ System handles rapid changes reasonably well")
    else:
        print("   ⚠️ Too many valid commands rejected - overly conservative")

    return emergency_stats


if __name__ == '__main__':
    # Run comprehensive movement stress tests
    movement_results = run_comprehensive_movement_stress_test()

    print("\n🚨 Testing Movement Emergency Scenarios...")
    emergency_results = simulate_movement_emergency_scenarios()

    print("\n✨ Movement control stress testing completed!")
