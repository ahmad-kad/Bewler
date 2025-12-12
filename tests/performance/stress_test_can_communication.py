#!/usr/bin/env python3
"""
URC 2026 CAN Bus Communication Stress Test

Tests CAN bus communication under extreme conditions harsher than real-world
scenarios, including bus overload, arbitration conflicts, and electrical faults.
"""

import asyncio
import random
import statistics
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from bridges.can_mock_simulator import CANBusMockSimulator


class CANStressLevel(Enum):
    """CAN bus stress test severity levels."""
    MODERATE = "moderate"
    SEVERE = "severe"
    EXTREME = "extreme"


class CANFaultType(Enum):
    """Types of CAN bus faults to simulate."""
    BUS_OVERLOAD = "bus_overload"
    ARBITRATION_COLLISION = "arbitration_collision"
    ELECTRICAL_FAULT = "electrical_fault"
    BUS_OFF = "bus_off"
    ACKNOWLEDGEMENT_ERROR = "acknowledgement_error"
    BIT_ERROR = "bit_error"


@dataclass
class CANStressConfig:
    """Configuration for CAN bus stress testing."""
    stress_level: CANStressLevel
    test_duration: float = 30.0
    message_frequency_hz: float = 100.0
    fault_injection_rate: float = 0.0
    bus_load_percentage: int = 50
    arbitration_conflicts: bool = False
    electrical_noise: bool = False

    def __post_init__(self):
        """Set default parameters based on stress level."""
        if self.stress_level == CANStressLevel.MODERATE:
            self.message_frequency_hz = 500.0
            self.fault_injection_rate = 0.02  # 2% faults
            self.bus_load_percentage = 70
        elif self.stress_level == CANStressLevel.SEVERE:
            self.message_frequency_hz = 1000.0
            self.fault_injection_rate = 0.1  # 10% faults
            self.bus_load_percentage = 90
            self.arbitration_conflicts = True
        elif self.stress_level == CANStressLevel.EXTREME:
            self.message_frequency_hz = 2000.0
            self.fault_injection_rate = 0.3  # 30% faults
            self.bus_load_percentage = 120  # Overload
            self.arbitration_conflicts = True
            self.electrical_noise = True


class CANStressSimulator:
    """Advanced CAN bus simulator with stress testing capabilities."""

    def __init__(self, config: CANStressConfig):
        self.config = config
        self.can_simulator = CANBusMockSimulator()
        self.message_count = 0
        self.error_count = 0
        self.collision_count = 0
        self.bus_off_events = 0
        self.latency_samples = []
        self.is_bus_off = False
        self.bus_recovery_time = 0

        # Stress state tracking
        self.active_faults = []
        self.bus_load_history = []
        self.error_history = []

    def inject_fault(self, fault_type: CANFaultType) -> bool:
        """Inject a specific CAN fault."""
        self.active_faults.append({
            'type': fault_type,
            'timestamp': time.time(),
            'duration': random.uniform(0.1, 2.0)  # Random fault duration
        })

        if fault_type == CANFaultType.BUS_OFF:
            self.is_bus_off = True
            self.bus_off_events += 1
            self.bus_recovery_time = time.time() + random.uniform(0.5, 3.0)
            return True

        return False

    def simulate_arbitration_collision(self) -> bool:
        """Simulate CAN arbitration collision."""
        if not self.config.arbitration_conflicts:
            return False

        # Higher collision probability under load
        collision_prob = min(0.1 + (len(self.bus_load_history) / 100.0), 0.5)
        if random.random() < collision_prob:
            self.collision_count += 1
            return True
        return False

    def apply_electrical_noise(self) -> bool:
        """Simulate electrical noise affecting message integrity."""
        if not self.config.electrical_noise:
            return False

        # Noise causes bit errors
        noise_prob = 0.05 + (self.config.bus_load_percentage / 200.0)
        if random.random() < noise_prob:
            self.error_count += 1
            return True
        return False

    def process_message_with_stress(self, sensor_name: str, message_data: Dict) -> Optional[Dict]:
        """Process a CAN message under stress conditions."""

        # Check if bus is off
        if self.is_bus_off:
            if time.time() > self.bus_recovery_time:
                self.is_bus_off = False
            else:
                return None  # Message lost

        # Simulate arbitration collision
        if self.simulate_arbitration_collision():
            return None  # Message lost due to collision

        # Simulate electrical noise
        if self.apply_electrical_noise():
            # Corrupt the message
            if 'data' in message_data:
                message_data['data'] = "CORRUPTED"
            message_data['error'] = 'electrical_noise'

        # Simulate random faults
        if random.random() < self.config.fault_injection_rate:
            fault_type = random.choice(list(CANFaultType))
            if self.inject_fault(fault_type):
                return None

        # Add latency based on bus load
        base_latency = 0.001  # 1ms base
        load_factor = self.config.bus_load_percentage / 100.0
        latency = base_latency * (1 + load_factor * 2)
        time.sleep(latency)
        self.latency_samples.append(latency * 1000)  # Store in ms

        self.message_count += 1
        return message_data

    def get_stress_stats(self) -> Dict:
        """Get comprehensive stress test statistics."""
        return {
            'messages_processed': self.message_count,
            'errors_total': self.error_count,
            'arbitration_collisions': self.collision_count,
            'bus_off_events': self.bus_off_events,
            'avg_latency_ms': statistics.mean(self.latency_samples) if self.latency_samples else 0,
            'max_latency_ms': max(self.latency_samples) if self.latency_samples else 0,
            'error_rate_percent': (self.error_count / max(self.message_count, 1)) * 100,
            'collision_rate_percent': (self.collision_count / max(self.message_count, 1)) * 100,
            'bus_off_rate': self.bus_off_events / max(self.test_duration, 1),  # per second
            'bus_availability_percent': ((self.config.test_duration - sum(f.get('duration', 0) for f in self.active_faults)) / self.config.test_duration) * 100
        }


def run_can_stress_test_worker(stress_config: CANStressConfig) -> Dict:
    """Worker function to run CAN stress test."""

    simulator = CANStressSimulator(stress_config)

    sensors = ['imu', 'gps', 'battery', 'motor_left', 'motor_right', 'encoder_left', 'encoder_right']
    message_interval = 1.0 / stress_config.message_frequency_hz

    start_time = time.time()

    while time.time() - start_time < stress_config.test_duration:
        # Select random sensor
        sensor = random.choice(sensors)

        # Generate message
        message_data = {
            'sensor': sensor,
            'timestamp': time.time(),
            'data': f"stress_test_data_{random.randint(0, 1000)}",
            'mock': True
        }

        # Process through stress simulator
        result = simulator.process_message_with_stress(sensor, message_data)

        # Small delay between messages
        time.sleep(message_interval)

    # Store test duration for stats
    simulator.test_duration = stress_config.test_duration

    return simulator.get_stress_stats()


def run_can_stress_test_level(level: CANStressLevel) -> Dict:
    """Run CAN stress test for a specific severity level."""

    print(f"🚗 Testing {level.value.upper()} CAN Bus Stress Conditions")
    print("-" * 60)

    # Configure stress test
    config = CANStressConfig(level)

    # Display test parameters
    print("   Test Configuration:")
    print(f"   • Duration: {config.test_duration}s")
    print(f"   • Message Frequency: {config.message_frequency_hz}Hz")
    print(f"   • Fault Injection Rate: {config.fault_injection_rate*100:.1f}%")
    print(f"   • Bus Load: {config.bus_load_percentage}%")
    print(f"   • Arbitration Conflicts: {config.arbitration_conflicts}")
    print(f"   • Electrical Noise: {config.electrical_noise}")

    # Run stress test
    start_time = time.time()
    results = run_can_stress_test_worker(config)
    actual_duration = time.time() - start_time

    print("
   Test Results:")
    print(f"   • Messages Processed: {results['messages_processed']}")
    print(".1f"    print(f"   • Arbitration Collisions: {results['arbitration_collisions']}")
    print(f"   • Bus Off Events: {results['bus_off_events']}")
    print(".1f"    print(".3f"    print(".1f"
    # Performance assessment
    if results['error_rate_percent'] < 5 and results['bus_availability_percent'] > 95:
        assessment = "✅ EXCELLENT - Handles stress well"
    elif results['error_rate_percent'] < 15 and results['bus_availability_percent'] > 85:
        assessment = "⚠️ GOOD - Some issues under stress"
    else:
        assessment = "❌ POOR - Significant problems"

    print(f"   • Assessment: {assessment}")

    results['stress_level'] = level.value
    results['config'] = config
    results['actual_duration'] = actual_duration

    return results


def run_comprehensive_can_stress_test():
    """Run comprehensive CAN bus stress testing across all severity levels."""

    print("🔥 URC 2026 CAN Bus Communication Stress Test Suite")
    print("=" * 65)

    results = {}

    # Test all stress levels
    stress_levels = [CANStressLevel.MODERATE, CANStressLevel.SEVERE, CANStressLevel.EXTREME]

    for level in stress_levels:
        results[level.value] = run_can_stress_test_level(level)
        print()  # Add spacing between tests

    # Comparative analysis
    print("📊 CAN BUS STRESS ANALYSIS")
    print("=" * 35)

    print("
Performance by Stress Level:")
    print("Level       | Error Rate | Bus Avail | Latency | Assessment")
    print("-" * 65)

    for level_name, result in results.items():
        level_short = level_name[:3].upper()
        error_rate = result['error_rate_percent']
        bus_avail = result['bus_availability_percent']
        latency = result['avg_latency_ms']

        if error_rate < 5 and bus_avail > 95:
            assessment = "✅"
        elif error_rate < 15 and bus_avail > 85:
            assessment = "⚠️"
        else:
            assessment = "❌"

        print(f"{level_short:8} | {error_rate:10.1f}% | {bus_avail:9.1f}% | {latency:7.1f}ms | {assessment}")

    # Overall assessment
    extreme_results = results['extreme']

    print("
🎯 CAN BUS STRESS TEST ASSESSMENT")

    if extreme_results['bus_availability_percent'] > 80:
        print("✅ CAN bus maintains good reliability under extreme stress")
    else:
        print("❌ CAN bus reliability degrades significantly under extreme conditions")

    if extreme_results['error_rate_percent'] < 20:
        print("✅ Error handling is effective under high fault conditions")
    else:
        print("⚠️ Error rates may impact system reliability")

    if extreme_results['avg_latency_ms'] < 50:
        print("✅ Latency remains acceptable under stress")
    else:
        print("⚠️ High latency may affect real-time control")

    print("
🛠️ MITIGATION STRATEGIES:")
    print("   • Implement CAN message prioritization")
    print("   • Add redundancy for critical sensors")
    print("   • Use error detection and retransmission")
    print("   • Monitor bus loading and implement rate limiting")
    print("   • Add electrical noise filtering")

    print("
📋 SYSTEM IMPACT:")
    print("   • Motor control reliability under electrical interference")
    print("   • Sensor data integrity during high bus utilization")
    print("   • Emergency stop responsiveness during faults")
    print("   • Navigation accuracy with encoder data corruption")

    return results


def simulate_can_bus_failure_recovery():
    """Test CAN bus failure and recovery scenarios."""

    print("🔄 CAN Bus Failure & Recovery Simulation")
    print("-" * 45)

    config = CANStressConfig(CANStressLevel.SEVERE)
    simulator = CANStressSimulator(config)

    # Simulate bus failure
    print("   Simulating bus failure...")
    simulator.inject_fault(CANFaultType.BUS_OFF)

    # Try to send messages during failure
    messages_during_failure = 0
    messages_lost = 0

    for i in range(50):
        result = simulator.process_message_with_stress('motor_left', {'data': f'test_{i}'})
        if result is None:
            messages_lost += 1
        else:
            messages_during_failure += 1

    recovery_time = simulator.bus_recovery_time - time.time()
    print(f"   Messages sent during failure: {messages_during_failure}")
    print(f"   Messages lost: {messages_lost}")
    print(".1f"
    # Wait for recovery
    if recovery_time > 0:
        print(".1f"        time.sleep(recovery_time + 0.1)

    # Test recovery
    print("   Testing recovery...")
    messages_after_recovery = 0

    for i in range(50):
        result = simulator.process_message_with_stress('motor_left', {'data': f'recovery_{i}'})
        if result is not None:
            messages_after_recovery += 1

    print(f"   Messages after recovery: {messages_after_recovery}")

    recovery_stats = simulator.get_stress_stats()
    print(".1f"
    if messages_after_recovery > 40:  # 80% success rate
        print("   ✅ Bus recovery successful")
    else:
        print("   ❌ Bus recovery failed")

    return recovery_stats


if __name__ == '__main__':
    import os

    # Add path for imports
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

    # Run comprehensive CAN stress tests
    can_results = run_comprehensive_can_stress_test()

    print("
🔄 Testing CAN Bus Failure Recovery...")
    recovery_results = simulate_can_bus_failure_recovery()

    print("
✨ CAN bus stress testing completed!")
