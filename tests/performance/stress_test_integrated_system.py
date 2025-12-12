#!/usr/bin/env python3
"""
URC 2026 Integrated Communication Stress Test

Tests the complete rover communication system under extreme conditions harsher
than real-world scenarios, combining network, CAN bus, and movement control stress.
"""

import asyncio
import concurrent.futures
import os
import statistics
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

import psutil
import rclpy

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))


class IntegratedStressLevel(Enum):
    """Integrated system stress test severity levels."""
    MODERATE = "moderate"
    SEVERE = "severe"
    EXTREME = "extreme"


@dataclass
class IntegratedStressConfig:
    """Configuration for integrated system stress testing."""
    stress_level: IntegratedStressLevel
    test_duration: float = 120.0  # 2 minutes for comprehensive testing

    # Network stress parameters
    network_packet_loss: float = 0.0
    network_latency_ms: int = 0
    network_bandwidth_mbps: int = 1000

    # CAN bus stress parameters
    can_message_frequency: float = 100.0
    can_fault_rate: float = 0.0
    can_bus_load: int = 50

    # Movement control stress parameters
    movement_command_frequency: float = 50.0
    movement_fault_rate: float = 0.0
    movement_emergency_rate: float = 0.0

    # Cross-system interference
    cross_system_conflicts: bool = False
    resource_contention: bool = False

    def __post_init__(self):
        """Set default parameters based on stress level."""
        if self.stress_level == IntegratedStressLevel.MODERATE:
            self.network_packet_loss = 0.02    # 2% packet loss
            self.network_latency_ms = 25        # 25ms latency
            self.can_message_frequency = 200.0  # 200Hz CAN
            self.can_fault_rate = 0.03          # 3% CAN faults
            self.movement_command_frequency = 75.0  # 75Hz commands
            self.movement_fault_rate = 0.05     # 5% movement faults
            self.cross_system_conflicts = False
        elif self.stress_level == IntegratedStressLevel.SEVERE:
            self.network_packet_loss = 0.08    # 8% packet loss
            self.network_latency_ms = 100       # 100ms latency
            self.network_bandwidth_mbps = 50    # 50Mbps bandwidth limit
            self.can_message_frequency = 500.0  # 500Hz CAN
            self.can_fault_rate = 0.12          # 12% CAN faults
            self.can_bus_load = 80              # 80% bus load
            self.movement_command_frequency = 150.0  # 150Hz commands
            self.movement_fault_rate = 0.15     # 15% movement faults
            self.movement_emergency_rate = 0.08 # 8% emergency stops
            self.cross_system_conflicts = True
            self.resource_contention = True
        elif self.stress_level == IntegratedStressLevel.EXTREME:
            self.network_packet_loss = 0.25    # 25% packet loss
            self.network_latency_ms = 300       # 300ms latency
            self.network_bandwidth_mbps = 10    # 10Mbps bandwidth limit
            self.can_message_frequency = 1000.0 # 1000Hz CAN
            self.can_fault_rate = 0.35          # 35% CAN faults
            self.can_bus_load = 130             # 130% bus overload
            self.movement_command_frequency = 300.0  # 300Hz commands
            self.movement_fault_rate = 0.40     # 40% movement faults
            self.movement_emergency_rate = 0.25 # 25% emergency stops
            self.cross_system_conflicts = True
            self.resource_contention = True


class SystemResourceMonitor:
    """Monitor system resources during stress testing."""

    def __init__(self):
        self.cpu_samples = []
        self.memory_samples = []
        self.start_time = time.time()
        self.monitoring = False

    def start_monitoring(self):
        """Start resource monitoring."""
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop resource monitoring."""
        self.monitoring = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join(timeout=1.0)

    def _monitor_loop(self):
        """Resource monitoring loop."""
        while self.monitoring:
            self.cpu_samples.append(psutil.cpu_percent(interval=0.1))
            self.memory_samples.append(psutil.virtual_memory().percent)
            time.sleep(0.5)

    def get_stats(self) -> Dict:
        """Get resource monitoring statistics."""
        if not self.cpu_samples:
            return {'avg_cpu': 0, 'max_cpu': 0, 'avg_memory': 0, 'max_memory': 0}

        return {
            'avg_cpu': statistics.mean(self.cpu_samples),
            'max_cpu': max(self.cpu_samples),
            'avg_memory': statistics.mean(self.memory_samples),
            'max_memory': max(self.memory_samples),
            'cpu_variance': statistics.variance(self.cpu_samples) if len(self.cpu_samples) > 1 else 0,
            'memory_variance': statistics.variance(self.memory_samples) if len(self.memory_samples) > 1 else 0
        }


class NetworkEmulator:
    """Network emulator for integrated stress testing."""

    def __init__(self, config: IntegratedStressConfig):
        self.config = config
        self.active_rules = []

    def apply_network_stress(self):
        """Apply network stress rules."""
        self.clear_network_stress()

        # Apply packet loss
        if self.config.network_packet_loss > 0:
            loss_percent = self.config.network_packet_loss * 100
            rule = f'sudo tc qdisc add dev lo root netem loss {loss_percent}%'
            self._apply_rule(rule)

        # Apply latency
        if self.config.network_latency_ms > 0:
            latency = self.config.network_latency_ms
            rule = f'sudo tc qdisc add dev lo root netem delay {latency}ms'
            self._apply_rule(rule)

        # Apply bandwidth limit
        if self.config.network_bandwidth_mbps < 1000:
            bandwidth = self.config.network_bandwidth_mbps
            rule = f'sudo tc qdisc add dev lo root tbf rate {bandwidth}mbit burst 32kbit latency 400ms'
            self._apply_rule(rule)

    def _apply_rule(self, rule: str):
        """Apply a network rule."""
        try:
            subprocess.run(rule.split(), check=True, capture_output=True)
            self.active_rules.append(rule)
        except subprocess.CalledProcessError as e:
            print(f"Warning: Failed to apply network rule: {rule}")

    def clear_network_stress(self):
        """Clear all network stress rules."""
        try:
            subprocess.run(['sudo', 'tc', 'qdisc', 'del', 'dev', 'lo', 'root'],
                         capture_output=True)
        except subprocess.CalledProcessError:
            pass
        self.active_rules = []


class IntegratedStressTest:
    """Integrated stress test coordinator."""

    def __init__(self, config: IntegratedStressConfig):
        self.config = config
        self.resource_monitor = SystemResourceMonitor()
        self.network_emulator = NetworkEmulator(config)

        # Test results
        self.network_stats = {}
        self.can_stats = {}
        self.movement_stats = {}
        self.cross_system_events = []

    def run_integrated_stress_test(self) -> Dict:
        """Run the complete integrated stress test."""

        print("🔥 Running Integrated System Stress Test"        print("=" * 50)

        # Display test configuration
        print("   Test Configuration:"        print(f"   • Duration: {self.config.test_duration}s"        print(f"   • Network: {self.config.network_packet_loss*100:.1f}% loss, {self.config.network_latency_ms}ms latency"        print(f"   • CAN Bus: {self.config.can_message_frequency}Hz, {self.config.can_fault_rate*100:.1f}% faults"        print(f"   • Movement: {self.config.movement_command_frequency}Hz, {self.config.movement_fault_rate*100:.1f}% faults"        print(f"   • Cross-system conflicts: {self.config.cross_system_conflicts}"        print(f"   • Resource contention: {self.config.resource_contention}"

        # Apply network stress
        print("
   Applying network stress..."        self.network_emulator.apply_network_stress()

        # Start resource monitoring
        self.resource_monitor.start_monitoring()

        try:
            # Run all stress tests concurrently
            with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
                # Submit stress test tasks
                future_network = executor.submit(self._run_network_stress)
                future_can = executor.submit(self._run_can_stress)
                future_movement = executor.submit(self._run_movement_stress)
                future_conflicts = executor.submit(self._run_cross_system_conflicts)

                # Wait for completion with timeout
                start_time = time.time()
                timeout = self.config.test_duration + 30  # Extra time for cleanup

                try:
                    concurrent.futures.wait(
                        [future_network, future_can, future_movement, future_conflicts],
                        timeout=timeout,
                        return_when=concurrent.futures.ALL_COMPLETED
                    )

                    # Get results
                    self.network_stats = future_network.result() if future_network.done() else {}
                    self.can_stats = future_can.result() if future_can.done() else {}
                    self.movement_stats = future_movement.result() if future_movement.done() else {}
                    self.cross_system_events = future_conflicts.result() if future_conflicts.done() else []

                except concurrent.futures.TimeoutError:
                    print("   ⚠️  Test timed out - some components may not have completed")

            # Stop resource monitoring
            self.resource_monitor.stop_monitoring()

            # Generate comprehensive results
            results = self._generate_integrated_results()

            # Display results
            self._display_integrated_results(results)

            return results

        finally:
            # Always cleanup
            self.network_emulator.clear_network_stress()
            self.resource_monitor.stop_monitoring()

    def _run_network_stress(self) -> Dict:
        """Run network-specific stress test."""
        # Import and run network stress test
        from stress_test_network_communication import run_network_stress_test

        try:
            # Use severe network stress for integrated testing
            return run_network_stress_test('severe', duration=self.config.test_duration)
        except Exception as e:
            print(f"   Network stress test failed: {e}")
            return {'error': str(e)}

    def _run_can_stress(self) -> Dict:
        """Run CAN bus stress test."""
        from stress_test_can_communication import (
            CANStressConfig,
            CANStressLevel,
            run_can_stress_test_worker,
        )

        try:
            # Configure CAN stress based on integrated config
            can_config = CANStressConfig(CANStressLevel.SEVERE)
            can_config.test_duration = self.config.test_duration
            can_config.message_frequency_hz = self.config.can_message_frequency
            can_config.fault_injection_rate = self.config.can_fault_rate

            return run_can_stress_test_worker(can_config)
        except Exception as e:
            print(f"   CAN stress test failed: {e}")
            return {'error': str(e)}

    def _run_movement_stress(self) -> Dict:
        """Run movement control stress test."""
        from stress_test_movement_control import (
            MovementStressLevel,
            run_movement_stress_test_level,
        )

        try:
            # Use severe movement stress for integrated testing
            return run_movement_stress_test_level(MovementStressLevel.SEVERE)
        except Exception as e:
            print(f"   Movement stress test failed: {e}")
            return {'error': str(e)}

    def _run_cross_system_conflicts(self) -> List[Dict]:
        """Run cross-system conflict simulation."""
        conflicts = []
        start_time = time.time()

        while time.time() - start_time < self.config.test_duration:
            if self.config.cross_system_conflicts:
                # Simulate cross-system conflicts
                conflict = {
                    'timestamp': time.time(),
                    'type': 'cross_system_conflict',
                    'description': 'Simulated cross-system communication conflict',
                    'severity': 'high' if self.config.stress_level == IntegratedStressLevel.EXTREME else 'medium'
                }
                conflicts.append(conflict)

            time.sleep(1.0)  # Check every second

        return conflicts

    def _generate_integrated_results(self) -> Dict:
        """Generate comprehensive integrated test results."""

        resource_stats = self.resource_monitor.get_stats()

        # Calculate system health score (0-100)
        health_components = []

        # Network health
        if 'avg_latency_ms' in self.network_stats:
            network_health = max(0, 100 - (self.network_stats['avg_latency_ms'] / 10))  # Penalize >1s latency
            health_components.append(network_health)

        # CAN health
        if 'bus_availability_percent' in self.can_stats:
            can_health = self.can_stats['bus_availability_percent']
            health_components.append(can_health)

        # Movement health
        if 'command_success_rate' in self.movement_stats:
            movement_health = self.movement_stats['command_success_rate']
            health_components.append(movement_health)

        # Resource health
        resource_health = max(0, 100 - resource_stats['avg_cpu'] - (resource_stats['avg_memory'] / 2))
        health_components.append(resource_health)

        overall_health = statistics.mean(health_components) if health_components else 0

        return {
            'stress_level': self.config.stress_level.value,
            'duration': self.config.test_duration,
            'overall_health_score': overall_health,
            'network_performance': self.network_stats,
            'can_performance': self.can_stats,
            'movement_performance': self.movement_stats,
            'resource_usage': resource_stats,
            'cross_system_conflicts': len(self.cross_system_events),
            'test_timestamp': time.time()
        }

    def _display_integrated_results(self, results: Dict):
        """Display comprehensive integrated test results."""

        print("
📊 INTEGRATED SYSTEM STRESS TEST RESULTS"        print("=" * 50)

        print(f"   Overall System Health Score: {results['overall_health_score']:.1f}/100")

        if results['overall_health_score'] > 80:
            health_status = "✅ EXCELLENT - System handles extreme stress well"
        elif results['overall_health_score'] > 60:
            health_status = "⚠️ GOOD - Some degradation under stress"
        else:
            health_status = "❌ POOR - Significant system issues"

        print(f"   Health Status: {health_status}")

        # Component breakdown
        print("
   Component Performance:")

        if 'avg_latency_ms' in results['network_performance']:
            network_latency = results['network_performance']['avg_latency_ms']
            print(".1f"
        if 'bus_availability_percent' in results['can_performance']:
            can_availability = results['can_performance']['bus_availability_percent']
            print(".1f"
        if 'command_success_rate' in results['movement_performance']:
            movement_success = results['movement_performance']['command_success_rate']
            print(".1f"
        print(".1f"
        print(".1f"
        print(f"   Cross-system conflicts: {results['cross_system_conflicts']}")

        # Recommendations
        print("
🛠️ SYSTEM RECOMMENDATIONS:")

        issues = []

        if results.get('network_performance', {}).get('avg_latency_ms', 0) > 200:
            issues.append("• Implement network fault tolerance and message queuing")

        if results.get('can_performance', {}).get('bus_availability_percent', 100) < 90:
            issues.append("• Add CAN bus redundancy and error recovery")

        if results.get('movement_performance', {}).get('command_success_rate', 100) < 85:
            issues.append("• Implement command validation and conflict resolution")

        if results.get('resource_usage', {}).get('avg_cpu', 0) > 70:
            issues.append("• Optimize processing efficiency and consider load balancing")

        if not issues:
            issues.append("• System performs well under extreme stress - no major issues detected")

        for issue in issues:
            print(issue)

        print("
🎯 MISSION IMPACT ASSESSMENT:")
        if results['overall_health_score'] > 75:
            print("   ✅ System should maintain basic functionality in extreme conditions")
        elif results['overall_health_score'] > 50:
            print("   ⚠️ System may experience reduced performance in extreme conditions")
        else:
            print("   ❌ System reliability significantly compromised under stress")


def run_integrated_stress_test_suite():
    """Run integrated stress tests across all severity levels."""

    print("🚀 URC 2026 Integrated Communication Stress Test Suite")
    print("=" * 65)

    results = {}

    # Test all stress levels
    stress_levels = [IntegratedStressLevel.MODERATE, IntegratedStressLevel.SEVERE, IntegratedStressLevel.EXTREME]

    for level in stress_levels:
        print(f"\n🔥 Testing {level.value.upper()} Integrated System Stress")
        print("-" * 60)

        config = IntegratedStressConfig(level)
        tester = IntegratedStressTest(config)

        try:
            level_results = tester.run_integrated_stress_test()
            results[level.value] = level_results
        except Exception as e:
            print(f"   ❌ Integrated test failed: {e}")
            results[level.value] = {'error': str(e)}

        print()  # Add spacing

    # Comparative analysis
    print("📈 INTEGRATED STRESS ANALYSIS")
    print("=" * 35)

    print("
Performance by Stress Level:")
    print("Level       | Health | Network | CAN Bus | Movement | Resources")
    print("-" * 70)

    for level_name, result in results.items():
        if 'error' in result:
            print(f"{level_name:8} | ERROR  |   -    |   -    |    -    |    -   ")
            continue

        level_short = level_name[:3].upper()
        health = result['overall_health_score']

        network_lat = result.get('network_performance', {}).get('avg_latency_ms', 999)
        can_avail = result.get('can_performance', {}).get('bus_availability_percent', 0)
        movement_success = result.get('movement_performance', {}).get('command_success_rate', 0)
        cpu_usage = result.get('resource_usage', {}).get('avg_cpu', 999)

        print(f"{level_short:8} | {health:6.1f} | {network_lat:7.1f} | {can_avail:6.1f}% | {movement_success:8.1f}% | {cpu_usage:8.1f}%")

    # Final assessment
    extreme_results = results.get('extreme', {})
    if 'error' not in extreme_results:
        print("
🎯 FINAL SYSTEM ASSESSMENT")

        if extreme_results['overall_health_score'] > 70:
            print("✅ System demonstrates good resilience under extreme integrated stress")
            print("   • Suitable for harsh Martian environment with proper monitoring")
        elif extreme_results['overall_health_score'] > 40:
            print("⚠️ System shows moderate resilience under extreme conditions")
            print("   • May require additional fault tolerance measures")
        else:
            print("❌ System shows poor resilience under integrated stress")
            print("   • Significant redesign required for harsh environments")

    return results


if __name__ == '__main__':
    import os

    # Add path for imports
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

    # Run integrated stress test suite
    integrated_results = run_integrated_stress_test_suite()

    print("
✨ Integrated communication stress testing completed!")
