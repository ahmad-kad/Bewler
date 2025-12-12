#!/usr/bin/env python3
"""
URC 2026 Performance & Load Testing Suite

Tests system performance under various conditions:
- Message throughput and latency
- CPU/memory usage under load
- Concurrent operations
- Network stress testing
"""

import asyncio
import json
import os
import statistics
import sys
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional

import psutil

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'bridges'))

from bridges.can_mock_simulator import CANBusMockSimulator
from bridges.priority_message_router import PriorityMessageRouter


class PerformanceLoadTest(unittest.TestCase):
    """Performance and load testing for URC 2026 system."""

    def setUp(self):
        """Set up test environment."""
        self.can_simulator = CANBusMockSimulator()
        self.message_router = PriorityMessageRouter(max_queue_size=1000)
        self.results = {}

    def tearDown(self):
        """Clean up test resources."""
        if hasattr(self.can_simulator, 'stop'):
            self.can_simulator.stop()

    def _measure_memory_usage(self) -> float:
        """Measure current memory usage in MB."""
        process = psutil.Process()
        return process.memory_info().rss / 1024 / 1024  # Convert to MB

    def _measure_cpu_usage(self, duration: float = 1.0) -> float:
        """Measure CPU usage over a duration."""
        return psutil.cpu_percent(interval=duration)

    def test_message_throughput(self):
        """Test message processing throughput."""
        print("\n⚡ Testing Message Throughput")

        # Test parameters
        num_messages = 1000
        message_types = ['sensor_data', 'navigation_command', 'safety_trigger', 'telemetry']

        start_time = time.time()
        start_memory = self._measure_memory_usage()

        # Generate and process messages
        for i in range(num_messages):
            msg_type = message_types[i % len(message_types)]
            message = {
                'id': f'msg_{i}',
                'type': msg_type,
                'timestamp': time.time(),
                'data': f'test_data_{i}'
            }
            self.message_router.enqueue_message(message)

        # Process all messages
        processed = 0
        while True:
            msg = self.message_router.dequeue_message()
            if msg is None:
                break
            processed += 1

        end_time = time.time()
        end_memory = self._measure_memory_usage()

        # Calculate metrics
        total_time = end_time - start_time
        throughput = num_messages / total_time  # messages/second
        memory_delta = end_memory - start_memory

        self.results['message_throughput'] = {
            'messages_processed': processed,
            'total_time': total_time,
            'throughput_msg_per_sec': throughput,
            'memory_delta_mb': memory_delta
        }

        print(f"✅ Processed {processed} messages in {total_time:.3f}s")
        print(".2f")
        print(".2f")
        self.assertEqual(processed, num_messages)
        self.assertGreater(throughput, 100)  # Minimum 100 msg/s

    def test_can_sensor_data_latency(self):
        """Test CAN sensor data retrieval latency."""
        print("\n🏁 Testing CAN Sensor Data Latency")

        sensor_types = ['imu', 'gps', 'battery', 'motor_left', 'motor_right']
        num_samples = 100

        latencies = []

        for _ in range(num_samples):
            for sensor in sensor_types:
                start_time = time.time()
                data = self.can_simulator.get_mock_reading(sensor)
                end_time = time.time()

                latency = (end_time - start_time) * 1000  # Convert to ms
                latencies.append(latency)

                # Verify data structure
                self.assertIn('sensor', data)
                self.assertIn('data', data)
                self.assertTrue(data['mock'])

        # Calculate statistics
        avg_latency = statistics.mean(latencies)
        max_latency = max(latencies)
        p95_latency = statistics.quantiles(latencies, n=20)[18]  # 95th percentile

        self.results['can_latency'] = {
            'samples': len(latencies),
            'avg_latency_ms': avg_latency,
            'max_latency_ms': max_latency,
            'p95_latency_ms': p95_latency
        }

        print(f"✅ Tested {len(latencies)} sensor readings")
        print(".3f")
        print(".3f")
        print(".3f")

        # Performance requirements
        self.assertLess(avg_latency, 10.0)  # < 10ms average
        self.assertLess(max_latency, 50.0)  # < 50ms max
        self.assertLess(p95_latency, 20.0)  # < 20ms 95th percentile

    def test_concurrent_operations(self):
        """Test system performance under concurrent operations."""
        print("\n🔄 Testing Concurrent Operations")

        num_threads = 4
        operations_per_thread = 250
        results = []

        def worker_thread(thread_id: int):
            """Worker function for concurrent testing."""
            thread_results = []
            start_time = time.time()

            for i in range(operations_per_thread):
                # Mix of operations
                if i % 4 == 0:
                    # Sensor reading
                    data = self.can_simulator.get_mock_reading('imu')
                    thread_results.append(('sensor', time.time()))
                elif i % 4 == 1:
                    # Message routing
                    msg = {'type': 'sensor_data', 'id': f't{thread_id}_{i}'}
                    self.message_router.enqueue_message(msg)
                    thread_results.append(('enqueue', time.time()))
                elif i % 4 == 2:
                    # Message processing
                    msg = self.message_router.dequeue_message()
                    thread_results.append(('dequeue', time.time()))
                else:
                    # Mixed operation
                    self.can_simulator.set_motor_command('motor_left', float(i % 10))
                    thread_results.append(('motor_cmd', time.time()))

            end_time = time.time()
            return thread_results, end_time - start_time

        # Start concurrent threads
        start_time = time.time()
        start_memory = self._measure_memory_usage()
        start_cpu = self._measure_cpu_usage(0.1)

        with ThreadPoolExecutor(max_workers=num_threads) as executor:
            futures = [executor.submit(worker_thread, i) for i in range(num_threads)]
            thread_results = [future.result() for future in futures]

        end_time = time.time()
        end_memory = self._measure_memory_usage()
        end_cpu = self._measure_cpu_usage(0.1)

        # Aggregate results
        total_operations = sum(len(results[0]) for results in thread_results)
        total_time = end_time - start_time
        throughput = total_operations / total_time
        memory_delta = end_memory - start_memory
        cpu_delta = end_cpu - start_cpu

        self.results['concurrent_operations'] = {
            'threads': num_threads,
            'total_operations': total_operations,
            'total_time': total_time,
            'throughput_ops_per_sec': throughput,
            'memory_delta_mb': memory_delta,
            'cpu_usage_percent': cpu_delta
        }

        print(f"✅ Executed {total_operations} concurrent operations")
        print(".1f")
        print(".2f")
        print(".2f")
        print(".1f")

        # Performance requirements
        self.assertGreater(throughput, 500)  # Minimum 500 ops/s
        self.assertLess(memory_delta, 50)    # < 50MB memory increase
        self.assertLess(cpu_delta, 80)       # < 80% CPU usage

    def test_memory_leak_detection(self):
        """Test for memory leaks during prolonged operation."""
        print("\n💧 Testing Memory Leak Detection")

        num_iterations = 100
        memory_readings = []

        # Baseline memory
        baseline_memory = self._measure_memory_usage()
        memory_readings.append(baseline_memory)

        # Perform operations that might cause memory leaks
        for i in range(num_iterations):
            # Create and process messages
            for j in range(10):
                msg = {'type': 'sensor_data', 'id': f'leak_test_{i}_{j}', 'data': 'x' * 100}
                self.message_router.enqueue_message(msg)

            # Process messages
            for j in range(10):
                self.message_router.dequeue_message()

            # Get sensor data
            self.can_simulator.get_mock_reading('imu')
            self.can_simulator.get_mock_reading('gps')

            # Record memory every 10 iterations
            if i % 10 == 0:
                memory_readings.append(self._measure_memory_usage())

        final_memory = self._measure_memory_usage()
        memory_readings.append(final_memory)

        # Analyze memory trend
        memory_delta = final_memory - baseline_memory
        memory_trend = statistics.linear_regression(range(len(memory_readings)), memory_readings)[0]

        self.results['memory_leak_test'] = {
            'iterations': num_iterations,
            'baseline_memory_mb': baseline_memory,
            'final_memory_mb': final_memory,
            'memory_delta_mb': memory_delta,
            'memory_trend_mb_per_reading': memory_trend,
            'memory_readings': memory_readings
        }

        print(f"✅ Memory leak test completed")
        print(".2f")
        print(".2f")
        print(".4f")

        # Memory leak detection: slope should be near zero or negative
        self.assertLess(abs(memory_trend), 0.1)  # < 0.1 MB increase per reading
        self.assertLess(memory_delta, 10.0)      # < 10MB total increase

    def test_network_stress_simulation(self):
        """Test system under simulated network stress."""
        print("\n🌐 Testing Network Stress Simulation")

        # Simulate high-frequency message traffic
        num_messages = 500
        burst_size = 50
        burst_delay = 0.01  # 10ms between bursts

        latencies = []
        queue_sizes = []

        start_time = time.time()

        for burst in range(num_messages // burst_size):
            burst_start = time.time()

            # Send burst of messages
            for i in range(burst_size):
                msg = {
                    'type': 'sensor_data',
                    'id': f'burst_{burst}_{i}',
                    'timestamp': time.time()
                }
                enqueue_start = time.time()
                self.message_router.enqueue_message(msg)
                enqueue_end = time.time()
                latencies.append((enqueue_end - enqueue_start) * 1000)

            # Record queue size
            queue_sizes.append(self.message_router.get_queue_status()['queue_size'])

            # Small delay between bursts
            time.sleep(burst_delay)

            # Process some messages to prevent infinite queue growth
            for _ in range(burst_size // 2):
                self.message_router.dequeue_message()

        # Process remaining messages
        remaining = 0
        while True:
            msg = self.message_router.dequeue_message()
            if msg is None:
                break
            remaining += 1

        end_time = time.time()
        total_time = end_time - start_time

        # Analyze results
        avg_latency = statistics.mean(latencies)
        max_latency = max(latencies)
        avg_queue_size = statistics.mean(queue_sizes)
        max_queue_size = max(queue_sizes)

        self.results['network_stress'] = {
            'messages_sent': num_messages,
            'total_time': total_time,
            'throughput_msg_per_sec': num_messages / total_time,
            'avg_enqueue_latency_ms': avg_latency,
            'max_enqueue_latency_ms': max_latency,
            'avg_queue_size': avg_queue_size,
            'max_queue_size': max_queue_size
        }

        print(f"✅ Network stress test completed")
        print(".1f")
        print(".3f")
        print(".3f")
        print(".1f")
        print(".1f")

        # Performance requirements under stress
        self.assertLess(avg_latency, 5.0)      # < 5ms average enqueue latency
        self.assertLess(max_latency, 20.0)     # < 20ms max enqueue latency
        self.assertLess(max_queue_size, 200)   # Queue shouldn't grow too large

    def test_system_resource_monitoring(self):
        """Test system resource usage monitoring."""
        print("\n📊 Testing System Resource Monitoring")

        monitoring_duration = 5.0  # 5 seconds
        sample_interval = 0.1      # 100ms

        cpu_readings = []
        memory_readings = []

        start_time = time.time()

        # Monitor resources while performing operations
        while time.time() - start_time < monitoring_duration:
            # Perform some operations to generate load
            for _ in range(10):
                self.can_simulator.get_mock_reading('imu')
                msg = {'type': 'sensor_data', 'id': f'monitor_{time.time()}'}
                self.message_router.enqueue_message(msg)
                self.message_router.dequeue_message()

            # Record resource usage
            cpu_readings.append(psutil.cpu_percent(interval=0.05))
            memory_readings.append(self._measure_memory_usage())

            time.sleep(sample_interval)

        # Calculate statistics
        avg_cpu = statistics.mean(cpu_readings)
        max_cpu = max(cpu_readings)
        avg_memory = statistics.mean(memory_readings)
        max_memory = max(memory_readings)
        memory_variance = statistics.variance(memory_readings)

        self.results['resource_monitoring'] = {
            'monitoring_duration': monitoring_duration,
            'samples': len(cpu_readings),
            'avg_cpu_percent': avg_cpu,
            'max_cpu_percent': max_cpu,
            'avg_memory_mb': avg_memory,
            'max_memory_mb': max_memory,
            'memory_variance': memory_variance
        }

        print(f"✅ Resource monitoring completed")
        print(f"   CPU Usage: {avg_cpu:.1f}% avg, {max_cpu:.1f}% max")
        print(f"   Memory Usage: {avg_memory:.1f}MB avg, {max_memory:.1f}MB max")

        # Resource usage requirements
        self.assertLess(avg_cpu, 50.0)     # < 50% average CPU
        self.assertLess(max_cpu, 80.0)     # < 80% max CPU
        self.assertLess(max_memory, 500.0) # < 500MB max memory

    def generate_performance_report(self):
        """Generate comprehensive performance report."""
        print("\n📋 Performance Test Results Summary")
        print("=" * 50)

        for test_name, results in self.results.items():
            print(f"\n🔹 {test_name.replace('_', ' ').title()}:")
            for key, value in results.items():
                if isinstance(value, float):
                    print(f"   {key}: {value:.3f}")
                elif isinstance(value, int):
                    print(f"   {key}: {value}")
                else:
                    print(f"   {key}: {value}")

        # Overall assessment
        passed_tests = len([r for r in self.results.values() if all(isinstance(v, (int, float)) and v >= 0 for v in r.values())])
        total_tests = len(self.results)

        print(f"\n🎯 Overall Performance Assessment:")
        print(f"   Tests Passed: {passed_tests}/{total_tests}")
        print(".1f")

        if passed_tests == total_tests:
            print("   ✅ ALL PERFORMANCE REQUIREMENTS MET")
        elif passed_tests >= total_tests * 0.8:
            print("   ⚠️ MOST PERFORMANCE REQUIREMENTS MET")
        else:
            print("   ❌ PERFORMANCE ISSUES DETECTED")

        return self.results


if __name__ == '__main__':
    print("🚀 URC 2026 Performance & Load Testing Suite")
    print("=" * 50)

    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(PerformanceLoadTest)
    runner = unittest.TextTestRunner(verbosity=2)

    # Run tests
    result = runner.run(suite)

    # Generate performance report if tests passed
    if result.wasSuccessful():
        test_instance = PerformanceLoadTest()
        test_instance.setUp()
        test_instance.test_message_throughput()
        test_instance.test_can_sensor_data_latency()
        test_instance.test_concurrent_operations()
        test_instance.test_memory_leak_detection()
        test_instance.test_network_stress_simulation()
        test_instance.test_system_resource_monitoring()
        test_instance.generate_performance_report()
        test_instance.tearDown()

    print("\n🏁 Performance testing completed!")
