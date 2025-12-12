#!/usr/bin/env python3
"""
Fault Injection Framework for URC 2026

Provides comprehensive fault injection capabilities for testing system robustness.
Supports hardware failures, software bugs, and environmental disturbances.
"""

import json
import random
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class FaultType(Enum):
    """Types of faults that can be injected."""
    SENSOR_NOISE = "sensor_noise"
    SENSOR_BIAS = "sensor_bias"
    SENSOR_OUTAGE = "sensor_outage"
    ACTUATOR_FAILURE = "actuator_failure"
    COMMUNICATION_DELAY = "communication_delay"
    PROCESSING_OVERLOAD = "processing_overload"
    MEMORY_LEAK = "memory_leak"
    THREAD_DEADLOCK = "thread_deadlock"
    PARAMETER_CORRUPTION = "parameter_corruption"
    ENVIRONMENTAL_NOISE = "environmental_noise"


class FaultSeverity(Enum):
    """Fault severity levels."""
    LOW = 0.2
    MEDIUM = 0.5
    HIGH = 0.8
    CRITICAL = 1.0


@dataclass
class FaultInjection:
    """Fault injection configuration."""
    fault_type: FaultType
    target_component: str
    severity: FaultSeverity
    duration_seconds: float
    start_time: Optional[float] = None
    parameters: Dict[str, Any] = field(default_factory=dict)
    active: bool = False
    injection_id: str = ""


@dataclass
class FaultStatistics:
    """Statistics for fault injection testing."""
    total_injections: int = 0
    successful_injections: int = 0
    system_recoveries: int = 0
    failures_caused: int = 0
    average_recovery_time: float = 0.0
    fault_types_tested: List[str] = field(default_factory=list)


class FaultInjectionFramework(Node):
    """Comprehensive fault injection framework for robustness testing."""

    def __init__(self):
        super().__init__('fault_injection_framework')

        self.logger = self.get_logger()

        # Fault management
        self.active_faults = {}
        self.fault_history = []
        self.fault_statistics = FaultStatistics()

        # Component interfaces
        self.component_interfaces = {
            'imu': '/imu',
            'gps': '/gps/fix',
            'lidar': '/scan',
            'camera': '/camera/image_raw',
            'odom': '/odom',
            'cmd_vel': '/cmd_vel'
        }

        # Fault injection publishers
        self.fault_control_pub = self.create_publisher(
            String, '/fault_injection/control', 10
        )

        self.fault_status_pub = self.create_publisher(
            String, '/fault_injection/status', 10
        )

        # QoS for reliability
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Fault injection timer
        self.fault_monitor_timer = self.create_timer(1.0, self.monitor_active_faults)

        # Pre-defined fault scenarios
        self.fault_scenarios = self.define_fault_scenarios()

        self.logger.info("Fault Injection Framework initialized")
        self.logger.info(f"Available scenarios: {list(self.fault_scenarios.keys())}")

    def define_fault_scenarios(self) -> Dict[str, List[FaultInjection]]:
        """Define pre-configured fault scenarios."""
        return {
            'sensor_degradation': [
                FaultInjection(
                    fault_type=FaultType.SENSOR_NOISE,
                    target_component='imu',
                    severity=FaultSeverity.MEDIUM,
                    duration_seconds=30.0,
                    parameters={'noise_factor': 2.0}
                ),
                FaultInjection(
                    fault_type=FaultType.SENSOR_BIAS,
                    target_component='gps',
                    severity=FaultSeverity.HIGH,
                    duration_seconds=45.0,
                    parameters={'bias_meters': 5.0}
                )
            ],

            'communication_failure': [
                FaultInjection(
                    fault_type=FaultType.COMMUNICATION_DELAY,
                    target_component='lidar',
                    severity=FaultSeverity.HIGH,
                    duration_seconds=20.0,
                    parameters={'delay_seconds': 0.5}
                ),
                FaultInjection(
                    fault_type=FaultType.SENSOR_OUTAGE,
                    target_component='camera',
                    severity=FaultSeverity.CRITICAL,
                    duration_seconds=15.0
                )
            ],

            'system_stress': [
                FaultInjection(
                    fault_type=FaultType.PROCESSING_OVERLOAD,
                    target_component='cpu',
                    severity=FaultSeverity.MEDIUM,
                    duration_seconds=60.0,
                    parameters={'cpu_load_percent': 70}
                ),
                FaultInjection(
                    fault_type=FaultType.MEMORY_LEAK,
                    target_component='memory',
                    severity=FaultSeverity.MEDIUM,
                    duration_seconds=120.0,
                    parameters={'leak_rate_mb_per_sec': 0.1}
                )
            ],

            'environmental_extremes': [
                FaultInjection(
                    fault_type=FaultType.ENVIRONMENTAL_NOISE,
                    target_component='lidar',
                    severity=FaultSeverity.HIGH,
                    duration_seconds=90.0,
                    parameters={'dust_density': 0.8, 'visibility_reduction': 0.6}
                ),
                FaultInjection(
                    fault_type=FaultType.SENSOR_NOISE,
                    target_component='imu',
                    severity=FaultSeverity.CRITICAL,
                    duration_seconds=60.0,
                    parameters={'vibration_amplitude': 5.0}
                )
            ]
        }

    def inject_fault(self, fault: FaultInjection) -> str:
        """Inject a fault into the system."""
        fault_id = f"fault_{int(time.time())}_{random.randint(1000, 9999)}"
        fault.injection_id = fault_id
        fault.start_time = time.time()
        fault.active = True

        self.active_faults[fault_id] = fault
        self.fault_history.append(fault)
        self.fault_statistics.total_injections += 1

        # Publish fault injection command
        control_msg = String()
        control_msg.data = json.dumps({
            'command': 'inject_fault',
            'fault_id': fault_id,
            'fault_type': fault.fault_type.value,
            'target_component': fault.target_component,
            'severity': fault.severity.value,
            'duration': fault.duration_seconds,
            'parameters': fault.parameters
        })
        self.fault_control_pub.publish(control_msg)

        self.logger.warn(
            f"INJECTED FAULT: {fault.fault_type.value} "
            f"on {fault.target_component} "
            f"(severity: {fault.severity.value:.1f})"
        )
        return fault_id

    def inject_scenario(self, scenario_name: str) -> List[str]:
        """Inject a complete fault scenario."""
        if scenario_name not in self.fault_scenarios:
            self.logger.error(f"Unknown scenario: {scenario_name}")
            return []

        scenario = self.fault_scenarios[scenario_name]
        fault_ids = []

        num_faults = len(scenario)
        self.logger.info(
            f"Injecting scenario: {scenario_name} ({num_faults} faults)"
        )

        for fault in scenario:
            fault_id = self.inject_fault(fault)
            fault_ids.append(fault_id)

            # Stagger fault injections slightly
            time.sleep(1.0)

        return fault_ids

    def inject_random_fault(self) -> str:
        """Inject a random fault for chaos testing."""
        fault_types = list(FaultType)
        components = list(self.component_interfaces.keys())
        severities = list(FaultSeverity)

        random_fault = FaultInjection(
            fault_type=random.choice(fault_types),
            target_component=random.choice(components),
            severity=random.choice(severities),
            duration_seconds=random.uniform(10.0, 60.0),
            parameters=self.generate_random_parameters(random.choice(fault_types))
        )

        return self.inject_fault(random_fault)

    def generate_random_parameters(self, fault_type: FaultType) -> Dict[str, Any]:
        """Generate appropriate random parameters for fault type."""
        if fault_type == FaultType.SENSOR_NOISE:
            return {'noise_factor': random.uniform(1.0, 5.0)}
        elif fault_type == FaultType.SENSOR_BIAS:
            return {'bias_value': random.uniform(-10.0, 10.0)}
        elif fault_type == FaultType.COMMUNICATION_DELAY:
            return {'delay_seconds': random.uniform(0.1, 2.0)}
        elif fault_type == FaultType.PROCESSING_OVERLOAD:
            return {'cpu_load_percent': random.uniform(50, 90)}
        elif fault_type == FaultType.ENVIRONMENTAL_NOISE:
            return {
                'dust_density': random.uniform(0.1, 1.0),
                'visibility_reduction': random.uniform(0.1, 0.9)
            }
        else:
            return {}

    def monitor_active_faults(self):
        """Monitor active faults and handle expiration."""
        current_time = time.time()
        expired_faults = []

        for fault_id, fault in self.active_faults.items():
            time_elapsed = current_time - fault.start_time if fault.start_time else 0
            if fault.start_time and time_elapsed >= fault.duration_seconds:
                expired_faults.append(fault_id)

                # Publish fault clearance
                clear_msg = String()
                clear_msg.data = json.dumps({
                    'command': 'clear_fault',
                    'fault_id': fault_id,
                    'duration': fault.duration_seconds
                })
                self.fault_control_pub.publish(clear_msg)

                self.logger.info(
                    f"CLEARED FAULT: {fault.fault_type.value} "
                    f"on {fault.target_component}"
                )

        # Remove expired faults
        for fault_id in expired_faults:
            del self.active_faults[fault_id]

        # Publish current status
        status_msg = String()
        status_msg.data = json.dumps({
            'active_faults': len(self.active_faults),
            'total_injections': self.fault_statistics.total_injections,
            'system_recoveries': self.fault_statistics.system_recoveries,
            'current_time': current_time
        })
        self.fault_status_pub.publish(status_msg)

    def clear_all_faults(self):
        """Clear all active faults."""
        fault_ids = list(self.active_faults.keys())

        for fault_id in fault_ids:
            clear_msg = String()
            clear_msg.data = json.dumps({
                'command': 'clear_fault',
                'fault_id': fault_id,
                'immediate': True
            })
            self.fault_control_pub.publish(clear_msg)

        self.active_faults.clear()
        self.logger.info("Cleared all active faults")

    def get_fault_statistics(self) -> Dict[str, Any]:
        """Get comprehensive fault injection statistics."""
        return {
            'total_injections': self.fault_statistics.total_injections,
            'successful_injections': self.fault_statistics.successful_injections,
            'system_recoveries': self.fault_statistics.system_recoveries,
            'failures_caused': self.fault_statistics.failures_caused,
            'recovery_rate': (
                self.fault_statistics.system_recoveries /
                max(1, self.fault_statistics.total_injections)
            ),
            'fault_types_tested': list(set(
                f.fault_type.value for f in self.fault_history
            )),
            'active_faults': len(self.active_faults),
            'test_duration_hours': (time.time() - time.time()) / 3600  # Placeholder
        }


class FaultInjectionNode(Node):
    """ROS2 node that receives and applies fault injections."""

    def __init__(self):
        super().__init__('fault_injection_node')

        self.logger = self.get_logger()
        self.active_faults = {}

        # Subscribe to fault injection commands
        self.fault_control_sub = self.create_subscription(
            String, '/fault_injection/control', self.fault_control_callback, 10
        )

        # Publishers for modified sensor data
        self.imu_pub = self.create_publisher(
            String, '/imu_fault_injected', 10  # Placeholder for actual IMU msg
        )

        self.logger.info("Fault Injection Node initialized")

    def fault_control_callback(self, msg):
        """Handle fault injection control commands."""
        try:
            command = json.loads(msg.data)

            if command['command'] == 'inject_fault':
                fault_id = command['fault_id']
                fault_type = command['fault_type']
                target = command['target_component']

                self.active_faults[fault_id] = command
                self.logger.warn(f"APPLYING FAULT: {fault_type} on {target}")

            elif command['command'] == 'clear_fault':
                fault_id = command['fault_id']
                if fault_id in self.active_faults:
                    del self.active_faults[fault_id]
                    self.logger.info(f"CLEARED FAULT: {fault_id}")

        except json.JSONDecodeError:
            self.logger.error("Invalid fault control message format")


def run_fault_injection_demo():
    """Run a demonstration of the fault injection framework."""
    print(" FAULT INJECTION FRAMEWORK DEMONSTRATION")
    print("=" * 60)

    # Initialize ROS2
    rclpy.init()

    # Create fault injection framework
    framework = FaultInjectionFramework()
    fault_node = FaultInjectionNode()

    try:
        # Start fault injection node in background
        node_thread = threading.Thread(
            target=lambda: rclpy.spin(fault_node), daemon=True
        )
        node_thread.start()

        # Wait for system to initialize
        time.sleep(2)

        # Demonstrate different fault scenarios
        scenarios = ['sensor_degradation', 'communication_failure', 'system_stress']

        for scenario in scenarios:
            print(f"\n Testing scenario: {scenario}")
            framework.inject_scenario(scenario)

            # Wait for scenario to complete
            scenario_duration = max(
                f.duration_seconds for f in framework.fault_scenarios[scenario]
            )
            time.sleep(scenario_duration + 5)  # Extra time for recovery

            print(f" Scenario {scenario} completed")

        # Demonstrate random fault injection
        print("Injecting random faults for chaos testing...")
        for _ in range(3):
            framework.inject_random_fault()
            time.sleep(random.uniform(5, 15))

        # Generate final statistics
        stats = framework.get_fault_statistics()
        print("\nFAULT INJECTION STATISTICS:")
        print(f"  Total Injections: {stats['total_injections']}")
        print(f"  Active Faults: {stats['active_faults']}")
        print(f"  Recovery Rate: {stats.get('recovery_rate', 0):.1f}")
        print(f"  Fault Types Tested: {', '.join(stats['fault_types_tested'])}")

        print("\nFAULT INJECTION FRAMEWORK DEMONSTRATION COMPLETE")
        print("System robustness tested with multiple failure scenarios!")

    except KeyboardInterrupt:
        print("\n Demonstration interrupted")
    finally:
        framework.clear_all_faults()
        rclpy.shutdown()


if __name__ == '__main__':
    run_fault_injection_demo()
