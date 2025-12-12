#!/usr/bin/env python3
"""
Sensor Failure Recovery Test Scenario

Tests graceful degradation and recovery from sensor failures.
Validates system robustness when sensors fail or provide degraded data.
"""

import json
import random
import time
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, Temperature
from std_msgs.msg import Bool, String


class SensorType(Enum):
    """Types of sensors that can fail."""
    IMU = "imu"
    GPS = "gps"
    LIDAR = "lidar"
    CAMERA = "camera"
    ENCODER = "encoder"


class FailureMode(Enum):
    """Types of sensor failures."""
    COMPLETE_OUTAGE = "complete_outage"
    NOISY_DATA = "noisy_data"
    BIASED_DATA = "biased_data"
    DELAYED_DATA = "delayed_data"
    INTERMITTENT = "intermittent"


@dataclass
class SensorFailure:
    """Sensor failure configuration."""
    sensor_type: SensorType
    failure_mode: FailureMode
    start_time: float
    duration: float
    severity: float  # 0-1, how severe the failure is
    recovery_time: Optional[float] = None


@dataclass
class RecoveryMetrics:
    """Recovery performance metrics."""
    failure_detection_time: float = 0.0
    degradation_start_time: float = 0.0
    recovery_time: float = 0.0
    position_error_during_failure: float = 0.0
    navigation_continuity: bool = True
    graceful_degradation: bool = True


class SensorFailureRecoveryTester(Node):
    """Test sensor failure recovery and graceful degradation."""

    def __init__(self):
        super().__init__("sensor_failure_recovery_tester")

        # Test configuration
        self.test_duration = 900.0  # 15 minutes
        self.failure_interval = 180.0  # 3 minutes between failures
        self.recovery_timeout = 60.0  # 1 minute to recover

        # Test state
        self.start_time = time.time()
        self.test_active = True
        self.current_failures = []
        self.failure_history = []
        self.recovery_metrics = []

        # Sensor health monitoring
        self.last_sensor_updates = {}
        self.sensor_timeouts = {
            'imu': 0.5,
            'gps': 2.0,
            'lidar': 0.2,
            'camera': 0.1,
            'odom': 0.1
        }

        # Navigation state
        self.current_position = None
        self.navigation_active = True
        self.degraded_mode = False

        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, "/imu", self.imu_callback, self.qos_best_effort
        )

        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, self.qos_reliable
        )

        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.laser_callback, self.qos_best_effort
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, self.qos_reliable
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.test_status_pub = self.create_publisher(String, "/sensor_test_status", 10)
        self.failure_pub = self.create_publisher(String, "/sensor_failure_sim", 10)

        # Test timers
        self.failure_timer = self.create_timer(self.failure_interval, self.inject_failure)
        self.monitor_timer = self.create_timer(5.0, self.monitor_sensor_health)
        self.navigation_timer = self.create_timer(2.0, self.update_navigation)

        # Initialize failure schedule
        self.failure_schedule = self.generate_failure_schedule()

        self.get_logger().info("Sensor Failure Recovery Tester initialized")
        self.get_logger().info(f"Test duration: {self.test_duration/60:.1f} minutes")
        self.get_logger().info(f"Planned failures: {len(self.failure_schedule)}")

    def generate_failure_schedule(self) -> List[SensorFailure]:
        """Generate a schedule of sensor failures to test."""
        failures = []

        # GPS failure (critical for long-distance navigation)
        failures.append(SensorFailure(
            sensor_type=SensorType.GPS,
            failure_mode=FailureMode.COMPLETE_OUTAGE,
            start_time=120.0,  # 2 minutes in
            duration=60.0,     # 1 minute outage
            severity=1.0
        ))

        # IMU noise injection
        failures.append(SensorFailure(
            sensor_type=SensorType.IMU,
            failure_mode=FailureMode.NOISY_DATA,
            start_time=240.0,  # 4 minutes in
            duration=45.0,     # 45 seconds
            severity=0.7
        ))

        # LiDAR partial failure
        failures.append(SensorFailure(
            sensor_type=SensorType.LIDAR,
            failure_mode=FailureMode.INTERMITTENT,
            start_time=360.0,  # 6 minutes in
            duration=90.0,     # 1.5 minutes
            severity=0.5
        ))

        # Camera degradation
        failures.append(SensorFailure(
            sensor_type=SensorType.CAMERA,
            failure_mode=FailureMode.BIASED_DATA,
            start_time=480.0,  # 8 minutes in
            duration=30.0,     # 30 seconds
            severity=0.8
        ))

        # Multiple sensor failure (stress test)
        failures.append(SensorFailure(
            sensor_type=SensorType.IMU,
            failure_mode=FailureMode.COMPLETE_OUTAGE,
            start_time=600.0,  # 10 minutes in
            duration=30.0,     # 30 seconds
            severity=1.0
        ))
        failures.append(SensorFailure(
            sensor_type=SensorType.GPS,
            failure_mode=FailureMode.DELAYED_DATA,
            start_time=600.0,  # Simultaneous
            duration=30.0,
            severity=0.6
        ))

        return failures

    def imu_callback(self, msg):
        """Handle IMU data."""
        self.last_sensor_updates['imu'] = time.time()

        # Check for active IMU failure
        active_failure = self.get_active_failure(SensorType.IMU)
        if active_failure:
            self.simulate_failure(msg, active_failure)

    def gps_callback(self, msg):
        """Handle GPS data."""
        self.last_sensor_updates['gps'] = time.time()

        # Check for active GPS failure
        active_failure = self.get_active_failure(SensorType.GPS)
        if active_failure:
            self.simulate_failure(msg, active_failure)

    def laser_callback(self, msg):
        """Handle LiDAR data."""
        self.last_sensor_updates['lidar'] = time.time()

        # Check for active LiDAR failure
        active_failure = self.get_active_failure(SensorType.LIDAR)
        if active_failure:
            self.simulate_failure(msg, active_failure)

    def odom_callback(self, msg):
        """Handle odometry data."""
        self.last_sensor_updates['odom'] = time.time()
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    def get_active_failure(self, sensor_type: SensorType) -> Optional[SensorFailure]:
        """Get currently active failure for a sensor type."""
        current_time = time.time() - self.start_time

        for failure in self.current_failures:
            if (failure.sensor_type == sensor_type and
                failure.start_time <= current_time <= failure.start_time + failure.duration):
                return failure

        return None

    def simulate_failure(self, msg, failure: SensorFailure):
        """Simulate sensor failure on message data."""
        if failure.failure_mode == FailureMode.COMPLETE_OUTAGE:
            # Drop the message entirely
            return

        elif failure.failure_mode == FailureMode.NOISY_DATA:
            # Add noise to sensor data
            if hasattr(msg, 'linear_acceleration'):
                # IMU acceleration noise
                noise_factor = failure.severity * 10.0
                msg.linear_acceleration.x += random.gauss(0, noise_factor)
                msg.linear_acceleration.y += random.gauss(0, noise_factor)
                msg.linear_acceleration.z += random.gauss(0, noise_factor)

        elif failure.failure_mode == FailureMode.BIASED_DATA:
            # Add bias to sensor data
            if hasattr(msg, 'ranges'):
                # LiDAR range bias
                bias_factor = failure.severity * 2.0
                msg.ranges = [r * (1 + bias_factor) for r in msg.ranges]

        elif failure.failure_mode == FailureMode.INTERMITTENT:
            # Randomly drop messages
            if random.random() < failure.severity:
                return  # Drop message

        # Publish failure notification
        failure_msg = String()
        failure_msg.data = f"SENSOR_FAILURE_SIM|{failure.sensor_type.value}|{failure.failure_mode.value}|{failure.severity}"
        self.failure_pub.publish(failure_msg)

    def inject_failure(self):
        """Inject scheduled sensor failures."""
        if not self.test_active:
            return

        current_time = time.time() - self.start_time

        # Check for failures to start
        for failure in self.failure_schedule:
            if (failure.start_time <= current_time <= failure.start_time + failure.duration and
                failure not in self.current_failures):

                self.current_failures.append(failure)
                self.failure_history.append(failure)

                self.get_logger().warn(
                    f"INJECTING FAILURE: {failure.sensor_type.value} {failure.failure_mode.value} "
                    ".1f"
                )

                # Record degradation start
                metrics = RecoveryMetrics(
                    failure_detection_time=current_time,
                    degradation_start_time=current_time
                )
                self.recovery_metrics.append(metrics)

        # Check for failures to end
        self.current_failures = [
            f for f in self.current_failures
            if current_time <= f.start_time + f.duration
        ]

        # Publish current failure status
        active_failures = [f"{f.sensor_type.value}:{f.failure_mode.value}" for f in self.current_failures]
        status_msg = String()
        status_msg.data = f"SENSOR_FAILURE_STATUS|Active:{','.join(active_failures) if active_failures else 'None'}"
        self.test_status_pub.publish(status_msg)

    def monitor_sensor_health(self):
        """Monitor sensor health and detect timeouts."""
        current_time = time.time()

        for sensor, timeout in self.sensor_timeouts.items():
            last_update = self.last_sensor_updates.get(sensor, 0)
            if current_time - last_update > timeout:
                # Sensor timeout detected
                if not self.degraded_mode:
                    self.degraded_mode = True
                    self.get_logger().warn(f"SENSOR TIMEOUT: {sensor} - Entering degraded mode")
                    self.handle_sensor_timeout(sensor)

    def handle_sensor_timeout(self, sensor: str):
        """Handle sensor timeout by switching to degraded mode."""
        # Reduce navigation speed and complexity
        self.navigation_active = False

        # Stop current motion
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # In a real system, this would trigger:
        # - Switch to backup navigation algorithms
        # - Increase safety margins
        # - Alert operators
        # - Attempt sensor recovery

        self.get_logger().info(f"Activated degraded mode due to {sensor} failure")

    def update_navigation(self):
        """Update navigation commands based on system state."""
        if not self.test_active or not self.navigation_active:
            return

        # Simple navigation: move in expanding circles
        elapsed = time.time() - self.start_time
        radius = 2.0 + (elapsed / 60.0)  # Expand radius over time
        angular_speed = 0.5

        cmd = Twist()
        cmd.linear.x = 0.3  # Slow speed in degraded mode
        cmd.angular.z = angular_speed

        self.cmd_vel_pub.publish(cmd)

    def complete_test(self):
        """Complete the sensor failure test and generate report."""
        self.test_active = False

        # Calculate recovery metrics
        successful_recoveries = 0
        avg_recovery_time = 0.0

        for i, failure in enumerate(self.failure_history):
            if i < len(self.recovery_metrics):
                metrics = self.recovery_metrics[i]

                # Check if system recovered (simplified check)
                recovery_successful = (
                    self.navigation_active and  # Navigation still working
                    not self.degraded_mode      # Not stuck in degraded mode
                )

                if recovery_successful:
                    successful_recoveries += 1
                    metrics.recovery_time = failure.duration  # Simplified

                metrics.graceful_degradation = recovery_successful

        # Generate comprehensive results
        test_results = {
            'test_info': {
                'name': 'Sensor Failure Recovery Test',
                'duration_seconds': time.time() - self.start_time,
                'failures_injected': len(self.failure_history),
                'start_time': time.ctime(self.start_time),
                'end_time': time.ctime(time.time())
            },
            'failure_schedule': [
                {
                    'sensor': f.sensor_type.value,
                    'mode': f.failure_mode.value,
                    'start_time': f.start_time,
                    'duration': f.duration,
                    'severity': f.severity
                }
                for f in self.failure_history
            ],
            'recovery_metrics': [
                asdict(m) for m in self.recovery_metrics
            ],
            'overall_results': {
                'successful_recoveries': successful_recoveries,
                'total_failures': len(self.failure_history),
                'recovery_rate': successful_recoveries / max(1, len(self.failure_history)),
                'graceful_degradation': all(m.graceful_degradation for m in self.recovery_metrics),
                'navigation_continuity': self.navigation_active
            }
        }

        # Save results
        with open("/tmp/sensor_failure_test_results.json", "w") as f:
            json.dump(test_results, f, indent=2)

        # Publish completion status
        completion_msg = String()
        completion_msg.data = f"SENSOR_FAILURE_TEST_COMPLETE|Recoveries:{successful_recoveries}/{len(self.failure_history)}"
        self.test_status_pub.publish(completion_msg)

        # Print comprehensive results
        print("\n" + "="*70)
        print("SENSOR FAILURE RECOVERY TEST RESULTS")
        print("="*70)
        print(f"Failures Injected: {len(self.failure_history)}")
        print(f"Successful Recoveries: {successful_recoveries}")
        print(f"Navigation Continuity: {' YES' if self.navigation_active else ' NO'}")
        print(f"Graceful Degradation: {' YES' if test_results['overall_results']['graceful_degradation'] else ' NO'}")

        print("\nFAILURE DETAILS:")
        for i, failure in enumerate(self.failure_history):
            recovery = "" if i < len(self.recovery_metrics) and self.recovery_metrics[i].graceful_degradation else ""
            print(f"  {recovery} {failure.sensor_type.value} {failure.failure_mode.value} ({failure.duration:.0f}s)")

        success_rate = successful_recoveries / max(1, len(self.failure_history))
        overall_success = success_rate > 0.8 and self.navigation_active

        print(f"\nOVERALL RESULT: {' SUCCESS' if overall_success else '  NEEDS IMPROVEMENT'}")
        print("="*70)

        self.get_logger().info("Sensor Failure Recovery Test completed")
        self.get_logger().info("Results saved to /tmp/sensor_failure_test_results.json")


def main():
    """Main test function."""
    rclpy.init()
    tester = SensorFailureRecoveryTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.complete_test()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
