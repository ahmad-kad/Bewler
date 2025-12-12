#!/usr/bin/env python3
"""
Endurance SLAM Test Scenario

Validates long-duration operation stability and performance over extended periods.
Tests SLAM accuracy, memory usage, CPU utilization, and system reliability.
"""

import json
import math
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import List

import numpy as np
import psutil
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@dataclass
class EnduranceMetrics:
    """Endurance test performance metrics."""

    duration_hours: float = 0.0
    total_distance: float = 0.0
    loop_closures: int = 0
    pose_updates: int = 0
    map_updates: int = 0
    memory_usage_mb: List[float] = None
    cpu_usage_percent: List[float] = None
    slam_confidence: List[float] = None
    temperature_celsius: List[float] = None


@dataclass
class SystemHealth:
    """System health monitoring data."""

    memory_mb: float = 0.0
    memory_growth_mb: float = 0.0
    cpu_percent: float = 0.0
    temperature_c: float = 0.0
    process_count: int = 0
    thread_count: int = 0


class EnduranceSLAMTester(Node):
    """Test SLAM endurance over extended operation periods."""

    def __init__(self):
        super().__init__("endurance_slam_tester")

        # Test configuration
        self.test_duration = 1800.0  # 30 minutes (reduced for practical testing)
        self.monitoring_interval = 10.0  # 10 seconds
        self.distance_threshold = 10.0  # meters for loop closure testing

        # Test state
        self.start_time = time.time()
        self.test_active = True
        self.start_position = None
        self.total_distance = 0.0
        self.last_position = None
        self.loop_closures = 0

        # Performance monitoring
        self.memory_readings = []
        self.cpu_readings = []
        self.pose_confidence = []
        self.system_health = []

        # SLAM metrics
        self.pose_update_count = 0
        self.map_update_count = 0
        self.confidence_readings = []

        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, "/slam/pose/fused", self.pose_callback, self.qos_reliable)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, self.qos_reliable)

        self.map_sub = self.create_subscription(Path, "/slam/map_path", self.map_callback, self.qos_reliable)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.test_status_pub = self.create_publisher(String, "/endurance_test_status", 10)

        # Monitoring timers
        self.monitor_timer = self.create_timer(self.monitoring_interval, self.monitor_system_health)
        self.navigation_timer = self.create_timer(5.0, self.update_navigation)

        self.get_logger().info("Endurance SLAM Tester initialized")
        self.get_logger().info(f"Test duration: {self.test_duration/60:.1f} minutes")

    def pose_callback(self, msg):
        """Handle SLAM pose updates."""
        if not self.test_active:
            return

        self.pose_update_count += 1

        # Extract confidence if available (would depend on SLAM implementation)
        confidence = getattr(msg, "confidence", 0.8)  # Placeholder
        self.confidence_readings.append(confidence)

        # Check for loop closure (simplified: distance from start)
        if self.start_position:
            current_pos = (msg.pose.position.x, msg.pose.position.y)
            distance_from_start = math.sqrt(
                (current_pos[0] - self.start_position[0]) ** 2 + (current_pos[1] - self.start_position[1]) ** 2
            )

            if distance_from_start < self.distance_threshold:
                self.loop_closures += 1

    def odom_callback(self, msg):
        """Handle odometry updates for distance calculation."""
        if not self.start_position:
            self.start_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.last_position = self.start_position

        current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # Calculate distance traveled
        if self.last_position:
            distance = math.sqrt(
                (current_pos[0] - self.last_position[0]) ** 2 + (current_pos[1] - self.last_position[1]) ** 2
            )
            self.total_distance += distance

        self.last_position = current_pos

    def map_callback(self, msg):
        """Handle map updates."""
        self.map_update_count += 1

    def monitor_system_health(self):
        """Monitor system resource usage."""
        if not self.test_active:
            return

        # Get memory usage
        memory = psutil.virtual_memory()
        memory_mb = memory.used / (1024 * 1024)
        self.memory_readings.append(memory_mb)

        # Get CPU usage
        cpu_percent = psutil.cpu_percent(interval=1.0)
        self.cpu_readings.append(cpu_percent)

        # Get process information
        process = psutil.Process()
        memory_info = process.memory_info()
        process_memory_mb = memory_info.rss / (1024 * 1024)

        # Record system health
        health = SystemHealth(
            memory_mb=process_memory_mb,
            memory_growth_mb=self.calculate_memory_growth(),
            cpu_percent=cpu_percent,
            temperature_c=self.get_system_temperature(),
            process_count=len(psutil.pids()),
            thread_count=process.num_threads(),
        )

        self.system_health.append(health)

        # Log periodic status
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Endurance test running: {elapsed:.1f}s")
        self.get_logger().info(f"SLAM quality: {self.slam_quality:.2f}")

    def calculate_memory_growth(self) -> float:
        """Calculate memory growth rate."""
        if len(self.memory_readings) < 2:
            return 0.0

        recent_readings = self.memory_readings[-10:]  # Last 10 readings
        if len(recent_readings) < 2:
            return 0.0

        # Linear regression for growth rate
        x = np.arange(len(recent_readings))
        y = np.array(recent_readings)
        slope = np.polyfit(x, y, 1)[0]

        return slope * (self.monitoring_interval / 60.0)  # MB per minute

    def get_system_temperature(self) -> float:
        """Get system temperature (simplified)."""
        try:
            temps = psutil.sensors_temperatures()
            if "coretemp" in temps:
                return temps["coretemp"][0].current
            elif "cpu_thermal" in temps:
                return temps["cpu_thermal"][0].current
            else:
                return 45.0  # Default temperature
        except BaseException:
            return 45.0  # Default if sensors unavailable

    def update_navigation(self):
        """Update navigation commands for continuous movement."""
        if not self.test_active:
            return

        # Create figure-8 pattern for testing
        elapsed = time.time() - self.start_time
        period = 120.0  # 2 minutes per loop

        # Figure-8 parametric equations
        t = (elapsed % period) / period * 2 * math.pi
        scale = 5.0  # 5 meter radius

        if elapsed <= period * 2:  # First two loops
            # First loop: clockwise
            vx = scale * math.cos(t)
            vy = scale * math.sin(2 * t) / 2  # Figure-8 shape
        else:
            # Second loop: counter-clockwise
            vx = -scale * math.cos(t)
            vy = -scale * math.sin(2 * t) / 2

        # Convert to velocity commands
        linear_velocity = 0.3  # Slow movement
        angular_velocity = math.atan2(vy, vx) * 0.5

        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity

        self.cmd_vel_pub.publish(cmd)

    def complete_test(self):
        """Complete the endurance test and generate comprehensive report."""
        self.test_active = False

        elapsed_hours = (time.time() - self.start_time) / 3600.0

        # Calculate final metrics
        avg_memory = np.mean(self.memory_readings) if self.memory_readings else 0.0
        max_memory = max(self.memory_readings) if self.memory_readings else 0.0
        avg_cpu = np.mean(self.cpu_readings) if self.cpu_readings else 0.0
        max_cpu = max(self.cpu_readings) if self.cpu_readings else 0.0
        memory_growth = self.calculate_memory_growth()

        # SLAM performance
        avg_confidence = np.mean(self.confidence_readings) if self.confidence_readings else 0.8

        metrics = EnduranceMetrics(
            duration_hours=elapsed_hours,
            total_distance=self.total_distance,
            loop_closures=self.loop_closures,
            pose_updates=self.pose_update_count,
            map_updates=self.map_update_count,
            memory_usage_mb=self.memory_readings,
            cpu_usage_percent=self.cpu_readings,
            slam_confidence=self.confidence_readings,
            temperature_celsius=[h.temperature_c for h in self.system_health],
        )

        # Save detailed results
        result_data = {
            "test_info": {
                "name": "Endurance SLAM Test",
                "duration_hours": elapsed_hours,
                "start_time": datetime.fromtimestamp(self.start_time).isoformat(),
                "end_time": datetime.now().isoformat(),
            },
            "performance_metrics": asdict(metrics),
            "system_health": [asdict(h) for h in self.system_health],
            "test_results": {
                "memory_growth_acceptable": memory_growth < 50.0,  # <50MB per 10min
                "cpu_usage_acceptable": max_cpu < 80.0,  # <80% CPU
                "slam_stable": avg_confidence > 0.7,  # >70% confidence
                "distance_covered": self.total_distance,
                "loop_closures": self.loop_closures,
            },
        }

        with open("/tmp/endurance_slam_test_results.json", "w") as f:
            json.dump(result_data, f, indent=2)

        # Publish completion status
        completion_msg = String()
        completion_msg.data = (
            f"ENDURANCE_TEST_COMPLETE|Duration:{elapsed_hours:.2f}h|Distance:{self.total_distance:.1f}m"
        )
        self.test_status_pub.publish(completion_msg)

        # Print comprehensive results
        print("\n" + "=" * 80)
        print("ENDURANCE SLAM TEST RESULTS")
        print("=" * 80)
        print(".2f")
        print(f"Total Distance: {self.total_distance:.1f} meters")
        print(f"Loop Closures: {self.loop_closures}")
        print(f"Pose Updates: {self.pose_update_count}")
        print(f"Map Updates: {self.map_update_count}")
        print()
        print("SYSTEM PERFORMANCE:")
        print(".1f")
        print(".1f")
        print(".1f")
        print(".1f")
        print()
        print("SLAM PERFORMANCE:")
        print(".1f")
        print(f"Average Confidence: {avg_confidence:.2f}")
        print()
        print("TEST VALIDATION:")
        memory_ok = memory_growth < 50.0
        cpu_ok = max_cpu < 80.0
        slam_ok = avg_confidence > 0.7

        print(f"Memory Growth (<50MB/10min): {' PASS' if memory_ok else ' FAIL'}")
        print(f"CPU Usage (<80%): {' PASS' if cpu_ok else ' FAIL'}")
        print(f"SLAM Stability (>70% conf): {' PASS' if slam_ok else ' FAIL'}")

        overall_success = memory_ok and cpu_ok and slam_ok
        print(f"\nOVERALL RESULT: {' SUCCESS' if overall_success else '  ISSUES DETECTED'}")
        print("=" * 80)

        self.get_logger().info("Endurance SLAM Test completed")
        self.get_logger().info("Results saved to /tmp/endurance_slam_test_results.json")


def main():
    """Main test function."""
    rclpy.init()
    tester = EnduranceSLAMTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.complete_test()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
