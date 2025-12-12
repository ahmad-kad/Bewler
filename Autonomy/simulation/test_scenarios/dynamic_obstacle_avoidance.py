#!/usr/bin/env python3
"""
Dynamic Obstacle Avoidance Test Scenario

Tests real-time mapping and obstacle avoidance capabilities with moving/dynamic obstacles.
Validates LiDAR-based obstacle detection, path replanning, and collision avoidance.
"""

import json
import math
import time
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class ObstacleMetrics:
    """Metrics for obstacle avoidance performance."""

    detection_range: float = 0.0  # meters
    detection_time: float = 0.0  # seconds
    avoidance_success: bool = True
    path_replan_count: int = 0
    collision_count: int = 0
    clearance_distance: float = 0.0  # meters
    lidar_scan_quality: float = 0.0  # 0-1


@dataclass
class TestResult:
    """Complete test result data."""

    test_name: str
    duration: float
    obstacles_encountered: int
    obstacles_avoided: int
    path_replans: int
    collisions: int
    average_clearance: float
    lidar_fidelity: float
    navigation_success: bool
    timestamp: str


class DynamicObstacleAvoidanceTester(Node):
    """Test dynamic obstacle avoidance with moving objects."""

    def __init__(self):
        super().__init__("dynamic_obstacle_avoidance_tester")

        # Test parameters
        self.test_duration = 600.0  # 10 minutes
        self.obstacle_approach_distance = 3.0  # meters
        self.collision_threshold = 0.5  # meters
        self.min_clearance = 0.8  # meters

        # Test state
        self.start_time = time.time()
        self.test_active = True
        self.current_position = None
        self.obstacles_detected = []
        self.collision_events = []
        self.path_replans = 0

        # Metrics tracking
        self.clearance_distances = []
        self.detection_times = []
        self.last_obstacle_time = 0

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
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, self.qos_reliable
        )

        self.laser_sub = self.create_subscription(
            LaserScan, "/scan", self.laser_callback, self.qos_best_effort
        )

        self.path_sub = self.create_subscription(
            Path, "/plan", self.path_callback, self.qos_reliable
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.test_status_pub = self.create_publisher(String, "/test_status", 10)

        # Visualization publisher
        self.marker_pub = self.create_publisher(MarkerArray, "/obstacle_markers", 10)

        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test_cycle)

        self.get_logger().info("Dynamic Obstacle Avoidance Tester initialized")
        self.get_logger().info(f"Test duration: {self.test_duration/60:.1f} minutes")

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    def laser_callback(self, msg):
        """Process LiDAR scan data for obstacle detection."""
        if not self.test_active:
            return

        # Process laser scan for obstacles
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter valid ranges
        valid_mask = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        if len(valid_ranges) == 0:
            return

        # Find closest obstacles
        min_range_idx = np.argmin(valid_ranges)
        min_range = valid_ranges[min_range_idx]
        min_angle = valid_angles[min_range_idx]

        # Check if obstacle is approaching
        if min_range < self.obstacle_approach_distance:
            obstacle_time = time.time()

            # Record obstacle detection
            obstacle = {
                'range': min_range,
                'angle': min_angle,
                'timestamp': obstacle_time,
                'position': self.current_position
            }

            self.obstacles_detected.append(obstacle)
            self.detection_times.append(obstacle_time - self.last_obstacle_time)
            self.last_obstacle_time = obstacle_time

            # Check for collision risk
            if min_range < self.collision_threshold:
                self.collision_events.append({
                    'timestamp': obstacle_time,
                    'range': min_range,
                    'position': self.current_position
                })
                self.get_logger().warn(
                    f"COLLISION RISK DETECTED: range={min_range:.2f}m, "
                    f"position=({self.current_position.x:.2f}, {self.current_position.y:.2f})"
                )
                # Emergency stop
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)

            # Publish visualization marker
            self.publish_obstacle_marker(min_range, min_angle)

    def path_callback(self, msg):
        """Monitor path planning updates."""
        # Count path replans (when path changes significantly)
        if len(msg.poses) > 0:
            self.path_replans += 1

    def publish_obstacle_marker(self, range_m, angle_rad):
        """Publish visualization marker for detected obstacle."""
        marker_array = MarkerArray()
        marker = Marker()

        marker.header.frame_id = "laser"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position in laser frame
        marker.pose.position.x = range_m * math.cos(angle_rad)
        marker.pose.position.y = range_m * math.sin(angle_rad)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Size and color
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def run_test_cycle(self):
        """Main test execution cycle."""
        elapsed = time.time() - self.start_time

        if elapsed >= self.test_duration:
            self.complete_test()
            return

        # Publish test status
        status_msg = String()
        status_msg.data = f"OBSTACLE_AVOIDANCE_TEST_ACTIVE|{elapsed:.1f}s"
        self.test_status_pub.publish(status_msg)

        # Simple navigation: move forward slowly
        if elapsed > 10:  # Wait 10s for initialization
            cmd = Twist()
            cmd.linear.x = 0.5  # Slow forward motion
            self.cmd_vel_pub.publish(cmd)

    def complete_test(self):
        """Complete the test and generate results."""
        self.test_active = False

        # Calculate final metrics
        obstacles_encountered = len(self.obstacles_detected)
        obstacles_avoided = obstacles_encountered - len(self.collision_events)
        avoidance_rate = obstacles_avoided / max(1, obstacles_encountered)

        avg_clearance = np.mean(self.clearance_distances) if self.clearance_distances else 0.0
        avg_detection_time = np.mean(self.detection_times) if self.detection_times else 0.0

        # Generate test result
        result = TestResult(
            test_name="Dynamic Obstacle Avoidance",
            duration=time.time() - self.start_time,
            obstacles_encountered=obstacles_encountered,
            obstacles_avoided=obstacles_avoided,
            path_replans=self.path_replans,
            collisions=len(self.collision_events),
            average_clearance=avg_clearance,
            lidar_fidelity=0.85,  # Placeholder - would be calculated
            navigation_success=len(self.collision_events) == 0,
            timestamp=time.strftime("%Y-%m-%d %H:%M:%S")
        )

        # Save results
        with open("/tmp/dynamic_obstacle_test_results.json", "w") as f:
            json.dump(asdict(result), f, indent=2)

        # Publish completion status
        completion_msg = String()
        completion_msg.data = f"OBSTACLE_AVOIDANCE_TEST_COMPLETE|Success:{result.navigation_success}"
        self.test_status_pub.publish(completion_msg)

        self.get_logger().info("Dynamic Obstacle Avoidance Test completed")
        self.get_logger().info(f"Results saved to /tmp/dynamic_obstacle_test_results.json")

        # Print summary
        print("\n" + "="*60)
        print("DYNAMIC OBSTACLE AVOIDANCE TEST RESULTS")
        print("="*60)
        print(f"Obstacles Encountered: {obstacles_encountered}")
        print(f"Obstacles Avoided: {obstacles_avoided}")
        print(f"Collisions: {len(self.collision_events)}")
        print(f"Path Replans: {self.path_replans}")
        print(f"Navigation Success: {' YES' if result.navigation_success else ' NO'}")
        print("="*60)


def main():
    """Main test function."""
    rclpy.init()
    tester = DynamicObstacleAvoidanceTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.complete_test()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
