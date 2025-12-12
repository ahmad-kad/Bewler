#!/usr/bin/env python3
"""
Autonomous Waypoint Navigation Test Scenario

Tests SLAM performance and waypoint navigation accuracy in desert terrain.
Validates sensor data quality and simulation adequacy for real hardware.
"""

import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import String


class WaypointNavigationTester(Node):
    """Test autonomous waypoint navigation with SLAM validation."""

    def __init__(self):
        super().__init__("waypoint_navigation_tester")

        # Test parameters
        self.waypoints = [
            [0.0, 0.0, 0.0],  # Start
            [100.0, 50.0, 0.0],  # Waypoint 1
            [200.0, -100.0, 0.0],  # Waypoint 2
            [-200.0, 150.0, 0.0],  # Waypoint 3
            [0.0, 0.0, 0.0],  # Return to start
        ]

        self.current_waypoint = 0
        self.waypoint_tolerance = 2.0  # meters
        self.max_test_time = 1800.0  # 30 minutes
        self.start_time = None

        # Test data collection
        self.slam_poses = []
        self.gps_poses = []
        self.odom_poses = []
        self.test_results = {}

        # QoS profiles
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subscribers
        self.slam_pose_sub = self.create_subscription(
            PoseStamped, "/slam/pose/fused", self.slam_pose_callback, qos_profile
        )
        self.gps_sub = self.create_subscription(NavSatFix, "/rover/gps/fix", self.gps_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)

        self.get_logger().info("Waypoint Navigation Tester initialized")
        self.get_logger().info(f"Testing {len(self.waypoints)} waypoints over {self.max_test_time/60:.1f} minutes")

    def slam_pose_callback(self, msg):
        """Record SLAM pose data."""
        pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.slam_poses.append({"pose": pose, "timestamp": time.time(), "frame_id": msg.header.frame_id})

    def gps_callback(self, msg):
        """Record GPS data."""
        # Convert lat/lon to local coordinates (simplified)
        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
        # This is a placeholder - real implementation would use proper projection
        x = (lon - -4.5895) * 111320 * np.cos(np.radians(lat))
        y = (lat - 137.4417) * 111320

        self.gps_poses.append(
            {
                "pose": [x, y, alt],
                "timestamp": time.time(),
                "accuracy": msg.position_covariance[0] if msg.position_covariance else 0.0,
            }
        )

    def odom_callback(self, msg):
        """Record odometry data."""
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.odom_poses.append(
            {
                "pose": pose,
                "timestamp": time.time(),
                "velocity": [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z],
            }
        )

    def laser_callback(self, msg):
        """Record laser scan data for obstacle detection."""
        # Count valid range readings
        valid_ranges = sum(1 for r in msg.ranges if msg.range_min <= r <= msg.range_max)
        self.get_logger().debug(f"Laser scan: {valid_ranges}/{len(msg.ranges)} valid ranges")

    def run_test(self):
        """Main test execution."""
        if self.start_time is None:
            self.start_time = time.time()
            self.publish_next_waypoint()
            return

        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # Check timeout
        if elapsed_time > self.max_test_time:
            self.get_logger().warn("Test timeout reached")
            self.complete_test()
            return

        # Check if current waypoint reached
        if self.check_waypoint_reached():
            self.get_logger().info(f"Waypoint {self.current_waypoint} reached!")
            self.current_waypoint += 1

            if self.current_waypoint >= len(self.waypoints):
                self.get_logger().info("All waypoints completed!")
                self.complete_test()
            else:
                self.publish_next_waypoint()

        # Publish test status
        self.publish_test_status(elapsed_time)

    def publish_next_waypoint(self):
        """Publish next waypoint goal."""
        if self.current_waypoint < len(self.waypoints):
            wp = self.waypoints[self.current_waypoint]

            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "map"
            goal.pose.position.x = wp[0]
            goal.pose.position.y = wp[1]
            goal.pose.position.z = wp[2]
            goal.pose.orientation.w = 1.0

            self.goal_pub.publish(goal)
            self.get_logger().info(f"Published waypoint {self.current_waypoint}: {wp}")

    def check_waypoint_reached(self):
        """Check if current waypoint has been reached."""
        if not self.slam_poses:
            return False

        current_pose = self.slam_poses[-1]["pose"]
        target_waypoint = self.waypoints[self.current_waypoint]

        distance = np.sqrt((current_pose[0] - target_waypoint[0]) ** 2 + (current_pose[1] - target_waypoint[1]) ** 2)

        return distance < self.waypoint_tolerance

    def publish_test_status(self, elapsed_time):
        """Publish current test status."""
        status = {
            "test_type": "waypoint_navigation",
            "elapsed_time": elapsed_time,
            "current_waypoint": self.current_waypoint,
            "total_waypoints": len(self.waypoints),
            "slam_poses_count": len(self.slam_poses),
            "gps_poses_count": len(self.gps_poses),
            "odom_poses_count": len(self.odom_poses),
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        # Would publish to status topic in real implementation

    def complete_test(self):
        """Complete test and generate results."""
        self.get_logger().info("Completing waypoint navigation test...")

        # Calculate test results
        self.test_results = self.calculate_test_results()

        # Log results
        self.log_test_results()

        # Stop test
        self.test_timer.cancel()

        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def calculate_test_results(self):
        """Calculate test performance metrics."""
        results = {}

        # Waypoint accuracy
        if len(self.slam_poses) > 0:
            final_pose = self.slam_poses[-1]["pose"]
            start_pose = self.slam_poses[0]["pose"]
            loop_closure_error = np.sqrt((final_pose[0] - start_pose[0]) ** 2 + (final_pose[1] - start_pose[1]) ** 2)
            results["loop_closure_error"] = loop_closure_error

        # SLAM performance
        results["slam_poses_count"] = len(self.slam_poses)
        results["gps_poses_count"] = len(self.gps_poses)
        results["odom_poses_count"] = len(self.odom_poses)

        # Data quality metrics
        if len(self.slam_poses) > 1:
            slam_distances = []
            for i in range(1, len(self.slam_poses)):
                p1 = self.slam_poses[i - 1]["pose"]
                p2 = self.slam_poses[i]["pose"]
                dist = np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
                slam_distances.append(dist)

            results["slam_distance_traveled"] = sum(slam_distances)
            results["slam_pose_frequency"] = len(self.slam_poses) / (time.time() - self.start_time)

        return results

    def log_test_results(self):
        """Log test results."""
        self.get_logger().info("=== WAYPOINT NAVIGATION TEST RESULTS ===")

        for key, value in self.test_results.items():
            self.get_logger().info(f"{key}: {value}")

        # Validation criteria
        self.get_logger().info("=== VALIDATION CRITERIA ===")

        if "loop_closure_error" in self.test_results:
            lce = self.test_results["loop_closure_error"]
            if lce < 1.0:
                self.get_logger().info(f"✅ Loop closure error: {lce:.3f}m (< 1m required)")
            else:
                self.get_logger().warn(f"❌ Loop closure error: {lce:.3f}m (>= 1m)")

        if "slam_pose_frequency" in self.test_results:
            freq = self.test_results["slam_pose_frequency"]
            if freq > 5.0:
                self.get_logger().info(f"✅ SLAM frequency: {freq:.1f} Hz (> 5Hz required)")
            else:
                self.get_logger().warn(f"❌ SLAM frequency: {freq:.1f} Hz (< 5Hz)")


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        tester = WaypointNavigationTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
    except Exception as e:
        tester.get_logger().error(f"Test failed: {str(e)}")
    finally:
        if "tester" in locals():
            tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
