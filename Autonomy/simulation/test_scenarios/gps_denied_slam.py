#!/usr/bin/env python3

"""
GPS-Denied SLAM Test Scenario

This test validates visual-inertial SLAM performance during GPS-denied operation
in an indoor warehouse environment with feature-rich walls and obstacles.
"""

import json
import math
import time
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, LaserScan, NavSatFix, PointCloud2
from std_msgs.msg import Bool, String


class TestStatus(Enum):
    """Test execution status."""
    INITIALIZING = "initializing"
    GPS_AVAILABLE = "gps_available"
    GPS_DENIED = "gps_denied"
    GPS_RECOVERY = "gps_recovery"
    COMPLETED = "completed"
    FAILED = "failed"
    TIMEOUT = "timeout"


@dataclass
class SLAMDriftMetrics:
    """SLAM drift metrics during GPS-denied operation."""
    initial_position: List[float] = None
    final_position: List[float] = None
    total_drift: float = 0.0  # meters
    drift_per_minute: float = 0.0  # meters per minute
    gps_reacquisition_time: float = 0.0  # seconds
    visual_features_count: int = 0
    depth_quality: float = 0.0  # 0-1
    navigation_continuity: bool = True


@dataclass
class TestResults:
    """Complete test results."""
    test_name: str
    start_time: float
    end_time: float
    duration: float
    status: TestStatus
    gps_denied_duration: float
    slam_drift_metrics: SLAMDriftMetrics
    gps_transitions: List[Dict[str, float]]
    visual_features_over_time: List[Dict[str, Any]]
    errors: List[str]
    warnings: List[str]


class GPSDeniedSLAMTest(Node):
    """Test node for GPS-denied SLAM operation validation."""

    def __init__(self):
        super().__init__('gps_denied_slam_test')

        # Test configuration
        self.test_name = "gps_denied_slam"
        self.start_time = time.time()
        self.status = TestStatus.INITIALIZING

        # Test phases
        self.gps_available_phase_duration = 30.0  # seconds
        self.gps_denied_phase_duration = 120.0  # seconds
        self.gps_recovery_phase_duration = 30.0  # seconds

        self.phase_start_time = None
        self.gps_disabled_time = None
        self.gps_recovery_start_time = None

        # Data collection
        self.ground_truth_poses = []
        self.slam_poses = []
        self.imu_data = []
        self.gps_data = []
        self.lidar_data = []
        self.camera_data = []
        self.visual_features_data = []

        # GPS transition tracking
        self.gps_transitions = []
        self.gps_available = True

        # SLAM drift tracking
        self.slam_drift_metrics = SLAMDriftMetrics()
        self.gps_denied_start_pose = None
        self.gps_denied_end_pose = None

        # Test state
        self.test_results = None
        self.errors = []
        self.warnings = []

        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.gps_enable_pub = self.create_publisher(Bool, '/rover/gps/enable', 10)
        self.test_status_pub = self.create_publisher(String, '/test/status', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/rover/odom', self.odom_callback, qos_profile
        )
        self.slam_pose_sub = self.create_subscription(
            PoseWithCovariance, '/slam/pose/fused', self.slam_pose_callback, qos_profile
        )
        self.imu_sub = self.create_subscription(
            Imu, '/rover/imu', self.imu_callback, qos_profile
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/rover/gps', self.gps_callback, qos_profile
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/rover/scan', self.lidar_callback, qos_profile
        )
        self.camera_sub = self.create_subscription(
            Image, '/rover/camera/image_raw', self.camera_callback, qos_profile
        )
        self.depth_sub = self.create_subscription(
            Image, '/rover/camera/depth/image_raw', self.depth_callback, qos_profile
        )
        self.points_sub = self.create_subscription(
            PointCloud2, '/rover/camera/depth/points', self.points_callback, qos_profile
        )

        # Timers
        self.test_timer = self.create_timer(0.1, self.test_loop)
        self.metrics_timer = self.create_timer(1.0, self.update_metrics)
        self.status_timer = self.create_timer(5.0, self.publish_status)

        # Initialize test
        self.get_logger().info(f"Starting {self.test_name} test")
        self.start_navigation()

    def odom_callback(self, msg: Odometry):
        """Record ground truth pose from odometry."""
        pose = msg.pose.pose
        self.ground_truth_poses.append({
            'timestamp': time.time(),
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'qx': pose.orientation.x,
            'qy': pose.orientation.y,
            'qz': pose.orientation.z,
            'qw': pose.orientation.w
        })

    def slam_pose_callback(self, msg: PoseWithCovariance):
        """Record SLAM estimated pose."""
        pose = msg.pose
        self.slam_poses.append({
            'timestamp': time.time(),
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'qx': pose.orientation.x,
            'qy': pose.orientation.y,
            'qz': pose.orientation.z,
            'qw': pose.orientation.w,
            'covariance': msg.covariance
        })

    def imu_callback(self, msg: Imu):
        """Record IMU data."""
        self.imu_data.append({
            'timestamp': time.time(),
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        })

    def gps_callback(self, msg: NavSatFix):
        """Record GPS data and track availability."""
        self.gps_data.append({
            'timestamp': time.time(),
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance': msg.position_covariance.tolist(),
            'status': msg.status.status
        })

        # Track GPS transitions
        if msg.status.status >= 0 and not self.gps_available:
            # GPS recovered
            self.gps_available = True
            self.gps_recovery_start_time = time.time()
            self.gps_transitions.append({
                'timestamp': time.time(),
                'event': 'gps_recovered',
                'status': msg.status.status
            })
            self.get_logger().info("GPS signal recovered")

        elif msg.status.status < 0 and self.gps_available:
            # GPS lost
            self.gps_available = False
            self.gps_transitions.append({
                'timestamp': time.time(),
                'event': 'gps_lost',
                'status': msg.status.status
            })
            self.get_logger().warn("GPS signal lost")

    def lidar_callback(self, msg: LaserScan):
        """Record LiDAR data."""
        self.lidar_data.append({
            'timestamp': time.time(),
            'ranges': msg.ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        })

    def camera_callback(self, msg: Image):
        """Record camera data and estimate visual features."""
        self.camera_data.append({
            'timestamp': time.time(),
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'step': msg.step
        })

        # Estimate visual features (simplified)
        # In real implementation, this would use OpenCV feature detection
        estimated_features = self.estimate_visual_features(msg)
        self.visual_features_data.append({
            'timestamp': time.time(),
            'feature_count': estimated_features,
            'gps_available': self.gps_available
        })

    def depth_callback(self, msg: Image):
        """Record depth camera data."""
        # Analyze depth data quality
        depth_quality = self.analyze_depth_quality(msg)
        if hasattr(self, 'depth_quality_samples'):
            self.depth_quality_samples.append(depth_quality)
        else:
            self.depth_quality_samples = [depth_quality]

    def points_callback(self, msg: PointCloud2):
        """Record point cloud data."""
        # This would be used for more detailed 3D analysis
        pass

    def estimate_visual_features(self, image_msg: Image) -> int:
        """Estimate number of visual features in image (simplified)."""
        # Simplified feature estimation based on image properties
        # In real implementation, would use OpenCV feature detection
        base_features = 100
        if image_msg.encoding == 'rgb8':
            base_features = 150
        elif image_msg.encoding == 'bgr8':
            base_features = 150

        # Add some variation based on image size
        size_factor = (image_msg.width * image_msg.height) / (640 * 480)
        return int(base_features * size_factor)

    def analyze_depth_quality(self, depth_msg: Image) -> float:
        """Analyze depth data quality (0-1, higher is better)."""
        # Simplified depth quality analysis
        # In real implementation, would analyze actual depth data
        if depth_msg.encoding == '32FC1':
            return 0.9  # Good depth format
        elif depth_msg.encoding == '16UC1':
            return 0.7  # Decent depth format
        else:
            return 0.3  # Poor depth format

    def start_navigation(self):
        """Start autonomous navigation for the test."""
        # Publish a simple forward movement command
        cmd = Twist()
        cmd.linear.x = 0.5  # 0.5 m/s forward
        cmd.angular.z = 0.1  # Slight turn for testing
        self.cmd_vel_pub.publish(cmd)

        self.phase_start_time = time.time()
        self.status = TestStatus.GPS_AVAILABLE
        self.get_logger().info("Started navigation with GPS available")

    def test_loop(self):
        """Main test execution loop."""
        if self.status == TestStatus.INITIALIZING:
            if len(self.slam_poses) > 0:
                self.status = TestStatus.GPS_AVAILABLE
                self.phase_start_time = time.time()
                self.get_logger().info("Test started - SLAM data received")
            return

        current_time = time.time()
        phase_duration = current_time - self.phase_start_time

        if self.status == TestStatus.GPS_AVAILABLE:
            if phase_duration >= self.gps_available_phase_duration:
                # Transition to GPS-denied phase
                self.disable_gps()
                self.status = TestStatus.GPS_DENIED
                self.phase_start_time = current_time
                self.gps_disabled_time = current_time
                self.gps_denied_start_pose = self.slam_poses[-1] if self.slam_poses else None
                self.get_logger().info("Transitioned to GPS-denied phase")

        elif self.status == TestStatus.GPS_DENIED:
            if phase_duration >= self.gps_denied_phase_duration:
                # Transition to GPS recovery phase
                self.enable_gps()
                self.status = TestStatus.GPS_RECOVERY
                self.phase_start_time = current_time
                self.gps_denied_end_pose = self.slam_poses[-1] if self.slam_poses else None
                self.get_logger().info("Transitioned to GPS recovery phase")

        elif self.status == TestStatus.GPS_RECOVERY:
            if phase_duration >= self.gps_recovery_phase_duration:
                # Test completed
                self.status = TestStatus.COMPLETED
                self.complete_test()

    def disable_gps(self):
        """Disable GPS sensor."""
        gps_enable_msg = Bool()
        gps_enable_msg.data = False
        self.gps_enable_pub.publish(gps_enable_msg)

    def enable_gps(self):
        """Enable GPS sensor."""
        gps_enable_msg = Bool()
        gps_enable_msg.data = True
        self.gps_enable_pub.publish(gps_enable_msg)

    def update_metrics(self):
        """Update SLAM drift metrics during GPS-denied operation."""
        if self.status != TestStatus.GPS_DENIED or not self.slam_poses:
            return

        # Calculate drift during GPS-denied period
        if self.gps_denied_start_pose and len(self.slam_poses) > 0:
            start_pose = self.gps_denied_start_pose
            current_pose = self.slam_poses[-1]

            dx = current_pose['x'] - start_pose['x']
            dy = current_pose['y'] - start_pose['y']
            total_drift = math.sqrt(dx*dx + dy*dy)

            # Calculate drift per minute
            gps_denied_duration = time.time() - self.gps_disabled_time
            drift_per_minute = total_drift / (gps_denied_duration / 60.0) if gps_denied_duration > 0 else 0.0

            self.slam_drift_metrics.total_drift = total_drift
            self.slam_drift_metrics.drift_per_minute = drift_per_minute

        # Update visual features count
        if self.visual_features_data:
            recent_features = [f['feature_count'] for f in self.visual_features_data[-10:]]
            self.slam_drift_metrics.visual_features_count = int(np.mean(recent_features))

        # Update depth quality
        if hasattr(self, 'depth_quality_samples') and self.depth_quality_samples:
            self.slam_drift_metrics.depth_quality = np.mean(self.depth_quality_samples[-10:])

    def complete_test(self):
        """Complete the test and generate results."""
        self.end_time = time.time()
        duration = self.end_time - self.start_time

        # Calculate final metrics
        self.calculate_final_metrics()

        # Create test results
        self.test_results = TestResults(
            test_name=self.test_name,
            start_time=self.start_time,
            end_time=self.end_time,
            duration=duration,
            status=self.status,
            gps_denied_duration=self.gps_denied_phase_duration,
            slam_drift_metrics=self.slam_drift_metrics,
            gps_transitions=self.gps_transitions,
            visual_features_over_time=self.visual_features_data,
            errors=self.errors,
            warnings=self.warnings
        )

        # Save results
        self.save_results()

        # Log completion
        self.get_logger().info(f"Test completed in {duration:.2f}s")
        self.get_logger().info(f"GPS-denied duration: {self.gps_denied_phase_duration:.2f}s")
        self.get_logger().info(f"Total drift: {self.slam_drift_metrics.total_drift:.3f}m")
        self.get_logger().info(f"Drift per minute: {self.slam_drift_metrics.drift_per_minute:.3f}m/min")
        self.get_logger().info(f"Visual features: {self.slam_drift_metrics.visual_features_count}")
        self.get_logger().info(f"Depth quality: {self.slam_drift_metrics.depth_quality:.2f}")

    def calculate_final_metrics(self):
        """Calculate final test metrics."""
        # Calculate GPS reacquisition time
        if self.gps_recovery_start_time and self.gps_available:
            self.slam_drift_metrics.gps_reacquisition_time = time.time() - self.gps_recovery_start_time

        # Calculate navigation continuity
        # Check if SLAM poses were continuous during GPS-denied period
        if len(self.slam_poses) > 10:
            pose_intervals = []
            for i in range(1, len(self.slam_poses)):
                interval = self.slam_poses[i]['timestamp'] - self.slam_poses[i-1]['timestamp']
                pose_intervals.append(interval)

            # Check for large gaps in pose updates
            max_interval = max(pose_intervals) if pose_intervals else 0
            self.slam_drift_metrics.navigation_continuity = max_interval < 2.0  # Less than 2s gap

        # Store initial and final positions
        if self.gps_denied_start_pose:
            self.slam_drift_metrics.initial_position = [
                self.gps_denied_start_pose['x'],
                self.gps_denied_start_pose['y'],
                self.gps_denied_start_pose['z']
            ]

        if self.gps_denied_end_pose:
            self.slam_drift_metrics.final_position = [
                self.gps_denied_end_pose['x'],
                self.gps_denied_end_pose['y'],
                self.gps_denied_end_pose['z']
            ]

    def save_results(self):
        """Save test results to file."""
        if not self.test_results:
            return

        results_file = f"/tmp/{self.test_name}_results_{int(self.start_time)}.json"

        # Convert to serializable format
        results_dict = asdict(self.test_results)
        results_dict['status'] = self.test_results.status.value

        with open(results_file, 'w') as f:
            json.dump(results_dict, f, indent=2)

        self.get_logger().info(f"Results saved to {results_file}")

    def publish_status(self):
        """Publish current test status."""
        status_msg = String()
        status_msg.data = f"{self.test_name}: {self.status.value} - GPS: {'ON' if self.gps_available else 'OFF'}"
        self.test_status_pub.publish(status_msg)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        test_node = GPSDeniedSLAMTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
    except Exception as e:
        test_node.get_logger().error(f"Test failed with error: {str(e)}")
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
