#!/usr/bin/env python3

"""
Sensor Quality Analyzer

This tool analyzes sensor data quality from Gazebo simulation and compares
it against real-world sensor specifications to assess simulation fidelity.
"""

import json
import math
import os
import time
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, LaserScan, NavSatFix, PointCloud2


@dataclass
class SensorCharacteristics:
    """Sensor data characteristics."""
    update_rate: float = 0.0  # Hz
    noise_level: float = 0.0  # Standard deviation
    bias_drift: float = 0.0  # Drift over time
    data_completeness: float = 0.0  # 0-1
    timestamp_sync_error: float = 0.0  # seconds
    range_accuracy: float = 0.0  # meters
    angular_resolution: float = 0.0  # degrees


@dataclass
class SensorQualityReport:
    """Complete sensor quality assessment report."""
    sensor_type: str
    simulation_characteristics: SensorCharacteristics
    real_world_specs: Dict[str, float]
    fidelity_score: float  # 0-1
    quality_grade: str  # A, B, C, D, F
    recommendations: List[str]
    test_duration: float
    data_points: int


class SensorQualityAnalyzer(Node):
    """Node for analyzing sensor data quality in simulation."""

    def __init__(self):
        super().__init__('sensor_quality_analyzer')

        # Configuration
        self.analysis_duration = 60.0  # seconds
        self.start_time = time.time()
        self.analysis_complete = False

        # Data collection
        self.imu_data = deque(maxlen=10000)
        self.gps_data = deque(maxlen=1000)
        self.lidar_data = deque(maxlen=1000)
        self.camera_data = deque(maxlen=1000)
        self.depth_data = deque(maxlen=1000)
        self.odom_data = deque(maxlen=1000)

        # Analysis results
        self.sensor_reports = {}

        # Load real-world specifications
        self.load_real_world_specs()

        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers
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
        self.odom_sub = self.create_subscription(
            Odometry, '/rover/odom', self.odom_callback, qos_profile
        )

        # Timer for analysis
        self.analysis_timer = self.create_timer(1.0, self.analysis_loop)

        self.get_logger().info("Sensor Quality Analyzer started")

    def load_real_world_specs(self):
        """Load real-world sensor specifications."""
        # Default specifications (would be loaded from config file)
        self.real_world_specs = {
            'imu': {
                'gyro_bias_stability': 0.02,  # degrees/s
                'gyro_noise_density': 0.01,  # degrees/s/√Hz
                'accel_bias_stability': 2.0,  # mg
                'accel_noise_density': 1.0,  # mg/√Hz
                'update_rate': 100.0  # Hz
            },
            'gps': {
                'position_accuracy': 2.5,  # meters (95% confidence)
                'velocity_accuracy': 0.1,  # m/s
                'update_rate': 1.0  # Hz
            },
            'lidar': {
                'range_accuracy': 0.03,  # meters
                'angular_resolution': 0.25,  # degrees
                'update_rate': 10.0  # Hz
            },
            'camera': {
                'depth_accuracy': 0.02,  # meters at 1m
                'rgb_resolution': 640,  # pixels
                'update_rate': 30.0  # Hz
            }
        }

    def imu_callback(self, msg: Imu):
        """Collect IMU data."""
        self.imu_data.append({
            'timestamp': time.time(),
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        })

    def gps_callback(self, msg: NavSatFix):
        """Collect GPS data."""
        self.gps_data.append({
            'timestamp': time.time(),
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance': msg.position_covariance.tolist(),
            'status': msg.status.status
        })

    def lidar_callback(self, msg: LaserScan):
        """Collect LiDAR data."""
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
        """Collect camera data."""
        self.camera_data.append({
            'timestamp': time.time(),
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'step': msg.step
        })

    def depth_callback(self, msg: Image):
        """Collect depth camera data."""
        self.depth_data.append({
            'timestamp': time.time(),
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'step': msg.step
        })

    def odom_callback(self, msg: Odometry):
        """Collect odometry data for reference."""
        self.odom_data.append({
            'timestamp': time.time(),
            'position': [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            'orientation': [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        })

    def analysis_loop(self):
        """Main analysis loop."""
        current_time = time.time()

        if current_time - self.start_time >= self.analysis_duration and not self.analysis_complete:
            self.perform_analysis()
            self.analysis_complete = True
            self.get_logger().info("Sensor quality analysis completed")

    def perform_analysis(self):
        """Perform comprehensive sensor quality analysis."""
        self.get_logger().info("Starting sensor quality analysis...")

        # Analyze each sensor type
        if self.imu_data:
            self.analyze_imu()

        if self.gps_data:
            self.analyze_gps()

        if self.lidar_data:
            self.analyze_lidar()

        if self.camera_data:
            self.analyze_camera()

        if self.depth_data:
            self.analyze_depth_camera()

        # Generate overall report
        self.generate_overall_report()

    def analyze_imu(self):
        """Analyze IMU data quality."""
        if len(self.imu_data) < 10:
            return

        # Extract data
        timestamps = [d['timestamp'] for d in self.imu_data]
        accel_data = np.array([d['linear_acceleration'] for d in self.imu_data])
        gyro_data = np.array([d['angular_velocity'] for d in self.imu_data])

        # Calculate update rate
        time_diffs = np.diff(timestamps)
        update_rate = 1.0 / np.mean(time_diffs) if len(time_diffs) > 0 else 0.0

        # Calculate noise levels
        accel_noise = np.std(accel_data, axis=0)
        gyro_noise = np.std(gyro_data, axis=0)

        # Calculate bias drift (simplified)
        accel_bias_drift = self.calculate_bias_drift(accel_data)
        gyro_bias_drift = self.calculate_bias_drift(gyro_data)

        # Calculate data completeness
        expected_samples = update_rate * self.analysis_duration
        data_completeness = len(self.imu_data) / expected_samples if expected_samples > 0 else 0.0

        # Calculate timestamp synchronization error
        timestamp_sync_error = np.std(time_diffs) if len(time_diffs) > 0 else 0.0

        # Create characteristics
        characteristics = SensorCharacteristics(
            update_rate=update_rate,
            noise_level=np.mean(np.concatenate([accel_noise, gyro_noise])),
            bias_drift=np.mean([accel_bias_drift, gyro_bias_drift]),
            data_completeness=min(data_completeness, 1.0),
            timestamp_sync_error=timestamp_sync_error
        )

        # Calculate fidelity score
        fidelity_score = self.calculate_imu_fidelity(characteristics)

        # Generate recommendations
        recommendations = self.generate_imu_recommendations(characteristics, fidelity_score)

        # Create report
        report = SensorQualityReport(
            sensor_type='IMU',
            simulation_characteristics=characteristics,
            real_world_specs=self.real_world_specs['imu'],
            fidelity_score=fidelity_score,
            quality_grade=self.score_to_grade(fidelity_score),
            recommendations=recommendations,
            test_duration=self.analysis_duration,
            data_points=len(self.imu_data)
        )

        self.sensor_reports['imu'] = report
        self.get_logger().info(f"IMU Analysis: Fidelity {fidelity_score:.2f}, Grade {report.quality_grade}")

    def analyze_gps(self):
        """Analyze GPS data quality."""
        if len(self.gps_data) < 5:
            return

        # Extract data
        timestamps = [d['timestamp'] for d in self.gps_data]
        positions = np.array([[d['latitude'], d['longitude'], d['altitude']] for d in self.gps_data])
        covariances = [d['position_covariance'] for d in self.gps_data]

        # Calculate update rate
        time_diffs = np.diff(timestamps)
        update_rate = 1.0 / np.mean(time_diffs) if len(time_diffs) > 0 else 0.0

        # Calculate position accuracy from covariance
        position_errors = []
        for cov in covariances:
            if len(cov) >= 9:
                # Extract diagonal elements (variances)
                var_x, var_y, var_z = cov[0], cov[4], cov[8]
                position_error = math.sqrt(var_x + var_y)  # 2D position error
                position_errors.append(position_error)

        position_accuracy = np.mean(position_errors) if position_errors else 0.0

        # Calculate data completeness
        expected_samples = update_rate * self.analysis_duration
        data_completeness = len(self.gps_data) / expected_samples if expected_samples > 0 else 0.0

        # Create characteristics
        characteristics = SensorCharacteristics(
            update_rate=update_rate,
            noise_level=position_accuracy,
            data_completeness=min(data_completeness, 1.0),
            range_accuracy=position_accuracy
        )

        # Calculate fidelity score
        fidelity_score = self.calculate_gps_fidelity(characteristics)

        # Generate recommendations
        recommendations = self.generate_gps_recommendations(characteristics, fidelity_score)

        # Create report
        report = SensorQualityReport(
            sensor_type='GPS',
            simulation_characteristics=characteristics,
            real_world_specs=self.real_world_specs['gps'],
            fidelity_score=fidelity_score,
            quality_grade=self.score_to_grade(fidelity_score),
            recommendations=recommendations,
            test_duration=self.analysis_duration,
            data_points=len(self.gps_data)
        )

        self.sensor_reports['gps'] = report
        self.get_logger().info(f"GPS Analysis: Fidelity {fidelity_score:.2f}, Grade {report.quality_grade}")

    def analyze_lidar(self):
        """Analyze LiDAR data quality."""
        if len(self.lidar_data) < 5:
            return

        # Extract data
        timestamps = [d['timestamp'] for d in self.lidar_data]
        ranges_data = [d['ranges'] for d in self.lidar_data]
        angle_increments = [d['angle_increment'] for d in self.lidar_data]

        # Calculate update rate
        time_diffs = np.diff(timestamps)
        update_rate = 1.0 / np.mean(time_diffs) if len(time_diffs) > 0 else 0.0

        # Calculate angular resolution
        angular_resolution = np.mean(angle_increments) * 180.0 / math.pi  # Convert to degrees

        # Calculate range accuracy (simplified - would need ground truth)
        range_accuracy = 0.03  # Placeholder - would calculate from actual data

        # Calculate data completeness
        expected_samples = update_rate * self.analysis_duration
        data_completeness = len(self.lidar_data) / expected_samples if expected_samples > 0 else 0.0

        # Create characteristics
        characteristics = SensorCharacteristics(
            update_rate=update_rate,
            data_completeness=min(data_completeness, 1.0),
            range_accuracy=range_accuracy,
            angular_resolution=angular_resolution
        )

        # Calculate fidelity score
        fidelity_score = self.calculate_lidar_fidelity(characteristics)

        # Generate recommendations
        recommendations = self.generate_lidar_recommendations(characteristics, fidelity_score)

        # Create report
        report = SensorQualityReport(
            sensor_type='LiDAR',
            simulation_characteristics=characteristics,
            real_world_specs=self.real_world_specs['lidar'],
            fidelity_score=fidelity_score,
            quality_grade=self.score_to_grade(fidelity_score),
            recommendations=recommendations,
            test_duration=self.analysis_duration,
            data_points=len(self.lidar_data)
        )

        self.sensor_reports['lidar'] = report
        self.get_logger().info(f"LiDAR Analysis: Fidelity {fidelity_score:.2f}, Grade {report.quality_grade}")

    def analyze_camera(self):
        """Analyze RGB camera data quality."""
        if len(self.camera_data) < 5:
            return

        # Extract data
        timestamps = [d['timestamp'] for d in self.camera_data]
        widths = [d['width'] for d in self.camera_data]
        heights = [d['height'] for d in self.camera_data]

        # Calculate update rate
        time_diffs = np.diff(timestamps)
        update_rate = 1.0 / np.mean(time_diffs) if len(time_diffs) > 0 else 0.0

        # Calculate resolution
        avg_width = np.mean(widths)
        avg_height = np.mean(heights)

        # Calculate data completeness
        expected_samples = update_rate * self.analysis_duration
        data_completeness = len(self.camera_data) / expected_samples if expected_samples > 0 else 0.0

        # Create characteristics
        characteristics = SensorCharacteristics(
            update_rate=update_rate,
            data_completeness=min(data_completeness, 1.0)
        )

        # Calculate fidelity score
        fidelity_score = self.calculate_camera_fidelity(characteristics, avg_width, avg_height)

        # Generate recommendations
        recommendations = self.generate_camera_recommendations(characteristics, fidelity_score)

        # Create report
        report = SensorQualityReport(
            sensor_type='RGB Camera',
            simulation_characteristics=characteristics,
            real_world_specs=self.real_world_specs['camera'],
            fidelity_score=fidelity_score,
            quality_grade=self.score_to_grade(fidelity_score),
            recommendations=recommendations,
            test_duration=self.analysis_duration,
            data_points=len(self.camera_data)
        )

        self.sensor_reports['camera'] = report
        self.get_logger().info(f"Camera Analysis: Fidelity {fidelity_score:.2f}, Grade {report.quality_grade}")

    def analyze_depth_camera(self):
        """Analyze depth camera data quality."""
        if len(self.depth_data) < 5:
            return

        # Extract data
        timestamps = [d['timestamp'] for d in self.depth_data]
        encodings = [d['encoding'] for d in self.depth_data]

        # Calculate update rate
        time_diffs = np.diff(timestamps)
        update_rate = 1.0 / np.mean(time_diffs) if len(time_diffs) > 0 else 0.0

        # Assess depth format quality
        depth_quality = 0.0
        if '32FC1' in encodings:
            depth_quality = 0.9
        elif '16UC1' in encodings:
            depth_quality = 0.7
        else:
            depth_quality = 0.3

        # Calculate data completeness
        expected_samples = update_rate * self.analysis_duration
        data_completeness = len(self.depth_data) / expected_samples if expected_samples > 0 else 0.0

        # Create characteristics
        characteristics = SensorCharacteristics(
            update_rate=update_rate,
            data_completeness=min(data_completeness, 1.0),
            range_accuracy=depth_quality * 0.02  # Scale depth quality to accuracy
        )

        # Calculate fidelity score
        fidelity_score = self.calculate_depth_camera_fidelity(characteristics, depth_quality)

        # Generate recommendations
        recommendations = self.generate_depth_camera_recommendations(characteristics, fidelity_score)

        # Create report
        report = SensorQualityReport(
            sensor_type='Depth Camera',
            simulation_characteristics=characteristics,
            real_world_specs=self.real_world_specs['camera'],
            fidelity_score=fidelity_score,
            quality_grade=self.score_to_grade(fidelity_score),
            recommendations=recommendations,
            test_duration=self.analysis_duration,
            data_points=len(self.depth_data)
        )

        self.sensor_reports['depth_camera'] = report
        self.get_logger().info(f"Depth Camera Analysis: Fidelity {fidelity_score:.2f}, Grade {report.quality_grade}")

    def calculate_bias_drift(self, data: np.ndarray) -> float:
        """Calculate bias drift over time."""
        if len(data) < 10:
            return 0.0

        # Calculate mean of first and last 10% of data
        first_10_percent = int(0.1 * len(data))
        last_10_percent = int(0.9 * len(data))

        initial_bias = np.mean(data[:first_10_percent], axis=0)
        final_bias = np.mean(data[last_10_percent:], axis=0)

        drift = np.linalg.norm(final_bias - initial_bias)
        return drift

    def calculate_imu_fidelity(self, characteristics: SensorCharacteristics) -> float:
        """Calculate IMU fidelity score."""
        score = 0.0

        # Update rate fidelity (0-0.3)
        expected_rate = self.real_world_specs['imu']['update_rate']
        rate_ratio = characteristics.update_rate / expected_rate
        score += 0.3 * min(rate_ratio, 1.0)

        # Noise level fidelity (0-0.3)
        expected_noise = self.real_world_specs['imu']['gyro_noise_density']
        noise_ratio = expected_noise / max(characteristics.noise_level, 0.001)
        score += 0.3 * min(noise_ratio, 1.0)

        # Data completeness (0-0.2)
        score += 0.2 * characteristics.data_completeness

        # Bias drift (0-0.2)
        expected_drift = self.real_world_specs['imu']['gyro_bias_stability']
        drift_ratio = expected_drift / max(characteristics.bias_drift, 0.001)
        score += 0.2 * min(drift_ratio, 1.0)

        return min(score, 1.0)

    def calculate_gps_fidelity(self, characteristics: SensorCharacteristics) -> float:
        """Calculate GPS fidelity score."""
        score = 0.0

        # Update rate fidelity (0-0.3)
        expected_rate = self.real_world_specs['gps']['update_rate']
        rate_ratio = characteristics.update_rate / expected_rate
        score += 0.3 * min(rate_ratio, 1.0)

        # Position accuracy fidelity (0-0.4)
        expected_accuracy = self.real_world_specs['gps']['position_accuracy']
        accuracy_ratio = expected_accuracy / max(characteristics.range_accuracy, 0.001)
        score += 0.4 * min(accuracy_ratio, 1.0)

        # Data completeness (0-0.3)
        score += 0.3 * characteristics.data_completeness

        return min(score, 1.0)

    def calculate_lidar_fidelity(self, characteristics: SensorCharacteristics) -> float:
        """Calculate LiDAR fidelity score."""
        score = 0.0

        # Update rate fidelity (0-0.3)
        expected_rate = self.real_world_specs['lidar']['update_rate']
        rate_ratio = characteristics.update_rate / expected_rate
        score += 0.3 * min(rate_ratio, 1.0)

        # Range accuracy fidelity (0-0.3)
        expected_accuracy = self.real_world_specs['lidar']['range_accuracy']
        accuracy_ratio = expected_accuracy / max(characteristics.range_accuracy, 0.001)
        score += 0.3 * min(accuracy_ratio, 1.0)

        # Angular resolution fidelity (0-0.2)
        expected_resolution = self.real_world_specs['lidar']['angular_resolution']
        resolution_ratio = expected_resolution / max(characteristics.angular_resolution, 0.001)
        score += 0.2 * min(resolution_ratio, 1.0)

        # Data completeness (0-0.2)
        score += 0.2 * characteristics.data_completeness

        return min(score, 1.0)

    def calculate_camera_fidelity(self, characteristics: SensorCharacteristics, width: float, height: float) -> float:
        """Calculate RGB camera fidelity score."""
        score = 0.0

        # Update rate fidelity (0-0.3)
        expected_rate = self.real_world_specs['camera']['update_rate']
        rate_ratio = characteristics.update_rate / expected_rate
        score += 0.3 * min(rate_ratio, 1.0)

        # Resolution fidelity (0-0.4)
        expected_resolution = self.real_world_specs['camera']['rgb_resolution']
        resolution_ratio = min(width, height) / expected_resolution
        score += 0.4 * min(resolution_ratio, 1.0)

        # Data completeness (0-0.3)
        score += 0.3 * characteristics.data_completeness

        return min(score, 1.0)

    def calculate_depth_camera_fidelity(self, characteristics: SensorCharacteristics, depth_quality: float) -> float:
        """Calculate depth camera fidelity score."""
        score = 0.0

        # Update rate fidelity (0-0.3)
        expected_rate = self.real_world_specs['camera']['update_rate']
        rate_ratio = characteristics.update_rate / expected_rate
        score += 0.3 * min(rate_ratio, 1.0)

        # Depth quality fidelity (0-0.4)
        score += 0.4 * depth_quality

        # Data completeness (0-0.3)
        score += 0.3 * characteristics.data_completeness

        return min(score, 1.0)

    def score_to_grade(self, score: float) -> str:
        """Convert fidelity score to letter grade."""
        if score >= 0.9:
            return 'A'
        elif score >= 0.8:
            return 'B'
        elif score >= 0.7:
            return 'C'
        elif score >= 0.6:
            return 'D'
        else:
            return 'F'

    def generate_imu_recommendations(self, characteristics: SensorCharacteristics, fidelity_score: float) -> List[str]:
        """Generate IMU improvement recommendations."""
        recommendations = []

        if characteristics.update_rate < self.real_world_specs['imu']['update_rate'] * 0.8:
            recommendations.append("Increase IMU update rate to match real-world specifications")

        if characteristics.noise_level > self.real_world_specs['imu']['gyro_noise_density'] * 2:
            recommendations.append("Reduce IMU noise levels to better match real sensors")

        if characteristics.data_completeness < 0.9:
            recommendations.append("Improve IMU data completeness - check for dropped messages")

        if characteristics.bias_drift > self.real_world_specs['imu']['gyro_bias_stability'] * 2:
            recommendations.append("Reduce IMU bias drift to match real-world stability")

        if fidelity_score < 0.7:
            recommendations.append("Overall IMU simulation needs significant improvement for realistic testing")

        return recommendations

    def generate_gps_recommendations(self, characteristics: SensorCharacteristics, fidelity_score: float) -> List[str]:
        """Generate GPS improvement recommendations."""
        recommendations = []

        if characteristics.update_rate < self.real_world_specs['gps']['update_rate'] * 0.5:
            recommendations.append("Increase GPS update rate to match real-world specifications")

        if characteristics.range_accuracy < self.real_world_specs['gps']['position_accuracy'] * 0.5:
            recommendations.append("GPS accuracy is too good - add realistic noise")
        elif characteristics.range_accuracy > self.real_world_specs['gps']['position_accuracy'] * 2:
            recommendations.append("GPS accuracy is too poor - reduce noise levels")

        if characteristics.data_completeness < 0.8:
            recommendations.append("Improve GPS data completeness")

        return recommendations

    def generate_lidar_recommendations(self, characteristics: SensorCharacteristics, fidelity_score: float) -> List[str]:
        """Generate LiDAR improvement recommendations."""
        recommendations = []

        if characteristics.update_rate < self.real_world_specs['lidar']['update_rate'] * 0.8:
            recommendations.append("Increase LiDAR update rate to match real-world specifications")

        if characteristics.range_accuracy > self.real_world_specs['lidar']['range_accuracy'] * 2:
            recommendations.append("Improve LiDAR range accuracy")

        if characteristics.angular_resolution > self.real_world_specs['lidar']['angular_resolution'] * 2:
            recommendations.append("Improve LiDAR angular resolution")

        return recommendations

    def generate_camera_recommendations(self, characteristics: SensorCharacteristics, fidelity_score: float) -> List[str]:
        """Generate RGB camera improvement recommendations."""
        recommendations = []

        if characteristics.update_rate < self.real_world_specs['camera']['update_rate'] * 0.8:
            recommendations.append("Increase camera update rate to match real-world specifications")

        if fidelity_score < 0.7:
            recommendations.append("Camera simulation needs improvement for realistic testing")

        return recommendations

    def generate_depth_camera_recommendations(self, characteristics: SensorCharacteristics, fidelity_score: float) -> List[str]:
        """Generate depth camera improvement recommendations."""
        recommendations = []

        if characteristics.update_rate < self.real_world_specs['camera']['update_rate'] * 0.8:
            recommendations.append("Increase depth camera update rate")

        if characteristics.range_accuracy > self.real_world_specs['camera']['depth_accuracy'] * 2:
            recommendations.append("Improve depth camera accuracy")

        return recommendations

    def generate_overall_report(self):
        """Generate overall sensor quality report."""
        if not self.sensor_reports:
            self.get_logger().warn("No sensor data collected for analysis")
            return

        # Calculate overall fidelity
        fidelity_scores = [report.fidelity_score for report in self.sensor_reports.values()]
        overall_fidelity = np.mean(fidelity_scores)

        # Generate overall recommendations
        all_recommendations = []
        for report in self.sensor_reports.values():
            all_recommendations.extend(report.recommendations)

        # Create overall report
        overall_report = {
            'analysis_timestamp': time.time(),
            'test_duration': self.analysis_duration,
            'overall_fidelity_score': overall_fidelity,
            'overall_grade': self.score_to_grade(overall_fidelity),
            'sensor_count': len(self.sensor_reports),
            'sensor_reports': {name: asdict(report) for name, report in self.sensor_reports.items()},
            'overall_recommendations': list(set(all_recommendations))  # Remove duplicates
        }

        # Save report
        report_file = f"/tmp/sensor_quality_report_{int(time.time())}.json"
        with open(report_file, 'w') as f:
            json.dump(overall_report, f, indent=2)

        self.get_logger().info(f"Overall sensor fidelity: {overall_fidelity:.2f} (Grade {self.score_to_grade(overall_fidelity)})")
        self.get_logger().info(f"Detailed report saved to {report_file}")

        # Print summary
        self.print_summary(overall_report)

    def print_summary(self, report: Dict[str, Any]):
        """Print analysis summary."""
        print("\n" + "="*60)
        print("SENSOR QUALITY ANALYSIS SUMMARY")
        print("="*60)
        print(f"Overall Fidelity Score: {report['overall_fidelity_score']:.2f}")
        print(f"Overall Grade: {report['overall_grade']}")
        print(f"Test Duration: {report['test_duration']:.1f} seconds")
        print(f"Sensors Analyzed: {report['sensor_count']}")
        print("\nSensor Details:")
        print("-" * 40)

        for name, sensor_report in report['sensor_reports'].items():
            print(f"{name:15} | Fidelity: {sensor_report['fidelity_score']:.2f} | Grade: {sensor_report['quality_grade']}")

        print("\nKey Recommendations:")
        print("-" * 40)
        for i, rec in enumerate(report['overall_recommendations'][:5], 1):
            print(f"{i}. {rec}")

        if len(report['overall_recommendations']) > 5:
            print(f"... and {len(report['overall_recommendations']) - 5} more recommendations")

        print("="*60)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        analyzer = SensorQualityAnalyzer()
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info("Analysis interrupted by user")
    except Exception as e:
        analyzer.get_logger().error(f"Analysis failed with error: {str(e)}")
    finally:
        if 'analyzer' in locals():
            analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
