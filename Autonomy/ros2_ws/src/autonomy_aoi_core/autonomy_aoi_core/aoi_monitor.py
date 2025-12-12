#!/usr/bin/env python3
"""
AoI Monitor Node

Central AoI monitoring for the URC 2026 autonomy system.
Provides system-wide AoI tracking with minimal resource usage.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from autonomy_interfaces.msg import AOIStatus, AOIMetrics
from autonomy_interfaces.srv import GetAOIStatus
from autonomy_aoi_core.aoi_tracker import AOITracker, SharedAOIBuffer, AOIQualityAssessor, AOIConfig
import time
import logging

logger = logging.getLogger(__name__)


class AOIMonitorNode(Node):
    """
    Central AoI monitoring node.

    Tracks AoI for all critical sensors and systems with minimal overhead.
    """

    def __init__(self):
        super().__init__("aoi_monitor")

        # Declare parameters
        self.declare_parameter("update_rate_hz", 5.0)
        self.declare_parameter("max_sensors", 16)
        self.declare_parameter("history_window", 32)
        self.declare_parameter("enable_detailed_logging", False)

        # Get parameters
        update_rate = self.get_parameter("update_rate_hz").value
        max_sensors = self.get_parameter("max_sensors").value
        history_window = self.get_parameter("history_window").value
        self.enable_logging = self.get_parameter("enable_detailed_logging").value

        # Initialize AoI infrastructure
        self.aoi_config = AOIConfig(history_window=history_window, update_interval=1.0 / update_rate)

        self.shared_buffer = SharedAOIBuffer(max_sensors=max_sensors, history_size=history_window)

        self.quality_assessor = AOIQualityAssessor()

        # Individual trackers for detailed monitoring
        self.trackers = {}

        # Publishers with minimal QoS
        aoi_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Only keep latest
            durability=DurabilityPolicy.VOLATILE,
        )

        self.aoi_status_pub = self.create_publisher(AOIStatus, "/system/aoi_status", aoi_qos)

        self.aoi_metrics_pub = self.create_publisher(AOIMetrics, "/system/aoi_metrics", aoi_qos)

        # Service for AoI queries
        self.aoi_service = self.create_service(GetAOIStatus, "/system/get_aoi_status", self.handle_aoi_query)

        # Subscribers for critical sensors (minimal processing)
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self.on_imu_data, 1, callback_group=self.create_callback_group()  # Minimal QoS
        )

        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.on_gps_data, 1, callback_group=self.create_callback_group()
        )

        self.slam_sub = self.create_subscription(
            PoseStamped, "/slam/pose", self.on_slam_pose, 1, callback_group=self.create_callback_group()
        )

        self.wheel_sub = self.create_subscription(
            Odometry, "/wheel/odom", self.on_wheel_odom, 1, callback_group=self.create_callback_group()
        )

        # Periodic status updates
        self.status_timer = self.create_timer(1.0 / update_rate, self.publish_status_updates)

        # Health monitoring
        self.last_update_time = time.time()
        self.update_count = 0

        self.get_logger().info(
            f"AoI Monitor initialized: {max_sensors} sensors max, "
            f"{update_rate}Hz updates, {history_window} sample history"
        )

    def on_imu_data(self, msg):
        """Track IMU AoI."""
        self._track_sensor_aoi("imu", msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)

    def on_gps_data(self, msg):
        """Track GPS AoI."""
        self._track_sensor_aoi("gps", msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)

    def on_slam_pose(self, msg):
        """Track SLAM pose AoI."""
        self._track_sensor_aoi("slam_pose", msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)

    def on_wheel_odom(self, msg):
        """Track wheel odometry AoI."""
        self._track_sensor_aoi("wheel_odom", msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)

    def _track_sensor_aoi(self, sensor_name: str, sensor_timestamp: float):
        """Update AoI tracking for sensor."""
        current_time = time.time()
        aoi = current_time - sensor_timestamp

        # Update shared buffer (minimal memory)
        success = self.shared_buffer.update_aoi(sensor_name, aoi)
        if not success:
            self.get_logger().warn(f"AoI buffer full, cannot track {sensor_name}")
            return

        # Update detailed tracker if logging enabled
        if self.enable_logging:
            if sensor_name not in self.trackers:
                self.trackers[sensor_name] = AOITracker(self.aoi_config)
            self.trackers[sensor_name].update(sensor_timestamp)

        # Update activity tracking
        self.last_update_time = current_time
        self.update_count += 1

    def publish_status_updates(self):
        """Publish periodic AoI status updates."""
        # Publish individual sensor statuses (only if significantly changed)
        self._publish_sensor_statuses()

        # Publish system metrics (every update)
        self._publish_system_metrics()

    def _publish_sensor_statuses(self):
        """Publish AoI status for sensors with significant changes."""
        for sensor_name in self.shared_buffer.sensor_ids.keys():
            current_aoi = self.shared_buffer.get_sensor_aoi(sensor_name)
            if current_aoi is None:
                continue

            # Get detailed stats if available
            if sensor_name in self.trackers:
                stats = self.trackers[sensor_name].get_statistics()
                quality_score, quality_category = self.quality_assessor.assess_quality(sensor_name, current_aoi)

                # Create status message
                status_msg = AOIStatus()
                status_msg.header.stamp = self.get_clock().now().to_msg()
                status_msg.header.frame_id = "aoi_monitor"
                status_msg.sensor_name = sensor_name
                status_msg.sensor_type = self._get_sensor_type(sensor_name)
                status_msg.current_aoi = current_aoi
                status_msg.average_aoi = stats["average_aoi"]
                status_msg.max_aoi = stats["max_aoi"]
                status_msg.min_aoi = stats["min_aoi"]
                status_msg.is_fresh = current_aoi <= self.aoi_config.acceptable_threshold
                status_msg.quality_score = quality_score
                status_msg.freshness_status = quality_category
                status_msg.acceptable_threshold = self.aoi_config.acceptable_threshold
                status_msg.optimal_threshold = self.aoi_config.optimal_threshold
                status_msg.sample_count = len(self.trackers[sensor_name].aoi_history)
                status_msg.freshness_ratio = stats["freshness_ratio"]

                self.aoi_status_pub.publish(status_msg)

    def _publish_system_metrics(self):
        """Publish system-wide AoI metrics."""
        system_stats = self.shared_buffer.get_system_stats()

        metrics_msg = AOIMetrics()
        metrics_msg.header.stamp = self.get_clock().now().to_msg()
        metrics_msg.header.frame_id = "aoi_monitor"

        # System summary
        metrics_msg.system_average_aoi = system_stats["average_aoi"]
        metrics_msg.total_sensors = system_stats["total_sensors"]
        metrics_msg.fresh_sensors = system_stats["fresh_sensors"]
        metrics_msg.stale_sensors = system_stats["total_sensors"] - system_stats["fresh_sensors"]
        metrics_msg.critical_sensors = 0  # TODO: implement critical threshold tracking

        # AoI distribution (simplified)
        all_aois = []
        for sensor_name in self.shared_buffer.sensor_ids.keys():
            history = self.shared_buffer.get_sensor_history(sensor_name, 10)
            all_aois.extend([aoi for _, aoi in history])

        if all_aois:
            sorted_aois = sorted(all_aois)
            n = len(sorted_aois)
            metrics_msg.aoi_p50 = sorted_aois[n // 2]
            metrics_msg.aoi_p90 = sorted_aois[int(n * 0.9)]
            metrics_msg.aoi_p95 = sorted_aois[int(n * 0.95)]
            metrics_msg.aoi_p99 = sorted_aois[int(n * 0.99)]

        # System health
        freshness_ratio = system_stats["freshness_ratio"]
        metrics_msg.system_healthy = freshness_ratio > 0.7  # 70% fresh sensors
        metrics_msg.health_score = freshness_ratio

        if freshness_ratio > 0.8:
            metrics_msg.health_status = "HEALTHY"
        elif freshness_ratio > 0.6:
            metrics_msg.health_status = "WARNING"
        else:
            metrics_msg.health_status = "CRITICAL"

        # Performance metrics
        current_time = time.time()
        time_since_start = current_time - self.get_clock().now().seconds_nanoseconds()[0]
        if time_since_start > 0:
            metrics_msg.update_rate_hz = self.update_count / time_since_start

        # Publish metrics
        self.aoi_metrics_pub.publish(metrics_msg)

    def handle_aoi_query(self, request, response):
        """Handle AoI status queries."""
        try:
            response.success = True
            response.message = "AoI query successful"

            # Get requested sensor data
            if request.sensor_name:
                # Specific sensor
                if request.sensor_name in self.shared_buffer.sensor_ids:
                    status = self._create_sensor_status(request.sensor_name)
                    if status:
                        response.sensor_status = [status]

                    if request.include_history and request.history_samples > 0:
                        history = self.shared_buffer.get_sensor_history(request.sensor_name, request.history_samples)
                        response.aoi_history = [aoi for _, aoi in history]
                        response.timestamp_history = [self.get_clock().now().to_msg() for _ in history]  # Simplified
                else:
                    response.success = False
                    response.message = f"Unknown sensor: {request.sensor_name}"
            else:
                # All sensors
                response.sensor_status = []
                for sensor_name in self.shared_buffer.sensor_ids.keys():
                    status = self._create_sensor_status(sensor_name)
                    if status:
                        response.sensor_status.append(status)

            # System metrics
            system_stats = self.shared_buffer.get_system_stats()
            response.system_metrics.system_average_aoi = system_stats["average_aoi"]
            response.system_metrics.total_sensors = system_stats["total_sensors"]
            response.system_metrics.fresh_sensors = system_stats["fresh_sensors"]

        except Exception as e:
            response.success = False
            response.message = f"AoI query failed: {str(e)}"
            self.get_logger().error(f"AoI query error: {e}")

        return response

    def _create_sensor_status(self, sensor_name: str):
        """Create AOIStatus message for sensor."""
        current_aoi = self.shared_buffer.get_sensor_aoi(sensor_name)
        if current_aoi is None:
            return None

        status = AOIStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "aoi_monitor"
        status.sensor_name = sensor_name
        status.sensor_type = self._get_sensor_type(sensor_name)
        status.current_aoi = current_aoi

        if sensor_name in self.trackers:
            stats = self.trackers[sensor_name].get_statistics()
            status.average_aoi = stats["average_aoi"]
            status.max_aoi = stats["max_aoi"]
            status.min_aoi = stats["min_aoi"]
            status.freshness_ratio = stats["freshness_ratio"]

        quality_score, quality_category = self.quality_assessor.assess_quality(sensor_name, current_aoi)
        status.quality_score = quality_score
        status.freshness_status = quality_category
        status.is_fresh = current_aoi <= self.aoi_config.acceptable_threshold

        return status

    def _get_sensor_type(self, sensor_name: str) -> str:
        """Get sensor type category."""
        type_mapping = {
            "imu": "sensor",
            "gps": "sensor",
            "camera": "sensor",
            "lidar": "sensor",
            "slam_pose": "fusion",
            "wheel_odom": "sensor",
        }
        return type_mapping.get(sensor_name, "unknown")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = AOIMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
