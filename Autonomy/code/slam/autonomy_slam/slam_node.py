#!/usr/bin/env python3
"""
RGB-D SLAM Orchestrator Node

Integrates:
- RTAB-Map RGB-D SLAM for local mapping and localization
- Depth preprocessing for desert noise reduction
- GPS fusion for global consistency
- Health monitoring and fallback mechanisms
"""

from collections import deque
from datetime import datetime
from typing import Optional

import numpy as np
import rclpy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class SLAMOrchestrator(Node):
    """
    Main SLAM orchestrator node.

    Responsibilities:
    - Coordinate RGB-D depth processing
    - Manage RTAB-Map SLAM configuration
    - Integrate GPS for global localization
    - Monitor system health
    - Publish unified pose estimates
    - Provide fallback mechanisms
    """

    def __init__(self):
        super().__init__("slam_orchestrator")

        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter("enable_depth_processing", True)
        self.declare_parameter("enable_gps_fusion", True)
        self.declare_parameter("enable_diagnostics", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("min_feature_count", 50)  # Minimum features for valid pose

        # Get parameters
        self.enable_depth_proc = self.get_parameter("enable_depth_processing").value
        self.enable_gps_fus = self.get_parameter("enable_gps_fusion").value
        self.enable_diag = self.get_parameter("enable_diagnostics").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.min_features = self.get_parameter("min_feature_count").value

        # System state tracking
        self.system_ready = False
        self.slam_pose: Optional[np.ndarray] = None
        self.fused_pose: Optional[np.ndarray] = None
        self.slam_confidence = 0.0
        self.feature_count = 0
        self.loop_closures = 0

        # Metrics tracking (for diagnostics)
        self.pose_history: deque = deque(maxlen=100)
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        self.slam_update_rate = 0.0

        # Publishers
        self.slam_status_pub = self.create_publisher(
            String, "slam/system/status", 10, callback_group=self.callback_group
        )
        self.slam_health_pub = self.create_publisher(
            String, "slam/system/health", 10, callback_group=self.callback_group
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            "slam/system/diagnostics",
            10,
            callback_group=self.callback_group,
        )

        # Subscribers for monitoring
        self.slam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "slam/pose",
            self.on_slam_pose,
            10,
            callback_group=self.callback_group,
        )

        self.fused_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "slam/pose/fused",
            self.on_fused_pose,
            10,
            callback_group=self.callback_group,
        )

        self.rtabmap_status_sub = self.create_subscription(
            String,
            "rtabmap/stat",
            self.on_rtabmap_status,
            10,
            callback_group=self.callback_group,
        )

        self.fusion_status_sub = self.create_subscription(
            String,
            "slam/fusion/status",
            self.on_fusion_status,
            10,
            callback_group=self.callback_group,
        )

        # Timer for health monitoring
        self.health_timer = self.create_timer(1.0, self.on_health_check, callback_group=self.callback_group)

        # TF broadcaster for diagnostics
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(
            f"SLAMOrchestrator initialized: "
            f"depth_proc={self.enable_depth_proc}, "
            f"gps_fusion={self.enable_gps_fus}, "
            f"diagnostics={self.enable_diag}"
        )

    def on_slam_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Receive SLAM pose estimate."""
        self.slam_pose = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )

        # Extract covariance-based confidence
        covariance = np.array(msg.pose.covariance).reshape(6, 6)
        position_uncertainty = np.trace(covariance[:3, :3]) / 3.0
        self.slam_confidence = 1.0 / (1.0 + position_uncertainty)

        self.pose_history.append((datetime.now(), self.slam_pose.copy()))

    def on_fused_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Receive GPS-fused pose estimate."""
        self.fused_pose = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )

    def on_rtabmap_status(self, msg: String) -> None:
        """Parse RTAB-Map status for feature count and loop closures."""
        try:
            # RTAB-Map status format: "Nodes=N Features=F LoopClosures=L ..."
            parts = msg.data.split()
            for part in parts:
                if part.startswith("Features="):
                    self.feature_count = int(part.split("=")[1])
                elif part.startswith("LoopClosures="):
                    self.loop_closures = int(part.split("=")[1])
        except Exception as e:
            self.get_logger().debug(f"Error parsing RTAB-Map status: {e}")

    def on_fusion_status(self, msg: String) -> None:
        """Monitor GPS fusion status."""
        # Status contains mode, confidence, GPS availability

    def on_health_check(self) -> None:
        """Periodic health check and diagnostics."""
        # Check system readiness
        self.system_ready = self._check_system_ready()

        # Publish health status
        self._publish_health_status()

        # Publish diagnostics
        if self.enable_diag:
            self._publish_diagnostics()

    def _check_system_ready(self) -> bool:
        """
        Check if SLAM system is ready for operation.

        Criteria:
        - SLAM pose available
        - Sufficient feature count
        - Reasonable confidence level
        """
        if self.slam_pose is None:
            return False
        if self.feature_count < self.min_features:
            return False
        if self.slam_confidence < 0.5:
            return False
        return True

    def _publish_health_status(self) -> None:
        """Publish system health status."""
        status_msg = String()

        status_components = [
            f'System: {"READY" if self.system_ready else "INITIALIZING"}',
            f"Features: {self.feature_count}/{self.min_features}",
            f"Confidence: {self.slam_confidence:.2f}",
            f"LoopClosures: {self.loop_closures}",
        ]

        if self.fused_pose is not None:
            status_components.append("GPS: FUSED")

        status_msg.data = " | ".join(status_components)
        self.slam_health_pub.publish(status_msg)

        # Also publish overall status
        system_status = String()
        system_status.data = f'SLAM System Status: {"OPERATIONAL" if self.system_ready else "INITIALIZING"}'
        self.slam_status_pub.publish(system_status)

    def _publish_diagnostics(self) -> None:
        """Publish diagnostic information for monitoring."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # SLAM status diagnostic
        slam_diag = DiagnosticStatus()
        slam_diag.name = "SLAM System"
        slam_diag.level = DiagnosticStatus.OK if self.system_ready else DiagnosticStatus.WARN
        slam_diag.message = "SLAM Operational" if self.system_ready else "SLAM Initializing"
        slam_diag.values = [
            self._kv_pair("Feature Count", str(self.feature_count)),
            self._kv_pair("SLAM Confidence", f"{self.slam_confidence:.3f}"),
            self._kv_pair("Loop Closures", str(self.loop_closures)),
        ]

        # Pose diagnostic
        pose_diag = DiagnosticStatus()
        pose_diag.name = "Pose Estimate"
        if self.slam_pose is not None:
            pose_diag.level = DiagnosticStatus.OK
            pose_diag.message = f"Pose: ({self.slam_pose[0]:.2f}, {self.slam_pose[1]:.2f})"
            pose_diag.values = [
                self._kv_pair("X", f"{self.slam_pose[0]:.3f}"),
                self._kv_pair("Y", f"{self.slam_pose[1]:.3f}"),
                self._kv_pair("Z", f"{self.slam_pose[2]:.3f}"),
            ]
        else:
            pose_diag.level = DiagnosticStatus.WARN
            pose_diag.message = "No pose estimate available"

        diag_array.status = [slam_diag, pose_diag]
        self.diagnostics_pub.publish(diag_array)

    @staticmethod
    def _kv_pair(key: str, value: str):
        """Create key-value pair for diagnostic message."""
        from diagnostic_msgs.msg import KeyValue

        kv = KeyValue()
        kv.key = key
        kv.value = value
        return kv


def main(args=None):
    """Entry point for SLAM orchestrator."""
    rclpy.init(args=args)
    node = SLAMOrchestrator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SLAMOrchestrator")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
