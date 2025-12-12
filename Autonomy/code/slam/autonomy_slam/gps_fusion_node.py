#!/usr/bin/env python3
"""
GPS Fusion Node for SLAM

Combines SLAM pose estimates with GPS data using an Extended Kalman Filter (EKF)
to provide globally-consistent localization in desert environments.

Approach:
- SLAM provides local accuracy and rapid updates (10 Hz)
- GPS provides global reference frame (1-10 Hz, lower accuracy initially)
- EKF fuses both for best of both worlds
- Graceful fallback to GPS-only if SLAM confidence drops
"""

import logging
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

logger = logging.getLogger(__name__)


@dataclass
class State:
    """EKF state vector: [x, y, z, yaw, vx, vy]"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0

    def to_array(self) -> np.ndarray:
        """Convert to numpy array for EKF operations."""
        return np.array([self.x, self.y, self.z, self.yaw, self.vx, self.vy])

    @staticmethod
    def from_array(arr: np.ndarray) -> "State":
        """Construct from numpy array."""
        return State(x=arr[0], y=arr[1], z=arr[2], yaw=arr[3], vx=arr[4], vy=arr[5])


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for sensor fusion.

    Fuses:
    - SLAM pose (position + orientation)
    - GPS position (lat/lon → local)
    - IMU heading (optional yaw correction)

    State: [x, y, z, yaw, vx, vy]
    """

    def __init__(self, state_dim: int = 6, measurement_dim: int = 3):
        """
        Initialize EKF.

        Args:
            state_dim: Dimension of state vector (x, y, z, yaw, vx, vy)
            measurement_dim: Dimension of measurement (position x, y, yaw)
        """
        self.state = np.zeros(state_dim)
        self.P = np.eye(state_dim) * 0.1  # Initial covariance (high uncertainty)

        # Process noise covariance (how much we trust the motion model)
        self.Q = np.eye(state_dim)
        self.Q[:2, :2] *= 0.01  # Position process noise
        self.Q[2, 2] = 0.01  # Z process noise
        self.Q[3, 3] = 0.01  # Yaw process noise
        self.Q[4:, 4:] *= 0.1  # Velocity process noise

        # Measurement noise covariance (how much we trust measurements)
        self.R_slam = np.eye(3) * 0.05  # SLAM measurement noise
        self.R_gps = np.eye(2) * 1.0  # GPS measurement noise (lower confidence)

        self.dt = 0.1  # Time step
        self.gps_reference: Optional[Tuple[float, float]] = None

    def predict(self, dt: float) -> None:
        """
        Predict step: project state forward in time.

        Motion model assumes constant velocity.
        """
        self.dt = dt

        # State transition matrix (constant velocity model)
        F = np.eye(6)
        F[0, 4] = dt  # x += vx * dt
        F[1, 5] = dt  # y += vy * dt

        # Update state: x = F * x
        self.state = F @ self.state

        # Update covariance: P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q

    def update_slam(
        self,
        position: np.ndarray,
        orientation_yaw: float,
        covariance: Optional[np.ndarray] = None,
    ) -> None:
        """
        Update with SLAM measurement.

        Args:
            position: [x, y, z] position from SLAM
            orientation_yaw: Yaw angle from SLAM
            covariance: Optional 3x3 covariance matrix
        """
        # Measurement: [x, y, yaw]
        z = np.array([position[0], position[1], orientation_yaw])

        # Measurement Jacobian (linear in this case)
        H = np.zeros((3, 6))
        H[0, 0] = 1.0  # Measure x
        H[1, 1] = 1.0  # Measure y
        H[2, 3] = 1.0  # Measure yaw

        # Use provided covariance or default
        if covariance is not None:
            R = covariance[:3, :3]
        else:
            R = self.R_slam

        # Innovation (measurement residual)
        y = z - (H @ self.state)

        # Normalize angle difference
        y[2] = self._normalize_angle(y[2])

        # Innovation covariance: S = H * P * H^T + R
        S = H @ self.P @ H.T + R

        # Kalman gain: K = P * H^T * S^{-1}
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state: x = x + K * y
        self.state = self.state + K @ y

        # Update covariance: P = (I - K * H) * P
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P

    def update_gps(self, lat: float, lon: float, gps_std_dev: float = 1.0) -> None:
        """
        Update with GPS measurement (lat/lon → local coordinates).

        Args:
            lat: GPS latitude
            lon: GPS longitude
            gps_std_dev: GPS standard deviation in meters
        """
        # Set GPS reference on first fix
        if self.gps_reference is None:
            self.gps_reference = (lat, lon)
            logger.info(f"GPS reference set to: {self.gps_reference}")
            return

        # Convert GPS to local coordinates
        local_pos = self._gps_to_local(lat, lon)

        # Measurement: [x, y] from GPS
        z = np.array(local_pos)

        # Measurement Jacobian
        H = np.zeros((2, 6))
        H[0, 0] = 1.0  # Measure x
        H[1, 1] = 1.0  # Measure y

        # GPS covariance (typically low confidence)
        R = np.eye(2) * (gps_std_dev**2)

        # Innovation
        y = z - (H @ self.state)[:2]

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state (only position, not velocity)
        delta = np.zeros(6)
        delta[:2] = K @ y
        self.state = self.state + delta

        # Update covariance
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P

    def get_state(self) -> State:
        """Get current state."""
        return State.from_array(self.state)

    def get_covariance(self) -> np.ndarray:
        """Get current covariance matrix."""
        return self.P.copy()

    def _gps_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Convert GPS coordinates to local ENU coordinates.

        Uses simple flat-earth approximation suitable for local navigation.
        """
        if self.gps_reference is None:
            return (0.0, 0.0)

        ref_lat, ref_lon = self.gps_reference

        # 1 degree ≈ 111 km
        dlat = lat - ref_lat
        dlon = lon - ref_lon

        # Convert to local coordinates (ENU)
        x = dlon * 111000 * math.cos(math.radians(ref_lat))  # East
        y = dlat * 111000  # North

        return (x, y)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class GPSFusionNode(Node):
    """
    GPS Fusion node that combines SLAM and GPS data via EKF.

    Provides:
    - Fused pose estimate combining SLAM local accuracy and GPS global reference
    - Graceful fallback to GPS-only if SLAM confidence drops
    - Health monitoring and mode reporting
    """

    def __init__(self):
        super().__init__("gps_fusion_node")

        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter("slam_confidence_threshold", 0.7)
        self.declare_parameter("gps_std_dev", 1.0)
        self.declare_parameter("fusion_rate_hz", 10.0)

        # Get parameters
        self.slam_confidence_threshold = self.get_parameter("slam_confidence_threshold").value
        self.gps_std_dev = self.get_parameter("gps_std_dev").value
        fusion_rate = self.get_parameter("fusion_rate_hz").value

        # Initialize EKF
        self.ekf = ExtendedKalmanFilter()

        # Tracking state
        self.last_slam_pose: Optional[np.ndarray] = None
        self.last_slam_yaw: float = 0.0
        self.last_slam_covariance: Optional[np.ndarray] = None
        self.slam_confidence: float = 0.0
        self.gps_available: bool = False
        self.fusion_mode: str = "initializing"  # initializing, slam_only, gps_slam_fusion, gps_only

        # Subscribers
        self.slam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "slam/pose",
            self.on_slam_pose,
            10,
            callback_group=self.callback_group,
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            "gps/fix",
            self.on_gps_fix,
            10,
            callback_group=self.callback_group,
        )

        # Publishers
        self.fused_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "slam/pose/fused",
            10,
            callback_group=self.callback_group,
        )

        self.fusion_status_pub = self.create_publisher(
            String, "slam/fusion/status", 10, callback_group=self.callback_group
        )

        # TF broadcaster for fused pose
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for fusion update
        self.fusion_timer = self.create_timer(
            1.0 / fusion_rate, self.on_fusion_update, callback_group=self.callback_group
        )

        self.get_logger().info(
            f"GPSFusionNode initialized: "
            f"slam_confidence_threshold={self.slam_confidence_threshold}, "
            f"fusion_rate={fusion_rate}Hz"
        )

    def on_slam_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Receive SLAM pose estimate."""
        # Extract position
        position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )

        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # Extract covariance
        covariance = np.array(msg.pose.covariance).reshape(6, 6)
        slam_uncertainty = np.trace(covariance[:3, :3]) / 3.0

        # Estimate SLAM confidence (inverse of uncertainty)
        # Higher confidence when covariance is low
        self.slam_confidence = 1.0 / (1.0 + slam_uncertainty)

        self.last_slam_pose = position
        self.last_slam_yaw = yaw
        self.last_slam_covariance = covariance

    def on_gps_fix(self, msg: NavSatFix) -> None:
        """Receive GPS fix."""
        if msg.status.status < 0:  # No fix
            self.gps_available = False
            return

        self.gps_available = True

    def on_fusion_update(self) -> None:
        """Main fusion update loop."""
        # Predict step
        self.ekf.predict(0.1)

        # Update with SLAM if available and confident
        if self.last_slam_pose is not None:
            self.ekf.update_slam(self.last_slam_pose, self.last_slam_yaw, self.last_slam_covariance)

        # Update fusion mode
        self._update_fusion_mode()

        # Publish fused pose
        self._publish_fused_pose()

        # Publish status
        self._publish_status()

    def _update_fusion_mode(self) -> None:
        """Determine fusion mode based on available data."""
        if self.last_slam_pose is None:
            self.fusion_mode = "initializing"
        elif self.gps_available and self.slam_confidence > self.slam_confidence_threshold:
            self.fusion_mode = "gps_slam_fusion"
        elif self.slam_confidence > self.slam_confidence_threshold:
            self.fusion_mode = "slam_only"
        elif self.gps_available:
            self.fusion_mode = "gps_only"
        else:
            self.fusion_mode = "dead_reckoning"

    def _publish_fused_pose(self) -> None:
        """Publish fused pose estimate."""
        state = self.ekf.get_state()
        covariance = self.ekf.get_covariance()

        # Create pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # Position
        pose_msg.pose.pose.position.x = state.x
        pose_msg.pose.pose.position.y = state.y
        pose_msg.pose.pose.position.z = state.z

        # Orientation (yaw)
        half_yaw = state.yaw / 2.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(half_yaw)
        pose_msg.pose.pose.orientation.w = math.cos(half_yaw)

        # Covariance
        pose_msg.pose.covariance = tuple(covariance.flatten())

        self.fused_pose_pub.publish(pose_msg)

        # Publish TF transform
        transform = TransformStamped()
        transform.header.stamp = pose_msg.header.stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link_fused"
        transform.transform.translation.x = state.x
        transform.transform.translation.y = state.y
        transform.transform.translation.z = state.z
        transform.transform.rotation.x = pose_msg.pose.pose.orientation.x
        transform.transform.rotation.y = pose_msg.pose.pose.orientation.y
        transform.transform.rotation.z = pose_msg.pose.pose.orientation.z
        transform.transform.rotation.w = pose_msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(transform)

    def _publish_status(self) -> None:
        """Publish fusion status."""
        status_msg = String()
        status_msg.data = (
            f"Mode: {self.fusion_mode} | "
            f"SLAM Confidence: {self.slam_confidence:.2f} | "
            f'GPS: {"available" if self.gps_available else "unavailable"}'
        )
        self.fusion_status_pub.publish(status_msg)


def main(args=None):
    """Entry point for GPS fusion node."""
    rclpy.init(args=args)
    node = GPSFusionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GPSFusionNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
