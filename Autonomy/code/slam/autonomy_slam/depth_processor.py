#!/usr/bin/env python3
"""
Depth Processing Node for RGB-D SLAM

Handles depth data cleaning, filtering, and synchronization for robust SLAM
in low-feature desert environments with sand/dust noise.
"""

import logging
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

logger = logging.getLogger(__name__)


class DepthProcessor(Node):
    """
    Process RGB-D data with specialized filtering for desert environments.

    Responsibilities:
    - Bilateral filtering to preserve edges while reducing sand noise
    - Temporal smoothing across frames
    - Outlier detection and removal
    - Dust particle detection
    - Frame synchronization (RGB + Depth + CameraInfo)
    """

    def __init__(self):
        super().__init__("depth_processor")

        # Callback group for concurrent subscriptions
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter("depth_diameter", 9)
        self.declare_parameter("depth_sigma_color", 75.0)
        self.declare_parameter("depth_sigma_space", 75.0)
        self.declare_parameter("temporal_smoothing_window", 3)
        self.declare_parameter("dust_threshold", 50)
        self.declare_parameter("min_depth_mm", 100)
        self.declare_parameter("max_depth_mm", 5000)
        self.declare_parameter("enable_temporal_smoothing", True)
        self.declare_parameter("enable_dust_detection", True)

        # Get parameters
        self.depth_diameter = self.get_parameter("depth_diameter").value
        self.depth_sigma_color = self.get_parameter("depth_sigma_color").value
        self.depth_sigma_space = self.get_parameter("depth_sigma_space").value
        self.temporal_window = self.get_parameter("temporal_smoothing_window").value
        self.dust_threshold = self.get_parameter("dust_threshold").value
        self.min_depth = self.get_parameter("min_depth_mm").value
        self.max_depth = self.get_parameter("max_depth_mm").value
        self.enable_temporal = self.get_parameter("enable_temporal_smoothing").value
        self.enable_dust = self.get_parameter("enable_dust_detection").value

        # CV Bridge for ROS image conversion
        self.bridge = CvBridge()

        # Temporal smoothing buffer
        self.depth_history: deque = deque(maxlen=self.temporal_window)

        # Message filter subscribers for synchronization
        rgb_sub = Subscriber(self, Image, "camera/rgb/image_raw", callback_group=self.callback_group)
        depth_sub = Subscriber(self, Image, "camera/depth/image_raw", callback_group=self.callback_group)
        info_sub = Subscriber(
            self,
            CameraInfo,
            "camera/depth/camera_info",
            callback_group=self.callback_group,
        )

        # Synchronize RGB, depth, and camera info
        self.time_synchronizer = ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], queue_size=10, slop=0.1)
        self.time_synchronizer.registerCallback(self.on_synchronized_data)

        # Publishers for processed data
        self.depth_processed_pub = self.create_publisher(
            Image, "slam/depth/processed", 10, callback_group=self.callback_group
        )
        self.rgb_pub = self.create_publisher(Image, "slam/rgb/image", 10, callback_group=self.callback_group)
        self.info_pub = self.create_publisher(
            CameraInfo, "slam/depth/camera_info", 10, callback_group=self.callback_group
        )

        self.get_logger().info(
            f"DepthProcessor initialized: "
            f"bilateral({self.depth_diameter}, {self.depth_sigma_color}, {self.depth_sigma_space}), "
            f"temporal_window={self.temporal_window}, "
            f"depth_range=[{self.min_depth}, {self.max_depth}]mm"
        )

    def on_synchronized_data(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        """
        Callback for synchronized RGB, depth, and camera info.

        Processes depth data through multiple filtering stages and publishes results.
        """
        try:
            # Convert ROS messages to OpenCV format
            rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

            # Process depth through pipeline
            processed_depth = self._process_depth(depth_cv, rgb_cv)

            # Publish processed depth
            processed_msg = self.bridge.cv2_to_imgmsg(processed_depth, encoding="passthrough")
            processed_msg.header = depth_msg.header
            self.depth_processed_pub.publish(processed_msg)

            # Republish synchronized RGB
            rgb_msg_out = self.bridge.cv2_to_imgmsg(rgb_cv, encoding="bgr8")
            rgb_msg_out.header = rgb_msg.header
            self.rgb_pub.publish(rgb_msg_out)

            # Republish camera info with processed data header
            info_msg.header = depth_msg.header
            self.info_pub.publish(info_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing synchronized data: {e}")

    def _process_depth(self, depth: np.ndarray, rgb: np.ndarray) -> np.ndarray:
        """
        Apply complete depth processing pipeline optimized for desert environments.

        Pipeline stages:
        1. Range filtering (remove out-of-range values)
        2. Bilateral filtering (preserve edges, reduce sand noise)
        3. Dust detection and removal (optional)
        4. Temporal smoothing (optional)
        """
        # Stage 1: Range filtering
        depth_filtered = self._range_filter(depth)

        # Stage 2: Bilateral filtering (primary noise reduction)
        depth_filtered = self._bilateral_filter(depth_filtered, rgb)

        # Stage 3: Dust detection (optional)
        if self.enable_dust:
            depth_filtered = self._detect_and_remove_dust(depth_filtered)

        # Stage 4: Temporal smoothing (optional)
        if self.enable_temporal:
            depth_filtered = self._temporal_smooth(depth_filtered)

        return depth_filtered

    def _range_filter(self, depth: np.ndarray) -> np.ndarray:
        """
        Filter depth values outside valid range.
        Handles both mm and normalized depth formats.
        """
        # Create mask for valid depth range
        mask = (depth >= self.min_depth) & (depth <= self.max_depth)

        # Set out-of-range values to 0 (invalid)
        filtered = depth.copy()
        filtered[~mask] = 0

        return filtered.astype(np.uint16)

    def _bilateral_filter(self, depth: np.ndarray, rgb: np.ndarray) -> np.ndarray:
        """
        Apply bilateral filtering to reduce sand/dust noise while preserving edges.

        Bilateral filter is key for desert: it smooths flat sand areas while
        keeping object boundaries sharp.
        """
        # Apply bilateral filter (slower but better edge preservation)
        # Use RGB as guidance for filtering
        filtered = cv2.bilateralFilter(rgb, self.depth_diameter, self.depth_sigma_color, self.depth_sigma_space)

        # Apply morphological closing to fill small holes from sand particles
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        filtered = cv2.morphologyEx(filtered, cv2.MORPH_CLOSE, kernel, iterations=1)

        return filtered

    def _detect_and_remove_dust(self, depth: np.ndarray) -> np.ndarray:
        """
        Detect and filter dust particles based on local depth variance.

        Dust creates isolated high-variance regions. This removes them by
        comparing local standard deviation to threshold.
        """
        filtered = depth.copy()

        # Use Laplacian for high-frequency detection (isolated dust)
        laplacian = cv2.Laplacian(depth.astype(np.float32), cv2.CV_32F)
        variance = np.abs(laplacian)

        # Create mask for high-variance (potential dust) regions
        dust_mask = variance > self.dust_threshold

        # Dilate mask to remove isolated pixels
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        dust_mask = cv2.dilate(dust_mask, kernel, iterations=1)

        # Remove dust by setting to 0 (invalid)
        filtered[dust_mask] = 0

        return filtered

    def _temporal_smooth(self, depth: np.ndarray) -> np.ndarray:
        """
        Smooth depth across temporal window using median filtering.

        Reduces temporal noise while preserving sudden changes in scene.
        """
        self.depth_history.append(depth.astype(np.float32))

        if len(self.depth_history) < self.temporal_window:
            # Not enough history, return current
            return depth

        # Stack history and compute median
        stacked = np.stack(list(self.depth_history), axis=2)
        temporal_smooth = np.median(stacked, axis=2)

        return temporal_smooth.astype(np.uint16)


def main(args=None):
    """Entry point for depth processor node."""
    rclpy.init(args=args)
    node = DepthProcessor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down DepthProcessor")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
