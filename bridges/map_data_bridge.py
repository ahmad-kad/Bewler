#!/usr/bin/env python3
"""
Map Data Bridge - ROS2 Node

Bridges SLAM map data to frontend visualization:
- Subscribes to SLAM map topics (occupancy grid, point clouds)
- Converts map data to frontend-compatible format
- Publishes map data via WebSocket/ROS bridge
- Handles map updates and compression

Author: URC 2026 Autonomy Team
"""

import base64
import json
import time
import zlib
from typing import Any, Dict, List, Optional

# Data processing
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped

# ROS2 Messages
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class MapDataBridge(Node):
    """
    Map Data Bridge Node

    Subscribes to SLAM map data and publishes it for frontend visualization.

    ROS2 Subscribers:
    - /map: Occupancy grid map from SLAM
    - /slam/pointcloud: Point cloud map from SLAM
    - /slam/pose: Robot pose for map centering
    - /slam/status: SLAM status for map validity

    ROS2 Publishers:
    - /frontend/map_data: Compressed map data for frontend
    - /frontend/map_metadata: Map metadata (resolution, origin, etc.)
    - /frontend/robot_pose: Robot pose in map coordinates
    - /frontend/map_status: Map update status

    Services:
    - /frontend/request_map: Request full map data
    - /frontend/request_region: Request specific map region
    """

    def __init__(self):
        super().__init__('map_data_bridge')

        # Configuration parameters
        self.declare_parameter('map_compression_level', 6)
        self.declare_parameter('max_map_size', 1000000)  # Max uncompressed bytes
        self.declare_parameter('update_interval', 1.0)   # Map update frequency
        self.declare_parameter('region_size', 50)        # Map region size in meters

        self.compression_level = self.get_parameter('map_compression_level').value
        self.max_map_size = self.get_parameter('max_map_size').value
        self.update_interval = self.get_parameter('update_interval').value
        self.region_size = self.get_parameter('region_size').value

        # Map data storage
        self.current_map: Optional[OccupancyGrid] = None
        self.current_pointcloud: Optional[PointCloud2] = None
        self.current_pose: Optional[PoseStamped] = None
        self.map_metadata: Optional[MapMetaData] = None

        # Publishers
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.map_data_pub = self.create_publisher(String, '/frontend/map_data', qos_reliable)
        self.map_metadata_pub = self.create_publisher(String, '/frontend/map_metadata', qos_reliable)
        self.robot_pose_pub = self.create_publisher(String, '/frontend/robot_pose', qos_reliable)
        self.map_status_pub = self.create_publisher(String, '/frontend/map_status', qos_reliable)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_reliable)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/slam/pointcloud', self.pointcloud_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/slam/pose', self.pose_callback, 10)
        self.slam_status_sub = self.create_subscription(
            String, '/slam/status', self.slam_status_callback, 10)

        # Services
        self.request_map_srv = self.create_service(
            Trigger, '/frontend/request_map', self.request_map_callback)

        # Update timers
        self.map_update_timer = self.create_timer(self.update_interval, self.publish_map_update)
        self.status_timer = self.create_timer(5.0, self.status_update)

        # Statistics
        self.maps_published = 0
        self.last_map_update = 0
        self.map_size_bytes = 0

        self.get_logger().info('Map Data Bridge initialized')

    # Data callbacks
    def map_callback(self, msg: OccupancyGrid):
        """Handle occupancy grid map updates"""
        self.current_map = msg
        self.map_metadata = msg.info
        self.last_map_update = time.time()

        # Publish metadata immediately when map changes
        self.publish_map_metadata()

        self.get_logger().debug(f'Received map update: {msg.info.width}x{msg.info.height}')

    def pointcloud_callback(self, msg: PointCloud2):
        """Handle point cloud map updates"""
        self.current_pointcloud = msg
        self.get_logger().debug('Received point cloud update')

    def pose_callback(self, msg: PoseStamped):
        """Handle robot pose updates"""
        self.current_pose = msg

        # Publish robot pose for frontend
        pose_data = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "qx": msg.pose.orientation.x,
            "qy": msg.pose.orientation.y,
            "qz": msg.pose.orientation.z,
            "qw": msg.pose.orientation.w,
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }

        pose_msg = String()
        pose_msg.data = json.dumps(pose_data)
        self.robot_pose_pub.publish(pose_msg)

    def slam_status_callback(self, msg: String):
        """Handle SLAM status updates"""
        # Could use this to validate map data
        pass

    # Service callbacks
    def request_map_callback(self, request, response):
        """Handle full map data request"""
        if self.current_map:
            map_data = self.prepare_map_data()
            if map_data:
                response.success = True
                response.message = json.dumps(map_data)
            else:
                response.success = False
                response.message = "Failed to prepare map data"
        else:
            response.success = False
            response.message = "No map data available"
        return response


    # Publishing methods
    def publish_map_update(self):
        """Periodic map data publishing"""
        if not self.current_map:
            return

        # Check if map has been updated recently
        if time.time() - self.last_map_update > self.update_interval:
            return

        map_data = self.prepare_map_data()
        if map_data:
            map_msg = String()
            map_msg.data = json.dumps(map_data)
            self.map_data_pub.publish(map_msg)

            self.maps_published += 1
            self.map_size_bytes = len(map_msg.data)

    def publish_map_metadata(self):
        """Publish map metadata"""
        if not self.map_metadata:
            return

        metadata = {
            "resolution": self.map_metadata.resolution,
            "width": self.map_metadata.width,
            "height": self.map_metadata.height,
            "origin_x": self.map_metadata.origin.position.x,
            "origin_y": self.map_metadata.origin.position.y,
            "origin_z": self.map_metadata.origin.position.z,
            "origin_qx": self.map_metadata.origin.orientation.x,
            "origin_qy": self.map_metadata.origin.orientation.y,
            "origin_qz": self.map_metadata.origin.orientation.z,
            "origin_qw": self.map_metadata.origin.orientation.w
        }

        metadata_msg = String()
        metadata_msg.data = json.dumps(metadata)
        self.map_metadata_pub.publish(metadata_msg)

    def prepare_map_data(self) -> Optional[Dict[str, Any]]:
        """Prepare compressed map data for frontend"""
        if not self.current_map:
            return None

        try:
            # Convert occupancy grid to numpy array
            width = self.current_map.info.width
            height = self.current_map.info.height
            data = np.array(self.current_map.data, dtype=np.int8).reshape((height, width))

            # Compress map data
            compressed_data = self.compress_map_data(data)

            if len(compressed_data) > self.max_map_size:
                self.get_logger().warning(f'Compressed map too large: {len(compressed_data)} bytes')
                return None

            # Prepare map data structure
            map_data = {
                "type": "occupancy_grid",
                "width": width,
                "height": height,
                "resolution": self.current_map.info.resolution,
                "origin": {
                    "x": self.current_map.info.origin.position.x,
                    "y": self.current_map.info.origin.position.y,
                    "z": self.current_map.info.origin.position.z,
                    "qx": self.current_map.info.origin.orientation.x,
                    "qy": self.current_map.info.origin.orientation.y,
                    "qz": self.current_map.info.origin.orientation.z,
                    "qw": self.current_map.info.origin.orientation.w
                },
                "data": compressed_data.decode('ascii'),
                "timestamp": self.get_clock().now().to_msg().sec,
                "compressed": True
            }

            return map_data

        except Exception as e:
            self.get_logger().error(f'Map data preparation error: {str(e)}')
            return None

    def compress_map_data(self, data: np.ndarray) -> bytes:
        """Compress map data using zlib"""
        # Convert to bytes
        data_bytes = data.tobytes()

        # Compress
        compressed = zlib.compress(data_bytes, level=self.compression_level)

        # Base64 encode for JSON transport
        return base64.b64encode(compressed)

    def status_update(self):
        """Periodic status update"""
        status_data = {
            "maps_published": self.maps_published,
            "last_update": self.last_map_update,
            "map_size_bytes": self.map_size_bytes,
            "has_map": self.current_map is not None,
            "has_pointcloud": self.current_pointcloud is not None,
            "has_pose": self.current_pose is not None
        }

        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.map_status_pub.publish(status_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        bridge = MapDataBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
