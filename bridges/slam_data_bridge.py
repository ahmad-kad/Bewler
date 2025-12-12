#!/usr/bin/env python3
"""
SLAM Data Bridge - Routes WebSocket sensor data to SLAM system

Routes IMU/GPS data from WebSocket sources to SLAM localization,
provides map data back to frontend, and maintains coordinate frame alignment.

Author: URC 2026 Autonomy Team
"""

import json
import math
import os
from typing import Any, Dict, Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class SLAMDataBridge(Node):
    """
    WebSocket to SLAM Data Bridge

    Responsibilities:
    - Subscribe to WebSocket sensor data topics
    - Republish with proper SLAM frame_ids and timestamps
    - Publish simplified map data for frontend visualization
    - Maintain TF transforms between coordinate frames
    - Provide SLAM pose data to frontend
    """

    def __init__(self):
        super().__init__('slam_data_bridge')

        # Parameters
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('gps_frame', 'gps_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')

        # Frame IDs
        self.imu_frame = self.get_parameter('imu_frame').value
        self.gps_frame = self.get_parameter('gps_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

        # Publishers for SLAM consumption
        self.imu_pub = self.create_publisher(Imu, '/imu/data_slam', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix_slam', 10)

        # Frontend map data publishers
        self.map_pub = self.create_publisher(String, '/frontend/map_data', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/frontend/robot_pose', 10)
        self.path_pub = self.create_publisher(Path, '/frontend/robot_path', 10)

        # Mission waypoint publisher
        self.waypoints_pub = self.create_publisher(String, '/frontend/waypoints', 10)

        # Subscribers to WebSocket data
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)

        # SLAM pose subscriber
        self.slam_sub = self.create_subscription(
            PoseStamped, '/slam/pose', self.slam_pose_callback, 10)

        # Mission status subscriber for waypoints
        self.mission_sub = self.create_subscription(
            String, '/mission/status', self.mission_status_callback, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Data storage
        self.latest_imu: Optional[Imu] = None
        self.latest_gps: Optional[NavSatFix] = None
        self.slam_pose: Optional[PoseStamped] = None
        self.robot_path = Path()
        self.robot_path.header.frame_id = self.map_frame
        self.current_mission_waypoints = []

        # Debug counters
        self.slam_pose_count = 0
        self.map_data_count = 0

        # GPS origin for local coordinates
        self.gps_origin = None

        # Status update timer
        self.status_timer = self.create_timer(1.0, self.publish_status_update)

        # Load configuration
        self.config = self.load_config()

        self.get_logger().info('SLAM Data Bridge initialized')

    def load_config(self):
        """Load configuration from YAML file based on environment"""
        env = os.environ.get('ROVER_ENV', 'production')  # Default to production
        config_file = f'{env}.yaml'

        # Try multiple possible config locations
        possible_paths = [
            os.path.join(os.path.dirname(__file__), '..', 'config', config_file),
            os.path.join(os.path.dirname(__file__), 'config', config_file),
            f'config/{config_file}'
        ]

        for config_path in possible_paths:
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    self.get_logger().info(f'Loaded {env} configuration from {config_path}')
                    return config
            except FileNotFoundError:
                continue

        self.get_logger().warn(f'Config file {config_file} not found in any expected location, using defaults')
        return {}

    def imu_callback(self, msg: Imu):
        """Process IMU data for SLAM"""
        self.latest_imu = msg

        # Ensure proper frame_id for SLAM
        msg.header.frame_id = self.imu_frame

        try:
            # Republish for SLAM consumption
            self.imu_pub.publish(msg)

            # Broadcast TF transform
            self.broadcast_imu_transform(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to process IMU data: {e}')

    def gps_callback(self, msg: NavSatFix):
        """Process GPS data for SLAM"""
        self.latest_gps = msg

        # Set GPS origin if not set
        if self.gps_origin is None:
            self.gps_origin = (msg.latitude, msg.longitude)
            self.get_logger().info('.6f')

        # Ensure proper frame_id for SLAM
        msg.header.frame_id = self.gps_frame

        try:
            # Republish for SLAM consumption
            self.gps_pub.publish(msg)

            # Broadcast TF transform
            self.broadcast_gps_transform(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to process GPS data: {e}')

    def slam_pose_callback(self, msg: PoseStamped):
        """Receive SLAM pose and publish frontend data"""
        self.slam_pose = msg
        self.slam_pose_count += 1

        # Only log based on config interval to reduce network overhead
        pose_interval = self.config.get('slam', {}).get('pose_logging_interval', 10)
        if self.slam_pose_count % pose_interval == 0:
            self.get_logger().info(
                f'Received SLAM pose #{self.slam_pose_count}: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        try:
            # Update robot path (limit based on config for memory efficiency)
            path_limit = self.config.get('slam', {}).get('path_history_limit', 50)
            self.robot_path.poses.append(msg)
            if len(self.robot_path.poses) > path_limit:
                self.robot_path.poses.pop(0)

            # Publish path
            self.path_pub.publish(self.robot_path)

            # Publish simplified pose for frontend
            self.pose_pub.publish(msg)

            # Create simplified map data for frontend
            map_data = self.create_map_data()
            json_msg = String()
            json_msg.data = json.dumps(map_data)
            self.map_pub.publish(json_msg)
            self.map_data_count += 1

            # Broadcast TF transforms
            self.broadcast_slam_transforms(msg)

            # Only log based on config interval
            map_interval = self.config.get('slam', {}).get('map_publish_interval', 10)
            if self.map_data_count % map_interval == 0:
                self.get_logger().info(
                    f'Published map data #{self.map_data_count} for robot at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        except Exception as e:
            self.get_logger().error(f'Failed to process SLAM pose: {e}')

    def mission_status_callback(self, msg):
        """Monitor mission status for waypoint visualization"""
        try:
            status = json.loads(msg.data)
            if 'waypoints' in status:
                self.current_mission_waypoints = status['waypoints']
                self.publish_waypoints()
        except (json.JSONDecodeError, Exception) as e:
            self.get_logger().error(f'Failed to process mission status: {e}')

    def create_map_data(self) -> Dict[str, Any]:
        """Create simplified map data for frontend"""
        if not self.slam_pose:
            return {}

        # Current robot position and orientation
        robot_x = self.slam_pose.pose.position.x
        robot_y = self.slam_pose.pose.position.y
        robot_heading = self.quaternion_to_heading(self.slam_pose.pose.orientation)

        # Current sensor data
        imu_data = {}
        if self.latest_imu:
            imu_data = {
                'accel_x': self.latest_imu.linear_acceleration.x,
                'accel_y': self.latest_imu.linear_acceleration.y,
                'accel_z': self.latest_imu.linear_acceleration.z,
                'gyro_x': self.latest_imu.angular_velocity.x,
                'gyro_y': self.latest_imu.angular_velocity.y,
                'gyro_z': self.latest_imu.angular_velocity.z
            }

        gps_data = {}
        if self.latest_gps:
            gps_data = {
                'lat': self.latest_gps.latitude,
                'lon': self.latest_gps.longitude,
                'altitude': self.latest_gps.altitude,
                'satellites': list(getattr(self.latest_gps, 'position_covariance', [0] * 9))
            }

        # Mission waypoints
        waypoints = []
        for wp in self.current_mission_waypoints:
            waypoints.append({
                'x': wp.get('x', 0),
                'y': wp.get('y', 0),
                'reached': wp.get('reached', False)
            })

        return {
            'robot': {
                'x': robot_x,
                'y': robot_y,
                'heading': robot_heading,
                'timestamp': self.slam_pose.header.stamp.sec + self.slam_pose.header.stamp.nanosec * 1e-9
            },
            'sensors': {
                'imu': imu_data,
                'gps': gps_data
            },
            'mission': {
                'waypoints': waypoints,
                'active': len(self.current_mission_waypoints) > 0
            },
            'path': [
                {
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y
                } for pose in self.robot_path.poses[-50:]  # Last 50 points
            ]
        }

    def publish_waypoints(self):
        """Publish current mission waypoints to frontend"""
        waypoints_data = {
            'waypoints': self.current_mission_waypoints,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

        msg = String()
        msg.data = json.dumps(waypoints_data)
        self.waypoints_pub.publish(msg)

    def quaternion_to_heading(self, orientation) -> float:
        """Convert quaternion to heading angle (degrees)"""
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def broadcast_imu_transform(self, imu_msg: Imu):
        """Broadcast IMU transform"""
        transform = TransformStamped()
        transform.header.stamp = imu_msg.header.stamp
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.imu_frame
        # IMU typically mounted at origin relative to base
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0  # Identity rotation

        self.tf_broadcaster.sendTransform(transform)

    def broadcast_gps_transform(self, gps_msg: NavSatFix):
        """Broadcast GPS transform"""
        transform = TransformStamped()
        transform.header.stamp = gps_msg.header.stamp
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.gps_frame
        # GPS antenna offset (adjust based on mounting)
        transform.transform.translation.x = 0.5  # 50cm forward
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.3  # 30cm up
        transform.transform.rotation.w = 1.0  # Identity rotation

        self.tf_broadcaster.sendTransform(transform)

    def broadcast_slam_transforms(self, pose_msg: PoseStamped):
        """Broadcast SLAM-related transforms"""
        # Map to odom transform (if SLAM provides it)
        # For now, assume identity if SLAM pose is in map frame
        if pose_msg.header.frame_id == self.map_frame:
            transform = TransformStamped()
            transform.header.stamp = pose_msg.header.stamp
            transform.header.frame_id = self.map_frame
            transform.child_frame_id = self.odom_frame
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(transform)

    def publish_status_update(self):
        """Periodic status update"""
        # Publish bridge status
        status = {
            'bridge_active': True,
            'imu_connected': self.latest_imu is not None,
            'gps_connected': self.latest_gps is not None,
            'slam_connected': self.slam_pose is not None,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        # Publish to a status topic if needed
        # self.status_pub.publish(status_msg)  # Uncomment if status topic added

    def gps_to_local(self, lat: float, lon: float) -> tuple:
        """Convert GPS coordinates to local map coordinates"""
        if not self.gps_origin:
            return (0.0, 0.0)

        # Simple flat earth approximation
        # 1 degree lat/lon ≈ 111,320 meters
        dlat = lat - self.gps_origin[0]
        dlon = lon - self.gps_origin[1]

        x = dlon * 111320.0 * math.cos(math.radians(lat))
        y = dlat * 111320.0

        return (x, y)


def main():
    """Main entry point"""
    rclpy.init()
    try:
        bridge = SLAMDataBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
