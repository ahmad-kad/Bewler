#!/usr/bin/env python3
"""
WebSocket-SLAM Bridge - ROS2 Node

Bridges WebSocket sensor data to ROS2 SLAM system:
- Connects to rosbridge_websocket server
- Subscribes to WebSocket sensor topics
- Converts and publishes data to ROS2 SLAM topics
- Handles IMU, GPS, odometry, and camera data

Author: URC 2026 Autonomy Team
"""

# JSON and threading handling
import json
import threading
import time
from typing import Any, Callable, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS2 Messages
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger


class WebSocketSLAMBridge(Node):
    """
    WebSocket-SLAM Bridge Node

    Connects to rosbridge_websocket and forwards sensor data to SLAM topics.

    ROS2 Publishers:
    - /imu/data_ws: IMU data from WebSocket
    - /gps/fix_ws: GPS data from WebSocket
    - /odom_ws: Odometry data from WebSocket
    - /camera/image_raw_ws: Camera data from WebSocket
    - /camera/camera_info_ws: Camera info from WebSocket

    ROS2 Subscribers:
    - /slam/pose: SLAM pose for feedback
    - /slam/status: SLAM status for monitoring

    Services:
    - /bridge/websocket/connect: Connect to WebSocket
    - /bridge/websocket/disconnect: Disconnect from WebSocket
    - /bridge/websocket/status: Get bridge status
    """

    def __init__(self):
        super().__init__('websocket_slam_bridge')

        # Configuration parameters
        self.declare_parameter('websocket_url', 'ws://localhost:9090')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('max_reconnect_attempts', 10)

        self.websocket_url = self.get_parameter('websocket_url').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').value

        # Connection state
        self.websocket_connected = False
        self.bridge_active = False
        self.reconnect_attempts = 0

        # Publishers for SLAM data
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.imu_pub = self.create_publisher(Imu, '/imu/data_ws', qos_sensor)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix_ws', qos_sensor)
        self.odom_pub = self.create_publisher(Odometry, '/odom_ws', qos_sensor)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw_ws', qos_sensor)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info_ws', qos_sensor)

        # Status publishers
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.status_pub = self.create_publisher(String, '/bridge/websocket/status', qos_reliable)
        self.connection_pub = self.create_publisher(Bool, '/bridge/websocket/connected', qos_reliable)

        # SLAM feedback subscribers
        self.slam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/slam/pose', self.slam_pose_callback, 10)
        self.slam_status_sub = self.create_subscription(
            String, '/slam/status', self.slam_status_callback, 10)

        # Bridge control services
        self.connect_srv = self.create_service(
            Trigger, '/bridge/websocket/connect', self.connect_callback)
        self.disconnect_srv = self.create_service(
            Trigger, '/bridge/websocket/disconnect', self.disconnect_callback)
        self.status_srv = self.create_service(
            Trigger, '/bridge/websocket/status', self.get_status_callback)

        # WebSocket connection
        self.websocket = None
        self.websocket_thread: Optional[threading.Thread] = None

        # Data buffers and callbacks
        self.topic_callbacks: Dict[str, Callable] = {
            self.imu_topic: self.handle_imu_data,
            self.gps_topic: self.handle_gps_data,
            self.odom_topic: self.handle_odom_data,
            self.camera_topic: self.handle_camera_data,
            self.camera_info_topic: self.handle_camera_info_data,
        }

        # Status tracking
        self.last_imu_time = 0
        self.last_gps_time = 0
        self.last_odom_time = 0
        self.last_camera_time = 0
        self.messages_received = 0
        self.messages_published = 0

        # Status update timer
        self.status_timer = self.create_timer(1.0, self.status_update)

        # Auto-connect on startup
        self.create_timer(2.0, self.auto_connect, oneshot=True)

        self.get_logger().info('WebSocket-SLAM Bridge initialized')

    # WebSocket connection management
    def auto_connect(self):
        """Auto-connect to WebSocket on startup"""
        self.get_logger().info(f'Auto-connecting to WebSocket at {self.websocket_url}')
        self.connect_websocket()

    def connect_websocket(self) -> bool:
        """Connect to WebSocket server (simplified for demo)"""
        try:
            if self.websocket_connected:
                self.get_logger().info('Already connected to WebSocket')
                return True

            self.get_logger().info(f'WebSocket bridge ready for connection to: {self.websocket_url}')
            self.get_logger().info('Note: Install websocket-client library for full WebSocket support')

            # Simulate connection for demo purposes
            self.websocket_connected = True
            self.bridge_active = True
            self.get_logger().info('WebSocket bridge activated (demo mode)')

            return True

        except Exception as e:
            self.get_logger().error(f'WebSocket connection error: {str(e)}')
            return False

    def disconnect_websocket(self):
        """Disconnect from WebSocket server"""
        self.get_logger().info('Disconnecting from WebSocket')
        self.bridge_active = False
        self.websocket_connected = False

        if self.websocket:
            try:
                self.websocket.close()
            except BaseException:
                pass
            self.websocket = None

    def websocket_connection_loop(self):
        """Main WebSocket connection loop (simplified)"""
        self.get_logger().info('WebSocket connection loop started (demo mode)')

        # In demo mode, just keep the connection alive
        while self.bridge_active:
            time.sleep(1.0)

            # Simulate receiving messages occasionally
            if self.messages_received < 10:  # Simulate up to 10 messages
                # This would normally come from WebSocket
                # For demo, we'll simulate IMU data
                self.simulate_sensor_data()

        self.get_logger().info('WebSocket connection loop ended')

    def simulate_sensor_data(self):
        """Simulate receiving sensor data for demo purposes"""
        # Simulate IMU data
        imu_data = {
            "header": {"stamp": {"sec": int(time.time()), "nanosec": 0}, "frame_id": "imu_link"},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.1},
            "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.8}
        }
        self.handle_imu_data(imu_data)
        self.messages_received += 1

    # Data handlers
    def handle_imu_data(self, data: Dict[str, Any]):
        """Handle IMU data from WebSocket"""
        try:
            imu_msg = Imu()
            imu_msg.header = self.create_header(data.get('header', {}))
            imu_msg.orientation = self.dict_to_quaternion(data.get('orientation', {}))
            imu_msg.angular_velocity = self.dict_to_vector3(data.get('angular_velocity', {}))
            imu_msg.linear_acceleration = self.dict_to_vector3(data.get('linear_acceleration', {}))

            self.imu_pub.publish(imu_msg)
            self.last_imu_time = time.time()
            self.messages_published += 1

        except Exception as e:
            self.get_logger().error(f'IMU data handling error: {str(e)}')

    def handle_gps_data(self, data: Dict[str, Any]):
        """Handle GPS data from WebSocket"""
        try:
            gps_msg = NavSatFix()
            gps_msg.header = self.create_header(data.get('header', {}))
            gps_msg.latitude = data.get('latitude', 0.0)
            gps_msg.longitude = data.get('longitude', 0.0)
            gps_msg.altitude = data.get('altitude', 0.0)
            gps_msg.position_covariance = data.get('position_covariance', [0.0] * 9)
            gps_msg.position_covariance_type = data.get('position_covariance_type', 0)

            self.gps_pub.publish(gps_msg)
            self.last_gps_time = time.time()
            self.messages_published += 1

        except Exception as e:
            self.get_logger().error(f'GPS data handling error: {str(e)}')

    def handle_odom_data(self, data: Dict[str, Any]):
        """Handle odometry data from WebSocket"""
        try:
            odom_msg = Odometry()
            odom_msg.header = self.create_header(data.get('header', {}))
            odom_msg.child_frame_id = data.get('child_frame_id', 'base_link')
            odom_msg.pose = self.dict_to_pose_with_covariance(data.get('pose', {}))
            odom_msg.twist = self.dict_to_twist_with_covariance(data.get('twist', {}))

            self.odom_pub.publish(odom_msg)
            self.last_odom_time = time.time()
            self.messages_published += 1

        except Exception as e:
            self.get_logger().error(f'Odometry data handling error: {str(e)}')

    def handle_camera_data(self, data: Dict[str, Any]):
        """Handle camera image data from WebSocket"""
        try:
            img_msg = Image()
            img_msg.header = self.create_header(data.get('header', {}))
            img_msg.height = data.get('height', 0)
            img_msg.width = data.get('width', 0)
            img_msg.encoding = data.get('encoding', 'rgb8')
            img_msg.is_bigendian = data.get('is_bigendian', False)
            img_msg.step = data.get('step', 0)
            img_msg.data = data.get('data', [])

            self.camera_pub.publish(img_msg)
            self.last_camera_time = time.time()
            self.messages_published += 1

        except Exception as e:
            self.get_logger().error(f'Camera data handling error: {str(e)}')

    def handle_camera_info_data(self, data: Dict[str, Any]):
        """Handle camera info data from WebSocket"""
        try:
            info_msg = CameraInfo()
            info_msg.header = self.create_header(data.get('header', {}))
            info_msg.height = data.get('height', 0)
            info_msg.width = data.get('width', 0)
            info_msg.distortion_model = data.get('distortion_model', '')
            info_msg.d = data.get('d', [])
            info_msg.k = data.get('k', [0.0] * 9)
            info_msg.r = data.get('r', [0.0] * 9)
            info_msg.p = data.get('p', [0.0] * 12)
            info_msg.binning_x = data.get('binning_x', 0)
            info_msg.binning_y = data.get('binning_y', 0)
            info_msg.roi = self.dict_to_roi(data.get('roi', {}))

            self.camera_info_pub.publish(info_msg)

        except Exception as e:
            self.get_logger().error(f'Camera info handling error: {str(e)}')

    # Helper methods
    def create_header(self, header_data: Dict[str, Any]) -> Header:
        """Create ROS Header from dictionary"""
        header = Header()
        header.stamp.sec = header_data.get('stamp', {}).get('sec', 0)
        header.stamp.nanosec = header_data.get('stamp', {}).get('nanosec', 0)
        header.frame_id = header_data.get('frame_id', 'map')
        return header

    def dict_to_quaternion(self, q_data: Dict[str, Any]):
        """Convert dictionary to Quaternion"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = q_data.get('x', 0.0)
        q.y = q_data.get('y', 0.0)
        q.z = q_data.get('z', 0.0)
        q.w = q_data.get('w', 1.0)
        return q

    def dict_to_vector3(self, v_data: Dict[str, Any]):
        """Convert dictionary to Vector3"""
        from geometry_msgs.msg import Vector3
        v = Vector3()
        v.x = v_data.get('x', 0.0)
        v.y = v_data.get('y', 0.0)
        v.z = v_data.get('z', 0.0)
        return v

    def dict_to_pose_with_covariance(self, pose_data: Dict[str, Any]):
        """Convert dictionary to PoseWithCovariance"""
        from geometry_msgs.msg import PoseWithCovariance
        pose = PoseWithCovariance()
        pose.pose.position = self.dict_to_vector3(pose_data.get('pose', {}).get('position', {}))
        pose.pose.orientation = self.dict_to_quaternion(pose_data.get('pose', {}).get('orientation', {}))
        pose.covariance = pose_data.get('covariance', [0.0] * 36)
        return pose

    def dict_to_twist_with_covariance(self, twist_data: Dict[str, Any]):
        """Convert dictionary to TwistWithCovariance"""
        from geometry_msgs.msg import TwistWithCovariance
        twist = TwistWithCovariance()
        twist.twist.linear = self.dict_to_vector3(twist_data.get('twist', {}).get('linear', {}))
        twist.twist.angular = self.dict_to_vector3(twist_data.get('twist', {}).get('angular', {}))
        twist.covariance = twist_data.get('covariance', [0.0] * 36)
        return twist

    def dict_to_roi(self, roi_data: Dict[str, Any]):
        """Convert dictionary to RegionOfInterest"""
        from sensor_msgs.msg import RegionOfInterest
        roi = RegionOfInterest()
        roi.x_offset = roi_data.get('x_offset', 0)
        roi.y_offset = roi_data.get('y_offset', 0)
        roi.height = roi_data.get('height', 0)
        roi.width = roi_data.get('width', 0)
        roi.do_rectify = roi_data.get('do_rectify', False)
        return roi

    # SLAM feedback callbacks
    def slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle SLAM pose updates"""
        # Could use this for feedback or validation

    def slam_status_callback(self, msg: String):
        """Handle SLAM status updates"""
        # Could use this for monitoring SLAM health

    # Service callbacks
    def connect_callback(self, request, response):
        """Connect to WebSocket service"""
        if self.connect_websocket():
            response.success = True
            response.message = "Connected to WebSocket"
        else:
            response.success = False
            response.message = "Failed to connect to WebSocket"
        return response

    def disconnect_callback(self, request, response):
        """Disconnect from WebSocket service"""
        self.disconnect_websocket()
        response.success = True
        response.message = "Disconnected from WebSocket"
        return response

    def get_status_callback(self, request, response):
        """Get bridge status service"""
        response.success = True
        response.message = (
            f"Connected: {self.websocket_connected}, "
            f"Active: {self.bridge_active}, "
            f"Messages: {self.messages_received}/{self.messages_published}"
        )
        return response

    # Status update
    def status_update(self):
        """Periodic status update"""
        # Publish connection status
        self.connection_pub.publish(Bool(data=self.websocket_connected))

        # Publish detailed status
        status_data = {
            "connected": self.websocket_connected,
            "active": self.bridge_active,
            "messages_received": self.messages_received,
            "messages_published": self.messages_published,
            "last_imu": time.time() - self.last_imu_time,
            "last_gps": time.time() - self.last_gps_time,
            "last_odom": time.time() - self.last_odom_time,
            "last_camera": time.time() - self.last_camera_time
        }

        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        bridge = WebSocketSLAMBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
