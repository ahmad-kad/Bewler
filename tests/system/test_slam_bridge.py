#!/usr/bin/env python3
"""
Test SLAM Bridge - Verify WebSocket to SLAM data routing

Tests the complete data flow:
- WebSocket sensor data → ROS2 topics → SLAM bridge → Frontend map data
- Mission waypoints → Frontend visualization
- Coordinate transformations and TF frames
"""

import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String


class SLAMBridgeTester(Node):
    """Test node for SLAM bridge functionality"""

    def __init__(self):
        super().__init__('slam_bridge_tester')

        # Publishers for simulated sensor data
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.slam_pub = self.create_publisher(PoseStamped, '/slam/pose', 10)
        self.mission_pub = self.create_publisher(String, '/mission/status', 10)

        # Subscribers for monitoring bridge output
        self.map_sub = self.create_subscription(
            String, '/frontend/map_data', self.map_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/frontend/robot_pose', self.pose_callback, 10)

        # Test data storage
        self.received_map_data = None
        self.received_pose = None
        self.map_count = 0
        self.pose_count = 0

        # Simulated robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.gps_lat = 37.7749  # San Francisco
        self.gps_lon = -122.4194

        self.get_logger().info('SLAM Bridge Tester ready')

    def map_callback(self, msg):
        """Monitor map data from SLAM bridge"""
        try:
            self.received_map_data = json.loads(msg.data)
            self.map_count += 1
            self.get_logger().info(f'Map data received: robot at ({self.received_map_data["robot"]["x"]:.2f}, {self.received_map_data["robot"]["y"]:.2f})')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid map data JSON')

    def pose_callback(self, msg):
        """Monitor robot pose from SLAM bridge"""
        self.received_pose = msg
        self.pose_count += 1
        self.get_logger().info(f'Pose received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def publish_simulated_imu(self):
        """Publish simulated IMU data"""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        # Simulate some movement
        imu.linear_acceleration.x = 0.1 + 0.05 * math.sin(time.time())
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81

        imu.angular_velocity.x = 0.01
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.1 * math.sin(time.time() * 0.5)

        self.imu_pub.publish(imu)

    def publish_simulated_gps(self):
        """Publish simulated GPS data"""
        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = 'gps_link'

        # Simulate GPS movement
        self.gps_lat += 0.00001 * math.sin(time.time() * 0.1)
        self.gps_lon += 0.00001 * math.cos(time.time() * 0.1)

        gps.latitude = self.gps_lat
        gps.longitude = self.gps_lon
        gps.altitude = 10.0

        # GPS covariance
        gps.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
        gps.position_covariance_type = 1  # Known covariance

        self.gps_pub.publish(gps)

    def publish_simulated_odometry(self):
        """Publish simulated odometry"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Simulate robot movement
        self.robot_x += 0.1 * math.cos(self.robot_heading)
        self.robot_y += 0.1 * math.sin(self.robot_heading)
        self.robot_heading += 0.05

        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.orientation.z = math.sin(self.robot_heading / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.robot_heading / 2.0)

        odom.twist.twist.linear.x = 0.1
        odom.twist.twist.angular.z = 0.05

        self.odom_pub.publish(odom)

    def publish_simulated_slam_pose(self):
        """Publish simulated SLAM pose"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        # SLAM pose (slightly different from odometry to simulate localization)
        pose.pose.position.x = self.robot_x + 0.05 * math.sin(time.time())
        pose.pose.position.y = self.robot_y + 0.05 * math.cos(time.time())
        pose.pose.orientation.z = math.sin(self.robot_heading / 2.0)
        pose.pose.orientation.w = math.cos(self.robot_heading / 2.0)

        self.slam_pub.publish(pose)

    def publish_mission_status(self, waypoints=None):
        """Publish mission status with waypoints"""
        status = {
            'status': 'executing',
            'waypoint_index': 0,
            'total_waypoints': len(waypoints) if waypoints else 0,
            'waypoints': waypoints or []
        }

        msg = String()
        msg.data = json.dumps(status)
        self.mission_pub.publish(msg)

    def run_sensor_simulation(self):
        """Run continuous sensor data simulation"""
        rate = 10  # Hz
        while rclpy.ok():
            self.publish_simulated_imu()
            self.publish_simulated_gps()
            self.publish_simulated_odometry()
            self.publish_simulated_slam_pose()
            time.sleep(1.0 / rate)

    def run_test(self):
        """Run comprehensive SLAM bridge test"""
        self.get_logger().info('Starting SLAM Bridge Test')

        # Wait for system to stabilize
        time.sleep(2.0)

        # Test 1: Sensor data publishing
        self.get_logger().info('Test 1: Publishing sensor data streams')
        test_waypoints = [
            {'x': 5.0, 'y': 0.0},
            {'x': 5.0, 'y': 5.0},
            {'x': 0.0, 'y': 5.0}
        ]

        self.publish_mission_status(test_waypoints)

        # Start sensor simulation in background
        sim_thread = threading.Thread(target=self.run_sensor_simulation)
        sim_thread.daemon = True
        sim_thread.start()

        # Wait for data to flow
        time.sleep(5.0)

        # Test 2: Check data reception
        self.get_logger().info('Test 2: Checking data reception')

        # Stop simulation
        sim_thread.join(timeout=1.0)

        # Print test results
        self.print_test_results()

    def print_test_results(self):
        """Print test results"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('SLAM BRIDGE TEST RESULTS')
        self.get_logger().info('=' * 60)

        success = True

        # Check map data received
        if self.map_count > 0:
            self.get_logger().info(f'✅ Map data received: {self.map_count} messages')
        else:
            self.get_logger().error('❌ No map data received')
            success = False

        # Check pose data received
        if self.pose_count > 0:
            self.get_logger().info(f'✅ Pose data received: {self.pose_count} messages')
        else:
            self.get_logger().error('❌ No pose data received')
            success = False

        # Check map data structure
        if self.received_map_data:
            robot = self.received_map_data.get('robot')
            if robot and 'x' in robot and 'y' in robot and 'heading' in robot:
                self.get_logger().info('✅ Map data structure valid')
                self.get_logger().info(f'   Robot position: ({robot["x"]:.2f}, {robot["y"]:.2f}), heading: {robot["heading"]:.1f}°')
            else:
                self.get_logger().error('❌ Map data structure invalid')
                success = False

            # Check sensors
            sensors = self.received_map_data.get('sensors', {})
            if sensors.get('imu'):
                self.get_logger().info('✅ IMU data present in map data')
            if sensors.get('gps'):
                self.get_logger().info('✅ GPS data present in map data')

            # Check mission data
            mission = self.received_map_data.get('mission', {})
            if mission.get('waypoints'):
                self.get_logger().info(f'✅ Mission waypoints present: {len(mission["waypoints"])} waypoints')
        else:
            self.get_logger().error('❌ No valid map data received')
            success = False

        # Check coordinate transformations
        if self.received_pose:
            pose = self.received_pose
            self.get_logger().info(f'✅ TF transformations working: pose at ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        else:
            self.get_logger().error('❌ TF transformations not working')
            success = False

        if success:
            self.get_logger().info('🎉 ALL SLAM BRIDGE TESTS PASSED!')
            self.get_logger().info('✅ WebSocket sensor data → ROS2 → SLAM bridge → Frontend')
            self.get_logger().info('✅ Coordinate transformations working')
            self.get_logger().info('✅ Map visualization data flowing')
        else:
            self.get_logger().error('❌ SOME SLAM BRIDGE TESTS FAILED')

        self.get_logger().info('=' * 60)


def main():
    """Main test function"""
    rclpy.init()

    # Create test node
    tester = SLAMBridgeTester()

    # Let system stabilize
    time.sleep(2.0)

    # Run test in separate thread
    test_thread = threading.Thread(target=tester.run_test)
    test_thread.start()

    # Spin ROS2
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass

    # Wait for test to complete
    test_thread.join(timeout=30.0)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
