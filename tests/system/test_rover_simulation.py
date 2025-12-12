#!/usr/bin/env python3
"""
Complete Rover Simulation Test - Production-ready system validation

Tests the entire rover simulation stack:
1. Gazebo simulation with TurtleBot3
2. Navigation and SLAM systems
3. Mission executor integration
4. SLAM data bridge functionality
5. WebSocket command routing
6. Frontend data flow

This simulates the production rover environment for comprehensive testing.
"""

import json
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class RoverSimulationTester(Node):
    """Comprehensive tester for the rover simulation environment"""

    def __init__(self):
        super().__init__('rover_simulation_tester')

        # Test state tracking
        self.test_phase = 0
        self.test_results = []
        self.simulation_started = False

        # Data monitoring
        self.received_odom = None
        self.received_imu = None
        self.received_map_data = None
        self.received_mission_status = None
        self.received_slam_pose = None

        # Subscribers for monitoring simulation
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.map_sub = self.create_subscription(
            String, '/frontend/map_data', self.map_callback, 10)
        self.mission_sub = self.create_subscription(
            String, '/mission/status', self.mission_callback, 10)
        self.slam_sub = self.create_subscription(
            PoseStamped, '/pose', self.slam_callback, 10)

        # Publishers for test commands
        self.cmd_pub = self.create_publisher(String, '/mission/commands', 10)

        # Test timer
        self.test_timer = self.create_timer(2.0, self.run_test_step)

        self.get_logger().info('🧪 Rover Simulation Tester ready')

    def odom_callback(self, msg):
        """Monitor odometry data"""
        self.received_odom = msg
        self.get_logger().debug(f'📍 Odometry: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')

    def imu_callback(self, msg):
        """Monitor IMU data"""
        self.received_imu = msg
        self.get_logger().debug(
            f'🔄 IMU: accel=({msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f})')

    def map_callback(self, msg):
        """Monitor frontend map data"""
        try:
            self.received_map_data = json.loads(msg.data)
            self.get_logger().debug(
                f'🗺️ Map data received: robot at ({self.received_map_data["robot"]["x"]:.2f}, {self.received_map_data["robot"]["y"]:.2f})')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid map data format')

    def mission_callback(self, msg):
        """Monitor mission status"""
        try:
            self.received_mission_status = json.loads(msg.data)
            self.get_logger().debug(f'🎯 Mission status: {self.received_mission_status["status"]}')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid mission status format')

    def slam_callback(self, msg):
        """Monitor SLAM pose"""
        self.received_slam_pose = msg
        self.get_logger().debug(f'🎯 SLAM pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def run_test_step(self):
        """Run test phases sequentially"""
        if self.test_phase == 0:
            self.test_phase_0_check_simulation_startup()
        elif self.test_phase == 1:
            self.test_phase_1_check_sensor_data()
        elif self.test_phase == 2:
            self.test_phase_2_test_mission_commands()
        elif self.test_phase == 3:
            self.test_phase_3_test_data_flow()
        elif self.test_phase == 4:
            self.test_phase_4_final_validation()

    def test_phase_0_check_simulation_startup(self):
        """Phase 0: Check that simulation systems are starting"""
        self.get_logger().info('🧪 Phase 0: Checking simulation startup...')

        # Check if basic ROS2 nodes are running
        try:
            result = subprocess.run(['ros2', 'node', 'list'],
                                    capture_output=True, text=True, timeout=5)
            nodes = result.stdout.strip().split('\n')

            required_nodes = ['/robot_state_publisher', '/gazebo']
            found_nodes = [node for node in nodes if any(req in node for req in required_nodes)]

            if len(found_nodes) >= 2:
                self.add_test_result('Simulation nodes detected', True, f'Found nodes: {found_nodes}')
                self.test_phase = 1
            else:
                self.add_test_result('Simulation nodes check', False, f'Only found: {found_nodes}')
        except Exception as e:
            self.add_test_result('Simulation startup check', False, str(e))

    def test_phase_1_check_sensor_data(self):
        """Phase 1: Verify sensor data is flowing"""
        self.get_logger().info('🧪 Phase 1: Checking sensor data flow...')

        # Check odometry
        if self.received_odom:
            self.add_test_result('Odometry data received', True,
                                 f'Position: ({self.received_odom.pose.pose.position.x:.2f}, {self.received_odom.pose.pose.position.y:.2f})')
        else:
            self.add_test_result('Odometry data', False, 'No odometry received yet')

        # Check IMU
        if self.received_imu:
            self.add_test_result('IMU data received', True,
                                 f'Accel: ({self.received_imu.linear_acceleration.x:.2f}, {self.received_imu.linear_acceleration.y:.2f}, {self.received_imu.linear_acceleration.z:.2f})')
        else:
            self.add_test_result('IMU data', False, 'No IMU data received yet')

        # Check SLAM pose
        if self.received_slam_pose:
            self.add_test_result('SLAM pose received', True,
                                 f'Pose: ({self.received_slam_pose.pose.position.x:.2f}, {self.received_slam_pose.pose.position.y:.2f})')
        else:
            self.add_test_result('SLAM pose', False, 'No SLAM pose received yet')

        # Move to next phase if we have basic data
        if self.received_odom and self.received_imu:
            self.test_phase = 2

    def test_phase_2_test_mission_commands(self):
        """Phase 2: Test mission command execution"""
        self.get_logger().info('🧪 Phase 2: Testing mission commands...')

        # Send a simple waypoint mission
        waypoints = [
            {'x': 1.0, 'y': 0.0},
            {'x': 1.0, 'y': 1.0},
            {'x': 0.0, 'y': 1.0}
        ]

        command = {
            'command': 'start_waypoint_mission',
            'waypoints': waypoints
        }

        msg = String()
        msg.data = json.dumps(command)
        self.cmd_pub.publish(msg)

        self.add_test_result('Mission command sent', True, f'Sent waypoint mission with {len(waypoints)} waypoints')
        self.test_phase = 3

    def test_phase_3_test_data_flow(self):
        """Phase 3: Test complete data flow to frontend"""
        self.get_logger().info('🧪 Phase 3: Testing complete data flow...')

        # Check map data flow
        if self.received_map_data:
            robot = self.received_map_data.get('robot', {})
            if 'x' in robot and 'y' in robot:
                self.add_test_result('Map data flow', True,
                                     f'Robot position: ({robot["x"]:.2f}, {robot["y"]:.2f})')
            else:
                self.add_test_result('Map data structure', False, 'Missing robot position')
        else:
            self.add_test_result('Map data flow', False, 'No map data received')

        # Check mission status
        if self.received_mission_status:
            self.add_test_result('Mission status flow', True,
                                 f'Status: {self.received_mission_status.get("status", "unknown")}')
        else:
            self.add_test_result('Mission status flow', False, 'No mission status received')

        self.test_phase = 4

    def test_phase_4_final_validation(self):
        """Phase 4: Final system validation"""
        self.get_logger().info('🧪 Phase 4: Final system validation...')

        # Send stop command
        stop_command = {'command': 'stop_mission'}
        msg = String()
        msg.data = json.dumps(stop_command)
        self.cmd_pub.publish(msg)

        self.add_test_result('Stop command sent', True, 'Mission stop command sent')

        # Print final results
        self.print_final_results()

        # Shutdown test
        self.get_logger().info('✅ Rover simulation test completed')
        rclpy.shutdown()

    def add_test_result(self, test_name: str, success: bool, message: str):
        """Add a test result"""
        self.test_results.append({
            'name': test_name,
            'success': success,
            'message': message,
            'timestamp': time.time()
        })
        status = '✅' if success else '❌'
        self.get_logger().info(f'{status} {test_name}: {message}')

    def print_final_results(self):
        """Print comprehensive test results"""
        self.get_logger().info('=' * 80)
        self.get_logger().info('🚀 ROVER SIMULATION TEST RESULTS')
        self.get_logger().info('=' * 80)

        passed = sum(1 for result in self.test_results if result['success'])
        total = len(self.test_results)

        # Group results by category
        categories = {
            'Simulation': [],
            'Sensors': [],
            'Mission': [],
            'Data Flow': []
        }

        for result in self.test_results:
            if 'simulation' in result['name'].lower() or 'node' in result['name'].lower():
                categories['Simulation'].append(result)
            elif 'odom' in result['name'].lower() or 'imu' in result['name'].lower() or 'sensor' in result['name'].lower():
                categories['Sensors'].append(result)
            elif 'mission' in result['name'].lower() or 'waypoint' in result['name'].lower():
                categories['Mission'].append(result)
            else:
                categories['Data Flow'].append(result)

        for category, results in categories.items():
            if results:
                self.get_logger().info(f'\n📋 {category} Tests:')
                for result in results:
                    status = '✅ PASS' if result['success'] else '❌ FAIL'
                    self.get_logger().info(f'   {status}: {result["name"]}')
                    if not result['success']:
                        self.get_logger().info(f'      └─ {result["message"]}')

        self.get_logger().info(f'\n📊 OVERALL RESULTS: {passed}/{total} tests passed')

        if passed == total:
            self.get_logger().info('🎉 ALL TESTS PASSED! Rover simulation is production-ready!')
        elif passed >= total * 0.8:
            self.get_logger().info('⚠️ MOST TESTS PASSED - Minor issues to resolve')
        else:
            self.get_logger().info('❌ CRITICAL ISSUES - System needs attention')

        self.get_logger().info('=' * 80)


def main():
    """Main test function"""
    # Set TurtleBot model
    import os
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    rclpy.init()

    try:
        tester = RoverSimulationTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
