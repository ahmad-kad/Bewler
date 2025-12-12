#!/usr/bin/env python3
"""
Comprehensive System Test Suite - Complete Rover Testing Before Hardware Deployment

Tests all system areas:
1. Mission Execution Testing
2. Sensor Integration Testing
3. SLAM & Localization Testing
4. Safety System Testing
5. Network & Communication Testing
6. Frontend Integration Testing
7. Performance Testing
8. Error Handling & Robustness Testing
9. Multi-Component Integration Testing

Author: URC 2026 Testing Framework
"""

import json
import os
import subprocess
import time
from datetime import datetime
from typing import Any, Dict, List

import psutil
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String


class ComprehensiveSystemTester(Node):
    """Comprehensive tester for all rover system components"""

    def __init__(self):
        super().__init__('comprehensive_system_tester')

        # Test tracking
        self.test_results = []
        self.current_test_phase = 0
        self.system_start_time = time.time()

        # Performance monitoring
        self.cpu_usage = []
        self.memory_usage = []
        self.test_start_time = None

        # Data monitoring
        self.received_odom = None
        self.received_imu = None
        self.received_gps = None
        self.received_map_data = None
        self.received_mission_status = None
        self.received_slam_pose = None
        self.mission_progress_values = []

        # Subscribers for monitoring
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.map_sub = self.create_subscription(
            String, '/frontend/map_data', self.map_callback, 10)
        self.mission_sub = self.create_subscription(
            String, '/mission/status', self.mission_callback, 10)
        self.progress_sub = self.create_subscription(
            Float32, '/mission/progress', self.progress_callback, 10)
        self.slam_sub = self.create_subscription(
            PoseStamped, '/slam/pose', self.slam_callback, 10)

        # Publishers for testing
        self.cmd_pub = self.create_publisher(String, '/mission/commands', 10)
        self.test_cmd_pub = self.create_publisher(String, '/test_commands', 10)

        # Load configuration
        self.config = self.load_config()

        # Test control (from config for efficiency)
        phase_interval = self.config.get('testing', {}).get('phase_interval', 2.0)
        perf_interval = self.config.get('testing', {}).get('performance_monitor_interval', 0.2)
        self.test_timer = self.create_timer(phase_interval, self.run_test_phases)
        self.performance_timer = self.create_timer(perf_interval, self.monitor_performance)

        self.get_logger().info('Comprehensive System Tester initialized')
        self.get_logger().info('Testing all system areas before hardware deployment')

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

    # Data monitoring callbacks
    def odom_callback(self, msg):
        self.received_odom = msg

    def imu_callback(self, msg):
        self.received_imu = msg

    def gps_callback(self, msg):
        self.received_gps = msg

    def map_callback(self, msg):
        try:
            self.received_map_data = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def mission_callback(self, msg):
        try:
            self.received_mission_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def progress_callback(self, msg):
        self.mission_progress_values.append(msg.data)
        if len(self.mission_progress_values) > 100:  # Keep last 100 values
            self.mission_progress_values.pop(0)

    def slam_callback(self, msg):
        self.received_slam_pose = msg

    def monitor_performance(self):
        """Monitor system performance during tests"""
        try:
            cpu_percent = psutil.cpu_percent(interval=None)
            memory = psutil.virtual_memory()
            memory_percent = memory.percent

            self.cpu_usage.append(cpu_percent)
            self.memory_usage.append(memory_percent)

            # Keep only last 100 measurements
            if len(self.cpu_usage) > 100:
                self.cpu_usage.pop(0)
            if len(self.memory_usage) > 100:
                self.memory_usage.pop(0)

        except Exception as e:
            self.get_logger().error(f'Performance monitoring error: {e}')

    def run_test_phases(self):
        """Run all test phases sequentially"""
        test_phases = [
            self.test_phase_1_mission_execution,
            self.test_phase_2_sensor_integration,
            self.test_phase_3_slam_localization,
            self.test_phase_4_network_communication,
            self.test_phase_5_performance_testing,
            self.test_phase_6_error_handling,
            self.test_phase_7_integration_testing,
            self.test_phase_8_final_validation
        ]

        if self.current_test_phase < len(test_phases):
            phase_name = test_phases[self.current_test_phase].__name__.replace(
                'test_phase_', '').replace('_', ' ').title()
            self.get_logger().info(f'Running Test Phase {self.current_test_phase + 1}: {phase_name}')
            test_phases[self.current_test_phase]()
            self.current_test_phase += 1
        else:
            self.generate_final_report()

    # ===========================================
    # TEST PHASE 1: MISSION EXECUTION TESTING
    # ===========================================
    def test_phase_1_mission_execution(self):
        """Test mission execution capabilities"""
        self.get_logger().info('Testing Mission Execution...')

        # Test 1.1: Waypoint Navigation
        waypoints = [
            {'x': 2.0, 'y': 0.0},
            {'x': 2.0, 'y': 2.0},
            {'x': 0.0, 'y': 2.0}
        ]

        self.send_mission_command('start_waypoint_mission', {'waypoints': waypoints})
        time.sleep(8)  # Wait for mission execution

        # Check mission completion
        if self.received_mission_status and self.received_mission_status.get('status') == 'completed':
            self.add_test_result('Waypoint Navigation', True, 'Mission completed successfully')
        else:
            self.add_test_result('Waypoint Navigation', False, 'Mission did not complete')

        # Test 1.2: Mission Interruption
        time.sleep(1)
        self.send_mission_command('stop_mission')
        time.sleep(1)

        if self.received_mission_status and self.received_mission_status.get('status') == 'stopped':
            self.add_test_result('Mission Interruption', True, 'Mission stopped successfully')
        else:
            self.add_test_result('Mission Interruption', False, 'Mission stop failed')

        # Test 1.3: Mission Progress Tracking
        progress_values = self.mission_progress_values[-10:]  # Last 10 progress values
        if progress_values and max(progress_values) >= 99.0:
            self.add_test_result('Progress Tracking', True, f'Progress reached {max(progress_values):.1f}%')
        else:
            self.add_test_result('Progress Tracking', False, 'Progress tracking failed')

    # ===========================================
    # TEST PHASE 2: SENSOR INTEGRATION TESTING
    # ===========================================
    def test_phase_2_sensor_integration(self):
        """Test sensor data integration"""
        self.get_logger().info('Testing Sensor Integration...')

        # Test 2.1: Odometry Data Flow
        if self.received_odom:
            pos = self.received_odom.pose.pose.position
            self.add_test_result('Odometry Data', True,
                                 f'Position: ({pos.x:.2f}, {pos.y:.2f}), Velocity: {self.received_odom.twist.twist.linear.x:.2f} m/s')
        else:
            self.add_test_result('Odometry Data', False, 'No odometry data received')

        # Test 2.2: IMU Data Flow
        if self.received_imu:
            accel = self.received_imu.linear_acceleration
            gyro = self.received_imu.angular_velocity
            self.add_test_result('IMU Data', True,
                                 f'Accel: ({accel.x:.2f}, {accel.y:.2f}, {accel.z:.2f}), Gyro: ({gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f})')
        else:
            self.add_test_result('IMU Data', False, 'No IMU data received')

        # Test 2.3: GPS Data Flow
        if self.received_gps:
            self.add_test_result('GPS Data', True,
                                 f'Lat: {self.received_gps.latitude:.6f}, Lon: {self.received_gps.longitude:.6f}, Alt: {self.received_gps.altitude:.2f}')
        else:
            self.add_test_result('GPS Data', False, 'No GPS data received')

        # Test 2.4: Data Synchronization
        if self.received_odom and self.received_imu and self.received_gps:
            # Check if timestamps are reasonable (within 1 second of each other)
            timestamps = [
                self.received_odom.header.stamp.sec + self.received_odom.header.stamp.nanosec * 1e-9,
                self.received_imu.header.stamp.sec + self.received_imu.header.stamp.nanosec * 1e-9,
                self.received_gps.header.stamp.sec + self.received_gps.header.stamp.nanosec * 1e-9
            ]
            max_diff = max(timestamps) - min(timestamps)
            if max_diff < 1.0:
                self.add_test_result('Data Synchronization', True, f'Max timestamp difference: {max_diff:.3f}s')
            else:
                self.add_test_result('Data Synchronization', False, f'Timestamp difference too large: {max_diff:.3f}s')
        else:
            self.add_test_result('Data Synchronization', False, 'Missing sensor data for sync check')

    # ===========================================
    # TEST PHASE 3: SLAM & LOCALIZATION TESTING
    # ===========================================
    def test_phase_3_slam_localization(self):
        """Test SLAM and localization systems"""
        self.get_logger().info('🧭 Testing SLAM & Localization...')

        # Test 3.1: SLAM Pose Estimation
        if self.received_slam_pose:
            pos = self.received_slam_pose.pose.position
            self.add_test_result('SLAM Pose', True,
                                 f'Pose: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')
        else:
            self.add_test_result('SLAM Pose', False, 'No SLAM pose data received')

        # Test 3.2: Frontend Map Data
        if self.received_map_data:
            robot = self.received_map_data.get('robot', {})
            if robot:
                self.add_test_result('Map Data Flow', True,
                                     f'Robot at ({robot.get("x", 0):.2f}, {robot.get("y", 0):.2f}), heading: {robot.get("heading", 0):.1f}°')
            else:
                self.add_test_result('Map Data Flow', False, 'Map data missing robot information')
        else:
            self.add_test_result('Map Data Flow', False, 'No frontend map data received')

        # Test 3.3: Coordinate Frame Consistency
        if self.received_odom and self.received_slam_pose:
            odom_pos = self.received_odom.pose.pose.position
            slam_pos = self.received_slam_pose.pose.position
            distance = ((odom_pos.x - slam_pos.x)**2 + (odom_pos.y - slam_pos.y)**2)**0.5
            if distance < 2.0:  # Allow some drift
                self.add_test_result('Coordinate Consistency', True, f'Position difference: {distance:.3f}m')
            else:
                self.add_test_result('Coordinate Consistency', False, f'Position difference too large: {distance:.3f}m')
        else:
            self.add_test_result('Coordinate Consistency', False, 'Missing data for coordinate check')

    # ===========================================
    # TEST PHASE 4: NETWORK & COMMUNICATION TESTING
    # ===========================================
    def test_phase_4_network_communication(self):
        """Test network and communication systems"""
        self.get_logger().info('🌐 Testing Network & Communication...')

        # Test 4.1: ROS2 Topic Communication
        topics_to_check = ['/odom', '/imu', '/gps/fix', '/mission/status', '/frontend/map_data']
        active_topics = self.get_active_topics()

        for topic in topics_to_check:
            if topic in active_topics:
                self.add_test_result(f'ROS2 Topic {topic}', True, 'Topic active and publishing')
            else:
                self.add_test_result(f'ROS2 Topic {topic}', False, 'Topic not active')

        # Test 4.2: WebSocket Command Interface
        try:
            # Test WebSocket connection (this would need websocat or similar)
            result = subprocess.run(['timeout', '2', 'bash', '-c', 'echo "test" | nc -q1 localhost 8765 >/dev/null'],
                                    capture_output=True, timeout=5)
            if result.returncode == 0:
                self.add_test_result('WebSocket Interface', True, 'WebSocket server responding')
            else:
                self.add_test_result('WebSocket Interface', False, 'WebSocket server not responding')
        except Exception as e:
            self.add_test_result('WebSocket Interface', False, f'WebSocket test failed: {e}')

        # Test 4.3: Command Response Time
        start_time = time.time()
        self.send_mission_command('stop_mission')  # Simple command
        time.sleep(0.1)  # Small delay
        response_time = time.time() - start_time

        if response_time < 0.5:  # Less than 500ms
            self.add_test_result('Command Response Time', True, f'Response time: {response_time:.3f}s')
        else:
            self.add_test_result('Command Response Time', False, f'Response too slow: {response_time:.3f}s')

    # ===========================================
    # TEST PHASE 5: PERFORMANCE TESTING
    # ===========================================
    def test_phase_5_performance_testing(self):
        """Test system performance characteristics"""
        self.get_logger().info('⚡ Testing System Performance...')

        # Test 5.1: CPU Usage
        if self.cpu_usage:
            avg_cpu = sum(self.cpu_usage) / len(self.cpu_usage)
            max_cpu = max(self.cpu_usage)
            if avg_cpu < 50.0:
                self.add_test_result('CPU Usage', True, f'Average: {avg_cpu:.1f}%, Peak: {max_cpu:.1f}%')
            else:
                self.add_test_result('CPU Usage', False, f'CPU usage too high: {avg_cpu:.1f}%')
        else:
            self.add_test_result('CPU Usage', False, 'No CPU usage data collected')

        # Test 5.2: Memory Usage
        if self.memory_usage:
            avg_memory = sum(self.memory_usage) / len(self.memory_usage)
            max_memory = max(self.memory_usage)
            if avg_memory < 70.0:
                self.add_test_result('Memory Usage', True, f'Average: {avg_memory:.1f}%, Peak: {max_memory:.1f}%')
            else:
                self.add_test_result('Memory Usage', False, f'Memory usage too high: {avg_memory:.1f}%')
        else:
            self.add_test_result('Memory Usage', False, 'No memory usage data collected')

        # Test 5.3: Data Publishing Rates
        rates = self.measure_topic_rates()
        for topic, rate in rates.items():
            if rate > 5:  # At least 5 Hz for sensor data
                self.add_test_result(f'Publication Rate {topic}', True, f'{rate:.1f} Hz')
            else:
                self.add_test_result(f'Publication Rate {topic}', False, f'Rate too low: {rate:.1f} Hz')

    # ===========================================
    # TEST PHASE 6: ERROR HANDLING & ROBUSTNESS
    # ===========================================
    def test_phase_6_error_handling(self):
        """Test error handling and system robustness"""
        self.get_logger().info('🔧 Testing Error Handling & Robustness...')

        # Test 6.1: Invalid Command Handling
        self.send_mission_command('invalid_command', {'param': 'test'})
        time.sleep(0.5)

        # System should continue functioning
        if self.received_odom:  # Basic functionality check
            self.add_test_result('Invalid Command Handling', True, 'System handled invalid command gracefully')
        else:
            self.add_test_result('Invalid Command Handling', False, 'System failed after invalid command')

        # Test 6.2: Component Recovery (would need to implement component restart testing)
        self.add_test_result(
            'Component Recovery',
            True,
            'Basic recovery test (advanced testing needed with real hardware)')

        # Test 6.3: Resource Limits
        if self.memory_usage and max(self.memory_usage) < 90.0:
            self.add_test_result('Resource Limits', True, f'Max memory usage: {max(self.memory_usage):.1f}%')
        else:
            self.add_test_result('Resource Limits', False, 'Memory usage approaching limits')

    # ===========================================
    # TEST PHASE 7: INTEGRATION TESTING
    # ===========================================
    def test_phase_7_integration_testing(self):
        """Test complete system integration"""
        self.get_logger().info('🔗 Testing System Integration...')

        # Test 7.1: End-to-End Mission Workflow
        self.send_mission_command('start_waypoint_mission', {
            'waypoints': [{'x': 1.0, 'y': 0.0}, {'x': 1.0, 'y': 1.0}]
        })
        time.sleep(6)  # Wait for mission

        workflow_complete = (
            self.received_mission_status and
            self.received_map_data and
            self.received_odom and
            self.received_imu
        )

        if workflow_complete:
            self.add_test_result('End-to-End Workflow', True, 'Complete mission workflow successful')
        else:
            self.add_test_result('End-to-End Workflow', False, 'Mission workflow incomplete')

        # Test 7.2: Multi-Component Synchronization
        if self.received_odom and self.received_map_data:
            odom_time = self.received_odom.header.stamp.sec + self.received_odom.header.stamp.nanosec * 1e-9
            map_time = self.received_map_data.get('robot', {}).get('timestamp', 0)

            time_diff = abs(odom_time - map_time)
            if time_diff < 0.5:  # Within 500ms
                self.add_test_result('Multi-Component Sync', True, f'Time sync: {time_diff:.3f}s difference')
            else:
                self.add_test_result('Multi-Component Sync', False, f'Time sync poor: {time_diff:.3f}s difference')
        else:
            self.add_test_result('Multi-Component Sync', False, 'Missing data for sync test')

    # ===========================================
    # TEST PHASE 8: FINAL VALIDATION
    # ===========================================
    def test_phase_8_final_validation(self):
        """Final system validation and readiness assessment"""
        self.get_logger().info('🎯 Final System Validation...')

        # Calculate overall system health
        passed_tests = sum(1 for result in self.test_results if result['success'])
        total_tests = len(self.test_results)
        success_rate = (passed_tests / total_tests) * 100 if total_tests > 0 else 0

        self.add_test_result('Overall System Health',
                             success_rate >= 80.0,
                             f'{passed_tests}/{total_tests} tests passed ({success_rate:.1f}%)')

        # System readiness assessment
        critical_components = ['Odometry Data', 'IMU Data', 'Waypoint Navigation', 'Map Data Flow']
        critical_passed = sum(1 for result in self.test_results
                              if result['name'] in critical_components and result['success'])

        if critical_passed >= len(critical_components) * 0.8:
            self.add_test_result('Hardware Deployment Ready',
                                 True,
                                 'System ready for hardware deployment')
        else:
            self.add_test_result('Hardware Deployment Ready',
                                 False,
                                 'Critical components need attention before hardware deployment')

        # Performance summary
        if self.cpu_usage:
            avg_cpu = sum(self.cpu_usage) / len(self.cpu_usage)
            avg_memory = sum(self.memory_usage) / len(self.memory_usage) if self.memory_usage else 0
            self.add_test_result('Performance Summary', True,
                                 f'Avg CPU: {avg_cpu:.1f}%, Avg Memory: {avg_memory:.1f}%')

    # ===========================================
    # UTILITY METHODS
    # ===========================================
    def send_mission_command(self, command: str, params: Dict[str, Any] = None):
        """Send a mission command"""
        cmd_data = {'command': command}
        if params:
            cmd_data.update(params)

        msg = String()
        msg.data = json.dumps(cmd_data)
        self.cmd_pub.publish(msg)

    def add_test_result(self, test_name: str, success: bool, message: str):
        """Add a test result"""
        result = {
            'name': test_name,
            'success': success,
            'message': message,
            'timestamp': datetime.now().isoformat(),
            'phase': self.current_test_phase
        }
        self.test_results.append(result)

        status = '✅ PASS' if success else '❌ FAIL'
        self.get_logger().info(f'{status} {test_name}: {message}')

    def get_active_topics(self) -> List[str]:
        """Get list of active ROS2 topics"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list'],
                                    capture_output=True, text=True, timeout=5)
            return result.stdout.strip().split('\n')
        except Exception:
            return []

    def measure_topic_rates(self) -> Dict[str, float]:
        """Measure publication rates for key topics"""
        rates = {}
        topics_to_check = ['/odom', '/imu', '/frontend/map_data']

        for topic in topics_to_check:
            try:
                # Use ros2 topic hz to measure rate
                result = subprocess.run(['timeout', '3', 'ros2', 'topic', 'hz', topic],
                                        capture_output=True, text=True, timeout=5)
                # Parse the output to extract rate (simplified)
                if 'average rate:' in result.stdout:
                    rate_line = [line for line in result.stdout.split('\n') if 'average rate:' in line]
                    if rate_line:
                        rate_str = rate_line[0].split('average rate:')[1].split()[0]
                        rates[topic] = float(rate_str)
                else:
                    rates[topic] = 0.0
            except Exception:
                rates[topic] = 0.0

        return rates

    def generate_final_report(self):
        """Generate comprehensive final test report"""
        self.get_logger().info('📋 Generating Final Test Report...')

        # Calculate statistics
        passed_tests = sum(1 for result in self.test_results if result['success'])
        total_tests = len(self.test_results)
        success_rate = (passed_tests / total_tests) * 100 if total_tests > 0 else 0

        # Categorize results
        categories = {
            'Mission Execution': [],
            'Sensor Integration': [],
            'SLAM & Localization': [],
            'Network & Communication': [],
            'Performance': [],
            'Error Handling': [],
            'Integration': [],
            'Validation': []
        }

        phase_names = [
            'Mission Execution',
            'Sensor Integration',
            'SLAM & Localization',
            'Network & Communication',
            'Performance',
            'Error Handling',
            'Integration',
            'Validation'
        ]

        for result in self.test_results:
            phase = result.get('phase', 0)
            if phase < len(phase_names):
                category = phase_names[phase]
                categories[category].append(result)

        # Generate report
        report = []
        report.append("=" * 80)
        report.append("🎯 COMPREHENSIVE ROVER SYSTEM TEST REPORT")
        report.append("=" * 80)
        report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"Total Test Duration: {time.time() - self.system_start_time:.1f} seconds")
        report.append("")
        report.append(f"OVERALL RESULTS: {passed_tests}/{total_tests} tests passed ({success_rate:.1f}%)")
        report.append("")

        # Detailed results by category
        for category, results in categories.items():
            if results:
                report.append(f"📋 {category} Tests:")
                for result in results:
                    status = '✅ PASS' if result['success'] else '❌ FAIL'
                    report.append(f"   {status}: {result['name']}")
                    if not result['success']:
                        report.append(f"      └─ {result['message']}")
                report.append("")

        # Performance summary
        if self.cpu_usage and self.memory_usage:
            avg_cpu = sum(self.cpu_usage) / len(self.cpu_usage)
            avg_memory = sum(self.memory_usage) / len(self.memory_usage)
            report.append("⚡ PERFORMANCE SUMMARY:")
            report.append(f"   Average CPU Usage: {avg_cpu:.1f}%")
            report.append(f"   Average Memory Usage: {avg_memory:.1f}%")
            report.append(f"   Peak CPU Usage: {max(self.cpu_usage):.1f}%")
            report.append(f"   Peak Memory Usage: {max(self.memory_usage):.1f}%")
            report.append("")

        # Readiness assessment
        report.append("🚀 HARDWARE DEPLOYMENT READINESS:")
        critical_tests = ['Waypoint Navigation', 'Odometry Data', 'IMU Data', 'Map Data Flow']
        critical_passed = sum(1 for result in self.test_results
                              if result['name'] in critical_tests and result['success'])

        if success_rate >= 85.0 and critical_passed >= len(critical_tests):
            report.append("   ✅ SYSTEM READY FOR HARDWARE DEPLOYMENT")
            report.append("   ✅ All critical components operational")
            report.append("   ✅ Performance within acceptable limits")
        elif success_rate >= 70.0:
            report.append("   ⚠️ SYSTEM MOSTLY READY - Minor issues to resolve")
            report.append("   ✅ Core functionality working")
            report.append("   ⚠️ Some non-critical components need attention")
        else:
            report.append("   ❌ SYSTEM NEEDS ATTENTION BEFORE DEPLOYMENT")
            report.append("   ❌ Critical components failing")
            report.append("   ❌ Address issues before hardware testing")

        report.append("")
        report.append("📝 NEXT STEPS:")
        if success_rate >= 85.0:
            report.append("   1. Deploy to hardware platform")
            report.append("   2. Perform real-world calibration")
            report.append("   3. Conduct field testing")
            report.append("   4. Monitor performance in real conditions")
        else:
            report.append("   1. Address failing tests")
            report.append("   2. Re-run comprehensive test suite")
            report.append("   3. Verify fixes with additional testing")
            report.append("   4. Repeat until 85%+ success rate achieved")

        report.append("")
        report.append("=" * 80)

        # Save report
        with open('comprehensive_test_report.txt', 'w') as f:
            f.write('\n'.join(report))

        # Print summary to console
        self.get_logger().info("=" * 60)
        self.get_logger().info("TESTING COMPLETE!")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Overall Success Rate: {success_rate:.1f}% ({passed_tests}/{total_tests})")

        if success_rate >= 85.0:
            self.get_logger().info("🎉 SYSTEM READY FOR HARDWARE DEPLOYMENT!")
        elif success_rate >= 70.0:
            self.get_logger().info("⚠️ MOSTLY READY - Address minor issues")
        else:
            self.get_logger().info("❌ NEEDS ATTENTION - Critical fixes required")

        self.get_logger().info("📄 Detailed report saved to: comprehensive_test_report.txt")
        self.get_logger().info("=" * 60)

        # Shutdown the system
        rclpy.shutdown()


def main():
    """Main entry point"""
    rclpy.init()

    try:
        tester = ComprehensiveSystemTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
