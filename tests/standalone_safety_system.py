#!/usr/bin/env python3
"""
Enhanced Standalone Safety System for URC 2026 Testing

Provides comprehensive ROS-compatible safety services and topics for testing
the frontend without requiring the full ROS autonomy stack.

This enhanced version provides:
- Emergency stop service
- Safety recovery service
- System diagnostics service
- Watchdog monitoring service
- Sensor health service
- Real-time safety status publishing
- Active alerts management
- Comprehensive simulation capabilities
"""

import json
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class EnhancedStandaloneSafetySystem(Node):
    """Enhanced standalone safety system node for comprehensive testing."""

    def __init__(self):
        super().__init__('standalone_safety_system')

        # Safety state management
        self.emergency_stop_active = False
        self.safety_recovery_active = False
        self.system_healthy = True
        self.active_alerts = []
        self.last_emergency_time = 0
        self.recovery_attempts = 0

        # Simulation parameters
        self.simulation_mode = True
        self.failure_probability = 0.1  # 10% chance of simulated failures
        self.recovery_delay = 2.0  # seconds

        # Publishers for safety topics
        self.system_health_pub = self.create_publisher(String, '/safety/system_health', 10)
        self.active_alerts_pub = self.create_publisher(String, '/safety/active_alerts', 10)
        self.watchdog_status_pub = self.create_publisher(String, '/safety/watchdog_status', 10)
        self.sensor_health_pub = self.create_publisher(String, '/safety/sensor_health', 10)
        self.emergency_status_pub = self.create_publisher(String, '/safety/emergency_status', 10)

        # Service servers
        self.emergency_stop_srv = self.create_service(
            Trigger, '/safety/emergency_stop',
            self.handle_emergency_stop
        )

        self.recover_from_safety_srv = self.create_service(
            Trigger, '/safety/recover_from_safety',
            self.handle_recover_from_safety
        )

        self.run_diagnostics_srv = self.create_service(
            Trigger, '/safety/run_diagnostics',
            self.handle_run_diagnostics
        )

        self.watchdog_monitoring_srv = self.create_service(
            Trigger, '/safety/watchdog_monitoring',
            self.handle_watchdog_monitoring
        )

        self.sensor_health_check_srv = self.create_service(
            Trigger, '/safety/sensor_health_check',
            self.handle_sensor_health_check
        )

        # Simulation timers
        self.health_timer = self.create_timer(1.0, self.publish_system_health)
        self.alerts_timer = self.create_timer(2.0, self.publish_active_alerts)
        self.watchdog_timer = self.create_timer(3.0, self.publish_watchdog_status)
        self.sensor_timer = self.create_timer(1.5, self.publish_sensor_health)

        self.get_logger().info('Enhanced Standalone Safety System started')
        self.get_logger().info('Providing services: emergency_stop, recover_from_safety, run_diagnostics, watchdog_monitoring, sensor_health_check')

    def handle_emergency_stop(self, request, response):
        """Handle emergency stop service calls."""
        self.get_logger().info('EMERGENCY STOP TRIGGERED!')

        self.emergency_stop_active = True
        self.last_emergency_time = time.time()
        self.system_healthy = False

        # Add emergency alert
        self.active_alerts.append({
            'alert_id': f'emergency_{int(time.time())}',
            'severity': 'CRITICAL',
            'message': 'Emergency stop activated',
            'timestamp': time.time(),
            'source': 'safety_system'
        })

        # Publish emergency status
        emergency_data = {
            'emergency_active': True,
            'emergency_type': 'SOFTWARE_ESTOP',
            'timestamp': time.time(),
            'operator_initiated': True
        }
        self.emergency_status_pub.publish(String(data=json.dumps(emergency_data)))

        response.success = True
        response.message = 'Emergency stop activated successfully'
        return response

    def handle_recover_from_safety(self, request, response):
        """Handle safety recovery service calls."""
        self.get_logger().info('Safety recovery requested')

        if not self.emergency_stop_active:
            response.success = False
            response.message = 'No active safety condition to recover from'
            return response

        # Simulate recovery delay
        time.sleep(self.recovery_delay)
        self.recovery_attempts += 1

        # Simulate recovery success/failure
        recovery_success = random.random() > self.failure_probability

        if recovery_success:
            self.emergency_stop_active = False
            self.safety_recovery_active = False
            self.system_healthy = True

            # Clear emergency alerts
            self.active_alerts = [alert for alert in self.active_alerts
                                  if alert.get('severity') != 'CRITICAL']

            response.success = True
            response.message = f'Safety recovery successful (attempt {self.recovery_attempts})'
            self.get_logger().info('Safety recovery completed successfully')
        else:
            response.success = False
            response.message = f'Safety recovery failed (attempt {self.recovery_attempts}) - manual intervention required'
            self.get_logger().warn('Safety recovery failed - manual intervention may be required')

        return response

    def handle_run_diagnostics(self, request, response):
        """Handle system diagnostics service calls."""
        self.get_logger().info('Running system diagnostics')

        # Simulate diagnostic checks
        diagnostics = {
            'overall_health': 'GOOD',
            'checks': {
                'communication_systems': 'PASS',
                'power_systems': 'PASS',
                'sensor_integrity': 'PASS',
                'actuator_health': 'PASS',
                'emergency_systems': 'PASS'
            },
            'timestamp': time.time()
        }

        # Simulate occasional diagnostic warnings
        if random.random() < 0.3:  # 30% chance
            diagnostics['overall_health'] = 'WARNING'
            diagnostics['checks']['sensor_integrity'] = 'WARNING'
            diagnostics['checks']['actuator_health'] = 'WARNING'

        response.success = diagnostics['overall_health'] != 'CRITICAL'
        response.message = f'Diagnostics completed: {diagnostics["overall_health"]}'
        return response

    def handle_watchdog_monitoring(self, request, response):
        """Handle watchdog monitoring service calls."""
        self.get_logger().info('Watchdog monitoring check requested')

        watchdog_status = {
            'watchdog_active': True,
            'last_heartbeat': time.time(),
            'system_responsive': True,
            'monitored_processes': ['state_machine', 'safety_system', 'navigation', 'control'],
            'alerts_detected': len([a for a in self.active_alerts if a.get('source') == 'watchdog'])
        }

        response.success = watchdog_status['system_responsive']
        response.message = f'Watchdog monitoring: {"PASS" if response.success else "FAIL"} - {len(watchdog_status["monitored_processes"])} processes monitored'
        return response

    def handle_sensor_health_check(self, request, response):
        """Handle sensor health check service calls."""
        self.get_logger().info('Sensor health check requested')

        sensor_status = {
            'overall_health': 'GOOD',
            'sensors_checked': ['imu', 'gps', 'camera_rgb', 'camera_depth', 'lidar'],
            'failed_sensors': [],
            'degraded_sensors': []
        }

        # Simulate sensor issues occasionally
        if random.random() < 0.2:  # 20% chance of sensor issues
            sensor_status['overall_health'] = 'WARNING'
            sensor_status['degraded_sensors'].append('camera_depth')
            if random.random() < 0.5:
                sensor_status['failed_sensors'].append('gps')

        if sensor_status['failed_sensors']:
            sensor_status['overall_health'] = 'CRITICAL'

        response.success = sensor_status['overall_health'] != 'CRITICAL'
        failed_count = len(sensor_status['failed_sensors'])
        degraded_count = len(sensor_status['degraded_sensors'])
        response.message = f'Sensor health: {sensor_status["overall_health"]} - {failed_count} failed, {degraded_count} degraded'
        return response

    def publish_system_health(self):
        """Publish comprehensive system health status."""
        health_data = {
            'system_health': 'CRITICAL' if self.emergency_stop_active else 'WARNING' if not self.system_healthy else 'HEALTHY',
            'emergency_stop_active': self.emergency_stop_active,
            'safety_recovery_active': self.safety_recovery_active,
            'overall_status': 'EMERGENCY' if self.emergency_stop_active else 'NOMINAL',
            'timestamp': time.time(),
            'uptime_seconds': time.time() - self.get_clock().now().seconds_nanoseconds()[0],
            'active_alerts_count': len(self.active_alerts),
            'last_emergency_time': self.last_emergency_time,
            'recovery_attempts': self.recovery_attempts
        }

        self.system_health_pub.publish(String(data=json.dumps(health_data)))

    def publish_active_alerts(self):
        """Publish current active alerts."""
        alerts_data = {
            'active_alerts': self.active_alerts,
            'alert_count': len(self.active_alerts),
            'timestamp': time.time(),
            'system_status': 'EMERGENCY' if self.emergency_stop_active else 'NOMINAL'
        }

        self.active_alerts_pub.publish(String(data=json.dumps(alerts_data)))

    def publish_watchdog_status(self):
        """Publish watchdog monitoring status."""
        watchdog_data = {
            'watchdog_active': True,
            'system_responsive': not self.emergency_stop_active,
            'last_check_time': time.time(),
            'monitored_systems': ['state_machine', 'safety_system', 'navigation', 'control', 'sensors'],
            'system_health_score': 0.9 if not self.emergency_stop_active else 0.2,
            'alerts_since_last_check': len([a for a in self.active_alerts if a.get('timestamp', 0) > time.time() - 3])
        }

        self.watchdog_status_pub.publish(String(data=json.dumps(watchdog_data)))

    def publish_sensor_health(self):
        """Publish sensor health status."""
        sensor_data = {
            'sensors': {
                'imu': {'status': 'HEALTHY', 'last_update': time.time(), 'quality_score': 0.95},
                'gps': {'status': 'HEALTHY', 'last_update': time.time(), 'quality_score': 0.90},
                'camera_rgb': {'status': 'HEALTHY', 'last_update': time.time(), 'quality_score': 0.92},
                'camera_depth': {'status': 'HEALTHY', 'last_update': time.time(), 'quality_score': 0.88},
                'lidar': {'status': 'HEALTHY', 'last_update': time.time(), 'quality_score': 0.94}
            },
            'overall_health': 'GOOD',
            'timestamp': time.time()
        }

        # Simulate occasional sensor degradation
        if random.random() < 0.1:  # 10% chance
            degraded_sensor = random.choice(list(sensor_data['sensors'].keys()))
            sensor_data['sensors'][degraded_sensor]['status'] = 'DEGRADED'
            sensor_data['sensors'][degraded_sensor]['quality_score'] = 0.7
            sensor_data['overall_health'] = 'WARNING'

        self.sensor_health_pub.publish(String(data=json.dumps(sensor_data)))


def main():
    rclpy.init()
    node = EnhancedStandaloneSafetySystem()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Enhanced Standalone Safety System Stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
