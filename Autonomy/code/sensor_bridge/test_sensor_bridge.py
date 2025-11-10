#!/usr/bin/env python3
"""
Test script for WebSocket Sensor Bridge

Simulates WebSocket sensor data and verifies ROS2 topic publishing.
"""

import asyncio
import websockets
import json
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, BatteryState, Odometry, Temperature


class SensorDataVerifier(Node):
    """ROS2 node to verify sensor data publishing"""

    def __init__(self):
        super().__init__('sensor_verifier')

        # Track received messages
        self.received_messages = {
            'imu': [],
            'gps': [],
            'battery': [],
            'wheel_odom': [],
            'temperature': []
        }

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.on_imu, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.on_gps, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery/status', self.on_battery, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/wheel/odom', self.on_wheel_odom, 10)
        self.temp_sub = self.create_subscription(
            Temperature, '/temperature/data', self.on_temperature, 10)

        self.get_logger().info('Sensor data verifier initialized')

    def on_imu(self, msg: Imu):
        self.received_messages['imu'].append({
            'timestamp': time.time(),
            'accel_x': msg.linear_acceleration.x,
            'gyro_z': msg.angular_velocity.z
        })
        self.get_logger().info(f'IMU: accel_x={msg.linear_acceleration.x:.2f}')

    def on_gps(self, msg: NavSatFix):
        self.received_messages['gps'].append({
            'timestamp': time.time(),
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        })
        self.get_logger().info(f'GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}')

    def on_battery(self, msg: BatteryState):
        self.received_messages['battery'].append({
            'timestamp': time.time(),
            'voltage': msg.voltage,
            'percentage': msg.percentage
        })
        self.get_logger().info(f'Battery: {msg.percentage:.1f}%')

    def on_wheel_odom(self, msg: Odometry):
        self.received_messages['wheel_odom'].append({
            'timestamp': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'vx': msg.twist.twist.linear.x
        })
        self.get_logger().info(f'Odom: x={msg.pose.pose.position.x:.2f}, vx={msg.twist.twist.linear.x:.2f}')

    def on_temperature(self, msg: Temperature):
        self.received_messages['temperature'].append({
            'timestamp': time.time(),
            'temperature': msg.temperature
        })
        self.get_logger().info(f'Temp: {msg.temperature:.1f}°C')

    def get_stats(self):
        """Get reception statistics"""
        return {
            sensor: len(messages)
            for sensor, messages in self.received_messages.items()
        }


async def mock_websocket_server(websocket, path):
    """Mock WebSocket server that sends simulated sensor data"""
    print("WebSocket client connected")

    message_count = 0
    base_time = time.time()

    try:
        while True:
            # Generate simulated sensor data
            timestamp = time.time()
            sensor_data = {
                "timestamp": timestamp,
                "sensors": {
                    "imu": {
                        "accel": {"x": 0.1, "y": 0.2, "z": 9.81},
                        "gyro": {"x": 0.01, "y": 0.02, "z": 0.03},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    },
                    "gps": {
                        "lat": 37.7749 + 0.001 * message_count,  # Slowly moving
                        "lon": -122.4194 + 0.001 * message_count,
                        "altitude": 10.5,
                        "status": 0,
                        "position_covariance": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
                    },
                    "battery": {
                        "voltage": 12.3,
                        "current": 2.1,
                        "percentage": 85.0,
                        "temperature": 35.0,
                        "status": 0,
                        "health": 0,
                        "technology": 0
                    },
                    "wheel_odom": {
                        "x": 0.1 * message_count,  # Moving forward
                        "y": 0.0,
                        "z": 0.0,
                        "qx": 0.0,
                        "qy": 0.0,
                        "qz": 0.0,
                        "qw": 1.0,
                        "vx": 0.5,
                        "vy": 0.0,
                        "vz": 0.0,
                        "wx": 0.0,
                        "wy": 0.0,
                        "wz": 0.1,
                        "pose_covariance": [0.1] * 36,
                        "twist_covariance": [0.1] * 36
                    },
                    "temperature": {
                        "temperature": 35.0 + 0.1 * message_count,  # Warming up
                        "variance": 0.1
                    }
                }
            }

            # Send JSON data
            await websocket.send(json.dumps(sensor_data))
            message_count += 1

            # Send at 10Hz (100ms intervals)
            await asyncio.sleep(0.1)

    except websockets.exceptions.ConnectionClosed:
        print("WebSocket client disconnected")


async def run_mock_server():
    """Run the mock WebSocket server"""
    server = await websockets.serve(mock_websocket_server, "localhost", 8080)
    print("Mock WebSocket server started on ws://localhost:8080")
    await server.wait_closed()


def run_mock_server_thread():
    """Run mock server in a separate thread"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_mock_server())


def main():
    """Main test function"""
    # Start ROS2
    rclpy.init()

    # Create verifier node
    verifier = SensorDataVerifier()

    # Start mock WebSocket server in background thread
    server_thread = threading.Thread(target=run_mock_server_thread, daemon=True)
    server_thread.start()

    # Give server time to start
    time.sleep(1)

    # Start sensor bridge (this would normally be done via ros2 run)
    # For testing, we'll simulate the bridge behavior

    print("Starting sensor data verification...")
    print("Test will run for 10 seconds...")

    start_time = time.time()
    while time.time() - start_time < 10.0:
        rclpy.spin_once(verifier, timeout_sec=0.1)

    # Print results
    stats = verifier.get_stats()
    print("\n=== Test Results ===")
    print(f"Duration: {time.time() - start_time:.1f} seconds")
    print("Messages received:")
    for sensor, count in stats.items():
        rate = count / 10.0  # messages per second
        print(f"  {sensor}: {count} messages ({rate:.1f} Hz)")

    # Check if we received reasonable data
    success = True
    for sensor, count in stats.items():
        if count < 50:  # Should get ~100 messages in 10 seconds at 10Hz
            print(f"❌ {sensor}: Too few messages ({count})")
            success = False
        else:
            print(f"✅ {sensor}: Good message rate")

    if success:
        print("\n🎉 Test PASSED: Sensor bridge is working correctly!")
    else:
        print("\n💥 Test FAILED: Check sensor bridge implementation")

    verifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
