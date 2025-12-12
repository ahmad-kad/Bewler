#!/usr/bin/env python3
"""
CAN Bus Mock Simulator - Simulates CAN bus data for testing

This simulator provides realistic mock data for CAN bus sensors and actuators,
clearly labeled as MOCK DATA to prevent confusion with real hardware readings.

Usage: Run as a service that WebSocket bridges can connect to for testing.
"""

import asyncio
import json
import math
import random
import threading
import time
from typing import Any, Dict

import websockets


class CANBusMockSimulator:
    """Mock CAN bus data simulator - NOT REAL HARDWARE"""

    def __init__(self, websocket_port: int = 8766):
        self.websocket_port = websocket_port
        self.mock_data = self._initialize_mock_data()
        self.last_update = time.time()
        self.update_interval = 0.1  # 10Hz updates

        # WebSocket server for external connections
        self.websocket_server = None
        self.connected_clients = set()

        # Start background update thread
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()

    def _initialize_mock_data(self) -> Dict[str, Any]:
        """Initialize mock sensor data with realistic values"""
        return {
            # Motor data
            'motor_left': {
                'position': 0.0,      # radians
                'velocity': 0.0,      # rad/s
                'current': 0.0,       # amps
                'temperature': 25.0,  # celsius
                'status': 'ok'
            },
            'motor_right': {
                'position': 0.0,
                'velocity': 0.0,
                'current': 0.0,
                'temperature': 25.0,
                'status': 'ok'
            },

            # IMU data
            'imu': {
                'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 9.81,  # m/s²
                'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,       # rad/s
                'temp': 25.0,  # celsius
                'calibrated': True
            },

            # GPS data
            'gps': {
                'latitude': 38.406,     # decimal degrees
                'longitude': -110.792,  # decimal degrees
                'altitude': 1500.0,    # meters
                'heading': 0.0,        # degrees
                'speed': 0.0,          # m/s
                'satellites': 12,
                'fix_quality': 2,      # 2 = DGPS fix
                'hdop': 0.8           # horizontal dilution of precision
            },

            # Battery monitoring
            'battery': {
                'voltage': 24.0,       # volts
                'current': 5.0,        # amps (positive = discharging)
                'temperature': 30.0,   # celsius
                'charge_level': 85.0,  # percentage
                'cell_count': 6,
                'status': 'charging'
            },

            # Environmental sensors
            'environment': {
                'cpu_temp': 45.0,      # celsius
                'motor_left_temp': 35.0,
                'motor_right_temp': 35.0,
                'ambient_temp': 22.0,
                'humidity': 45.0,      # percentage
                'pressure': 1013.25    # hPa
            },

            # System status
            'system': {
                'uptime': 0.0,         # seconds
                'cpu_usage': 15.0,     # percentage
                'memory_usage': 45.0,  # percentage
                'can_bus_status': 'ok',
                'error_count': 0,
                'last_reset': time.time()
            }
        }

    def _update_loop(self):
        """Background thread to update mock data"""
        while self.running:
            self._update_mock_data()
            time.sleep(self.update_interval)

    def _update_mock_data(self):
        """Update mock data with realistic variations"""
        now = time.time()

        # Update system uptime
        self.mock_data['system']['uptime'] = now - self.mock_data['system']['last_reset']

        # Add realistic noise to sensors
        self._add_realistic_noise()

        # Simulate battery drain/charge
        self._simulate_battery_dynamics()

        # Update GPS with slow movement if "moving"
        self._simulate_gps_movement()

        self.last_update = now

    def _add_realistic_noise(self):
        """Add realistic noise to sensor readings"""
        # IMU noise (accelerometer)
        for axis in ['x', 'y', 'z']:
            key = f'accel_{axis}'
            base_value = self.mock_data['imu'][key]
            # Add ±0.01 m/s² noise
            noise = random.uniform(-0.01, 0.01)
            self.mock_data['imu'][key] = base_value + noise

        # Gyro noise (±0.001 rad/s)
        for axis in ['x', 'y', 'z']:
            key = f'gyro_{axis}'
            noise = random.uniform(-0.001, 0.001)
            self.mock_data['imu'][key] += noise

        # Temperature variations
        for sensor in ['imu', 'battery', 'environment']:
            if 'temp' in self.mock_data[sensor]:
                temp_noise = random.uniform(-0.1, 0.1)
                self.mock_data[sensor]['temp'] += temp_noise

        # GPS HDOP variation
        hdop_noise = random.uniform(-0.05, 0.05)
        self.mock_data['gps']['hdop'] = max(0.5, min(2.0, self.mock_data['gps']['hdop'] + hdop_noise))

    def _simulate_battery_dynamics(self):
        """Simulate realistic battery behavior"""
        battery = self.mock_data['battery']

        # Simulate discharge when current > 0
        if battery['current'] > 0:
            # Discharge rate based on current draw
            discharge_rate = battery['current'] * 0.0001  # Simplified
            battery['charge_level'] = max(0.0, battery['charge_level'] - discharge_rate)

            # Voltage drops as charge decreases
            base_voltage = 25.2  # Full charge
            voltage_drop = (100.0 - battery['charge_level']) * 0.01
            battery['voltage'] = base_voltage - voltage_drop

        # Temperature increases with current draw
        temp_increase = abs(battery['current']) * 0.01
        battery['temperature'] += temp_increase - 0.05  # Gradual cooling

        # Update status
        if battery['charge_level'] < 20.0:
            battery['status'] = 'low'
        elif battery['charge_level'] < 10.0:
            battery['status'] = 'critical'
        else:
            battery['status'] = 'normal'

    def _simulate_gps_movement(self):
        """Simulate GPS movement when robot is active"""
        gps = self.mock_data['gps']

        # Very slow movement (0.001 degrees per second ≈ 111m at equator)
        lat_change = random.uniform(-0.00001, 0.00001)
        lon_change = random.uniform(-0.00001, 0.00001)

        gps['latitude'] += lat_change
        gps['longitude'] += lon_change

        # Update heading and speed based on movement
        if abs(lat_change) > 0.000001 or abs(lon_change) > 0.000001:
            gps['speed'] = random.uniform(0.1, 2.0)  # 0.1-2.0 m/s
            gps['heading'] = math.degrees(math.atan2(lon_change, lat_change))
        else:
            gps['speed'] = 0.0

    def get_mock_reading(self, sensor_type: str) -> Dict[str, Any]:
        """Get mock sensor reading with clear labeling"""
        if sensor_type in self.mock_data:
            return {
                "sensor": sensor_type,
                "data": self.mock_data[sensor_type].copy(),
                "timestamp": time.time(),
                "mock": True,
                "source": "CAN_BUS_MOCK_SIMULATOR",
                "warning": "NOT REAL CAN BUS DATA - SIMULATED FOR TESTING ONLY",
                "units": self._get_units_for_sensor(sensor_type)
            }
        else:
            return {
                "sensor": sensor_type,
                "error": f"Unknown sensor type: {sensor_type}",
                "mock": True,
                "timestamp": time.time()
            }

    def _get_units_for_sensor(self, sensor_type: str) -> Dict[str, str]:
        """Get units for sensor readings"""
        unit_maps = {
            'motor_left': {
                'position': 'radians',
                'velocity': 'rad/s',
                'current': 'amps',
                'temperature': 'celsius'
            },
            'motor_right': {
                'position': 'radians',
                'velocity': 'rad/s',
                'current': 'amps',
                'temperature': 'celsius'
            },
            'imu': {
                'accel_x': 'm/s²', 'accel_y': 'm/s²', 'accel_z': 'm/s²',
                'gyro_x': 'rad/s', 'gyro_y': 'rad/s', 'gyro_z': 'rad/s',
                'temp': 'celsius'
            },
            'gps': {
                'latitude': 'degrees', 'longitude': 'degrees', 'altitude': 'meters',
                'heading': 'degrees', 'speed': 'm/s', 'hdop': 'dimensionless'
            },
            'battery': {
                'voltage': 'volts', 'current': 'amps', 'temperature': 'celsius',
                'charge_level': 'percentage'
            },
            'environment': {
                'cpu_temp': 'celsius', 'motor_left_temp': 'celsius',
                'motor_right_temp': 'celsius', 'ambient_temp': 'celsius',
                'humidity': 'percentage', 'pressure': 'hPa'
            }
        }
        return unit_maps.get(sensor_type, {})

    def set_motor_command(self, motor: str, velocity: float):
        """Simulate motor command (for testing control systems)"""
        if motor in ['motor_left', 'motor_right']:
            self.mock_data[motor]['velocity'] = velocity
            # Simulate current draw based on velocity
            self.mock_data[motor]['current'] = abs(velocity) * 2.0
            self.get_logger().info(f"Mock motor command: {motor} -> {velocity} rad/s")

    def get_logger(self):
        """Mock logger for compatibility"""
        class MockLogger:
            def info(self, msg): print(f"[CAN_MOCK] {msg}")
            def warning(self, msg): print(f"[CAN_MOCK] WARNING: {msg}")
            def error(self, msg): print(f"[CAN_MOCK] ERROR: {msg}")
        return MockLogger()

    async def start_websocket_server(self):
        """Start WebSocket server for external connections"""
        async def handler(websocket, path):
            self.connected_clients.add(websocket)
            try:
                self.get_logger().info("CAN Mock Simulator: Client connected")

                # Send initial status
                await websocket.send(json.dumps({
                    "type": "status",
                    "message": "CAN Bus Mock Simulator Active",
                    "mock": True,
                    "timestamp": time.time()
                }))

                async for message in websocket:
                    try:
                        data = json.loads(message)
                        if data.get('type') == 'sensor_request':
                            sensor_data = self.get_mock_reading(data['sensor'])
                            await websocket.send(json.dumps(sensor_data))
                        elif data.get('type') == 'motor_command':
                            self.set_motor_command(data['motor'], data['velocity'])
                            await websocket.send(json.dumps({
                                "type": "ack",
                                "command": "motor_command",
                                "mock": True
                            }))
                    except json.JSONDecodeError:
                        await websocket.send(json.dumps({
                            "error": "Invalid JSON",
                            "mock": True
                        }))
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                self.connected_clients.discard(websocket)

        self.websocket_server = await websockets.serve(
            handler, "0.0.0.0", self.websocket_port
        )
        self.get_logger().info(f"CAN Mock Simulator WebSocket server started on port {self.websocket_port}")

        # Keep server running
        await self.websocket_server.wait_closed()

    def stop(self):
        """Stop the simulator"""
        self.running = False
        if self.websocket_server:
            self.websocket_server.close()


async def main():
    """Main entry point for running the CAN mock simulator"""
    simulator = CANBusMockSimulator()

    try:
        await simulator.start_websocket_server()
        print("🚗 CAN Bus Mock Simulator Started")
        print("📡 WebSocket server on port 8766")
        print("⚠️  WARNING: This provides MOCK DATA only - NOT REAL CAN BUS")
        print("Press Ctrl+C to stop...")

        # Keep running
        while simulator.running:
            await asyncio.sleep(1)

    except KeyboardInterrupt:
        print("\n🛑 Stopping CAN Mock Simulator...")
        simulator.stop()


if __name__ == "__main__":
    asyncio.run(main())
