#!/usr/bin/env python3
"""
Mock Sensor Interfaces for Testing

Provides realistic mock implementations of sensor interfaces for testing.
"""

import time
from typing import Dict, List, Tuple

import numpy as np


class MockGPS:
    """Mock GPS sensor interface."""

    def __init__(self, initial_position: Tuple[float, float] = (37.7749, -122.4194)):
        self.position = initial_position
        self.altitude = 10.0
        self.accuracy = 2.5  # meters
        self.satellites = 8
        self.fix_type = "3D"

    def get_position(self) -> Tuple[float, float, float]:
        """Get current GPS position (lat, lon, alt)."""
        # Add small random variation to simulate GPS noise
        noise = np.random.normal(0, 0.1, 3)
        return (
            self.position[0] + noise[0],
            self.position[1] + noise[1],
            self.altitude + noise[2]
        )

    def get_accuracy(self) -> float:
        """Get position accuracy in meters."""
        return self.accuracy

    def has_fix(self) -> bool:
        """Check if GPS has a valid fix."""
        return self.satellites >= 4

    def get_satellite_count(self) -> int:
        """Get number of satellites in view."""
        return self.satellites

    def simulate_gps_denied(self):
        """Simulate GPS signal loss."""
        self.satellites = 0
        self.fix_type = "NO_FIX"

    def simulate_gps_recovery(self):
        """Simulate GPS signal recovery."""
        self.satellites = 8
        self.fix_type = "3D"


class MockIMU:
    """Mock IMU sensor interface."""

    def __init__(self):
        self.orientation = (0.0, 0.0, 0.0, 1.0)  # Quaternion (w, x, y, z)
        self.angular_velocity = (0.0, 0.0, 0.0)    # rad/s
        self.linear_acceleration = (0.0, 0.0, 9.81)  # m/s²
        self.temperature = 25.0  # Celsius
        self.bias_drift = 0.001  # rad/s drift

    def get_orientation(self) -> Tuple[float, float, float, float]:
        """Get orientation as quaternion."""
        return self.orientation

    def get_angular_velocity(self) -> Tuple[float, float, float]:
        """Get angular velocity (roll, pitch, yaw rates)."""
        # Add small noise and bias drift
        noise = np.random.normal(0, 0.01, 3)
        bias = np.random.normal(0, self.bias_drift, 3)
        return tuple(np.array(self.angular_velocity) + noise + bias)

    def get_linear_acceleration(self) -> Tuple[float, float, float]:
        """Get linear acceleration."""
        noise = np.random.normal(0, 0.1, 3)
        return tuple(np.array(self.linear_acceleration) + noise)

    def get_temperature(self) -> float:
        """Get IMU temperature."""
        return self.temperature + np.random.normal(0, 0.5)

    def calibrate(self):
        """Perform IMU calibration."""
        # Reset biases
        self.bias_drift = 0.0001  # Reduced after calibration


class MockCamera:
    """Mock camera interface."""

    def __init__(self, resolution: Tuple[int, int] = (640, 480)):
        self.resolution = resolution
        self.is_connected = True
        self.exposure = 100  # microseconds
        self.gain = 1.0
        self.frame_rate = 30.0

    def capture_frame(self) -> np.ndarray:
        """Capture a frame from the camera."""
        if not self.is_connected:
            raise Exception("Camera not connected")

        # Generate synthetic image with some features
        image = np.random.randint(0, 255, (*self.resolution[::-1], 3), dtype=np.uint8)

        # Add some mock features (bright spots)
        for _ in range(5):
            x, y = np.random.randint(0, self.resolution[0], 2)
            cv2.circle(image, (x, y), 10, (255, 255, 255), -1)

        return image

    def set_exposure(self, exposure: int):
        """Set camera exposure."""
        self.exposure = exposure

    def set_gain(self, gain: float):
        """Set camera gain."""
        self.gain = gain

    def get_intrinsics(self) -> Dict:
        """Get camera intrinsic parameters."""
        return {
            'fx': 600.0, 'fy': 600.0,
            'cx': self.resolution[0] / 2, 'cy': self.resolution[1] / 2,
            'k1': -0.1, 'k2': 0.01, 'p1': 0.0, 'p2': 0.0, 'k3': 0.0
        }

    def disconnect(self):
        """Simulate camera disconnection."""
        self.is_connected = False

    def reconnect(self):
        """Simulate camera reconnection."""
        self.is_connected = True


class MockLiDAR:
    """Mock LiDAR sensor interface."""

    def __init__(self, range_max: float = 10.0, fov: float = 270.0):
        self.range_max = range_max
        self.fov = fov  # degrees
        self.angle_increment = 1.0  # degrees
        self.is_scanning = True

    def get_scan(self) -> np.ndarray:
        """Get LiDAR scan data."""
        if not self.is_scanning:
            return np.array([])

        # Generate synthetic scan data
        num_points = int(self.fov / self.angle_increment)
        angles = np.linspace(-self.fov / 2, self.fov / 2, num_points)

        # Create distances with some obstacles
        distances = np.full(num_points, self.range_max)

        # Add some mock obstacles
        obstacle_indices = np.random.choice(num_points, size=5, replace=False)
        distances[obstacle_indices] = np.random.uniform(1.0, 5.0, 5)

        return distances

    def start_scan(self):
        """Start LiDAR scanning."""
        self.is_scanning = True

    def stop_scan(self):
        """Stop LiDAR scanning."""
        self.is_scanning = False


class MockEncoder:
    """Mock wheel encoder interface."""

    def __init__(self, pulses_per_revolution: int = 1000):
        self.pulses_per_revolution = pulses_per_revolution
        self.pulse_count = 0
        self.wheel_circumference = 0.5  # meters
        self.is_enabled = True

    def get_pulse_count(self) -> int:
        """Get current pulse count."""
        if not self.is_enabled:
            return self.pulse_count

        # Simulate movement (add some pulses)
        movement_pulses = np.random.randint(0, 10)
        self.pulse_count += movement_pulses
        return self.pulse_count

    def reset_count(self):
        """Reset pulse count."""
        self.pulse_count = 0

    def get_distance(self) -> float:
        """Get distance traveled based on pulse count."""
        revolutions = self.pulse_count / self.pulses_per_revolution
        return revolutions * self.wheel_circumference

    def get_velocity(self, time_delta: float = 1.0) -> float:
        """Get velocity based on pulse rate."""
        # Mock velocity calculation
        return np.random.uniform(0, 2.0)  # m/s


class MockActuator:
    """Mock actuator interface for testing."""

    def __init__(self):
        self.position = 0.0  # radians or meters
        self.velocity = 0.0
        self.effort = 0.0    # torque or force
        self.is_enabled = True
        self.temperature = 25.0

    def set_position(self, position: float):
        """Set actuator position."""
        if self.is_enabled:
            self.position = position

    def set_velocity(self, velocity: float):
        """Set actuator velocity."""
        if self.is_enabled:
            self.velocity = velocity

    def set_effort(self, effort: float):
        """Set actuator effort."""
        if self.is_enabled:
            self.effort = effort

    def get_position(self) -> float:
        """Get current position."""
        return self.position + np.random.normal(0, 0.01)

    def get_velocity(self) -> float:
        """Get current velocity."""
        return self.velocity + np.random.normal(0, 0.1)

    def get_temperature(self) -> float:
        """Get actuator temperature."""
        return self.temperature + np.random.normal(0, 2.0)

    def enable(self):
        """Enable actuator."""
        self.is_enabled = True

    def disable(self):
        """Disable actuator."""
        self.is_enabled = False


class MockBattery:
    """Mock battery interface."""

    def __init__(self, capacity: float = 100.0):
        self.capacity = capacity
        self.voltage = 24.0  # volts
        self.current = 2.0   # amps
        self.temperature = 25.0
        self.charge_level = 95.0  # percent

    def get_voltage(self) -> float:
        """Get battery voltage."""
        return self.voltage + np.random.normal(0, 0.1)

    def get_current(self) -> float:
        """Get battery current."""
        return self.current + np.random.normal(0, 0.1)

    def get_charge_level(self) -> float:
        """Get battery charge level (0-100%)."""
        return max(0, min(100, self.charge_level + np.random.normal(0, 1.0)))

    def get_temperature(self) -> float:
        """Get battery temperature."""
        return self.temperature + np.random.normal(0, 1.0)

    def is_critical(self) -> bool:
        """Check if battery is at critical level."""
        return self.get_charge_level() < 10.0

    def simulate_discharge(self, rate: float = 1.0):
        """Simulate battery discharge."""
        self.charge_level = max(0, self.charge_level - rate)


class SensorFusionMock:
    """Mock sensor fusion system."""

    def __init__(self):
        self.gps = MockGPS()
        self.imu = MockIMU()
        self.camera = MockCamera()
        self.lidar = MockLiDAR()
        self.encoders = [MockEncoder() for _ in range(4)]  # 4 wheels
        self.battery = MockBattery()

    def get_robot_pose(self) -> Dict:
        """Get fused robot pose estimate."""
        gps_pos = self.gps.get_position()

        return {
            'position': (gps_pos[0], gps_pos[1], gps_pos[2]),
            'orientation': self.imu.get_orientation(),
            'accuracy': self.gps.get_accuracy(),
            'timestamp': time.time()
        }

    def get_obstacle_data(self) -> List[Dict]:
        """Get obstacle detection data."""
        scan = self.lidar.get_scan()

        obstacles = []
        angles = np.linspace(-self.lidar.fov / 2, self.lidar.fov / 2, len(scan))

        for i, distance in enumerate(scan):
            if distance < self.lidar.range_max:
                angle = angles[i]
                x = distance * np.cos(np.radians(angle))
                y = distance * np.sin(np.radians(angle))

                obstacles.append({
                    'position': (x, y, 0.0),
                    'distance': distance,
                    'angle': angle,
                    'type': 'lidar_obstacle'
                })

        return obstacles

    def get_system_health(self) -> Dict:
        """Get overall system health status."""
        return {
            'gps': {
                'healthy': self.gps.has_fix(),
                'satellites': self.gps.get_satellite_count(),
                'accuracy': self.gps.get_accuracy()
            },
            'imu': {
                'healthy': True,  # IMU usually reliable
                'temperature': self.imu.get_temperature()
            },
            'camera': {
                'healthy': self.camera.is_connected,
                'resolution': self.camera.resolution
            },
            'lidar': {
                'healthy': self.lidar.is_scanning,
                'range': self.lidar.range_max
            },
            'battery': {
                'healthy': not self.battery.is_critical(),
                'charge_level': self.battery.get_charge_level(),
                'voltage': self.battery.get_voltage()
            }
        }


# Convenience function to create complete mock sensor suite
def create_mock_sensor_suite() -> SensorFusionMock:
    """Create a complete mock sensor suite for testing."""
    return SensorFusionMock()


# Test data generators
def generate_test_trajectory(length: int = 100) -> List[Tuple[float, float, float]]:
    """Generate a test trajectory for navigation testing."""
    trajectory = []
    for i in range(length):
        x = i * 0.1  # 10cm steps
        y = np.sin(i * 0.1) * 2.0  # Sine wave pattern
        z = 0.0
        trajectory.append((x, y, z))
    return trajectory


def generate_test_waypoints(count: int = 5) -> List[Dict]:
    """Generate test mission waypoints."""
    waypoints = []
    for i in range(count):
        waypoints.append({
            'id': i + 1,
            'position': (i * 10.0, (i % 2) * 5.0, 0.0),  # Alternating pattern
            'type': 'gnss_waypoint',
            'accuracy_required': 3.0
        })
    return waypoints
