#!/usr/bin/env python3
"""
Teleoperation Data Processor
Handles validation, filtering, and processing of teleoperation sensor data
"""

from typing import Any, Dict, List, Optional

import rclpy
import structlog
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Float32MultiArray

from .monitoring_system import monitor_data_quality

# Configure structured logging
logger = structlog.get_logger(__name__)


class TeleoperationDataProcessor:
    """
    Processes and validates teleoperation sensor data from control systems.
    Handles data quality assurance, filtering, and state management.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config

        # Data storage with history for filtering
        self.latest_motor_data: Optional[Dict] = None
        self.actual_chassis_velocity: Optional[Dict] = None
        self.motor_temperatures: List[float] = []
        self.system_status: Optional[Dict] = None

        # Filtering history
        self.temperature_history: List[List[float]] = []
        self.velocity_history: List[List[float]] = []
        self.previous_positions: Optional[List[float]] = None

        # Data quality tracking
        self.data_quality_metrics: Dict[str, Dict] = {}

        # Configuration thresholds
        self.teleop_config = config.get("teleoperation", {})

    def process_joint_state_data(self, msg: JointState) -> bool:
        """
        Process joint state data from teleoperation monitoring.
        Returns True if data is valid and processed successfully.
        """
        if not self._validate_joint_state_data(msg):
            return False

        # Apply filtering
        filtered_positions = self._filter_position_data(msg.position)
        filtered_velocities = self._filter_velocity_data(msg.velocity)

        # Store processed data
        self.latest_motor_data = {
            "positions": filtered_positions,
            "velocities": filtered_velocities,
            "names": msg.name,
            "timestamp": msg.header.stamp,
        }

        # Update data quality metrics
        self._update_data_quality_metrics("joint_states", True)

        return True

    def process_chassis_velocity_data(self, msg: TwistStamped) -> bool:
        """
        Process chassis velocity data from teleoperation monitoring.
        Returns True if data is valid and processed successfully.
        """
        if not self._validate_chassis_velocity_data(msg):
            return False

        # Store processed data
        self.actual_chassis_velocity = {
            "linear_x": msg.twist.linear.x,
            "linear_y": msg.twist.linear.y,
            "angular_z": msg.twist.angular.z,
            "timestamp": msg.header.stamp,
        }

        # Update data quality metrics
        self._update_data_quality_metrics("chassis_velocity", True)

        return True

    def process_motor_temperature_data(self, msg: Float32MultiArray) -> bool:
        """
        Process motor temperature data from teleoperation monitoring.
        Returns True if data is valid and processed successfully.
        """
        if not self._validate_motor_temperature_data(msg):
            return False

        # Apply filtering
        filtered_temperatures = self._filter_temperature_data(msg.data)

        # Store processed data
        self.motor_temperatures = filtered_temperatures

        # Update data quality metrics
        self._update_data_quality_metrics("motor_temperatures", True)

        return True

    def process_system_status_data(self, msg: BatteryState) -> bool:
        """
        Process system status data from teleoperation monitoring.
        Returns True if data is valid and processed successfully.
        """
        if not self._validate_system_status_data(msg):
            return False

        # Store processed data
        self.system_status = {
            "battery_voltage": msg.voltage,
            "battery_percentage": msg.percentage,
            "battery_current": msg.current,
            "timestamp": msg.header.stamp,
        }

        # Update data quality metrics
        self._update_data_quality_metrics("system_status", True)

        return True

    def _validate_joint_state_data(self, msg: JointState) -> bool:
        """Validate incoming joint state data"""
        try:
            # Check timestamp freshness
            if not self._is_timestamp_fresh(msg.header.stamp):
                return False

            # Check structural integrity
            if not msg.name or not msg.position or not msg.velocity:
                return False

            # Check data consistency
            if len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity):
                return False

            # Check value ranges
            motor_limits = self.teleop_config.get("motor_limits", {})
            max_velocity = motor_limits.get("max_velocity", 15.0)
            position_tolerance = motor_limits.get("position_tolerance", 10.0)

            for pos, vel in zip(msg.position, msg.velocity):
                if abs(pos) > position_tolerance or abs(vel) > max_velocity:
                    return False

            return True

        except Exception:
            return False

    def _validate_chassis_velocity_data(self, msg: TwistStamped) -> bool:
        """Validate incoming chassis velocity data"""
        try:
            if not self._is_timestamp_fresh(msg.header.stamp):
                return False

            # Check for NaN or extreme values
            linear = msg.twist.linear
            angular = msg.twist.angular

            for value in [linear.x, linear.y, angular.z]:
                if not (value == value) or abs(value) > 1000:  # NaN check
                    return False

            return True

        except Exception:
            return False

    def _validate_motor_temperature_data(self, msg: Float32MultiArray) -> bool:
        """Validate incoming motor temperature data"""
        try:
            if not msg.data:
                return False

            thermal_limits = self.teleop_config.get("thermal_limits", {})
            emergency_temp = thermal_limits.get("emergency_temp", 100.0)

            for temp in msg.data:
                if not (temp == temp) or temp < -50 or temp > emergency_temp:
                    return False

            return True

        except Exception:
            return False

    def _validate_system_status_data(self, msg: BatteryState) -> bool:
        """Validate incoming system status data"""
        try:
            # Reasonable voltage range (12V-30V typical for robotics)
            if msg.voltage < 10.0 or msg.voltage > 35.0:
                return False

            # Percentage range
            if msg.percentage < 0.0 or msg.percentage > 100.0:
                return False

            return True

        except Exception:
            return False

    def _is_timestamp_fresh(self, stamp) -> bool:
        """Check if message timestamp is reasonably fresh"""
        try:
            current_time = rclpy.time.Time()
            msg_time = rclpy.time.Time.from_msg(stamp)
            max_age = rclpy.duration.Duration(seconds=2.0)
            age = current_time - msg_time
            return age < max_age
        except Exception:
            return False

    def _filter_position_data(self, positions: List[float]) -> List[float]:
        """Apply basic filtering to position data"""
        if not positions:
            return positions

        # Simple outlier detection based on previous readings
        if hasattr(self, "previous_positions") and self.previous_positions:
            filtered_positions = []
            for i, (curr_pos, prev_pos) in enumerate(
                zip(positions, self.previous_positions)
            ):
                diff = abs(curr_pos - prev_pos)
                # If position changed too much, it might be noise
                if diff > 1.0:  # 1 radian threshold
                    filtered_positions.append(prev_pos)  # Use previous value
                else:
                    filtered_positions.append(curr_pos)
            return filtered_positions

        return positions

    def _filter_velocity_data(self, velocities: List[float]) -> List[float]:
        """Apply basic filtering to velocity data"""
        if not velocities:
            return velocities

        # Simple moving average filter
        if not self.velocity_history:
            self.velocity_history = [velocities]
            return velocities

        # Keep last 3 readings for averaging
        self.velocity_history.append(velocities)
        if len(self.velocity_history) > 3:
            self.velocity_history.pop(0)

        # Compute moving average
        filtered_velocities = []
        for i in range(len(velocities)):
            avg_velocity = sum(reading[i] for reading in self.velocity_history) / len(
                self.velocity_history
            )
            filtered_velocities.append(avg_velocity)

        return filtered_velocities

    def _filter_temperature_data(self, temperatures: List[float]) -> List[float]:
        """Apply filtering to temperature data"""
        if not temperatures:
            return temperatures

        # Add to history for smoothing
        self.temperature_history.append(temperatures)

        # Keep last 5 readings
        if len(self.temperature_history) > 5:
            self.temperature_history.pop(0)

        # Simple moving average
        if len(self.temperature_history) > 1:
            filtered_temps = []
            for i in range(len(temperatures)):
                avg_temp = sum(
                    reading[i] for reading in self.temperature_history
                ) / len(self.temperature_history)
                filtered_temps.append(avg_temp)
            return filtered_temps

        return temperatures

    def _update_data_quality_metrics(self, data_type: str, is_valid: bool):
        """Track data quality metrics"""
        if data_type not in self.data_quality_metrics:
            self.data_quality_metrics[data_type] = {
                "total_received": 0,
                "valid_count": 0,
                "last_valid_time": None,
                "consecutive_failures": 0,
            }

        metrics = self.data_quality_metrics[data_type]
        metrics["total_received"] += 1

        if is_valid:
            metrics["valid_count"] += 1
            metrics["last_valid_time"] = rclpy.time.Time()
            metrics["consecutive_failures"] = 0
        else:
            metrics["consecutive_failures"] += 1

            # Monitor data quality issues (non-invasive)
            if metrics["consecutive_failures"] >= 5:
                monitor_data_quality(
                    "teleoperation_data_processor",
                    {
                        "data_type": data_type,
                        "consecutive_failures": metrics["consecutive_failures"],
                        "total_received": metrics["total_received"],
                        "valid_percentage": (metrics["valid_count"] / metrics["total_received"]) * 100
                    }
                )

    def get_data_quality_report(self) -> Dict[str, Dict]:
        """Get data quality metrics for monitoring"""
        return self.data_quality_metrics.copy()

    def reset_filters(self):
        """Reset all filtering history"""
        self.temperature_history.clear()
        self.velocity_history.clear()
        self.previous_positions = None
