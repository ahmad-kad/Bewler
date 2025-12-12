"""
Redundant Safety Monitor.

Independent safety monitoring system that validates the primary safety system.
Provides redundant safety state machine and cross-checks safety decisions.
"""

import time
import json
from enum import Enum
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from sensor_msgs.msg import Imu, BatteryState, Temperature
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from autonomy_interfaces.msg import SafetyStatus


class RedundantSafetyState(Enum):
    """Redundant safety state machine states."""

    NORMAL = "NORMAL"  # All systems normal
    MONITORING = "MONITORING"  # Active monitoring, no violations
    WARNING = "WARNING"  # Warning conditions detected
    CRITICAL = "CRITICAL"  # Critical conditions, prepare for intervention
    EMERGENCY = "EMERGENCY"  # Emergency halt required
    RECOVERY = "RECOVERY"  # Recovery in progress


class SafetyConsistency(Enum):
    """Consistency check results between primary and redundant systems."""

    CONSISTENT = "CONSISTENT"  # Systems agree
    MINOR_DISCREPANCY = "MINOR_DISCREPANCY"  # Minor differences, acceptable
    MAJOR_DISCREPANCY = "MAJOR_DISCREPANCY"  # Major differences, investigate
    SYSTEM_FAILURE = "SYSTEM_FAILURE"  # One system has failed


@dataclass
class SafetyThresholds:
    """Safety thresholds for redundant monitoring."""

    battery_critical: float = 10.0  # Battery percentage
    battery_warning: float = 20.0  # Battery percentage
    temperature_critical: float = 85.0  # Celsius
    temperature_warning: float = 70.0  # Celsius
    imu_accel_max: float = 50.0  # m/s² (unreasonable acceleration)
    imu_gyro_max: float = 20.0  # rad/s (unreasonable rotation)
    position_uncertainty_max: float = 10.0  # meters (GPS uncertainty)
    velocity_max: float = 5.0  # m/s (safe velocity limit)
    communication_timeout: float = 5.0  # seconds


@dataclass
class SensorHealth:
    """Sensor health tracking."""

    last_update: float = 0.0
    update_count: int = 0
    error_count: int = 0
    consecutive_failures: int = 0
    data_quality_score: float = 1.0  # 0.0 to 1.0


class RedundantSafetyMonitor(Node):
    """
    Redundant Safety Monitor.

    Provides independent safety monitoring and validation of the primary safety system.
    Implements redundant safety logic and cross-checks safety decisions.
    """

    def __init__(self, thresholds: Optional[SafetyThresholds] = None):
        super().__init__("redundant_safety_monitor")

        self.thresholds = thresholds or SafetyThresholds()
        self.logger = self.get_logger()

        # Safety state tracking
        self.current_state = RedundantSafetyState.NORMAL
        self.primary_safety_status: Optional[SafetyStatus] = None
        self.last_primary_update = 0.0
        self.consistency_history: deque = deque(maxlen=100)

        # Sensor health tracking
        self.sensor_health: Dict[str, SensorHealth] = {
            "imu": SensorHealth(),
            "gps": SensorHealth(),
            "battery": SensorHealth(),
            "temperature": SensorHealth(),
            "odometry": SensorHealth(),
        }

        # Data validation buffers (sliding windows)
        self.imu_accel_buffer: deque = deque(maxlen=50)
        self.imu_gyro_buffer: deque = deque(maxlen=50)
        self.battery_level_buffer: deque = deque(maxlen=20)
        self.temperature_buffer: deque = deque(maxlen=20)

        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10
        )

        self.qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=20
        )

        # Callback groups
        self.monitoring_group = MutuallyExclusiveCallbackGroup()
        self.validation_group = MutuallyExclusiveCallbackGroup()

        # Setup publishers and subscribers
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_timers()

        self.logger.info("Redundant Safety Monitor initialized")

    def _setup_publishers(self):
        """Setup ROS2 publishers for redundant safety monitoring."""
        self.redundant_status_pub = self.create_publisher(
            SafetyStatus, "/safety/redundant_status", 10, callback_group=self.monitoring_group
        )

        self.consistency_check_pub = self.create_publisher(
            String, "/safety/consistency_check", 10, callback_group=self.monitoring_group
        )

        self.sensor_health_pub = self.create_publisher(
            String, "/safety/sensor_health", 10, callback_group=self.monitoring_group
        )

        self.redundant_diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/safety/redundant_diagnostics", 10, callback_group=self.monitoring_group
        )

    def _setup_subscribers(self):
        """Setup ROS2 subscribers for independent monitoring."""
        # Primary safety system monitoring
        self.create_subscription(
            SafetyStatus,
            "/state_machine/safety_status",
            self._primary_safety_callback,
            10,
            callback_group=self.monitoring_group,
        )

        # Independent sensor monitoring
        self.create_subscription(
            Imu,
            "/imu/data",
            self._imu_safety_check,
            10,
            qos_profile=self.qos_best_effort,
            callback_group=self.validation_group,
        )

        self.create_subscription(
            BatteryState,
            "/battery/status",
            self._battery_safety_check,
            10,
            qos_profile=self.qos_best_effort,
            callback_group=self.validation_group,
        )

        self.create_subscription(
            Temperature,
            "/temperature/data",
            self._temperature_safety_check,
            10,
            qos_profile=self.qos_best_effort,
            callback_group=self.validation_group,
        )

        self.create_subscription(
            Odometry,
            "/wheel/odom",
            self._odometry_safety_check,
            10,
            qos_profile=self.qos_best_effort,
            callback_group=self.validation_group,
        )

    def _setup_timers(self):
        """Setup monitoring and validation timers."""
        # Consistency checking timer
        self.consistency_timer = self.create_timer(
            2.0, self._perform_consistency_check, callback_group=self.monitoring_group
        )

        # Sensor health monitoring timer
        self.health_timer = self.create_timer(5.0, self._publish_sensor_health, callback_group=self.monitoring_group)

        # Redundant safety evaluation timer
        self.safety_timer = self.create_timer(
            1.0, self._evaluate_redundant_safety, callback_group=self.validation_group
        )

        # Diagnostics publishing timer
        self.diagnostics_timer = self.create_timer(
            10.0, self._publish_redundant_diagnostics, callback_group=self.monitoring_group
        )

    def _primary_safety_callback(self, msg: SafetyStatus):
        """Monitor primary safety system status."""
        self.primary_safety_status = msg
        self.last_primary_update = time.time()

        # Update redundant state based on primary status
        self._update_redundant_state_from_primary()

    def _imu_safety_check(self, msg: Imu):
        """Independent IMU safety validation."""
        current_time = time.time()

        # Update sensor health
        self._update_sensor_health("imu", current_time, True)

        # Extract acceleration magnitude
        accel_magnitude = (
            msg.linear_acceleration.x**2 + msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2
        ) ** 0.5

        # Extract gyro magnitude
        gyro_magnitude = (msg.angular_velocity.x**2 + msg.angular_velocity.y**2 + msg.angular_velocity.z**2) ** 0.5

        # Add to validation buffers
        self.imu_accel_buffer.append(accel_magnitude)
        self.imu_gyro_buffer.append(gyro_magnitude)

        # Check for safety violations
        if accel_magnitude > self.thresholds.imu_accel_max:
            self._trigger_redundant_violation(
                "imu_acceleration_spike",
                f"IMU acceleration spike: {accel_magnitude:.2f} m/s²",
                {"accel_magnitude": accel_magnitude, "threshold": self.thresholds.imu_accel_max},
            )

        if gyro_magnitude > self.thresholds.imu_gyro_max:
            self._trigger_redundant_violation(
                "imu_rotation_spike",
                f"IMU rotation spike: {gyro_magnitude:.2f} rad/s",
                {"gyro_magnitude": gyro_magnitude, "threshold": self.thresholds.imu_gyro_max},
            )

        # Check for sensor failure (unrealistic constant values)
        if len(self.imu_accel_buffer) >= 10:
            recent_accel = list(self.imu_accel_buffer)[-10:]
            if max(recent_accel) - min(recent_accel) < 0.01:  # Very little variation
                self._trigger_redundant_violation(
                    "imu_sensor_stuck",
                    "IMU sensor appears stuck (no variation in acceleration)",
                    {"accel_range": max(recent_accel) - min(recent_accel)},
                )

    def _battery_safety_check(self, msg: BatteryState):
        """Independent battery safety validation."""
        current_time = time.time()

        # Update sensor health
        self._update_sensor_health("battery", current_time, True)

        # Add to validation buffer
        self.battery_level_buffer.append(msg.percentage)

        # Check safety thresholds
        if msg.percentage <= self.thresholds.battery_critical:
            self._trigger_redundant_violation(
                "battery_critical_redundant",
                f"Battery critically low (redundant check): {msg.percentage:.1f}%",
                {"battery_level": msg.percentage, "threshold": self.thresholds.battery_critical},
            )
        elif msg.percentage <= self.thresholds.battery_warning:
            if self.current_state == RedundantSafetyState.NORMAL:
                self._update_redundant_state(RedundantSafetyState.WARNING, f"Battery warning: {msg.percentage:.1f}%")

        # Check for unrealistic battery changes
        if len(self.battery_level_buffer) >= 5:
            recent_levels = list(self.battery_level_buffer)[-5:]
            max_change = max(recent_levels) - min(recent_levels)
            if max_change > 50.0:  # 50% change in short period is suspicious
                self._trigger_redundant_violation(
                    "battery_level_anomaly",
                    f"Unrealistic battery level change: {max_change:.1f}%",
                    {"max_change": max_change, "recent_levels": recent_levels},
                )

    def _temperature_safety_check(self, msg: Temperature):
        """Independent temperature safety validation."""
        current_time = time.time()

        # Update sensor health
        self._update_sensor_health("temperature", current_time, True)

        # Add to validation buffer
        self.temperature_buffer.append(msg.temperature)

        # Check safety thresholds
        if msg.temperature >= self.thresholds.temperature_critical:
            self._trigger_redundant_violation(
                "temperature_critical_redundant",
                f"Temperature critically high (redundant check): {msg.temperature:.1f}°C",
                {"temperature": msg.temperature, "threshold": self.thresholds.temperature_critical},
            )
        elif msg.temperature >= self.thresholds.temperature_warning:
            if self.current_state == RedundantSafetyState.NORMAL:
                self._update_redundant_state(
                    RedundantSafetyState.WARNING, f"Temperature warning: {msg.temperature:.1f}°C"
                )

    def _odometry_safety_check(self, msg: Odometry):
        """Independent odometry safety validation."""
        current_time = time.time()

        # Update sensor health
        self._update_sensor_health("odometry", current_time, True)

        # Extract velocity magnitude
        velocity_magnitude = (
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2 + msg.twist.twist.linear.z**2
        ) ** 0.5

        # Check velocity limits
        if velocity_magnitude > self.thresholds.velocity_max:
            self._trigger_redundant_violation(
                "velocity_limit_exceeded",
                f"Velocity limit exceeded: {velocity_magnitude:.2f} m/s",
                {"velocity": velocity_magnitude, "threshold": self.thresholds.velocity_max},
            )

    def _update_sensor_health(self, sensor_name: str, timestamp: float, success: bool):
        """Update sensor health tracking."""
        health = self.sensor_health[sensor_name]

        if success:
            health.last_update = timestamp
            health.update_count += 1
            health.consecutive_failures = 0

            # Improve data quality score
            health.data_quality_score = min(1.0, health.data_quality_score + 0.01)
        else:
            health.error_count += 1
            health.consecutive_failures += 1

            # Degrade data quality score
            health.data_quality_score = max(0.0, health.data_quality_score - 0.1)

    def _trigger_redundant_violation(self, violation_id: str, description: str, context: Dict[str, Any]):
        """Trigger a redundant safety violation."""
        self.logger.warning(f"Redundant safety violation: {violation_id} - {description}")

        # Update redundant state
        if "critical" in violation_id.lower() or "emergency" in violation_id.lower():
            self._update_redundant_state(RedundantSafetyState.CRITICAL, description)
        else:
            self._update_redundant_state(RedundantSafetyState.WARNING, description)

        # Publish violation status
        violation_msg = SafetyStatus()
        violation_msg.header.stamp = self.get_clock().now().to_msg()
        violation_msg.header.frame_id = "redundant_safety_monitor"
        violation_msg.is_safe = self.current_state not in [
            RedundantSafetyState.EMERGENCY,
            RedundantSafetyState.CRITICAL,
        ]
        violation_msg.safety_level = self.current_state.value
        violation_msg.active_triggers = [violation_id]
        violation_msg.trigger_type = violation_id
        violation_msg.trigger_source = "redundant_safety_monitor"
        violation_msg.trigger_time = self.get_clock().now().to_msg()
        violation_msg.trigger_description = description
        violation_msg.requires_manual_intervention = self.current_state == RedundantSafetyState.EMERGENCY
        violation_msg.can_auto_recover = self.current_state != RedundantSafetyState.EMERGENCY
        violation_msg.recovery_steps = ["Investigate redundant safety violation", "Verify primary safety system"]
        violation_msg.estimated_recovery_time = 60.0  # 1 minute
        violation_msg.context_state = "redundant_monitoring"
        violation_msg.mission_phase = "unknown"

        self.redundant_status_pub.publish(violation_msg)

    def _update_redundant_state(self, new_state: RedundantSafetyState, reason: str):
        """Update redundant safety state."""
        if new_state != self.current_state:
            self.logger.info(
                f"Redundant safety state changed: {self.current_state.value} -> {new_state.value} ({reason})"
            )
            self.current_state = new_state

    def _update_redundant_state_from_primary(self):
        """Update redundant state based on primary safety status."""
        if not self.primary_safety_status:
            return

        primary_level = self.primary_safety_status.safety_level

        # Map primary safety levels to redundant states
        if primary_level == "EMERGENCY":
            if self.current_state != RedundantSafetyState.EMERGENCY:
                self._update_redundant_state(RedundantSafetyState.EMERGENCY, "Primary system emergency detected")
        elif primary_level == "CRITICAL":
            if self.current_state not in [RedundantSafetyState.EMERGENCY, RedundantSafetyState.CRITICAL]:
                self._update_redundant_state(
                    RedundantSafetyState.CRITICAL, "Primary system critical condition detected"
                )

    def _perform_consistency_check(self):
        """Perform consistency check between primary and redundant safety systems."""
        if not self.primary_safety_status:
            consistency = SafetyConsistency.SYSTEM_FAILURE
            description = "Primary safety system not reporting"
        else:
            time_since_primary = time.time() - self.last_primary_update

            if time_since_primary > self.thresholds.communication_timeout:
                consistency = SafetyConsistency.SYSTEM_FAILURE
                description = f"Primary safety system timeout: {time_since_primary:.1f}s"
            else:
                # Compare safety levels
                primary_level = self.primary_safety_status.safety_level
                redundant_level = self.current_state.value

                if primary_level == redundant_level:
                    consistency = SafetyConsistency.CONSISTENT
                    description = "Safety systems consistent"
                elif primary_level in ["NORMAL", "WARNING"] and redundant_level in ["NORMAL", "WARNING", "MONITORING"]:
                    consistency = SafetyConsistency.MINOR_DISCREPANCY
                    description = f"Minor discrepancy: Primary={primary_level}, Redundant={redundant_level}"
                else:
                    consistency = SafetyConsistency.MAJOR_DISCREPANCY
                    description = f"Major discrepancy: Primary={primary_level}, Redundant={redundant_level}"

        # Record consistency result
        consistency_result = {
            "timestamp": time.time(),
            "consistency": consistency.value,
            "description": description,
            "primary_level": self.primary_safety_status.safety_level if self.primary_safety_status else "unknown",
            "redundant_level": self.current_state.value,
        }

        self.consistency_history.append(consistency_result)

        # Publish consistency check
        consistency_msg = String()
        consistency_msg.data = json.dumps(consistency_result, indent=2)
        self.consistency_check_pub.publish(consistency_msg)

        # Log issues
        if consistency in [SafetyConsistency.MAJOR_DISCREPANCY, SafetyConsistency.SYSTEM_FAILURE]:
            self.logger.error(f"Safety system consistency issue: {description}")

    def _evaluate_redundant_safety(self):
        """Evaluate overall redundant safety status."""
        # Check sensor health
        unhealthy_sensors = [
            sensor
            for sensor, health in self.sensor_health.items()
            if health.data_quality_score < 0.5 or health.consecutive_failures > 5
        ]

        if unhealthy_sensors:
            if self.current_state == RedundantSafetyState.NORMAL:
                self._update_redundant_state(RedundantSafetyState.WARNING, f"Unhealthy sensors: {unhealthy_sensors}")

        # Check for overall system degradation
        active_violations = len(
            [r for r in self.consistency_history if r["consistency"] != SafetyConsistency.CONSISTENT]
        )

        if active_violations > 10:  # Many recent inconsistencies
            recent_consistency = list(self.consistency_history)[-10:]
            major_issues = sum(
                1
                for r in recent_consistency
                if r["consistency"]
                in [SafetyConsistency.MAJOR_DISCREPANCY.value, SafetyConsistency.SYSTEM_FAILURE.value]
            )

            if major_issues > 5 and self.current_state != RedundantSafetyState.CRITICAL:
                self._update_redundant_state(
                    RedundantSafetyState.CRITICAL, f"Multiple safety system inconsistencies: {major_issues}/10"
                )

    def _publish_sensor_health(self):
        """Publish sensor health status."""
        health_data = {"timestamp": time.time(), "sensors": {}}

        for sensor_name, health in self.sensor_health.items():
            health_data["sensors"][sensor_name] = {
                "last_update": health.last_update,
                "age": time.time() - health.last_update,
                "update_count": health.update_count,
                "error_count": health.error_count,
                "consecutive_failures": health.consecutive_failures,
                "data_quality_score": health.data_quality_score,
                "healthy": health.data_quality_score > 0.7 and health.consecutive_failures == 0,
            }

        health_msg = String()
        health_msg.data = json.dumps(health_data, indent=2)
        self.sensor_health_pub.publish(health_msg)

    def _publish_redundant_diagnostics(self):
        """Publish redundant safety diagnostics."""
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.header.frame_id = "redundant_safety_monitor"

        # Redundant safety diagnostic
        safety_status = DiagnosticStatus()
        safety_status.name = "Redundant Safety Monitor"
        safety_status.hardware_id = "redundant_safety_v1.0"

        if self.current_state == RedundantSafetyState.NORMAL:
            safety_status.level = DiagnosticStatus.OK
            safety_status.message = "Redundant safety monitoring normal"
        elif self.current_state == RedundantSafetyState.WARNING:
            safety_status.level = DiagnosticStatus.WARN
            safety_status.message = "Redundant safety monitoring warning conditions"
        elif self.current_state in [RedundantSafetyState.CRITICAL, RedundantSafetyState.EMERGENCY]:
            safety_status.level = DiagnosticStatus.ERROR
            safety_status.message = f"Redundant safety monitoring: {self.current_state.value}"
        else:
            safety_status.level = DiagnosticStatus.OK
            safety_status.message = f"Redundant safety monitoring: {self.current_state.value}"

        # Add sensor health information
        for sensor_name, health in self.sensor_health.items():
            safety_status.values.append(
                {
                    "key": f"{sensor_name}_health",
                    "value": f"score={health.data_quality_score:.2f}, failures={health.consecutive_failures}",
                }
            )

        diagnostics.status = [safety_status]
        self.redundant_diagnostics_pub.publish(diagnostics)

    def get_consistency_history(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Get recent consistency check history."""
        return list(self.consistency_history)[-limit:]

    def get_sensor_health_status(self) -> Dict[str, Dict[str, Any]]:
        """Get comprehensive sensor health status."""
        return {
            sensor_name: {
                "last_update": health.last_update,
                "age": time.time() - health.last_update,
                "update_count": health.update_count,
                "error_count": health.error_count,
                "consecutive_failures": health.consecutive_failures,
                "data_quality_score": health.data_quality_score,
                "healthy": health.data_quality_score > 0.7 and health.consecutive_failures == 0,
            }
            for sensor_name, health in self.sensor_health.items()
        }


def main(args=None):
    """Main entry point for redundant safety monitor node."""
    rclpy.init(args=args)

    # Create redundant safety monitor
    monitor = RedundantSafetyMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.logger.info("Redundant Safety Monitor shutting down")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
