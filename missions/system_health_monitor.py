#!/usr/bin/env python3
"""
System Health Monitor
Monitors thermal, battery, and motor health conditions
"""

import uuid
from typing import Any, Callable, Dict, List, Optional

import structlog

# Configure structured logging
logger = structlog.get_logger(__name__)


class SystemHealthMonitor:
    """
    Monitors system health including thermal management, battery monitoring,
    and motor health assessment.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.teleop_config = config.get("teleoperation", {})

        # Health state tracking
        self.thermal_speed_factor = 1.0
        self.battery_speed_factor = 1.0
        self.emergency_mode = False

        # Health thresholds from config
        self.thermal_limits = self.teleop_config.get("thermal_limits", {})
        self.battery_limits = self.teleop_config.get("battery_limits", {})
        self.motor_limits = self.teleop_config.get("motor_limits", {})

        # Mission adaptation settings
        self.mission_adaptation = self.teleop_config.get("mission_adaptation", {})

        # Callbacks for emergency responses
        self.emergency_callbacks: Dict[str, Callable] = {}

    def register_emergency_callback(self, emergency_type: str, callback: Callable):
        """Register callback for emergency situations"""
        self.emergency_callbacks[emergency_type] = callback

    def monitor_motor_health(self, motor_data: Optional[Dict]) -> bool:
        """
        Monitor motor health using joint state data.
        Returns True if motors are healthy, False if issues detected.
        """
        if not motor_data:
            return True  # No data to check

        positions = motor_data.get("positions", [])
        velocities = motor_data.get("velocities", [])

        # Check for motor stalls (high velocity commanded but low actual)
        # This would require comparing commanded vs actual velocities
        # For now, just basic health checks

        # Check for position discontinuities (potential encoder issues)
        if hasattr(self, "previous_positions") and self.previous_positions:
            for i, (prev_pos, curr_pos) in enumerate(
                zip(self.previous_positions, positions)
            ):
                diff = abs(curr_pos - prev_pos)
                if diff > 1.0:  # Large position jump
                    correlation_id = str(uuid.uuid4())
                    logger.warning(
                        "Position discontinuity detected",
                        motor_index=i,
                        position_difference=diff,
                        correlation_id=correlation_id,
                    )
                    return False

        self.previous_positions = positions.copy()
        return True

    def monitor_thermal_limits(self, temperatures: List[float]) -> Dict[str, Any]:
        """
        Monitor motor temperatures and determine thermal status.
        Returns thermal status and recommended actions.
        """
        if not temperatures:
            return {"status": "unknown", "action": "none"}

        max_temp = max(temperatures)
        avg_temp = sum(temperatures) / len(temperatures)

        # Get thresholds from config
        critical_temp = self.thermal_limits.get("critical_temp", 70.0)
        warning_temp = self.thermal_limits.get("warning_temp", 55.0)
        emergency_temp = self.thermal_limits.get("emergency_temp", 80.0)

        # Determine thermal status
        if max_temp >= emergency_temp:
            status = "emergency"
            action = "shutdown"
            self._trigger_emergency("thermal_overload")
        elif max_temp >= critical_temp:
            status = "critical"
            action = "reduce_speed"
            speed_factor = self.mission_adaptation.get("thermal_throttle_factor", 0.3)
            self.thermal_speed_factor = speed_factor
        elif max_temp >= warning_temp:
            status = "warning"
            action = "monitor"
            speed_factor = 0.8
            self.thermal_speed_factor = speed_factor
        else:
            status = "normal"
            action = "none"
            self.thermal_speed_factor = 1.0

        return {
            "status": status,
            "action": action,
            "max_temp": max_temp,
            "avg_temp": avg_temp,
            "speed_factor": self.thermal_speed_factor,
        }

    def monitor_system_health(self, system_status: Optional[Dict]) -> Dict[str, Any]:
        """
        Monitor overall system health including battery.
        Returns system health status and recommended actions.
        """
        if not system_status:
            return {"status": "unknown", "action": "none"}

        battery_pct = system_status.get("battery_percentage", 100.0)

        # Get battery thresholds from config
        low_threshold = self.battery_limits.get("low_threshold", 20.0)
        critical_threshold = self.battery_limits.get("critical_threshold", 10.0)

        # Determine battery status
        if battery_pct <= critical_threshold:
            status = "critical"
            action = "return_to_base"
            self._trigger_emergency("battery_critical")
        elif battery_pct <= low_threshold:
            status = "low"
            action = "conserve_energy"
            speed_factor = self.mission_adaptation.get(
                "battery_conservation_factor", 0.8
            )
            self.battery_speed_factor = speed_factor
        else:
            status = "normal"
            action = "none"
            self.battery_speed_factor = 1.0

        return {
            "status": status,
            "action": action,
            "battery_percentage": battery_pct,
            "speed_factor": self.battery_speed_factor,
        }

    def get_overall_health_status(self) -> Dict[str, Any]:
        """
        Get comprehensive health status across all monitored systems.
        """
        # This would aggregate health from all monitored systems
        return {
            "thermal_factor": self.thermal_speed_factor,
            "battery_factor": self.battery_speed_factor,
            "effective_speed_factor": min(
                self.thermal_speed_factor, self.battery_speed_factor
            ),
            "emergency_mode": self.emergency_mode,
            "overall_status": "critical" if self.emergency_mode else "normal",
        }

    def _adjust_speed_for_thermal_limits(self, max_temp: float) -> float:
        """Calculate speed adjustment factor for thermal conditions"""
        critical_temp = self.thermal_limits.get("critical_temp", 70.0)
        warning_temp = self.thermal_limits.get("warning_temp", 55.0)

        if max_temp > critical_temp:
            return self.mission_adaptation.get("thermal_throttle_factor", 0.3)
        elif max_temp > warning_temp + 10:
            return self.mission_adaptation.get("thermal_throttle_factor", 0.7)
        elif max_temp > warning_temp:
            return 0.8
        else:
            return 1.0

    def _adjust_mission_for_battery_level(self, battery_pct: float) -> Dict[str, Any]:
        """Determine mission adjustments for battery conservation"""
        low_threshold = self.battery_limits.get("low_threshold", 20.0)
        critical_threshold = self.battery_limits.get("critical_threshold", 10.0)

        if battery_pct < critical_threshold:
            return {
                "action": "emergency_return",
                "speed_factor": 1.0,  # Don't reduce speed for emergency return
                "simplify_mission": True,
            }
        elif battery_pct < low_threshold:
            return {
                "action": "conserve_energy",
                "speed_factor": self.mission_adaptation.get(
                    "battery_conservation_factor", 0.8
                ),
                "simplify_mission": True,
            }
        else:
            return {
                "action": "normal_operation",
                "speed_factor": 1.0,
                "simplify_mission": False,
            }

    def _trigger_emergency(self, emergency_type: str):
        """Trigger emergency response"""
        self.emergency_mode = True

        if emergency_type in self.emergency_callbacks:
            self.emergency_callbacks[emergency_type]()

        correlation_id = str(uuid.uuid4())
        logger.critical(
            "Emergency mode activated",
            emergency_type=emergency_type,
            correlation_id=correlation_id,
        )

    def reset_emergency_mode(self):
        """Reset emergency mode (use with caution)"""
        self.emergency_mode = False
        self.thermal_speed_factor = 1.0
        self.battery_speed_factor = 1.0
