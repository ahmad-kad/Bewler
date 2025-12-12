#!/usr/bin/env python3
"""
Custom Exception Hierarchy for URC Robotics Platform

Provides specific, meaningful exceptions for different failure modes
to improve error handling and debugging.
"""

from typing import Any, Dict, Optional

import structlog

logger = structlog.get_logger(__name__)


class RoboticsException(Exception):
    """
    Base exception for all robotics platform errors.

    Provides structured error information and logging.
    """

    def __init__(self, message: str, context: Optional[Dict[str, Any]] = None):
        super().__init__(message)
        self.message = message
        self.context = context or {}

        # Auto-log critical errors
        logger.error(f"{self.__class__.__name__}: {message}", **self.context)


class ConfigurationError(RoboticsException):
    """Raised when configuration is invalid or missing."""


class ValidationError(RoboticsException):
    """Raised when data validation fails."""


class TeleoperationError(RoboticsException):
    """Base class for teleoperation-related errors."""


class DataValidationError(TeleoperationError):
    """Raised when teleoperation data fails validation."""


class DataQualityError(TeleoperationError):
    """Raised when data quality issues are detected."""


class MotorControlError(TeleoperationError):
    """Raised when motor control operations fail."""


class SystemHealthError(RoboticsException):
    """Base class for system health-related errors."""


class ThermalError(SystemHealthError):
    """Raised when thermal limits are exceeded."""


class BatteryError(SystemHealthError):
    """Raised when battery conditions are critical."""


class EmergencyError(SystemHealthError):
    """Raised during emergency situations."""


class MissionError(RoboticsException):
    """Base class for mission-related errors."""


class NavigationError(MissionError):
    """Raised when navigation operations fail."""


class CommunicationError(RoboticsException):
    """Raised when communication with external systems fails."""


class HardwareError(RoboticsException):
    """Raised when hardware operations fail."""


class SensorError(HardwareError):
    """Raised when sensor operations fail."""


class ActuatorError(HardwareError):
    """Raised when actuator operations fail."""


# Convenience functions for common error patterns
def validate_required_env_var(var_name: str) -> str:
    """
    Validate that a required environment variable is set.

    Raises:
        ConfigurationError: If the environment variable is not set
    """
    import os

    value = os.getenv(var_name)
    if value is None:
        raise ConfigurationError(
            f"Required environment variable '{var_name}' is not set",
            context={"env_var": var_name},
        )
    return value


def validate_data_range(
    value: float, min_val: float, max_val: float, field_name: str
) -> None:
    """
    Validate that a numeric value is within acceptable range.

    Raises:
        ValidationError: If the value is outside the acceptable range
    """
    if not (min_val <= value <= max_val):
        raise ValidationError(
            f"Value {value} for {field_name} is outside acceptable range "
            f"[{min_val}, {max_val}]",
            context={
                "field": field_name,
                "value": value,
                "min": min_val,
                "max": max_val,
            },
        )
