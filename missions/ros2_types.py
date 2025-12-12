"""
ROS2 Type Definitions and Utilities for URC 2026 Mission System

Provides type annotations and utilities to improve static analysis and MyPy support
for ROS2 message types and interfaces.
"""

from typing import Any, Dict, List, Optional, Protocol


# ROS2 Message Type Protocols for better static analysis
class ROS2Message(Protocol):
    """Protocol for ROS2 message types."""

    def __init__(self) -> None: ...


class Header(Protocol):
    """ROS2 Header message protocol."""
    stamp: Any  # builtin_interfaces/Time
    frame_id: str


class Pose(Protocol):
    """ROS2 Pose message protocol."""
    position: Any  # geometry_msgs/Point
    orientation: Any  # geometry_msgs/Quaternion


class PoseStamped(Protocol):
    """ROS2 PoseStamped message protocol."""
    header: Header
    pose: Pose


class Point(Protocol):
    """ROS2 Point message protocol."""
    x: float
    y: float
    z: float


class Quaternion(Protocol):
    """ROS2 Quaternion message protocol."""
    x: float
    y: float
    z: float
    w: float


class Twist(Protocol):
    """ROS2 Twist message protocol."""
    linear: Any  # geometry_msgs/Vector3
    angular: Any  # geometry_msgs/Vector3


class Vector3(Protocol):
    """ROS2 Vector3 message protocol."""
    x: float
    y: float
    z: float


class Imu(Protocol):
    """ROS2 IMU message protocol."""
    header: Header
    orientation: Optional[Quaternion]
    orientation_covariance: List[float]
    angular_velocity: Vector3
    angular_velocity_covariance: List[float]
    linear_acceleration: Vector3
    linear_acceleration_covariance: List[float]


class NavSatFix(Protocol):
    """ROS2 NavSatFix message protocol."""
    header: Header
    latitude: float
    longitude: float
    altitude: float
    position_covariance: List[float]
    position_covariance_type: int


class Odometry(Protocol):
    """ROS2 Odometry message protocol."""
    header: Header
    pose: Any  # geometry_msgs/PoseWithCovarianceStamped equivalent
    twist: Any  # geometry_msgs/TwistWithCovarianceStamped equivalent


class BatteryState(Protocol):
    """ROS2 BatteryState message protocol."""
    header: Header
    voltage: float
    temperature: Optional[float]
    current: Optional[float]
    charge: Optional[float]
    capacity: Optional[float]
    percentage: Optional[float]
    power_supply_status: int
    power_supply_health: int
    power_supply_technology: int
    present: bool


class Temperature(Protocol):
    """ROS2 Temperature message protocol."""
    header: Header
    temperature: float
    variance: float


class String(Protocol):
    """ROS2 String message protocol."""
    data: str

# Mission-specific types


class Waypoint:
    """Waypoint data structure."""
    latitude: float
    longitude: float
    altitude: float
    name: str
    precision_required: bool


class MissionConfig:
    """Mission configuration structure."""
    mission_type: str
    waypoints: List[Waypoint]
    timeout: float
    retry_count: int
    emergency_procedures: Dict[str, Any]


class SystemHealth:
    """System health monitoring structure."""
    cpu_usage: float
    memory_usage: float
    temperature: float
    battery_level: float
    communication_status: str
    subsystem_status: Dict[str, str]

# Utility functions for type-safe operations


def create_pose_stamped(
    x: float,
    y: float,
    z: float,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0,
    frame_id: str = "map"
) -> Any:
    """Create a PoseStamped message with proper typing."""
    # This would use actual ROS2 messages in implementation
    # For now, return a dict for type checking
    return {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": frame_id},
        "pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}
        }
    }


def create_twist(
    linear_x: float = 0.0,
    linear_y: float = 0.0,
    linear_z: float = 0.0,
    angular_x: float = 0.0,
    angular_y: float = 0.0,
    angular_z: float = 0.0
) -> Any:
    """Create a Twist message with proper typing."""
    return {
        "linear": {"x": linear_x, "y": linear_y, "z": linear_z},
        "angular": {"x": angular_x, "y": angular_y, "z": angular_z}
    }


def validate_message_type(message: Any, expected_type: str) -> bool:
    """Validate that a message matches expected type."""
    # Type checking utility
    if not hasattr(message, '__class__'):
        return False
    return expected_type in str(type(message))


def safe_get_message_attribute(message: Any, attribute_path: str, default: Any = None) -> Any:
    """Safely get nested message attributes."""
    try:
        current = message
        for attr in attribute_path.split('.'):
            current = getattr(current, attr, None)
            if current is None:
                return default
        return current
    except AttributeError:
        return default

# Constants for better type safety


class ROS2QoSProfiles:
    """QoS profile constants for consistent configuration."""
    SENSOR_DATA = {
        "reliability": "BEST_EFFORT",
        "durability": "VOLATILE",
        "depth": 10
    }

    CONTROL_COMMANDS = {
        "reliability": "RELIABLE",
        "durability": "VOLATILE",
        "depth": 5
    }

    STATE_INFORMATION = {
        "reliability": "RELIABLE",
        "durability": "TRANSIENT_LOCAL",
        "depth": 1
    }


class MissionStatus:
    """Mission status enumeration."""
    IDLE = "idle"
    INITIALIZING = "initializing"
    EXECUTING = "executing"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    EMERGENCY = "emergency"


class ErrorCodes:
    """Standardized error codes for mission system."""
    SUCCESS = 0
    INVALID_PARAMETERS = 1
    HARDWARE_FAILURE = 2
    TIMEOUT = 3
    COMMUNICATION_ERROR = 4
    SAFETY_VIOLATION = 5
    RESOURCE_EXHAUSTED = 6


# Type aliases for better readability
PoseStampedMsg = Any  # Would be geometry_msgs.msg.PoseStamped
TwistMsg = Any        # Would be geometry_msgs.msg.Twist
ImuMsg = Any          # Would be sensor_msgs.msg.Imu
StringMsg = Any       # Would be std_msgs.msg.String

# Export commonly used types
__all__ = [
    'ROS2Message', 'Header', 'Pose', 'PoseStamped', 'Point', 'Quaternion',
    'Twist', 'Vector3', 'Imu', 'NavSatFix', 'Odometry', 'BatteryState',
    'Temperature', 'String', 'Waypoint', 'MissionConfig', 'SystemHealth',
    'create_pose_stamped', 'create_twist', 'validate_message_type',
    'safe_get_message_attribute', 'ROS2QoSProfiles', 'MissionStatus', 'ErrorCodes',
    'PoseStampedMsg', 'TwistMsg', 'ImuMsg', 'StringMsg'
]
