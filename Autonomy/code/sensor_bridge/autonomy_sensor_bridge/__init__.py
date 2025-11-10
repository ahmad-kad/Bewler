"""
Autonomy Sensor Bridge Package.

WebSocket-to-ROS2 sensor data bridge for URC 2026 Mars Rover.
Provides seamless sensor data integration with migration path to direct ROS2.
"""

__version__ = "0.1.0"
__author__ = "URC Machiato Team"

from .websocket_bridge import WebSocketSensorBridgeNode

__all__ = [
    "WebSocketSensorBridgeNode",
]
