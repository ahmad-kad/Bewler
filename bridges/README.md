# Bridges Directory

Data bridge components for connecting different system parts.

## Bridge Files

- `map_data_bridge.py` - Bridge for map data between SLAM and mission control
- `slam_data_bridge.py` - SLAM data processing and bridging
- `websocket_mission_bridge.py` - WebSocket bridge for mission control
- `websocket_slam_bridge.py` - WebSocket bridge for SLAM data

## Purpose

These bridges handle data flow between different subsystems:
- ROS2 ↔ WebSocket communication
- SLAM ↔ Mission control data exchange
- External sensor data integration



