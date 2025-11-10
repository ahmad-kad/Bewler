.. _sensor_bridge:

Sensor Bridge Documentation
===========================

The WebSocket Sensor Bridge provides a robust interface for integrating external sensor data sources with the URC 2026 Mars Rover's ROS2 autonomy system. This bridge enables seamless data flow from WebSocket-connected sensors to ROS2 topics while maintaining high reliability and performance.

.. toctree::
   :maxdepth: 2
   :caption: Sensor Bridge Documentation

   overview
   integration
   usage
   configuration
   api
   examples
   migration

Overview
--------

The sensor bridge serves as the primary interface between external sensor hardware and the rover's autonomy stack. Key features include:

- **WebSocket-to-ROS2 Translation**: Converts JSON sensor data to standard ROS2 sensor messages
- **ROS2 Timer Architecture**: Uses ROS2 timers instead of threads for maximum reliability
- **Comprehensive Validation**: Type checking, range validation, and sanity checks for all sensor data
- **Adaptive Reconnection**: Exponential backoff with configurable retry limits
- **Health Monitoring**: Real-time status and performance metrics
- **QoS Optimization**: Sensor-specific QoS profiles for optimal performance
- **Graceful Degradation**: Fallback modes during connection failures
- **Migration Ready**: Designed for easy transition to direct ROS2 transport

Architecture
-------------

.. figure:: ../_static/sensor_bridge_architecture.png
   :alt: Sensor Bridge Architecture
   :align: center
   :scale: 75%

   Sensor Bridge Architecture Overview

The bridge operates as a ROS2 node that:

1. **Receives** JSON sensor data via WebSocket
2. **Validates** all incoming data for type and range correctness
3. **Converts** validated data to standard ROS2 sensor messages
4. **Publishes** messages to ROS2 topics with appropriate QoS settings
5. **Monitors** connection health and system performance
6. **Reports** status and diagnostics for external monitoring

Quick Start
-----------

1. **Launch the bridge:**

   .. code-block:: bash

      ros2 launch autonomy_sensor_bridge sensor_bridge.launch.py

2. **Send sensor data via WebSocket:**

   .. code-block:: javascript

      const ws = new WebSocket('ws://localhost:8080');
      ws.send(JSON.stringify({
        timestamp: Date.now() / 1000,
        sensors: {
          imu: {
            accel: {x: 0.1, y: 0.2, z: 9.8},
            gyro: {x: 0.01, y: 0.02, z: 0.03}
          }
        }
      }));

3. **Subscribe to ROS2 topics:**

   .. code-block:: bash

      ros2 topic echo /imu/data

Supported Sensors
------------------

The bridge supports the following sensor types with full validation and ROS2 message conversion:

- **IMU** (`/imu/data`) - Inertial measurement data
- **GPS** (`/gps/fix`) - Global positioning data
- **Battery** (`/battery/status`) - Power system monitoring
- **Wheel Odometry** (`/wheel/odom`) - Wheel encoder data
- **Temperature** (`/temperature/data`) - Thermal monitoring

All sensors publish standard ROS2 message types with appropriate QoS settings for their data characteristics.
