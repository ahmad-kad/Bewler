.. _sensor_bridge_integration:

Integration Guide
=================

This guide explains how to integrate the WebSocket Sensor Bridge into your ROS2 autonomy system, including system requirements, launch configuration, and integration patterns.

System Requirements
-------------------

Hardware Requirements
~~~~~~~~~~~~~~~~~~~~~

- **CPU**: Minimum 100MHz for basic operation, 500MHz+ recommended for high-frequency sensors
- **Memory**: 32MB RAM minimum, 64MB recommended
- **Network**: Ethernet or WiFi with WebSocket support
- **Storage**: 10MB free space for ROS2 installation and configuration

Software Requirements
~~~~~~~~~~~~~~~~~~~~~

- **ROS2**: Humble Hawksbill or later
- **Python**: 3.8 or later
- **WebSocket Library**: Included in package dependencies
- **ROS2 Packages**:
  - ``sensor_msgs``
  - ``nav_msgs``
  - ``std_msgs``
  - ``diagnostic_msgs``

Installation
------------

Package Installation
~~~~~~~~~~~~~~~~~~~~

1. **Clone the repository:**

   .. code-block:: bash

      cd ~/urc-machiato-2026/Autonomy/ros2_ws/src
      # Package is already included in the workspace

2. **Build the package:**

   .. code-block:: bash

      cd ~/urc-machiato-2026/Autonomy/ros2_ws
      colcon build --packages-select autonomy_sensor_bridge

3. **Source the workspace:**

   .. code-block:: bash

      source install/setup.bash

4. **Verify installation:**

   .. code-block:: bash

      ros2 pkg list | grep sensor_bridge
      ros2 run autonomy_sensor_bridge sensor_bridge_node --help

Launch Configuration
--------------------

Basic Launch
~~~~~~~~~~~~

Launch the sensor bridge with default configuration:

.. code-block:: bash

   ros2 launch autonomy_sensor_bridge sensor_bridge.launch.py

Custom Configuration
~~~~~~~~~~~~~~~~~~~~

Launch with custom WebSocket endpoint:

.. code-block:: bash

   ros2 launch autonomy_sensor_bridge sensor_bridge.launch.py \
      websocket_url:=ws://192.168.1.100:9090 \
      log_level:=debug

Launch with custom config file:

.. code-block:: bash

   ros2 launch autonomy_sensor_bridge sensor_bridge.launch.py \
      config_file:=/path/to/custom_config.yaml

System Integration Patterns
---------------------------

Integration with Navigation Stack
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The sensor bridge integrates seamlessly with ROS2 navigation components:

.. code-block:: xml

   <!-- Launch file snippet -->
   <include file="$(find-pkg-share autonomy_sensor_bridge)/launch/sensor_bridge.launch.py">
     <arg name="websocket_url" value="ws://sensor-hub:8080"/>
   </include>

   <include file="$(find-pkg-share nav2_bringup)/launch/navigation_launch.py">
     <arg name="use_sim_time" value="false"/>
   </include>

Subscribe to sensor data in your navigation node:

.. code-block:: python

   import rclpy
   from sensor_msgs.msg import Imu, NavSatFix
   from nav_msgs.msg import Odometry

   class NavigationNode(Node):
       def __init__(self):
           super().__init__('navigation_node')

           # Subscribe to sensor bridge topics
           self.imu_sub = self.create_subscription(
               Imu, '/imu/data', self.imu_callback, 10)

           self.gps_sub = self.create_subscription(
               NavSatFix, '/gps/fix', self.gps_callback, 10)

           self.odom_sub = self.create_subscription(
               Odometry, '/wheel/odom', self.odom_callback, 10)

Integration with SLAM
~~~~~~~~~~~~~~~~~~~~~

The bridge provides data for SLAM algorithms:

.. code-block:: python

   from autonomy_slam.gps_fusion_node import GPSFusionNode

   # GPS fusion node automatically subscribes to /gps/fix
   # IMU data available for sensor fusion
   # Wheel odometry for motion estimation

Integration with State Machine
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Sensor data influences system state decisions:

.. code-block:: python

   from autonomy_state_management.state_machine_director import StateMachineDirector

   # Battery status affects state transitions
   # IMU data used for stability monitoring
   # GPS data required for navigation states

Integration with Diagnostics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The bridge publishes diagnostic information:

.. code-block:: python

   # Subscribe to bridge diagnostics
   diag_sub = self.create_subscription(
       DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)

   # Monitor bridge health
   status_sub = self.create_subscription(
       String, '/sensor_bridge/status', self.status_callback, 10)

   # Access performance metrics
   metrics_sub = self.create_subscription(
       String, '/sensor_bridge/metrics', self.metrics_callback, 10)

WebSocket Sensor Integration
-----------------------------

Connecting Sensor Hardware
~~~~~~~~~~~~~~~~~~~~~~~~~~

1. **Configure WebSocket endpoint:**

   .. code-block:: yaml

      # config/sensor_bridge.yaml
      sensor_bridge:
        websocket_url: "ws://your-sensor-ip:8080"

2. **Implement sensor WebSocket server:**

   .. code-block:: python

      import asyncio
      import websockets
      import json
      import time

      async def sensor_server(websocket, path):
          print("Sensor connected")
          while True:
              # Collect sensor data
              sensor_data = {
                  'timestamp': time.time(),
                  'sensors': {
                      'imu': get_imu_data(),
                      'gps': get_gps_data(),
                      'battery': get_battery_data()
                  }
              }

              # Send to bridge
              await websocket.send(json.dumps(sensor_data))
              await asyncio.sleep(0.1)  # 10Hz

      start_server = websockets.serve(sensor_server, "0.0.0.0", 8080)
      asyncio.get_event_loop().run_until_complete(start_server)
      asyncio.get_event_loop().run_forever()

3. **Verify data flow:**

   .. code-block:: bash

      # Monitor incoming data
      ros2 topic hz /imu/data

      # Check data quality
      ros2 topic echo /imu/data --once

Integration with Existing Systems
----------------------------------

Migrating from Direct ROS2
~~~~~~~~~~~~~~~~~~~~~~~~~~~

If migrating from direct ROS2 sensor nodes:

1. **Keep existing topic names** for compatibility
2. **Configure QoS settings** to match original nodes
3. **Gradually transition** sensor hardware to WebSocket

.. code-block:: yaml

   # Maintain compatibility with existing systems
   sensor_bridge:
     sensors:
       imu:
         topic_name: "/sensors/imu"  # Existing topic name
         qos_profile: compatible_settings

Integration with Simulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~

For Gazebo or simulation environments:

.. code-block:: python

   # Simulation sensor bridge
   class SimulatedSensorBridge(WebSocketSensorBridgeNode):
       def _simulate_sensor_data(self):
           # Generate realistic sensor data
           # Publish via WebSocket for testing

Integration Testing
-------------------

Testing Data Flow
~~~~~~~~~~~~~~~~~

1. **Launch the bridge:**

   .. code-block:: bash

      ros2 launch autonomy_sensor_bridge sensor_bridge.launch.py

2. **Send test data:**

   .. code-block:: bash

      # Use the included test script
      python3 test_sensor_bridge.py

3. **Verify topics:**

   .. code-block:: bash

      ros2 topic list | grep sensor_bridge
      ros2 topic hz /imu/data /gps/fix /battery/status

4. **Check diagnostics:**

   .. code-block:: bash

      ros2 topic echo /diagnostics

Testing Reliability
~~~~~~~~~~~~~~~~~~~

1. **Test reconnection:**

   .. code-block:: bash

      # Kill WebSocket server
      # Verify automatic reconnection
      ros2 topic echo /sensor_bridge/status

2. **Test data validation:**

   .. code-block:: bash

      # Send invalid data
      # Verify error handling and continued operation

3. **Test graceful degradation:**

   .. code-block:: bash

      # Disconnect network
      # Verify system continues with last known data

Troubleshooting Integration
---------------------------

Common Issues
~~~~~~~~~~~~~

**Bridge won't start:**

.. code-block:: bash

   # Check package installation
   ros2 pkg list | grep sensor_bridge

   # Verify Python dependencies
   python3 -c "import websockets"

**No sensor data:**

.. code-block:: bash

   # Check WebSocket connection
   ros2 topic echo /sensor_bridge/status

   # Verify sensor data format
   # Check bridge logs for validation errors

**High latency:**

.. code-block:: bash

   # Check QoS settings
   ros2 topic info /imu/data

   # Monitor processing time
   ros2 topic echo /sensor_bridge/metrics

**Memory leaks:**

.. code-block:: bash

   # Monitor resource usage
   ros2 run rqt_top rqt_top

   # Check for proper cleanup on shutdown

Performance Optimization
------------------------

Optimizing for High-Frequency Sensors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For IMU and wheel odometry (100Hz+):

.. code-block:: yaml

   sensor_bridge:
     sensors:
       imu:
         enabled: true
         topic_name: "/imu/data"
         qos_profile:
           reliability: best_effort
           history: keep_last
           depth: 20

Optimizing for Network Constraints
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For bandwidth-limited networks:

.. code-block:: yaml

   sensor_bridge:
     sensors:
       gps:
         # Reduce update rate
         update_rate: 5.0  # 5Hz instead of 10Hz

Optimizing for Processing Power
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For resource-constrained devices:

.. code-block:: yaml

   sensor_bridge:
     # Reduce monitoring frequency
     health_check_interval: 5.0  # Less frequent checks
     message_timeout: 30.0       # Longer timeout

Monitoring Integration
----------------------

Health Monitoring
~~~~~~~~~~~~~~~~~

Integrate with system health monitoring:

.. code-block:: python

   from diagnostic_msgs.msg import DiagnosticStatus

   def check_sensor_bridge_health(self):
       """Check bridge health status"""
       # Subscribe to /diagnostics
       # Check connection status
       # Verify data flow
       # Return diagnostic status

Logging Integration
~~~~~~~~~~~~~~~~~~~

Configure logging for integration:

.. code-block:: yaml

   sensor_bridge:
     # Adjust log level for integration
     log_level: info  # debug, info, warn, error, fatal

Metrics Integration
~~~~~~~~~~~~~~~~~~~

Export metrics for monitoring systems:

.. code-block:: python

   # Prometheus metrics example
   from prometheus_client import Gauge, Counter

   bridge_connected = Gauge('sensor_bridge_connected', 'Bridge connection status')
   messages_processed = Counter('sensor_bridge_messages_processed', 'Messages processed')

   # Update metrics from /sensor_bridge/metrics topic

Best Practices
--------------

1. **Use standard ROS2 message types** for maximum compatibility
2. **Configure QoS appropriately** for your sensor characteristics
3. **Monitor bridge health** in production deployments
4. **Test failure scenarios** during integration
5. **Document sensor data formats** for maintenance
6. **Plan migration path** to direct ROS2 transport
7. **Implement proper error handling** in consuming nodes
8. **Use semantic versioning** for configuration changes

By following this integration guide, you can seamlessly incorporate the WebSocket Sensor Bridge into your ROS2 autonomy system while maintaining high reliability and performance standards.
