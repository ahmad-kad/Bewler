.. _sensor_bridge_usage:

Usage Guide
===========

This guide provides practical instructions for using the WebSocket Sensor Bridge, including how to read sensor data, monitor bridge status, and troubleshoot common issues.

Reading Sensor Data
-------------------

Subscribing to Sensor Topics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The bridge publishes sensor data on standard ROS2 topics. Subscribe to these topics to access sensor data:

**IMU Data**

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Imu

   class ImuReader(Node):
       def __init__(self):
           super().__init__('imu_reader')

           # Subscribe to IMU data
           self.subscription = self.create_subscription(
               Imu,
               '/imu/data',
               self.imu_callback,
               10  # QoS depth
           )

       def imu_callback(self, msg: Imu):
           # Access IMU data
           accel_x = msg.linear_acceleration.x
           accel_y = msg.linear_acceleration.y
           accel_z = msg.linear_acceleration.z

           gyro_x = msg.angular_velocity.x
           gyro_y = msg.angular_velocity.y
           gyro_z = msg.angular_velocity.z

           # Orientation (if available)
           if hasattr(msg, 'orientation'):
               qx = msg.orientation.x
               qy = msg.orientation.y
               qz = msg.orientation.z
               qw = msg.orientation.w

           self.get_logger().info(
               f'IMU: accel=({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}) '
               f'gyro=({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f})'
           )

**GPS Data**

.. code-block:: python

   from sensor_msgs.msg import NavSatFix

   class GpsReader(Node):
       def __init__(self):
           super().__init__('gps_reader')

           self.subscription = self.create_subscription(
               NavSatFix,
               '/gps/fix',
               self.gps_callback,
               10
           )

       def gps_callback(self, msg: NavSatFix):
           latitude = msg.latitude
           longitude = msg.longitude
           altitude = msg.altitude
           status = msg.status.status

           self.get_logger().info(
               f'GPS: {latitude:.6f}, {longitude:.6f}, {altitude:.2f}m '
               f'Status: {status}'
           )

**Battery Data**

.. code-block:: python

   from sensor_msgs.msg import BatteryState

   class BatteryReader(Node):
       def __init__(self):
           super().__init__('battery_reader')

           self.subscription = self.create_subscription(
               BatteryState,
               '/battery/status',
               self.battery_callback,
               10
           )

       def battery_callback(self, msg: BatteryState):
           voltage = msg.voltage
           current = msg.current
           percentage = msg.percentage
           temperature = msg.temperature

           self.get_logger().info(
               f'Battery: {voltage:.2f}V, {current:.2f}A, '
               f'{percentage:.1f}%, {temperature:.1f}°C'
           )

**Wheel Odometry**

.. code-block:: python

   from nav_msgs.msg import Odometry

   class OdometryReader(Node):
       def __init__(self):
           super().__init__('odometry_reader')

           self.subscription = self.create_subscription(
               Odometry,
               '/wheel/odom',
               self.odom_callback,
               10
           )

       def odom_callback(self, msg: Odometry):
           x = msg.pose.pose.position.x
           y = msg.pose.pose.position.y
           yaw = msg.pose.pose.orientation.z  # Simplified

           vx = msg.twist.twist.linear.x
           vy = msg.twist.twist.linear.y

           self.get_logger().info(
               f'Odom: pos=({x:.2f}, {y:.2f}) vel=({vx:.2f}, {vy:.2f})'
           )

**Temperature Data**

.. code-block:: python

   from sensor_msgs.msg import Temperature

   class TemperatureReader(Node):
       def __init__(self):
           super().__init__('temperature_reader')

           self.subscription = self.create_subscription(
               Temperature,
               '/temperature/data',
               self.temp_callback,
               10
           )

       def temp_callback(self, msg: Temperature):
           temperature = msg.temperature
           variance = msg.variance

           self.get_logger().info(
               f'Temperature: {temperature:.2f}°C (variance: {variance:.4f})'
           )

Command Line Access
~~~~~~~~~~~~~~~~~~~

Access sensor data from the command line:

**Monitor IMU data:**

.. code-block:: bash

   ros2 topic echo /imu/data

**Check GPS fix:**

.. code-block:: bash

   ros2 topic echo /gps/fix --once

**Monitor battery status:**

.. code-block:: bash

   ros2 topic hz /battery/status

**Check topic frequencies:**

.. code-block:: bash

   # Check all sensor topics
   ros2 topic hz /imu/data /gps/fix /battery/status /wheel/odom /temperature/data

**List available topics:**

.. code-block:: bash

   ros2 topic list | grep -E "(imu|gps|battery|odom|temperature)"

Publishing to the Bridge
-------------------------

WebSocket Data Format
~~~~~~~~~~~~~~~~~~~~~

Send sensor data to the bridge via WebSocket using this JSON format:

.. code-block:: json

   {
     "timestamp": 1234567890.123,
     "sensors": {
       "imu": {
         "accel": {"x": 0.1, "y": 0.2, "z": 9.81},
         "gyro": {"x": 0.01, "y": 0.02, "z": 0.03},
         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
         "accel_covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
         "gyro_covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
         "orientation_covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
       },
       "gps": {
         "lat": 37.7749,
         "lon": -122.4194,
         "altitude": 10.5,
         "status": 0,
         "service": 1,
         "position_covariance": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
       },
       "battery": {
         "voltage": 12.3,
         "current": 2.1,
         "percentage": 85.0,
         "temperature": 35.0,
         "status": 0,
         "health": 0,
         "technology": 0,
         "capacity": 100.0,
         "design_capacity": 100.0,
         "charge": 85.0,
         "cell_voltages": [3.7, 3.7, 3.7, 3.7],
         "cell_temperatures": [35.0, 35.0, 35.0, 35.0]
       },
       "wheel_odom": {
         "x": 1.5,
         "y": 2.3,
         "z": 0.0,
         "qx": 0.0,
         "qy": 0.0,
         "qz": 0.0,
         "qw": 1.0,
         "vx": 0.5,
         "vy": 0.0,
         "vz": 0.0,
         "wx": 0.0,
         "wy": 0.0,
         "wz": 0.1,
         "pose_covariance": [0.1] * 36,
         "twist_covariance": [0.1] * 36
       },
       "temperature": {
         "temperature": 35.0,
         "variance": 0.1
       }
     }
   }

WebSocket Client Examples
~~~~~~~~~~~~~~~~~~~~~~~~~

**Python WebSocket Client:**

.. code-block:: python

   import asyncio
   import websockets
   import json
   import time
   import random

   async def sensor_client():
       uri = "ws://localhost:8080"
       async with websockets.connect(uri) as websocket:
           print("Connected to sensor bridge")

           while True:
               # Generate simulated sensor data
               sensor_data = {
                   "timestamp": time.time(),
                   "sensors": {
                       "imu": {
                           "accel": {
                               "x": random.gauss(0, 0.1),
                               "y": random.gauss(0, 0.1),
                               "z": 9.81 + random.gauss(0, 0.01)
                           },
                           "gyro": {
                               "x": random.gauss(0, 0.01),
                               "y": random.gauss(0, 0.01),
                               "z": random.gauss(0, 0.01)
                           }
                       },
                       "gps": {
                           "lat": 37.7749 + random.gauss(0, 0.0001),
                           "lon": -122.4194 + random.gauss(0, 0.0001),
                           "altitude": 10.5 + random.gauss(0, 0.1)
                       },
                       "battery": {
                           "voltage": 12.3 + random.gauss(0, 0.1),
                           "percentage": 85.0 + random.gauss(0, 1.0),
                           "temperature": 35.0 + random.gauss(0, 2.0)
                       }
                   }
               }

               # Send data
               await websocket.send(json.dumps(sensor_data))
               print(f"Sent sensor data at {sensor_data['timestamp']}")

               # Wait for next update (10Hz)
               await asyncio.sleep(0.1)

   # Run the client
   asyncio.run(sensor_client())

**JavaScript WebSocket Client:**

.. code-block:: javascript

   // Web browser or Node.js
   const WebSocket = require('ws');

   const ws = new WebSocket('ws://localhost:8080');

   ws.on('open', function open() {
       console.log('Connected to sensor bridge');

       // Send sensor data periodically
       setInterval(() => {
           const sensorData = {
               timestamp: Date.now() / 1000,
               sensors: {
                   imu: {
                       accel: { x: 0.1, y: 0.2, z: 9.81 },
                       gyro: { x: 0.01, y: 0.02, z: 0.03 }
                   },
                   gps: {
                       lat: 37.7749,
                       lon: -122.4194,
                       altitude: 10.5
                   },
                   battery: {
                       voltage: 12.3,
                       percentage: 85.0,
                       temperature: 35.0
                   }
               }
           };

           ws.send(JSON.stringify(sensorData));
           console.log('Sent sensor data');
       }, 100); // 10Hz
   });

   ws.on('error', function error(err) {
       console.error('WebSocket error:', err);
   });

   ws.on('close', function close() {
       console.log('WebSocket connection closed');
   });

Monitoring Bridge Status
------------------------

Connection Status
~~~~~~~~~~~~~~~~~

Monitor the bridge connection status:

.. code-block:: bash

   # Real-time status updates
   ros2 topic echo /sensor_bridge/status

Status message format:

.. code-block:: json

   {
     "connection_state": "connected",
     "reconnect_attempts": 0,
     "last_message_time": 1234567890.123,
     "uptime_seconds": 3600.5,
     "timestamp": 1234567890.456
   }

**Connection States:**
- ``disconnected``: No WebSocket connection
- ``connecting``: Attempting to connect
- ``connected``: Successfully connected
- ``reconnecting``: Connection lost, retrying
- ``failed``: Max reconnection attempts reached

Performance Metrics
~~~~~~~~~~~~~~~~~~~

Monitor bridge performance:

.. code-block:: bash

   # Performance metrics
   ros2 topic echo /sensor_bridge/metrics

Metrics include:

.. code-block:: json

   {
     "messages_received": 36000,
     "messages_processed": 35985,
     "messages_failed": 15,
     "connection_attempts": 2,
     "connection_failures": 1,
     "avg_processing_time": 0.0023,
     "last_message_timestamp": 1234567890.123,
     "uptime_seconds": 3600.5
   }

ROS2 Diagnostics
~~~~~~~~~~~~~~~~

Access standard ROS2 diagnostics:

.. code-block:: bash

   # Diagnostic information
   ros2 topic echo /diagnostics

The bridge publishes two diagnostic statuses:

1. **Connection Diagnostics**: WebSocket connection health
2. **Performance Diagnostics**: Message processing statistics

Health Monitoring
~~~~~~~~~~~~~~~~~

Create a health monitoring node:

.. code-block:: python

   from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
   from std_msgs.msg import String
   import json

   class BridgeMonitor(Node):
       def __init__(self):
           super().__init__('bridge_monitor')

           # Subscribe to bridge topics
           self.diag_sub = self.create_subscription(
               DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)

           self.status_sub = self.create_subscription(
               String, '/sensor_bridge/status', self.status_callback, 10)

           self.metrics_sub = self.create_subscription(
               String, '/sensor_bridge/metrics', self.metrics_callback, 10)

       def diagnostics_callback(self, msg: DiagnosticArray):
           for status in msg.status:
               if 'WebSocket Sensor Bridge' in status.name:
                   level = status.level
                   message = status.message

                   if level == DiagnosticStatus.ERROR:
                       self.get_logger().error(f'Bridge error: {message}')
                   elif level == DiagnosticStatus.WARN:
                       self.get_logger().warn(f'Bridge warning: {message}')

       def status_callback(self, msg: String):
           status = json.loads(msg.data)
           state = status['connection_state']

           if state != 'connected':
               self.get_logger().warn(f'Bridge not connected: {state}')

       def metrics_callback(self, msg: String):
           metrics = json.loads(msg.data)
           success_rate = 0.0

           if metrics['messages_received'] > 0:
               success_rate = (metrics['messages_processed'] /
                             metrics['messages_received']) * 100

           if success_rate < 95.0:
               self.get_logger().warn(f'Low success rate: {success_rate:.1f}%')

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**No sensor data appearing:**

.. code-block:: bash

   # Check connection status
   ros2 topic echo /sensor_bridge/status --once

   # Check for error messages
   ros2 run autonomy_sensor_bridge sensor_bridge_node  # Look for errors

   # Verify WebSocket connection
   # Check if sensor client is sending data

**High latency or dropped messages:**

.. code-block:: bash

   # Check QoS settings
   ros2 topic info /imu/data

   # Monitor processing time
   ros2 topic echo /sensor_bridge/metrics --once

   # Check system resources
   ros2 run rqt_top rqt_top

**Connection instability:**

.. code-block:: bash

   # Check reconnection settings
   ros2 param get /websocket_sensor_bridge reconnect_interval
   ros2 param get /websocket_sensor_bridge max_reconnect_attempts

   # Monitor connection attempts
   ros2 topic echo /sensor_bridge/metrics --filter "connection_attempts > 0"

**Invalid sensor data:**

.. code-block:: bash

   # Check bridge logs for validation errors
   ros2 run autonomy_sensor_bridge sensor_bridge_node --log-level debug

   # Verify sensor data format matches documentation

Debug Commands
~~~~~~~~~~~~~~

**Enable debug logging:**

.. code-block:: bash

   ros2 launch autonomy_sensor_bridge sensor_bridge.launch.py log_level:=debug

**Monitor all bridge topics:**

.. code-block:: bash

   # List bridge topics
   ros2 topic list | grep sensor_bridge

   # Monitor all topics simultaneously
   ros2 run rqt_topic rqt_topic

**Check bridge parameters:**

.. code-block:: bash

   # List all parameters
   ros2 param list /websocket_sensor_bridge

   # Get specific parameter
   ros2 param get /websocket_sensor_bridge websocket_url

**Test with mock data:**

.. code-block:: bash

   # Use the included test script
   cd ~/urc-machiato-2026/Autonomy/code/sensor_bridge
   python3 test_sensor_bridge.py

Performance Tuning
------------------

Optimizing for Your Use Case
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**High-frequency sensors (IMU, wheel odometry):**

.. code-block:: yaml

   sensor_bridge:
     sensors:
       imu:
         enabled: true
         qos_profile:
           reliability: best_effort  # Minimize latency
           depth: 20                 # Buffer more messages

**Reliable delivery (GPS, battery):**

.. code-block:: yaml

   sensor_bridge:
     sensors:
       gps:
         enabled: true
         qos_profile:
           reliability: reliable     # Ensure delivery
           durability: transient_local  # Persist for late joiners

**Network-constrained environments:**

.. code-block:: yaml

   sensor_bridge:
     # Reduce monitoring overhead
     health_check_interval: 5.0     # Less frequent checks
     message_timeout: 30.0          # Longer timeout

**Resource-constrained devices:**

.. code-block:: yaml

   sensor_bridge:
     # Reduce processing frequency
     sensors:
       temperature:
         update_rate: 1.0           # Lower frequency

Monitoring Best Practices
~~~~~~~~~~~~~~~~~~~~~~~~~

1. **Monitor connection status** in production
2. **Set up alerts** for connection failures
3. **Track message success rates** (>95% target)
4. **Monitor processing latency** (<5ms target)
5. **Log sensor data validation errors** for debugging
6. **Regular health checks** during operation

By following this usage guide, you can effectively integrate the WebSocket Sensor Bridge into your ROS2 system and monitor its performance for reliable operation.
