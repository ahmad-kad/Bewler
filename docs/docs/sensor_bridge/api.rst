.. _sensor_bridge_api:

API Reference
=============

This section provides detailed API documentation for the WebSocket Sensor Bridge, including class definitions, methods, parameters, and message formats.

WebSocketSensorBridgeNode
--------------------------

.. py:class:: WebSocketSensorBridgeNode

   Main ROS2 node for the WebSocket Sensor Bridge.

   This node handles WebSocket connections, sensor data validation, ROS2 message conversion, and health monitoring.

   .. py:method:: __init__()

      Initialize the sensor bridge node.

      Sets up ROS2 parameters, publishers, subscribers, and WebSocket connection management.

   .. py:method:: _declare_parameters()

      Declare all ROS2 parameters with validation.

      **Parameters declared:**

      - ``websocket_url`` (string, default: "ws://localhost:8080"): WebSocket server URL
      - ``reconnect_interval`` (float, default: 3.0): Base reconnection delay in seconds
      - ``max_reconnect_attempts`` (int, default: 10): Maximum reconnection attempts
      - ``connection_timeout`` (float, default: 5.0): Connection timeout in seconds
      - ``health_check_interval`` (float, default: 1.0): Health check frequency in seconds
      - ``message_timeout`` (float, default: 10.0): Message timeout threshold in seconds
      - ``enable_graceful_degradation`` (bool, default: true): Enable fallback modes
      - ``max_reconnect_interval`` (float, default: 60.0): Maximum backoff delay
      - ``reconnect_backoff_multiplier`` (float, default: 1.5): Exponential backoff multiplier

   .. py:method:: _create_sensor_configs()

      Create sensor configurations with appropriate QoS settings.

      **Returns:**
         Dict[str, SensorConfig]: Dictionary of sensor configurations

   .. py:method:: _setup_publishers()

      Setup ROS2 publishers for enabled sensors.

      Creates publishers for IMU, GPS, battery, wheel odometry, and temperature sensors.

   .. py:method:: _setup_monitoring_publishers()

      Setup publishers for monitoring and diagnostics.

      Creates publishers for status, diagnostics, and metrics topics.

   .. py:method:: _schedule_connection_attempt()

      Schedule a WebSocket connection attempt using ROS2 timer.

   .. py:method:: _attempt_connection()

      Attempt WebSocket connection using ROS2 timer.

   .. py:method:: _on_connection_success()

      Handle successful WebSocket connection.

   .. py:method:: _on_connection_failure(error_msg)

      Handle WebSocket connection failure.

      :param error_msg: Error message describing the failure

   .. py:method:: _handle_websocket_message(message)

      Process incoming WebSocket message with validation.

      :param message: JSON message string from WebSocket

   .. py:method:: _validate_message_structure(data)

      Validate WebSocket message structure.

      :param data: Parsed JSON data
      :returns: Dict with 'valid' boolean and optional 'error' message

   .. py:method:: _process_sensor_data(sensor_name, sensor_data, timestamp)

      Process individual sensor data with validation.

      :param sensor_name: Name of the sensor (e.g., 'imu', 'gps')
      :param sensor_data: Sensor data dictionary
      :param timestamp: Message timestamp
      :returns: bool indicating success

   .. py:method:: _validate_sensor_data(sensor_name, sensor_data)

      Validate sensor-specific data.

      :param sensor_name: Name of the sensor
      :param sensor_data: Sensor data to validate
      :returns: Dict with 'valid' boolean and optional 'error' message

   .. py:method:: _convert_imu_data(data, timestamp)

      Convert IMU JSON data to ROS2 Imu message.

      :param data: IMU data dictionary
      :param timestamp: Message timestamp
      :returns: sensor_msgs/Imu message or None on error

   .. py:method:: _convert_gps_data(data, timestamp)

      Convert GPS JSON data to ROS2 NavSatFix message.

      :param data: GPS data dictionary
      :param timestamp: Message timestamp
      :returns: sensor_msgs/NavSatFix message or None on error

   .. py:method:: _convert_battery_data(data, timestamp)

      Convert battery JSON data to ROS2 BatteryState message.

      :param data: Battery data dictionary
      :param timestamp: Message timestamp
      :returns: sensor_msgs/BatteryState message or None on error

   .. py:method:: _convert_wheel_odom(data, timestamp)

      Convert wheel odometry JSON data to ROS2 Odometry message.

      :param data: Odometry data dictionary
      :param timestamp: Message timestamp
      :returns: nav_msgs/Odometry message or None on error

   .. py:method:: _convert_temperature(data, timestamp)

      Convert temperature JSON data to ROS2 Temperature message.

      :param data: Temperature data dictionary
      :param timestamp: Message timestamp
      :returns: sensor_msgs/Temperature message or None on error

   .. py:method:: _check_connection_health()

      Monitor connection health and handle graceful degradation.

   .. py:method:: _enter_graceful_degradation_mode()

      Enter graceful degradation mode during connection issues.

   .. py:method:: _exit_graceful_degradation_mode()

      Exit graceful degradation mode when connection recovers.

   .. py:method:: _publish_connection_status()

      Publish current connection status.

   .. py:method:: _publish_metrics()

      Publish performance metrics.

   .. py:method:: _publish_diagnostics()

      Publish ROS2 diagnostic information.

   .. py:method:: destroy_node()

      Clean shutdown with comprehensive resource cleanup.

Published Topics
----------------

Sensor Data Topics
~~~~~~~~~~~~~~~~~~

**/imu/data** (sensor_msgs/Imu)
   Inertial measurement unit data.

   **QoS:** BEST_EFFORT, KEEP_LAST, depth=20

   **Fields:**
   - header (std_msgs/Header): Timestamp and frame information
   - linear_acceleration (geometry_msgs/Vector3): Acceleration in m/s²
   - angular_velocity (geometry_msgs/Vector3): Angular velocity in rad/s
   - orientation (geometry_msgs/Quaternion): Orientation quaternion (optional)

**/gps/fix** (sensor_msgs/NavSatFix)
   GPS position and status information.

   **QoS:** RELIABLE, KEEP_LAST, depth=10

   **Fields:**
   - header (std_msgs/Header): Timestamp and frame information
   - latitude (float64): Latitude in degrees (-90 to 90)
   - longitude (float64): Longitude in degrees (-180 to 180)
   - altitude (float64): Altitude in meters
   - status (sensor_msgs/NavSatStatus): GPS fix status

**/battery/status** (sensor_msgs/BatteryState)
   Battery status and health information.

   **QoS:** RELIABLE, TRANSIENT_LOCAL, KEEP_LAST, depth=5

   **Fields:**
   - header (std_msgs/Header): Timestamp and frame information
   - voltage (float32): Battery voltage in volts
   - current (float32): Battery current in amperes
   - percentage (float32): Battery charge percentage (0-100)
   - temperature (float32): Battery temperature in Celsius

**/wheel/odom** (nav_msgs/Odometry)
   Wheel encoder odometry data.

   **QoS:** BEST_EFFORT, KEEP_LAST, depth=20

   **Fields:**
   - header (std_msgs/Header): Timestamp and frame information
   - child_frame_id (string): Child frame ID ("base_link")
   - pose (geometry_msgs/PoseWithCovariance): Position and orientation
   - twist (geometry_msgs/TwistWithCovariance): Linear and angular velocity

**/temperature/data** (sensor_msgs/Temperature)
   Temperature sensor readings.

   **QoS:** RELIABLE, KEEP_LAST, depth=10

   **Fields:**
   - header (std_msgs/Header): Timestamp and frame information
   - temperature (float64): Temperature in Celsius
   - variance (float64): Measurement variance

Monitoring Topics
~~~~~~~~~~~~~~~~~

**/sensor_bridge/status** (std_msgs/String)
   Real-time connection status information.

   **QoS:** RELIABLE, TRANSIENT_LOCAL, KEEP_LAST, depth=1

   **Format:**

   .. code-block:: json

      {
        "connection_state": "connected|connecting|disconnected|reconnecting|failed",
        "reconnect_attempts": 0,
        "last_message_time": 1234567890.123,
        "uptime_seconds": 3600.5,
        "timestamp": 1234567890.456
      }

**/sensor_bridge/metrics** (std_msgs/String)
   Performance metrics and statistics.

   **QoS:** RELIABLE, KEEP_LAST, depth=5

   **Format:**

   .. code-block:: json

      {
        "timestamp": 1234567890.123,
        "uptime_seconds": 3600.5,
        "messages_received": 36000,
        "messages_processed": 35985,
        "messages_failed": 15,
        "connection_attempts": 2,
        "connection_failures": 1,
        "avg_processing_time": 0.0023,
        "last_message_timestamp": 1234567890.123
      }

**/diagnostics** (diagnostic_msgs/DiagnosticArray)
   ROS2 diagnostic information for monitoring systems.

   **QoS:** RELIABLE, KEEP_LAST, depth=10

   **Contains:**
   - Connection diagnostics (connectivity status)
   - Performance diagnostics (processing statistics)

WebSocket Message Format
------------------------

The bridge expects WebSocket messages in the following JSON format:

.. code-block:: json

   {
     "timestamp": 1234567890.123,
     "sensors": {
       "sensor_name": {
         // Sensor-specific data fields
       }
     }
   }

Required Fields
~~~~~~~~~~~~~~~

- **timestamp** (number): Unix timestamp in seconds (required)
- **sensors** (object): Container for sensor data (required)

Sensor Data Formats
~~~~~~~~~~~~~~~~~~~

IMU Sensor
^^^^^^^^^^

.. code-block:: json

   {
     "imu": {
       "accel": {
         "x": 0.1,
         "y": 0.2,
         "z": 9.81
       },
       "gyro": {
         "x": 0.01,
         "y": 0.02,
         "z": 0.03
       },
       "orientation": {
         "x": 0.0,
         "y": 0.0,
         "z": 0.0,
         "w": 1.0
       },
       "accel_covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
       "gyro_covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
       "orientation_covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
     }
   }

GPS Sensor
^^^^^^^^^^

.. code-block:: json

   {
     "gps": {
       "lat": 37.7749,
       "lon": -122.4194,
       "altitude": 10.5,
       "status": 0,
       "service": 1,
       "position_covariance": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
     }
   }

Battery Sensor
^^^^^^^^^^^^^^

.. code-block:: json

   {
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
     }
   }

Wheel Odometry Sensor
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: json

   {
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
       "pose_covariance": [0.1, 0.0, 0.0, ..., 0.1],
       "twist_covariance": [0.1, 0.0, 0.0, ..., 0.1]
     }
   }

Temperature Sensor
^^^^^^^^^^^^^^^^^^

.. code-block:: json

   {
     "temperature": {
       "temperature": 35.0,
       "variance": 0.1
     }
   }

Data Validation Rules
---------------------

The bridge performs comprehensive validation on incoming sensor data:

IMU Validation
~~~~~~~~~~~~~~

- **Accelerometer**: Numeric values, reasonable range (-100 to 100 m/s²)
- **Gyroscope**: Numeric values, reasonable range (-50 to 50 rad/s)
- **Orientation**: Valid quaternion (optional)
- **Covariances**: 3x3 matrices with positive diagonal elements

GPS Validation
~~~~~~~~~~~~~~

- **Latitude**: Numeric, range -90 to 90 degrees
- **Longitude**: Numeric, range -180 to 180 degrees
- **Altitude**: Numeric, reasonable range (-1000 to 10000 meters)
- **Status/Service**: Valid enum values

Battery Validation
~~~~~~~~~~~~~~~~~~

- **Voltage**: Numeric, range 0-50V
- **Percentage**: Numeric, range 0-100%
- **Current**: Numeric (can be negative for discharge)
- **Temperature**: Numeric, range -50 to 150°C
- **Status fields**: Valid enum values
- **Cell data**: Arrays of appropriate size and value ranges

Wheel Odometry Validation
~~~~~~~~~~~~~~~~~~~~~~~~~

- **Position**: Numeric values for x, y, z coordinates
- **Orientation**: Valid quaternion components
- **Velocity**: Numeric linear and angular velocity values
- **Covariances**: 6x6 matrices for pose and twist

Temperature Validation
~~~~~~~~~~~~~~~~~~~~~~

- **Temperature**: Numeric, range -50 to 150°C
- **Variance**: Positive numeric value

Error Handling
--------------

The bridge implements comprehensive error handling:

Validation Errors
~~~~~~~~~~~~~~~~~

- **Invalid JSON**: Logged with message preview, message skipped
- **Missing fields**: Logged with field name, message skipped
- **Type errors**: Logged with expected vs actual types, message skipped
- **Range violations**: Logged with actual values, message may be processed with warnings

Connection Errors
~~~~~~~~~~~~~~~~~

- **Connection failures**: Automatic retry with exponential backoff
- **Timeout errors**: Configurable timeout detection
- **Protocol errors**: WebSocket protocol violations handled gracefully

Processing Errors
~~~~~~~~~~~~~~~~~

- **ROS2 publishing failures**: Logged with topic and error details
- **Memory allocation errors**: Graceful degradation
- **Threading issues**: Prevented by ROS2 timer architecture

Recovery Mechanisms
~~~~~~~~~~~~~~~~~~~

- **Connection recovery**: Automatic reconnection with backoff
- **Graceful degradation**: Continued operation with last known values
- **Resource cleanup**: Proper cleanup on shutdown
- **State reset**: Clean state transitions

Configuration Parameters
------------------------

All parameters can be set via ROS2 parameter server or launch files:

Core Parameters
~~~~~~~~~~~~~~~

- **websocket_url**: WebSocket server endpoint
- **reconnect_interval**: Base delay between reconnection attempts
- **max_reconnect_attempts**: Maximum number of reconnection attempts
- **connection_timeout**: Timeout for connection attempts

Monitoring Parameters
~~~~~~~~~~~~~~~~~~~~~

- **health_check_interval**: Frequency of health checks (seconds)
- **message_timeout**: Timeout for message reception (seconds)
- **enable_graceful_degradation**: Enable fallback modes during failures

Advanced Parameters
~~~~~~~~~~~~~~~~~~~

- **max_reconnect_interval**: Maximum backoff delay (seconds)
- **reconnect_backoff_multiplier**: Exponential backoff multiplier

QoS Profiles
~~~~~~~~~~~~

The bridge uses optimized QoS profiles for different sensor types:

High-Frequency Sensors (IMU, Odometry)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   QoSProfile(
       reliability=ReliabilityPolicy.BEST_EFFORT,
       history=HistoryPolicy.KEEP_LAST,
       depth=20,
       durability=DurabilityPolicy.VOLATILE
   )

Standard Sensors (GPS, Temperature)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,
       history=HistoryPolicy.KEEP_LAST,
       depth=10,
       durability=DurabilityPolicy.VOLATILE
   )

Critical Sensors (Battery)
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,
       durability=DurabilityPolicy.TRANSIENT_LOCAL,
       history=HistoryPolicy.KEEP_LAST,
       depth=5
   )

This API reference provides comprehensive information for integrating with and extending the WebSocket Sensor Bridge functionality.
