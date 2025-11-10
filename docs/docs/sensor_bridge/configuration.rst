.. _sensor_bridge_configuration:

Configuration Guide
===================

This guide covers all configuration options for the WebSocket Sensor Bridge, including parameter descriptions, sensor-specific settings, and optimization recommendations.

Parameter Reference
-------------------

Core Parameters
~~~~~~~~~~~~~~~

**websocket_url** (string, default: "ws://localhost:8080")
   WebSocket server URL for sensor data connection.

   **Example:** ``ws://192.168.1.100:9090``

**reconnect_interval** (float, default: 3.0)
   Base delay in seconds between reconnection attempts.

   **Range:** 0.1 - 60.0 seconds

**max_reconnect_attempts** (int, default: 10)
   Maximum number of reconnection attempts before giving up.

   **Range:** 1 - 100 attempts

**connection_timeout** (float, default: 5.0)
   Timeout in seconds for WebSocket connection attempts.

   **Range:** 1.0 - 30.0 seconds

Monitoring Parameters
~~~~~~~~~~~~~~~~~~~~~

**health_check_interval** (float, default: 1.0)
   Frequency in seconds for connection health checks.

   **Range:** 0.1 - 10.0 seconds
   **Impact:** Lower values provide faster failure detection but increase CPU usage

**message_timeout** (float, default: 10.0)
   Timeout in seconds for detecting missing messages.

   **Range:** 1.0 - 300.0 seconds
   **Impact:** Shorter timeouts detect issues faster but may cause false positives

**enable_graceful_degradation** (bool, default: true)
   Enable fallback modes during connection failures.

   **Impact:** When enabled, system continues with last known values during outages

Advanced Parameters
~~~~~~~~~~~~~~~~~~~

**max_reconnect_interval** (float, default: 60.0)
   Maximum delay in seconds for exponential backoff.

   **Range:** 5.0 - 300.0 seconds

**reconnect_backoff_multiplier** (float, default: 1.5)
   Multiplier for exponential backoff calculation.

   **Range:** 1.1 - 3.0
   **Example:** With base interval 3.0 and multiplier 1.5:
   Attempt 1: 3.0s, Attempt 2: 4.5s, Attempt 3: 6.75s, etc.

Sensor Configuration
--------------------

Sensor Enablement
~~~~~~~~~~~~~~~~~~

Each sensor can be individually enabled or disabled:

.. code-block:: yaml

   sensor_bridge:
     sensors:
       imu:
         enabled: true          # Enable IMU sensor processing
         topic_name: "/imu/data"
         update_rate: 100.0

       gps:
         enabled: true          # Enable GPS sensor processing
         topic_name: "/gps/fix"
         update_rate: 10.0

       battery:
         enabled: false         # Disable battery sensor processing
         topic_name: "/battery/status"
         update_rate: 1.0

Topic Name Customization
~~~~~~~~~~~~~~~~~~~~~~~~~

Customize ROS2 topic names to avoid conflicts or match existing systems:

.. code-block:: yaml

   sensor_bridge:
     sensors:
       imu:
         enabled: true
         topic_name: "/sensors/imu"      # Custom topic name
         update_rate: 100.0

       gps:
         enabled: true
         topic_name: "/navigation/gps"   # Custom topic name
         update_rate: 10.0

QoS Profile Optimization
------------------------

Understanding QoS Profiles
~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS2 QoS (Quality of Service) profiles control message delivery behavior:

**Reliability:**
- ``BEST_EFFORT``: Fast delivery, may drop messages under load
- ``RELIABLE``: Guaranteed delivery, may be slower

**History:**
- ``KEEP_LAST``: Keep N most recent messages
- ``KEEP_ALL``: Keep all messages (use with caution)

**Durability:**
- ``VOLATILE``: Messages lost if no subscriber
- ``TRANSIENT_LOCAL``: Messages persist for late-joining subscribers

Default QoS Profiles
~~~~~~~~~~~~~~~~~~~~

The bridge uses optimized QoS profiles for different sensor types:

**High-Frequency Sensors (IMU, Wheel Odometry):**

.. code-block:: yaml

   # Optimized for low latency, high throughput
   qos_profile:
     reliability: BEST_EFFORT
     history: KEEP_LAST
     depth: 20
     durability: VOLATILE

**Standard Sensors (GPS, Temperature):**

.. code-block:: yaml

   # Balanced reliability and performance
   qos_profile:
     reliability: RELIABLE
     history: KEEP_LAST
     depth: 10
     durability: VOLATILE

**Critical Sensors (Battery):**

.. code-block:: yaml

   # Maximum reliability for safety-critical data
   qos_profile:
     reliability: RELIABLE
     history: KEEP_LAST
     depth: 5
     durability: TRANSIENT_LOCAL

Custom QoS Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~

Override default QoS settings for specific requirements:

.. code-block:: yaml

   sensor_bridge:
     sensors:
       imu:
         enabled: true
         topic_name: "/imu/data"
         qos_profile:
           reliability: RELIABLE      # Override default BEST_EFFORT
           history: KEEP_LAST
           depth: 50                  # Larger buffer
           durability: VOLATILE

       battery:
         enabled: true
         topic_name: "/battery/status"
         qos_profile:
           reliability: RELIABLE
           history: KEEP_LAST
           depth: 10                  # Keep more history
           durability: TRANSIENT_LOCAL

Configuration Files
-------------------

Main Configuration File
~~~~~~~~~~~~~~~~~~~~~~~

Location: ``config/sensor_bridge.yaml``

Complete configuration example:

.. code-block:: yaml

   # WebSocket Sensor Bridge Configuration
   # URC 2026 Mars Rover

   sensor_bridge:
     # WebSocket connection settings
     websocket_url: "ws://localhost:8080"
     reconnect_interval: 3.0
     max_reconnect_attempts: 10
     connection_timeout: 5.0

     # Monitoring settings
     health_check_interval: 1.0
     message_timeout: 10.0
     enable_graceful_degradation: true

     # Advanced settings
     max_reconnect_interval: 60.0
     reconnect_backoff_multiplier: 1.5

     # Sensor configurations
     sensors:
       imu:
         enabled: true
         topic_name: "/imu/data"
         update_rate: 100.0

       gps:
         enabled: true
         topic_name: "/gps/fix"
         update_rate: 10.0

       battery:
         enabled: true
         topic_name: "/battery/status"
         update_rate: 1.0

       wheel_odom:
         enabled: true
         topic_name: "/wheel/odom"
         update_rate: 50.0

       temperature:
         enabled: true
         topic_name: "/temperature/data"
         update_rate: 5.0

Launch File Parameters
~~~~~~~~~~~~~~~~~~~~~~

Override configuration via launch files:

.. code-block:: xml

   <!-- Launch file with parameter overrides -->
   <launch>
     <include file="$(find-pkg-share autonomy_sensor_bridge)/launch/sensor_bridge.launch.py">
       <arg name="websocket_url" value="ws://sensor-hub:8080"/>
       <arg name="log_level" value="debug"/>
     </include>
   </launch>

Runtime Parameter Changes
~~~~~~~~~~~~~~~~~~~~~~~~~

Modify parameters at runtime using ROS2 commands:

.. code-block:: bash

   # Change WebSocket URL at runtime
   ros2 param set /websocket_sensor_bridge websocket_url "ws://192.168.1.100:9090"

   # Adjust reconnection settings
   ros2 param set /websocket_sensor_bridge reconnect_interval 5.0
   ros2 param set /websocket_sensor_bridge max_reconnect_attempts 15

   # List all parameters
   ros2 param list /websocket_sensor_bridge

Environment-Specific Configurations
-----------------------------------

Development Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~

For development and testing environments:

.. code-block:: yaml

   sensor_bridge:
     # Local WebSocket server
     websocket_url: "ws://localhost:8080"

     # Aggressive reconnection for testing
     reconnect_interval: 1.0
     max_reconnect_attempts: 5

     # Verbose monitoring
     health_check_interval: 0.5
     message_timeout: 5.0

     # Enable all sensors for testing
     sensors:
       imu:
         enabled: true
         topic_name: "/imu/data"
       gps:
         enabled: true
         topic_name: "/gps/fix"
       battery:
         enabled: true
         topic_name: "/battery/status"
       wheel_odom:
         enabled: true
         topic_name: "/wheel/odom"
       temperature:
         enabled: true
         topic_name: "/temperature/data"

Competition Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~

For competition environments:

.. code-block:: yaml

   sensor_bridge:
     # Rover network WebSocket server
     websocket_url: "ws://10.0.0.100:8080"

     # Robust reconnection for competition
     reconnect_interval: 5.0
     max_reconnect_attempts: 20
     max_reconnect_interval: 120.0

     # Stable monitoring
     health_check_interval: 2.0
     message_timeout: 15.0
     enable_graceful_degradation: true

     # Optimized QoS for competition
     sensors:
       imu:
         enabled: true
         topic_name: "/imu/data"
         qos_profile:
           reliability: BEST_EFFORT
           depth: 30  # Extra buffer for competition stress

       gps:
         enabled: true
         topic_name: "/gps/fix"
         qos_profile:
           reliability: RELIABLE
           durability: TRANSIENT_LOCAL  # Critical for navigation

       battery:
         enabled: true
         topic_name: "/battery/status"
         qos_profile:
           reliability: RELIABLE
           durability: TRANSIENT_LOCAL  # Always available for safety

Simulation Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~

For Gazebo simulation environments:

.. code-block:: yaml

   sensor_bridge:
     # Simulation WebSocket endpoint
     websocket_url: "ws://localhost:9090"

     # Fast reconnection for simulation
     reconnect_interval: 0.5
     max_reconnect_attempts: 3

     # Simulation-appropriate timeouts
     health_check_interval: 0.5
     message_timeout: 3.0

     # Simulation sensor topics
     sensors:
       imu:
         enabled: true
         topic_name: "/sensors/imu"
       gps:
         enabled: true
         topic_name: "/sensors/gps"
       battery:
         enabled: true
         topic_name: "/sensors/battery"

Performance Tuning
------------------

High-Frequency Optimization
~~~~~~~~~~~~~~~~~~~~~~~~~~~

For IMU and wheel odometry sensors:

.. code-block:: yaml

   sensor_bridge:
     sensors:
       imu:
         enabled: true
         topic_name: "/imu/data"
         qos_profile:
           reliability: BEST_EFFORT    # Minimize latency
           history: KEEP_LAST
           depth: 50                   # Large buffer for 100Hz data
           durability: VOLATILE

       wheel_odom:
         enabled: true
         topic_name: "/wheel/odom"
         qos_profile:
           reliability: BEST_EFFORT
           history: KEEP_LAST
           depth: 30
           durability: VOLATILE

Reliability Optimization
~~~~~~~~~~~~~~~~~~~~~~~~

For critical navigation and safety sensors:

.. code-block:: yaml

   sensor_bridge:
     sensors:
       gps:
         enabled: true
         topic_name: "/gps/fix"
         qos_profile:
           reliability: RELIABLE       # Guaranteed delivery
           history: KEEP_LAST
           depth: 20                   # Sufficient for 10Hz data
           durability: TRANSIENT_LOCAL # Late joiner support

       battery:
         enabled: true
         topic_name: "/battery/status"
         qos_profile:
           reliability: RELIABLE
           history: KEEP_LAST
           depth: 10
           durability: TRANSIENT_LOCAL # Always available

Resource-Constrained Optimization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For devices with limited resources:

.. code-block:: yaml

   sensor_bridge:
     # Reduce monitoring overhead
     health_check_interval: 5.0      # Less frequent checks
     message_timeout: 30.0           # Longer timeout acceptable

     sensors:
       temperature:
         enabled: true
         topic_name: "/temperature/data"
         qos_profile:
           reliability: RELIABLE
           history: KEEP_LAST
           depth: 5                   # Minimal buffer
           durability: VOLATILE

Network Optimization
~~~~~~~~~~~~~~~~~~~~

For bandwidth-constrained networks:

.. code-block:: yaml

   sensor_bridge:
     # Reduce update frequencies
     sensors:
       gps:
         update_rate: 5.0             # Reduce GPS rate
       temperature:
         update_rate: 1.0             # Reduce temperature rate

       battery:
         qos_profile:
           depth: 3                  # Smaller history

Configuration Validation
------------------------

Automatic Validation
~~~~~~~~~~~~~~~~~~~~~

The bridge validates configuration on startup:

**Parameter Validation:**
- WebSocket URL format checking
- Numeric parameter range validation
- Sensor topic name conflicts detection

**Sensor Validation:**
- Required field presence checking
- QoS parameter compatibility validation
- Topic name uniqueness verification

Manual Validation
~~~~~~~~~~~~~~~~~

Validate configuration manually:

.. code-block:: bash

   # Check parameter values
   ros2 param get /websocket_sensor_bridge websocket_url
   ros2 param get /websocket_sensor_bridge reconnect_interval

   # Verify topic publishing
   ros2 topic list | grep sensor_bridge
   ros2 topic hz /imu/data

   # Check QoS settings
   ros2 topic info /imu/data

Configuration Troubleshooting
-----------------------------

Common Configuration Issues
~~~~~~~~~~~~~~~~~~~~~~~~~~~

**WebSocket Connection Fails:**

.. code-block:: bash

   # Check URL format
   ros2 param get /websocket_sensor_bridge websocket_url

   # Test WebSocket connectivity
   curl -I http://websocket-server:8080

   # Verify firewall settings
   sudo ufw status

**Sensors Not Publishing:**

.. code-block:: bash

   # Check sensor enablement
   ros2 param get /websocket_sensor_bridge sensors.imu.enabled

   # Verify topic names
   ros2 topic list | grep imu

   # Check for configuration errors in logs
   ros2 run autonomy_sensor_bridge sensor_bridge_node 2>&1 | grep ERROR

**High Latency or Dropped Messages:**

.. code-block:: bash

   # Check QoS settings
   ros2 topic info /imu/data

   # Monitor message rates
   ros2 topic hz /imu/data

   # Check system resources
   top -p $(pgrep -f sensor_bridge)

**Memory Usage Issues:**

.. code-block:: bash

   # Monitor memory usage
   ps aux | grep sensor_bridge

   # Check QoS history depths
   ros2 param get /websocket_sensor_bridge sensors.imu.qos_profile.depth

   # Reduce buffer sizes if needed
   ros2 param set /websocket_sensor_bridge sensors.imu.qos_profile.depth 10

Configuration Best Practices
----------------------------

1. **Use descriptive topic names** that match your system's naming conventions
2. **Configure QoS appropriately** for each sensor's requirements and network conditions
3. **Test configurations** in development before deploying to production
4. **Document custom configurations** for team maintenance
5. **Use parameter overrides** in launch files for environment-specific settings
6. **Monitor performance** and adjust QoS settings based on observed behavior
7. **Plan for network conditions** when configuring timeouts and buffer sizes
8. **Implement graceful degradation** for critical systems

This configuration guide provides comprehensive information for optimizing the WebSocket Sensor Bridge for your specific use case and environment.
