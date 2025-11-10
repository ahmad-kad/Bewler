.. _sensor_bridge_migration:

ROS2 Migration Guide
====================

This guide outlines the transition from WebSocket-based sensor data to direct ROS2 DDS transport, including benefits, implementation strategies, and performance improvements.

Migration Benefits
------------------

Performance Improvements
~~~~~~~~~~~~~~~~~~~~~~~~~

**Latency Reduction**

Current WebSocket Bridge:
- JSON serialization: ~1-2ms
- WebSocket transport: ~2-3ms
- Python processing: ~1-2ms
- **Total: 4-7ms latency**

Direct ROS2 Transport:
- Binary serialization: ~0.1ms
- DDS transport: ~0.5ms
- C++ processing: ~0.2ms
- **Total: 0.8-1.3ms latency**

**Improvement: 75-85% latency reduction**

**Bandwidth Efficiency**

WebSocket JSON (IMU example):
- Message size: ~250 bytes
- Overhead: JSON structure + WebSocket framing
- Effective throughput: ~40KB/s per sensor

Direct ROS2 Binary:
- Message size: ~120 bytes
- Overhead: DDS headers only
- Effective throughput: ~100KB/s per sensor

**Improvement: 60% bandwidth reduction**

Reliability Enhancements
~~~~~~~~~~~~~~~~~~~~~~~~

**Connection Resilience**

WebSocket Bridge:
- Single connection point of failure
- Manual reconnection logic required
- Intermediate JSON parsing layer

Direct ROS2 DDS:
- Automatic peer discovery
- Built-in connection management
- Native ROS2 middleware reliability

**Data Integrity**

WebSocket Bridge:
- JSON parsing can fail silently
- No built-in data validation
- Manual error detection required

Direct ROS2 DDS:
- Type-safe message definitions
- Built-in serialization validation
- Automatic error detection and reporting

Migration Phases
----------------

Phase 1: Hybrid Operation (Recommended Starting Point)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Maintain WebSocket bridge while gradually migrating sensors:

.. code-block:: python

   class HybridSensorBridge(Node):
       """Bridge supporting both WebSocket and direct ROS2 sensors"""

       def __init__(self):
           super().__init__('hybrid_sensor_bridge')

           # Existing WebSocket connection for legacy sensors
           self.websocket_bridge = WebSocketSensorBridge()

           # New direct ROS2 subscriptions for migrated sensors
           self.imu_sub = self.create_subscription(
               Imu, '/imu/data', self.on_direct_imu, 10)

           # Publishers for WebSocket sensors (renamed to avoid conflicts)
           self.gps_pub = self.create_publisher(
               NavSatFix, '/gps/fix_legacy', 10)

           # Publishers for direct sensors
           self.imu_pub = self.create_publisher(
               Imu, '/imu/data', 10)

Phase 2: Sensor-by-Sensor Migration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Migrate sensors based on priority and hardware capabilities:

**High Priority (Migrate First):**
- IMU sensors (100Hz, critical for navigation)
- Wheel odometry (50Hz, motion estimation)
- GPS receivers (10Hz, global positioning)

**Medium Priority:**
- LiDAR data (high bandwidth, processing intensive)
- Camera metadata (moderate bandwidth, structured)
- Battery monitors (low bandwidth, critical safety)

**Low Priority:**
- Temperature sensors (low bandwidth, monitoring)
- Status indicators (very low bandwidth)

Phase 3: Full ROS2 Operation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Complete migration with optimized DDS configuration:

.. code-block:: xml

   <!-- CycloneDDS configuration for rover network -->
   <CycloneDDS>
     <Domain>
       <General>
         <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
         <AllowMulticast>true</AllowMulticast>
         <MaxMessageSize>65536</MaxMessageSize>
       </General>

       <Discovery>
         <ParticipantIndex>auto</ParticipantIndex>
         <MaxAutoParticipantIndex>20</MaxAutoParticipantIndex>
       </Discovery>
     </Domain>
   </CycloneDDS>

Implementation Strategies
-------------------------

Strategy 1: Raspberry Pi Sensor Nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Deploy ROS2 nodes directly on sensor hardware:

**Hardware Requirements:**
- Raspberry Pi Zero W or similar
- ROS2 Humble (micro-ROS for resource-constrained devices)
- WiFi connectivity for DDS communication

**Implementation:**

.. code-block:: python

   #!/usr/bin/env python3
   """
   IMU sensor node for Raspberry Pi
   """

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Imu
   from mpu6050 import MPU6050  # Hardware interface
   import time

   class RaspberryPiImuNode(Node):
       def __init__(self):
           super().__init__('rpi_imu_sensor')

           # Initialize hardware
           self.imu = MPU6050()

           # Publisher with optimized QoS
           qos_profile = QoSProfile(
               reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST,
               depth=10,
               durability=DurabilityPolicy.VOLATILE
           )

           self.publisher = self.create_publisher(
               Imu, '/imu/data', qos_profile)

           # High-frequency timer
           self.timer = self.create_timer(0.01, self.publish_reading)

           self.get_logger().info('Raspberry Pi IMU sensor initialized')

       def publish_reading(self):
           """Read IMU and publish ROS2 message"""
           try:
               # Read from hardware
               accel_data = self.imu.get_accel_data()
               gyro_data = self.imu.get_gyro_data()

               # Create ROS2 message
               msg = Imu()
               msg.header.stamp = self.get_clock().now().to_msg()
               msg.header.frame_id = 'imu_link'

               # Fill message
               msg.linear_acceleration.x = accel_data['x']
               msg.linear_acceleration.y = accel_data['y']
               msg.linear_acceleration.z = accel_data['z']

               msg.angular_velocity.x = gyro_data['x']
               msg.angular_velocity.y = gyro_data['y']
               msg.angular_velocity.z = gyro_data['z']

               # Publish
               self.publisher.publish(msg)

           except Exception as e:
               self.get_logger().error(f'IMU reading failed: {e}')

   def main():
       rclpy.init()
       node = RaspberryPiImuNode()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

Strategy 2: ESP32 Micro-ROS Nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For ultra-low-power sensors using ESP32:

**Advantages:**
- Extremely low power consumption
- WiFi connectivity built-in
- Micro-ROS support for resource constraints

**Implementation:**

.. code-block:: cpp

   // ESP32 IMU sensor with Micro-ROS
   #include <micro_ros_arduino.h>
   #include <MPU6050.h>

   rcl_publisher_t imu_publisher;
   sensor_msgs__msg__Imu imu_msg;
   MPU6050 mpu;

   void setup() {
       // Initialize Micro-ROS
       set_microros_wifi_transports("SSID", "PASSWORD", "192.168.1.100", 8888);
       delay(2000);

       allocator = rcl_get_default_allocator();
       rclc_support_init(&support, 0, NULL, &allocator);
       rclc_node_init_default(&node, "esp32_imu_sensor", "", &support);

       // Create publisher
       rclc_publisher_init_default(
           &imu_publisher,
           &node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
           "/imu/data");

       // Initialize sensor
       Wire.begin();
       mpu.initialize();
   }

   void loop() {
       // Read sensor data
       int16_t ax, ay, az, gx, gy, gz;
       mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

       // Convert to ROS2 message
       imu_msg.header.stamp.sec = millis() / 1000;
       imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

       imu_msg.linear_acceleration.x = ax / 16384.0 * 9.81;
       imu_msg.linear_acceleration.y = ay / 16384.0 * 9.81;
       imu_msg.linear_acceleration.z = az / 16384.0 * 9.81;

       imu_msg.angular_velocity.x = gx / 131.0 * M_PI / 180.0;
       imu_msg.angular_velocity.y = gy / 131.0 * M_PI / 180.0;
       imu_msg.angular_velocity.z = gz / 131.0 * M_PI / 180.0;

       // Publish
       rcl_publish(&imu_publisher, &imu_msg, NULL);

       rclc_spin_node_once(&node, 10);
       delay(10); // 100Hz
   }

Strategy 3: Arduino-ROS Bridge
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For legacy Arduino sensors:

.. code-block:: cpp

   // Arduino with rosserial
   #include <ros.h>
   #include <sensor_msgs/Imu.h>
   #include <Wire.h>
   #include <MPU6050.h>

   ros::NodeHandle nh;
   sensor_msgs::Imu imu_msg;
   ros::Publisher imu_pub("imu/data", &imu_msg);

   MPU6050 mpu;

   void setup() {
       nh.initNode();
       nh.advertise(imu_pub);

       Wire.begin();
       mpu.initialize();
   }

   void loop() {
       // Read sensor
       int16_t ax, ay, az, gx, gy, gz;
       mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

       // Fill message
       imu_msg.header.stamp = nh.now();
       imu_msg.header.frame_id = "imu_link";

       imu_msg.linear_acceleration.x = ax / 16384.0 * 9.81;
       imu_msg.linear_acceleration.y = ay / 16384.0 * 9.81;
       imu_msg.linear_acceleration.z = az / 16384.0 * 9.81;

       imu_msg.angular_velocity.x = gx / 131.0 * M_PI / 180.0;
       imu_msg.angular_velocity.y = gy / 131.0 * M_PI / 180.0;
       imu_msg.angular_velocity.z = gz / 131.0 * M_PI / 180.0;

       // Publish
       imu_pub.publish(&imu_msg);
       nh.spinOnce();

       delay(10); // 100Hz
   }

Network Configuration
---------------------

DDS Configuration for Rovers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Optimize DDS for mobile robotic networks:

.. code-block:: xml

   <!-- FAST-DDS configuration for rover -->
   <profiles>
     <participant profile_name="rover_participant">
       <rtps>
         <builtin>
           <discovery_config>
             <discoveryProtocol>BEST_EFFORT</discoveryProtocol>
             <leaseDuration>10</leaseDuration>
           </discovery_config>
         </builtin>

         <sendBuffers>
           <sendBufferSize>1048576</sendBufferSize>
         </sendBuffers>

         <throughputController>
           <bytesPerPeriod>1048576</bytesPerPeriod>
           <periodMillisecs>1000</periodMillisecs>
         </throughputController>
       </rtps>
     </participant>

     <data_writer profile_name="sensor_writer">
       <qos>
         <reliability>
           <kind>BEST_EFFORT_RELIABILITY_QOS</kind>
         </reliability>
         <durability>
           <kind>VOLATILE_DURABILITY_QOS</kind>
         </durability>
         <history>
           <kind>KEEP_LAST_HISTORY_QOS</kind>
           <depth>10</depth>
         </history>
       </qos>
     </data_writer>
   </profiles>

QoS Optimization
~~~~~~~~~~~~~~~~~

Configure QoS for sensor-specific requirements:

**High-Frequency Sensors (IMU, Wheel Odometry):**

.. code-block:: python

   # Minimal latency, best effort delivery
   high_freq_qos = QoSProfile(
       reliability=ReliabilityPolicy.BEST_EFFORT,
       history=HistoryPolicy.KEEP_LAST,
       depth=15,  # Buffer for network jitter
       durability=DurabilityPolicy.VOLATILE,
       liveliness=LivelinessPolicy.AUTOMATIC,
       liveliness_lease_duration=Duration(seconds=1)  # Fast failure detection
   )

**Navigation Sensors (GPS, Odometry):**

.. code-block:: python

   # Reliable delivery for critical navigation data
   nav_qos = QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,
       history=HistoryPolicy.KEEP_LAST,
       depth=10,
       durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Late joiner support
       liveliness=LivelinessPolicy.AUTOMATIC,
       liveliness_lease_duration=Duration(seconds=5)
   )

**Monitoring Sensors (Battery, Temperature):**

.. code-block:: python

   # Reliable with persistence for diagnostics
   monitor_qos = QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,
       history=HistoryPolicy.KEEP_LAST,
       depth=5,
       durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Always available
       liveliness=LivelinessPolicy.AUTOMATIC,
       liveliness_lease_duration=Duration(seconds=30)  # Longer timeout OK
   )

Migration Testing
-----------------

Performance Validation
~~~~~~~~~~~~~~~~~~~~~~~

Test performance improvements during migration:

.. code-block:: python

   class MigrationPerformanceTest(Node):
       """Test performance before and after migration"""

       def __init__(self):
           super().__init__('migration_test')

           # Subscribe to sensor data
           self.imu_sub = self.create_subscription(
               Imu, '/imu/data', self.imu_callback, 100)

           self.latencies = []
           self.message_count = 0
           self.start_time = time.time()

       def imu_callback(self, msg: Imu):
           """Measure message latency"""
           receive_time = time.time()
           send_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
           latency = receive_time - send_time

           self.latencies.append(latency)
           self.message_count += 1

           if self.message_count % 100 == 0:
               avg_latency = sum(self.latencies[-100:]) / 100
               self.get_logger().info(
                   f'Average latency (last 100): {avg_latency*1000:.1f}ms'
               )

Reliability Testing
~~~~~~~~~~~~~~~~~~~

Test connection reliability:

.. code-block:: python

   class ReliabilityTest(Node):
       """Test connection reliability during migration"""

       def __init__(self):
           super().__init__('reliability_test')

           self.message_counts = {}
           self.last_message_times = {}
           self.connection_failures = 0

           # Subscribe to all sensor topics
           sensor_topics = ['/imu/data', '/gps/fix', '/battery/status']
           for topic in sensor_topics:
               self.create_subscription(
                   topic, self.sensor_callback, 10,
                   callback_group=MutuallyExclusiveCallbackGroup())

       def sensor_callback(self, msg, topic_name):
           """Monitor sensor message reception"""
           current_time = time.time()

           if topic_name not in self.message_counts:
               self.message_counts[topic_name] = 0
               self.last_message_times[topic_name] = current_time

           self.message_counts[topic_name] += 1

           # Check for message gaps (missed messages)
           time_since_last = current_time - self.last_message_times[topic_name]
           expected_interval = 0.1  # 10Hz

           if time_since_last > expected_interval * 2:
               self.get_logger().warn(
                   f'Message gap on {topic_name}: {time_since_last:.1f}s'
               )
               self.connection_failures += 1

           self.last_message_times[topic_name] = current_time

Data Integrity Testing
~~~~~~~~~~~~~~~~~~~~~~~

Verify data correctness during migration:

.. code-block:: python

   class DataIntegrityTest(Node):
       """Verify data integrity during migration"""

       def __init__(self):
           super().__init__('data_integrity_test')

           self.imu_sub = self.create_subscription(
               Imu, '/imu/data', self.imu_validation_callback, 10)

           self.validation_stats = {
               'total_messages': 0,
               'invalid_messages': 0,
               'range_violations': 0,
               'timestamp_issues': 0
           }

       def imu_validation_callback(self, msg: Imu):
           """Validate IMU message integrity"""
           self.validation_stats['total_messages'] += 1

           # Check timestamp reasonableness
           current_time = self.get_clock().now().nanoseconds * 1e-9
           msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

           if abs(current_time - msg_time) > 5.0:  # 5 second tolerance
               self.validation_stats['timestamp_issues'] += 1
               self.get_logger().warn(f'Timestamp issue: {abs(current_time - msg_time):.1f}s')

           # Check accelerometer ranges
           accel_mag = (msg.linear_acceleration.x**2 +
                       msg.linear_acceleration.y**2 +
                       msg.linear_acceleration.z**2)**0.5

           if accel_mag > 50:  # Unreasonable acceleration
               self.validation_stats['range_violations'] += 1
               self.get_logger().warn(f'Unusual acceleration magnitude: {accel_mag:.1f} m/s²')

           # Check gyro ranges
           gyro_mag = (msg.angular_velocity.x**2 +
                      msg.angular_velocity.y**2 +
                      msg.angular_velocity.z**2)**0.5

           if gyro_mag > 20:  # Unreasonable angular velocity
               self.validation_stats['range_violations'] += 1
               self.get_logger().warn(f'Unusual angular velocity magnitude: {gyro_mag:.1f} rad/s')

Rollback Strategies
-------------------

Safe Migration with Rollback
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Implement rollback capability:

.. code-block:: python

   class MigrationController(Node):
       """Control sensor migration with rollback capability"""

       def __init__(self):
           super().__init__('migration_controller')

           # Migration state
           self.migration_state = {
               'imu': 'websocket',     # websocket, hybrid, direct
               'gps': 'websocket',
               'battery': 'websocket'
           }

           # Performance baselines
           self.baselines = {}

           # Timers for monitoring
           self.monitoring_timer = self.create_timer(5.0, self.monitor_migration)

       def migrate_sensor(self, sensor_name: str):
           """Migrate sensor to direct ROS2 with rollback"""
           try:
               self.get_logger().info(f'Starting migration of {sensor_name}')

               # Establish baseline performance
               self.baselines[sensor_name] = self.measure_performance(sensor_name)

               # Switch to hybrid mode
               self.enable_hybrid_mode(sensor_name)
               time.sleep(10)  # Allow stabilization

               # Verify hybrid performance
               if self.verify_performance(sensor_name):
                   # Switch to direct mode
                   self.enable_direct_mode(sensor_name)
                   time.sleep(10)

                   # Final verification
                   if self.verify_performance(sensor_name):
                       self.migration_state[sensor_name] = 'direct'
                       self.get_logger().info(f'✅ Migration successful for {sensor_name}')
                   else:
                       self.rollback_sensor(sensor_name)
               else:
                   self.rollback_sensor(sensor_name)

           except Exception as e:
               self.get_logger().error(f'Migration failed for {sensor_name}: {e}')
               self.rollback_sensor(sensor_name)

       def rollback_sensor(self, sensor_name: str):
           """Rollback sensor to WebSocket mode"""
           try:
               self.disable_direct_mode(sensor_name)
               self.disable_hybrid_mode(sensor_name)
               self.migration_state[sensor_name] = 'websocket'
               self.get_logger().info(f'🔄 Rolled back {sensor_name} to WebSocket mode')
           except Exception as e:
               self.get_logger().error(f'Rollback failed for {sensor_name}: {e}')

       def measure_performance(self, sensor_name: str) -> dict:
           """Measure current performance metrics"""
           # Implementation would collect latency, throughput, error rates
           return {}

       def verify_performance(self, sensor_name: str) -> bool:
           """Verify performance meets requirements"""
           current = self.measure_performance(sensor_name)
           baseline = self.baselines.get(sensor_name, {})

           # Check latency improvement
           if current.get('avg_latency', 0) > baseline.get('avg_latency', 0) * 1.1:
               return False

           # Check error rate
           if current.get('error_rate', 0) > 0.05:  # 5% max errors
               return False

           return True

Monitoring Migration Progress
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Track migration status:

.. code-block:: python

   def publish_migration_status(self):
       """Publish migration progress"""
       status_msg = String()
       status_msg.data = json.dumps({
           'migration_state': self.migration_state,
           'performance_baselines': self.baselines,
           'timestamp': time.time()
       })
       self.migration_status_pub.publish(status_msg)

Success Metrics
---------------

Performance Targets
~~~~~~~~~~~~~~~~~~~

**Latency Improvements:**
- IMU: < 1ms (from 4-7ms)
- GPS: < 5ms (from 10-15ms)
- Battery: < 10ms (from 20-30ms)

**Reliability Targets:**
- Message loss rate: < 0.1%
- Connection recovery time: < 5 seconds
- Data integrity: 100%

**Resource Usage:**
- CPU reduction: 50-70%
- Memory usage: < 80% of WebSocket bridge
- Network bandwidth: 60% reduction

Migration Checklist
-------------------

Pre-Migration Preparation
~~~~~~~~~~~~~~~~~~~~~~~~~

- [ ] Hardware assessment complete for all sensors
- [ ] Network infrastructure tested for DDS compatibility
- [ ] ROS2 installation verified on target devices
- [ ] Performance baselines established
- [ ] Rollback procedures documented
- [ ] Testing environments prepared

Migration Execution
~~~~~~~~~~~~~~~~~~~

- [ ] Hybrid bridge deployed and tested
- [ ] High-priority sensors migrated first (IMU, GPS, odometry)
- [ ] Performance monitoring active during migration
- [ ] Gradual rollout with rollback capability
- [ ] Integration testing at each phase
- [ ] Documentation updated for new architecture

Post-Migration Validation
~~~~~~~~~~~~~~~~~~~~~~~~~

- [ ] All performance targets achieved
- [ ] System stability verified under load
- [ ] Error rates within acceptable limits
- [ ] Network utilization optimized
- [ ] Team trained on new architecture
- [ ] Monitoring and alerting configured

This migration guide provides a comprehensive roadmap for transitioning from WebSocket-based sensor data to direct ROS2 DDS transport, ensuring reliability, performance, and maintainability improvements.
