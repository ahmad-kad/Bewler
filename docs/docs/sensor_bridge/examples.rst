.. _sensor_bridge_examples:

Code Examples
=============

This section provides practical code examples for integrating with and using the WebSocket Sensor Bridge in various scenarios.

Basic Sensor Data Reading
-------------------------

Simple IMU Reader
~~~~~~~~~~~~~~~~~

.. code-block:: python

   #!/usr/bin/env python3
   """
   Basic IMU data reader for WebSocket Sensor Bridge
   """

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Imu

   class SimpleImuReader(Node):
       def __init__(self):
           super().__init__('simple_imu_reader')

           # Subscribe to IMU data from sensor bridge
           self.subscription = self.create_subscription(
               Imu,
               '/imu/data',  # Topic published by sensor bridge
               self.imu_callback,
               10
           )

           self.get_logger().info('IMU reader initialized')

       def imu_callback(self, msg: Imu):
           # Extract IMU data
           accel_x = msg.linear_acceleration.x
           accel_y = msg.linear_acceleration.y
           accel_z = msg.linear_acceleration.z

           gyro_x = msg.angular_velocity.x
           gyro_y = msg.angular_velocity.y
           gyro_z = msg.angular_velocity.z

           # Simple logging
           self.get_logger().info(
               f'IMU: Accel({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}) '
               f'Gyro({gyro_x:.3f}, {gyro_y:.3f}, {gyro_z:.3f})'
           )

   def main(args=None):
       rclpy.init(args=args)

       reader = SimpleImuReader()

       try:
           rclpy.spin(reader)
       except KeyboardInterrupt:
           pass
       finally:
           reader.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()

GPS Navigation Integration
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   #!/usr/bin/env python3
   """
   GPS navigation node integrated with sensor bridge
   """

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import NavSatFix
   import math

   class GpsNavigationNode(Node):
       def __init__(self):
           super().__init__('gps_navigation')

           # Subscribe to GPS data
           self.gps_sub = self.create_subscription(
               NavSatFix,
               '/gps/fix',
               self.gps_callback,
               10
           )

           # Store current position
           self.current_lat = None
           self.current_lon = None
           self.current_alt = None

           # Target position (example: URC 2026 competition site)
           self.target_lat = 37.7749
           self.target_lon = -122.4194

           self.get_logger().info('GPS navigation node initialized')

       def gps_callback(self, msg: NavSatFix):
           """Process GPS fix data"""
           self.current_lat = msg.latitude
           self.current_lon = msg.longitude
           self.current_alt = msg.altitude

           # Calculate distance to target
           distance = self.calculate_distance(
               self.current_lat, self.current_lon,
               self.target_lat, self.target_lon
           )

           bearing = self.calculate_bearing(
               self.current_lat, self.current_lon,
               self.target_lat, self.target_lon
           )

           self.get_logger().info(
               '.6f'
               '.2f'
           )

       def calculate_distance(self, lat1, lon1, lat2, lon2):
           """Calculate distance between two GPS coordinates in meters"""
           R = 6371000  # Earth's radius in meters

           lat1_rad = math.radians(lat1)
           lat2_rad = math.radians(lat2)
           delta_lat = math.radians(lat2 - lat1)
           delta_lon = math.radians(lon2 - lon1)

           a = (math.sin(delta_lat/2) * math.sin(delta_lat/2) +
                math.cos(lat1_rad) * math.cos(lat2_rad) *
                math.sin(delta_lon/2) * math.sin(delta_lon/2))
           c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

           return R * c

       def calculate_bearing(self, lat1, lon1, lat2, lon2):
           """Calculate bearing from point 1 to point 2"""
           lat1_rad = math.radians(lat1)
           lat2_rad = math.radians(lat2)
           delta_lon = math.radians(lon2 - lon1)

           y = math.sin(delta_lon) * math.cos(lat2_rad)
           x = (math.cos(lat1_rad) * math.sin(lat2_rad) -
                math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))

           bearing_rad = math.atan2(y, x)
           return math.degrees(bearing_rad) % 360

   def main(args=None):
       rclpy.init(args=args)
       node = GpsNavigationNode()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

WebSocket Sensor Client
-----------------------

Python WebSocket Client
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   #!/usr/bin/env python3
   """
   WebSocket client for sending sensor data to the bridge
   """

   import asyncio
   import websockets
   import json
   import time
   import random
   from typing import Dict, Any

   class SensorClient:
       def __init__(self, websocket_url: str = "ws://localhost:8080"):
           self.websocket_url = websocket_url
           self.websocket = None
           self.connected = False

       async def connect(self):
           """Connect to WebSocket sensor bridge"""
           try:
               self.websocket = await websockets.connect(self.websocket_url)
               self.connected = True
               print(f"Connected to sensor bridge at {self.websocket_url}")
           except Exception as e:
               print(f"Failed to connect: {e}")
               self.connected = False

       async def send_sensor_data(self, sensor_data: Dict[str, Any]):
           """Send sensor data to bridge"""
           if not self.connected or not self.websocket:
               print("Not connected to sensor bridge")
               return False

           try:
               # Add timestamp if not present
               if 'timestamp' not in sensor_data:
                   sensor_data['timestamp'] = time.time()

               message = json.dumps(sensor_data)
               await self.websocket.send(message)
               print(f"Sent sensor data: {len(message)} bytes")
               return True
           except Exception as e:
               print(f"Failed to send data: {e}")
               return False

       async def generate_and_send_data(self):
           """Generate and send simulated sensor data"""
           while self.connected:
               # Generate IMU data
               imu_data = {
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
               }

               # Generate GPS data
               gps_data = {
                   "lat": 37.7749 + random.gauss(0, 0.0001),
                   "lon": -122.4194 + random.gauss(0, 0.0001),
                   "altitude": 10.5 + random.gauss(0, 0.1)
               }

               # Generate battery data
               battery_data = {
                   "voltage": 12.3 + random.gauss(0, 0.1),
                   "current": 2.1 + random.gauss(0, 0.1),
                   "percentage": max(0, min(100, 85.0 + random.gauss(0, 1.0))),
                   "temperature": 35.0 + random.gauss(0, 2.0)
               }

               # Package all sensor data
               sensor_message = {
                   "timestamp": time.time(),
                   "sensors": {
                       "imu": imu_data,
                       "gps": gps_data,
                       "battery": battery_data
                   }
               }

               await self.send_sensor_data(sensor_message)

               # Send at 10Hz
               await asyncio.sleep(0.1)

       async def run(self):
           """Main client loop"""
           await self.connect()

           if self.connected:
               try:
                   await self.generate_and_send_data()
               except KeyboardInterrupt:
                   print("Stopping sensor client...")
               finally:
                   if self.websocket:
                       await self.websocket.close()
                       self.connected = False

   async def main():
       client = SensorClient()
       await client.run()

   if __name__ == '__main__':
       asyncio.run(main())

JavaScript WebSocket Client
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: javascript

   // WebSocket sensor client for browsers or Node.js
   class SensorBridgeClient {
       constructor(websocketUrl = 'ws://localhost:8080') {
           this.websocketUrl = websocketUrl;
           this.websocket = null;
           this.connected = false;
           this.reconnectInterval = 3000; // 3 seconds
       }

       connect() {
           try {
               this.websocket = new WebSocket(this.websocketUrl);

               this.websocket.onopen = (event) => {
                   console.log('Connected to sensor bridge');
                   this.connected = true;
                   this.startSendingData();
               };

               this.websocket.onmessage = (event) => {
                   console.log('Received:', event.data);
               };

               this.websocket.onclose = (event) => {
                   console.log('Disconnected from sensor bridge');
                   this.connected = false;
                   // Auto-reconnect
                   setTimeout(() => this.connect(), this.reconnectInterval);
               };

               this.websocket.onerror = (error) => {
                   console.error('WebSocket error:', error);
               };

           } catch (error) {
               console.error('Failed to connect:', error);
               setTimeout(() => this.connect(), this.reconnectInterval);
           }
       }

       sendSensorData(sensorData) {
           if (!this.connected || !this.websocket) {
               console.warn('Not connected to sensor bridge');
               return false;
           }

           try {
               // Add timestamp if not present
               if (!sensorData.timestamp) {
                   sensorData.timestamp = Date.now() / 1000;
               }

               const message = JSON.stringify(sensorData);
               this.websocket.send(message);
               console.log(`Sent ${message.length} bytes of sensor data`);
               return true;
           } catch (error) {
               console.error('Failed to send sensor data:', error);
               return false;
           }
       }

       generateImuData() {
           return {
               accel: {
                   x: (Math.random() - 0.5) * 0.2,      // ±0.1 m/s²
                   y: (Math.random() - 0.5) * 0.2,
                   z: 9.81 + (Math.random() - 0.5) * 0.02 // ~9.81 m/s²
               },
               gyro: {
                   x: (Math.random() - 0.5) * 0.02,     // ±0.01 rad/s
                   y: (Math.random() - 0.5) * 0.02,
                   z: (Math.random() - 0.5) * 0.02
               }
           };
       }

       generateGpsData() {
           return {
               lat: 37.7749 + (Math.random() - 0.5) * 0.0002,  // Small variation
               lon: -122.4194 + (Math.random() - 0.5) * 0.0002,
               altitude: 10.5 + (Math.random() - 0.5) * 0.2
           };
       }

       generateBatteryData() {
           return {
               voltage: 12.3 + (Math.random() - 0.5) * 0.2,
               current: 2.1 + (Math.random() - 0.5) * 0.2,
               percentage: Math.max(0, Math.min(100, 85 + (Math.random() - 0.5) * 2)),
               temperature: 35 + (Math.random() - 0.5) * 4
           };
       }

       startSendingData() {
           if (!this.connected) return;

           // Send data every 100ms (10Hz)
           this.dataInterval = setInterval(() => {
               const sensorMessage = {
                   timestamp: Date.now() / 1000,
                   sensors: {
                       imu: this.generateImuData(),
                       gps: this.generateGpsData(),
                       battery: this.generateBatteryData()
                   }
               };

               this.sendSensorData(sensorMessage);
           }, 100);
       }

       disconnect() {
           if (this.dataInterval) {
               clearInterval(this.dataInterval);
           }
           if (this.websocket) {
               this.websocket.close();
           }
       }
   }

   // Usage
   const client = new SensorBridgeClient();
   client.connect();

   // Cleanup on exit (Node.js)
   process.on('SIGINT', () => {
       console.log('Shutting down sensor client...');
       client.disconnect();
       process.exit(0);
   });

Bridge Monitoring
-----------------

Health Monitor Node
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   #!/usr/bin/env python3
   """
   Sensor bridge health monitoring node
   """

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
   import json
   import time

   class BridgeHealthMonitor(Node):
       def __init__(self):
           super().__init__('bridge_health_monitor')

           # Subscribe to bridge monitoring topics
           self.status_sub = self.create_subscription(
               String,
               '/sensor_bridge/status',
               self.status_callback,
               10
           )

           self.metrics_sub = self.create_subscription(
               String,
               '/sensor_bridge/metrics',
               self.metrics_callback,
               10
           )

           self.diagnostics_sub = self.create_subscription(
               DiagnosticArray,
               '/diagnostics',
               self.diagnostics_callback,
               10
           )

           # Alert thresholds
           self.alerts = {
               'connection_failures': {'threshold': 5, 'alerted': False},
               'low_success_rate': {'threshold': 95.0, 'alerted': False},
               'high_processing_time': {'threshold': 0.005, 'alerted': False}
           }

           # Health status
           self.last_status_update = time.time()
           self.connection_state = 'unknown'

           self.get_logger().info('Bridge health monitor initialized')

       def status_callback(self, msg: String):
           """Monitor connection status"""
           try:
               status = json.loads(msg.data)
               self.connection_state = status['connection_state']
               self.last_status_update = time.time()

               if self.connection_state != 'connected':
                   self.get_logger().warn(
                       f'Bridge connection issue: {self.connection_state}'
                   )
               else:
                   self.get_logger().debug('Bridge connection healthy')

           except json.JSONDecodeError as e:
               self.get_logger().error(f'Failed to parse status: {e}')

       def metrics_callback(self, msg: String):
           """Monitor performance metrics"""
           try:
               metrics = json.loads(msg.data)

               # Calculate success rate
               messages_received = metrics.get('messages_received', 0)
               messages_processed = metrics.get('messages_processed', 0)

               if messages_received > 0:
                   success_rate = (messages_processed / messages_received) * 100

                   # Check success rate alert
                   if success_rate < self.alerts['low_success_rate']['threshold']:
                       if not self.alerts['low_success_rate']['alerted']:
                           self.get_logger().error(
                               '.1f'
                           )
                           self.alerts['low_success_rate']['alerted'] = True
                   else:
                       self.alerts['low_success_rate']['alerted'] = False

               # Check processing time
               processing_time = metrics.get('avg_processing_time', 0)
               if processing_time > self.alerts['high_processing_time']['threshold']:
                   if not self.alerts['high_processing_time']['alerted']:
                       self.get_logger().warn(
                           '.3f'
                       )
                       self.alerts['high_processing_time']['alerted'] = True
               else:
                   self.alerts['high_processing_time']['alerted'] = False

               # Log metrics periodically
               self.get_logger().debug(
                   '.1f'
               )

           except json.JSONDecodeError as e:
               self.get_logger().error(f'Failed to parse metrics: {e}')

       def diagnostics_callback(self, msg: DiagnosticArray):
           """Process ROS2 diagnostics"""
           for status in msg.status:
               if 'WebSocket Sensor Bridge' in status.name:
                   level = status.level
                   message = status.message

                   if level == DiagnosticStatus.ERROR:
                       self.get_logger().error(f'Bridge diagnostic error: {message}')
                   elif level == DiagnosticStatus.WARN:
                       self.get_logger().warn(f'Bridge diagnostic warning: {message}')

   def main(args=None):
       rclpy.init(args=args)
       monitor = BridgeHealthMonitor()
       rclpy.spin(monitor)
       monitor.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

Integration with Navigation Stack
----------------------------------

Navigation Integration Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   #!/usr/bin/env python3
   """
   Example navigation node that integrates sensor bridge data
   """

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Imu, NavSatFix
   from nav_msgs.msg import Odometry
   from geometry_msgs.msg import TwistStamped
   import math

   class IntegratedNavigationNode(Node):
       def __init__(self):
           super().__init__('integrated_navigation')

           # Sensor bridge subscriptions
           self.imu_sub = self.create_subscription(
               Imu, '/imu/data', self.imu_callback, 10)

           self.gps_sub = self.create_subscription(
               NavSatFix, '/gps/fix', self.gps_callback, 10)

           self.odom_sub = self.create_subscription(
               Odometry, '/wheel/odom', self.odom_callback, 10)

           # Publishers for integrated navigation
           self.velocity_pub = self.create_publisher(
               TwistStamped, '/integrated_velocity', 10)

           # State variables
           self.imu_data = None
           self.gps_data = None
           self.odom_data = None

           # Integration timer (50Hz)
           self.integration_timer = self.create_timer(
               0.02, self.integration_callback)

           self.get_logger().info('Integrated navigation node initialized')

       def imu_callback(self, msg: Imu):
           """Store latest IMU data"""
           self.imu_data = {
               'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
               'accel': {
                   'x': msg.linear_acceleration.x,
                   'y': msg.linear_acceleration.y,
                   'z': msg.linear_acceleration.z
               },
               'gyro': {
                   'x': msg.angular_velocity.x,
                   'y': msg.angular_velocity.y,
                   'z': msg.angular_velocity.z
               }
           }

       def gps_callback(self, msg: NavSatFix):
           """Store latest GPS data"""
           self.gps_data = {
               'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
               'lat': msg.latitude,
               'lon': msg.longitude,
               'alt': msg.altitude,
               'status': msg.status.status
           }

       def odom_callback(self, msg: Odometry):
           """Store latest odometry data"""
           self.odom_data = {
               'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
               'x': msg.pose.pose.position.x,
               'y': msg.pose.pose.position.y,
               'vx': msg.twist.twist.linear.x,
               'vy': msg.twist.twist.linear.y,
               'omega': msg.twist.twist.angular.z
           }

       def integration_callback(self):
           """Integrate sensor data for navigation"""
           if not all([self.imu_data, self.gps_data, self.odom_data]):
               return  # Not all sensors have data yet

           # Simple sensor fusion example
           # In practice, you'd use EKF or similar

           # Use odometry for primary velocity estimate
           linear_x = self.odom_data['vx']
           linear_y = self.odom_data['vy']
           angular_z = self.odom_data['omega']

           # Use IMU to detect anomalies (high acceleration might indicate issues)
           accel_magnitude = math.sqrt(
               self.imu_data['accel']['x']**2 +
               self.imu_data['accel']['y']**2 +
               self.imu_data['accel']['z']**2
           )

           # GPS for position reference
           lat = self.gps_data['lat']
           lon = self.gps_data['lon']

           # Publish integrated velocity
           velocity_msg = TwistStamped()
           velocity_msg.header.stamp = self.get_clock().now().to_msg()
           velocity_msg.header.frame_id = 'base_link'

           velocity_msg.twist.linear.x = linear_x
           velocity_msg.twist.linear.y = linear_y
           velocity_msg.twist.angular.z = angular_z

           self.velocity_pub.publish(velocity_msg)

           # Log integrated state
           self.get_logger().info(
               '.2f'
               '.6f'
           )

   def main(args=None):
       rclpy.init(args=args)
       node = IntegratedNavigationNode()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

Launch File Integration
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: xml

   <!-- Complete launch file example -->
   <launch>
       <!-- Sensor Bridge -->
       <include file="$(find-pkg-share autonomy_sensor_bridge)/launch/sensor_bridge.launch.py">
           <arg name="websocket_url" value="ws://sensor-hub:8080"/>
           <arg name="log_level" value="info"/>
       </include>

       <!-- Navigation Integration -->
       <node pkg="autonomy_sensor_bridge" exec="integrated_navigation_node.py" name="integrated_navigation">
           <param name="use_sim_time" value="false"/>
       </node>

       <!-- Bridge Health Monitor -->
       <node pkg="autonomy_sensor_bridge" exec="bridge_health_monitor.py" name="bridge_monitor">
           <param name="alert_threshold_success_rate" value="95.0"/>
           <param name="alert_threshold_processing_time" value="0.005"/>
       </node>

       <!-- WebSocket Sensor Client (if running on rover) -->
       <node pkg="autonomy_sensor_bridge" exec="websocket_sensor_client.py" name="sensor_client">
           <param name="websocket_url" value="ws://localhost:8080"/>
           <param name="sensor_update_rate" value="10.0"/>
       </node>
   </launch>

These examples demonstrate how to effectively integrate the WebSocket Sensor Bridge into ROS2 autonomy systems, from basic sensor reading to complex navigation integration.
