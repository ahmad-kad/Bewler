# 🚀 ROS2 Migration Guide - WebSocket Sensor Bridge

**Transitioning from WebSocket Bridge to Direct ROS2 Transport**

This guide outlines the step-by-step process for migrating sensor data from WebSocket JSON streams to direct ROS2 DDS transport.

---

## 📊 Current vs Future Architecture

### **Current: WebSocket Bridge**
```
Sensor Devices → JSON over WebSocket → Python Bridge Node → ROS2 Topics
    ↓                                                            ↓
External Network                                           ROS2 Ecosystem
```

**Characteristics:**
- ✅ Easy deployment and debugging
- ✅ Works with any WebSocket-capable device
- ✅ Centralized data aggregation
- ❌ 2-5ms latency overhead
- ❌ JSON serialization overhead
- ❌ WebSocket connection dependency

### **Future: Direct ROS2 Transport**
```
Sensor Devices → ROS2 Node → DDS Transport → ROS2 Topics
    ↓                           ↓
ROS2 Ecosystem              ROS2 Ecosystem
```

**Characteristics:**
- ✅ 0.1-1ms latency
- ✅ Binary efficiency (no JSON)
- ✅ Native ROS2 discovery and QoS
- ✅ 1000Hz+ sensor rates possible
- ❌ Requires ROS2 on sensor devices
- ❌ DDS network configuration needed

---

## 🎯 Migration Strategy

### **Phase 1: Assessment & Planning**
#### **1.1 Sensor Capability Assessment**
Evaluate each sensor device:

```python
# Sensor Capability Matrix
SENSOR_CAPABILITIES = {
    'imu_mpu6050': {
        'compute': 'raspberry_pi_zero',  # Can run ROS2?
        'network': 'wifi',               # DDS multicast capable?
        'power': 'low_power',           # ROS2 overhead acceptable?
        'rate': 100,                    # Hz required
        'criticality': 'high'          # Mission impact
    },
    'gps_ublox': {
        'compute': 'arduino_esp32',     # Limited compute
        'network': 'wifi',              # Possible but constrained
        'power': 'battery_sensitive',   # Power-critical
        'rate': 10,                     # Hz acceptable
        'criticality': 'medium'
    }
}
```

#### **1.2 Network Infrastructure Assessment**
```yaml
# Current Network
network:
  topology: wifi_mesh
  bandwidth: 50Mbps
  latency: 5ms
  dds_support: partial  # Some devices support multicast

# Required for Full ROS2
network_requirements:
  multicast_support: required
  bandwidth_min: 10Mbps_per_device
  latency_max: 1ms
  jitter_max: 0.5ms
```

#### **1.3 Migration Priority Matrix**
```python
# Migration Order (High → Low Priority)
MIGRATION_PRIORITIES = [
    'imu',           # 100Hz, fusion-critical
    'wheel_odom',    # 50Hz, navigation-critical
    'gps',           # 10Hz, global reference
    'lidar',         # High bandwidth, compute intensive
    'camera_meta',   # Moderate rate, structured data
    'temperature',   # Low rate, simple data
    'battery'        # Critical but low rate
]
```

### **Phase 2: Hybrid Implementation**
#### **2.1 Create ROS2 Sensor Node Template**
```python
#!/usr/bin/env python3
"""
ROS2 Sensor Node Template
Deploy on sensor devices for direct ROS2 transport
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class GenericSensorNode(Node):
    """Template for ROS2 sensor nodes"""

    def __init__(self, sensor_name: str, sensor_class):
        super().__init__(f'{sensor_name}_sensor')

        # Sensor-specific setup
        self.sensor = sensor_class()
        self.update_rate = 100  # Hz

        # QoS optimized for sensor type
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher
        self.publisher = self.create_publisher(
            Imu, f'/{sensor_name}/data', qos_profile
        )

        # Timer for periodic publishing
        self.timer = self.create_timer(
            1.0 / self.update_rate, self.publish_data
        )

        self.get_logger().info(f'{sensor_name} sensor node started')

    def publish_data(self):
        """Read sensor and publish ROS2 message"""
        try:
            # Read from hardware
            raw_data = self.sensor.read()

            # Convert to ROS2 message
            msg = self.create_message(raw_data)

            # Publish
            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Sensor read error: {e}')


# IMU-specific implementation
class ImuSensorNode(GenericSensorNode):
    def __init__(self):
        super().__init__('imu', MPU6050)

    def create_message(self, raw_data) -> Imu:
        """Convert IMU data to ROS2 Imu message"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Accelerometer
        msg.linear_acceleration.x = raw_data.accel_x
        msg.linear_acceleration.y = raw_data.accel_y
        msg.linear_acceleration.z = raw_data.accel_z

        # Gyroscope
        msg.angular_velocity.x = raw_data.gyro_x
        msg.angular_velocity.y = raw_data.gyro_y
        msg.angular_velocity.z = raw_data.gyro_z

        return msg
```

#### **2.2 Hybrid Bridge Implementation**
```python
class HybridSensorBridgeNode(Node):
    """
    Hybrid bridge supporting both WebSocket and direct ROS2 sensors
    """

    def __init__(self):
        super().__init__('hybrid_sensor_bridge')

        # Direct ROS2 subscriptions (migrated sensors)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.on_imu_data, 10
        )

        # WebSocket for legacy sensors
        self.websocket_client = WebSocketClient('ws://legacy-sensors:8080')
        self.websocket_client.on_message = self.on_websocket_message

        # Publishers for WebSocket sensors
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery/status', 10)

    def on_imu_data(self, msg: Imu):
        """IMU data from direct ROS2 sensor"""
        # Forward or process as needed
        self.get_logger().debug('Received direct ROS2 IMU data')

    def on_websocket_message(self, json_data: str):
        """Legacy sensor data from WebSocket"""
        data = json.loads(json_data)

        # Publish GPS data
        if 'gps' in data['sensors']:
            gps_msg = self.convert_gps_data(data['sensors']['gps'])
            self.gps_pub.publish(gps_msg)

        # Publish battery data
        if 'battery' in data['sensors']:
            battery_msg = self.convert_battery_data(data['sensors']['battery'])
            self.battery_pub.publish(battery_msg)
```

### **Phase 3: Full ROS2 Migration**
#### **3.1 Network Configuration**
```xml
<!-- CycloneDDS configuration for sensor network -->
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>192.168.1.0/24</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65536</MaxMessageSize>
    </General>

    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
    </Discovery>

    <Internal>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
        <WhcLow>100kB</WhcLow>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

#### **3.2 QoS Optimization for Direct Transport**
```python
# Optimized QoS profiles for direct ROS2 transport
QOS_PROFILES = {
    'sensor_high_freq': QoSProfile(           # IMU, wheel odometry
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=100000000  # 100ms
    ),

    'sensor_standard': QoSProfile(            # GPS, temperature
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=1000000000  # 1s
    ),

    'sensor_critical': QoSProfile(            # Battery, safety sensors
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=5000000000  # 5s
    )
}
```

#### **3.3 Hardware-Specific Optimizations**
```python
# Raspberry Pi optimization
class RaspberryPiSensorNode(Node):
    def __init__(self):
        super().__init__('rpi_sensor_node')

        # Optimize for RPi constraints
        self.declare_parameter('cpu_affinity', 2)  # Pin to CPU core
        self.declare_parameter('memory_lock', True)  # Lock memory
        self.declare_parameter('priority', 50)      # Real-time priority

        # Use efficient DDS settings
        self.configure_dds_for_embedded()

    def configure_dds_for_embedded(self):
        """Configure DDS for resource-constrained devices"""
        # Smaller message buffers
        # Reduced discovery traffic
        # Optimized serialization
        pass

# ESP32 optimization
class Esp32SensorNode(Node):
    def __init__(self):
        super().__init__('esp32_sensor_node')

        # Micro-ROS optimizations
        self.declare_parameter('xml_profile', 'microxrcedds_client.xml')

        # Minimal QoS for bandwidth-constrained networks
        self.optimize_for_low_bandwidth()
```

---

## 📈 Performance Improvements

### **Latency Reduction**
| Component | WebSocket Bridge | Direct ROS2 | Improvement |
|-----------|------------------|-------------|-------------|
| Serialization | JSON (1-2ms) | Binary (0.1ms) | 90% faster |
| Transport | TCP+WebSocket | DDS | 80% lower overhead |
| Processing | Python parsing | Native C++ | 50% faster |
| **Total Latency** | **2-5ms** | **0.1-1ms** | **75-95% faster** |

### **Bandwidth Efficiency**
```python
# WebSocket JSON message (IMU)
websocket_msg = {
    "timestamp": 1234567890.123,
    "sensors": {
        "imu": {
            "accel": {"x": 0.1, "y": 0.2, "z": 9.8},
            "gyro": {"x": 0.01, "y": 0.02, "z": 0.03}
        }
    }
}
# Size: ~150 bytes

# Direct ROS2 message (IMU)
ros2_msg = Imu()
ros2_msg.linear_acceleration.x = 0.1
# Size: ~120 bytes (binary)
# Efficiency: 20% bandwidth reduction
```

### **CPU Usage Reduction**
```python
# WebSocket Bridge CPU profile
cpu_usage_websocket = {
    'json_parsing': '15%',
    'websocket_handling': '10%',
    'ros2_publishing': '5%',
    'total': '30%'
}

# Direct ROS2 CPU profile
cpu_usage_direct = {
    'sensor_reading': '10%',
    'ros2_publishing': '3%',
    'dds_transport': '2%',
    'total': '15%'
}
# Improvement: 50% CPU reduction
```

---

## 🛠️ Implementation Steps

### **Step 1: Environment Setup**
```bash
# Install ROS2 on sensor devices
# Raspberry Pi
sudo apt install ros-humble-ros-base

# ESP32 (Micro-ROS)
# Follow micro-ROS setup guide
```

### **Step 2: Device-Specific Configurations**
```python
# Raspberry Pi 4 configuration
RASPBERRY_PI_CONFIG = {
    'dds': {
        'config_file': '/opt/ros/humble/share/rmw_cyclonedds_cpp/config.xml',
        'transport': 'udp',
        'interface': 'wlan0'
    },
    'qos_overrides': {
        'depth': 50,  # Higher buffer for better CPU
        'reliability': 'best_effort'
    }
}

# ESP32 configuration
ESP32_CONFIG = {
    'micro_ros': {
        'transport': 'serial',
        'baud_rate': 115200,
        'device': '/dev/ttyUSB0'
    },
    'qos_overrides': {
        'depth': 5,   # Lower buffer for memory constraints
        'reliability': 'best_effort'
    }
}
```

### **Step 3: Testing & Validation**
```python
class MigrationTestSuite:
    """Comprehensive testing for migration validation"""

    def test_latency_improvement(self):
        """Verify latency reduction after migration"""
        websocket_latency = measure_websocket_latency()
        direct_latency = measure_direct_latency()

        assert direct_latency < websocket_latency * 0.5
        assert direct_latency < 1000000  # < 1ms

    def test_data_integrity(self):
        """Ensure data accuracy through migration"""
        test_data = generate_test_sensor_data()

        websocket_result = process_websocket_data(test_data)
        direct_result = process_direct_data(test_data)

        assert np.allclose(websocket_result, direct_result, rtol=1e-6)

    def test_network_resilience(self):
        """Test network interruption handling"""
        # Simulate network issues
        simulate_network_partition()

        # Verify DDS automatic reconnection
        assert self.wait_for_reconnection(timeout=5.0)
        assert self.verify_data_flow_resumed()

    def test_resource_usage(self):
        """Monitor CPU/memory usage improvements"""
        websocket_usage = measure_resource_usage(websocket_bridge)
        direct_usage = measure_resource_usage(direct_nodes)

        assert direct_usage.cpu < websocket_usage.cpu * 0.7
        assert direct_usage.memory < websocket_usage.memory * 0.8
```

### **Step 4: Rollback Strategy**
```python
class MigrationController:
    """Manages migration with rollback capability"""

    def __init__(self):
        self.migration_state = 'websocket_only'  # websocket_only, hybrid, direct_only
        self.rollback_timeout = 30.0  # seconds

    def migrate_sensor(self, sensor_name: str):
        """Migrate individual sensor with rollback"""
        try:
            # Start direct ROS2 sensor
            self.start_direct_sensor(sensor_name)

            # Wait for data flow
            if self.wait_for_sensor_data(sensor_name, timeout=5.0):
                # Stop WebSocket equivalent
                self.stop_websocket_sensor(sensor_name)
                self.migration_state = 'hybrid'
                self.get_logger().info(f'✅ Migrated {sensor_name} to direct ROS2')
            else:
                # Rollback
                self.rollback_sensor(sensor_name)
                self.get_logger().error(f'❌ Migration failed for {sensor_name}')

        except Exception as e:
            self.rollback_sensor(sensor_name)
            raise MigrationError(f'Migration failed: {e}')

    def rollback_sensor(self, sensor_name: str):
        """Rollback sensor to WebSocket transport"""
        try:
            self.stop_direct_sensor(sensor_name)
            self.start_websocket_sensor(sensor_name)
            self.get_logger().info(f'🔄 Rolled back {sensor_name} to WebSocket')
        except Exception as e:
            self.get_logger().critical(f'Rollback failed for {sensor_name}: {e}')
```

---

## 📊 Migration Metrics & Success Criteria

### **Performance Targets**
- **Latency**: < 1ms for high-frequency sensors (IMU, wheel odometry)
- **CPU Usage**: < 50% of WebSocket bridge CPU usage
- **Memory Usage**: < 80% of WebSocket bridge memory usage
- **Data Integrity**: 100% message delivery (within QoS bounds)
- **Network Resilience**: Automatic recovery within 5 seconds

### **Success Metrics**
```python
MIGRATION_SUCCESS_CRITERIA = {
    'latency_improvement': lambda old, new: new < old * 0.5,
    'cpu_reduction': lambda old, new: new < old * 0.7,
    'memory_reduction': lambda old, new: new < old * 0.8,
    'data_integrity': lambda: message_loss_rate < 0.001,  # < 0.1%
    'network_resilience': lambda: recovery_time < 5.0,     # < 5 seconds
    'deployment_success': lambda: deployment_failure_rate < 0.05  # < 5%
}
```

### **Monitoring Dashboard**
```python
class MigrationDashboard(Node):
    """Real-time migration monitoring"""

    def __init__(self):
        super().__init__('migration_dashboard')

        # Subscribe to all sensor topics
        self.sensor_topics = [
            '/imu/data', '/gps/fix', '/wheel/odom',
            '/battery/status', '/temperature/data'
        ]

        self.topic_monitors = {}
        for topic in self.sensor_topics:
            self.topic_monitors[topic] = TopicMonitor(topic)

        # Migration metrics
        self.metrics_pub = self.create_publisher(
            String, '/migration/metrics', 1
        )

        # Update timer
        self.create_timer(1.0, self.publish_metrics)

    def publish_metrics(self):
        """Publish comprehensive migration metrics"""
        metrics = {
            'timestamp': self.get_clock().now().to_msg(),
            'topics': {},
            'performance': {},
            'health': {}
        }

        for topic, monitor in self.topic_monitors.items():
            metrics['topics'][topic] = {
                'hz': monitor.get_frequency(),
                'latency': monitor.get_latency(),
                'transport': monitor.get_transport_type()  # 'websocket' or 'direct'
            }

        # Calculate aggregate metrics
        direct_topics = [t for t, m in metrics['topics'].items()
                        if m['transport'] == 'direct']
        websocket_topics = [t for t, m in metrics['topics'].items()
                           if m['transport'] == 'websocket']

        metrics['performance'] = {
            'direct_topic_count': len(direct_topics),
            'websocket_topic_count': len(websocket_topics),
            'migration_percentage': len(direct_topics) / len(self.sensor_topics) * 100,
            'avg_latency_direct': statistics.mean([m['latency'] for m in direct_topics.values()]),
            'avg_latency_websocket': statistics.mean([m['latency'] for m in websocket_topics.values()])
        }

        # Publish metrics
        msg = String()
        msg.data = json.dumps(metrics)
        self.metrics_pub.publish(msg)
```

---

## 🎯 Migration Checklist

### **Pre-Migration**
- [ ] Sensor hardware assessment complete
- [ ] Network infrastructure evaluated
- [ ] ROS2 installation tested on target devices
- [ ] QoS profiles defined for each sensor type
- [ ] Rollback procedures documented
- [ ] Performance baselines established

### **During Migration**
- [ ] Hybrid bridge deployed and tested
- [ ] Individual sensors migrated incrementally
- [ ] Performance monitoring active
- [ ] Rollback procedures tested
- [ ] Data integrity verified at each step

### **Post-Migration**
- [ ] WebSocket bridge components removed
- [ ] Network configuration optimized
- [ ] Performance targets validated
- [ ] Documentation updated
- [ ] Team trained on new architecture

### **Success Validation**
- [ ] All performance targets met
- [ ] Zero message loss during migration
- [ ] Automatic recovery from network issues
- [ ] CPU/memory usage within targets
- [ ] All stakeholders approve migration

---

## 🔧 Tools & Utilities

### **Migration Automation Scripts**
```bash
#!/bin/bash
# migrate_sensor.sh - Automated sensor migration

SENSOR_NAME=$1
SENSOR_DEVICE=$2

echo "Migrating $SENSOR_NAME to direct ROS2 transport..."

# Deploy ROS2 node to device
scp sensor_node.py $SENSOR_DEVICE:/opt/ros/humble/lib/sensor_nodes/
ssh $SENSOR_DEVICE "ros2 run sensor_nodes ${SENSOR_NAME}_node &"

# Wait for data flow
sleep 5

# Verify data is being received
if ros2 topic hz /$SENSOR_NAME/data | grep -q "average rate"; then
    echo "✅ Direct ROS2 data flow confirmed"
    # Stop WebSocket equivalent
    ros2 service call /sensor_bridge/disable_sensor "sensor: '$SENSOR_NAME'"
    echo "✅ Migration complete"
else
    echo "❌ Migration failed - rolling back"
    ssh $SENSOR_DEVICE "pkill -f ${SENSOR_NAME}_node"
    exit 1
fi
```

### **Performance Comparison Script**
```python
#!/usr/bin/env python3
"""
Performance comparison between WebSocket and direct ROS2 transport
"""

import time
import statistics
from typing import List, Dict

class PerformanceComparator:
    def __init__(self):
        self.websocket_latencies: List[float] = []
        self.direct_latencies: List[float] = []

    def measure_websocket_latency(self, samples: int = 1000) -> Dict:
        """Measure WebSocket bridge latency"""
        # Implementation would send test messages and measure round-trip time
        pass

    def measure_direct_latency(self, samples: int = 1000) -> Dict:
        """Measure direct ROS2 latency"""
        # Implementation would publish/subscribe test messages
        pass

    def generate_report(self) -> str:
        """Generate performance comparison report"""
        websocket_stats = self.calculate_stats(self.websocket_latencies)
        direct_stats = self.calculate_stats(self.direct_latencies)

        improvement = (websocket_stats['mean'] - direct_stats['mean']) / websocket_stats['mean'] * 100

        report = f"""
        Performance Comparison Report
        ==============================

        WebSocket Bridge:
        - Mean latency: {websocket_stats['mean']:.3f}ms
        - 95th percentile: {websocket_stats['p95']:.3f}ms
        - 99th percentile: {websocket_stats['p99']:.3f}ms

        Direct ROS2:
        - Mean latency: {direct_stats['mean']:.3f}ms
        - 95th percentile: {direct_stats['p95']:.3f}ms
        - 99th percentile: {direct_stats['p99']:.3f}ms

        Improvement: {improvement:.1f}%
        Status: {'✅ Target met' if improvement > 50 else '❌ Below target'}
        """

        return report

    @staticmethod
    def calculate_stats(latencies: List[float]) -> Dict:
        """Calculate latency statistics"""
        return {
            'mean': statistics.mean(latencies),
            'median': statistics.median(latencies),
            'p95': statistics.quantiles(latencies, n=20)[18],
            'p99': statistics.quantiles(latencies, n=20)[19],
            'min': min(latencies),
            'max': max(latencies)
        }
```

---

*"Migration to direct ROS2 transport represents the future of robotic sensor integration - lower latency, higher reliability, and better performance for demanding applications."*
