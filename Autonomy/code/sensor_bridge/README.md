# WebSocket Sensor Bridge - URC 2026 Mars Rover

**WebSocket-to-ROS2 Sensor Data Bridge**

Converts real-time sensor data from WebSocket streams to ROS2 topics, designed for easy migration to direct ROS2 transport.

---

## 🏗️ Architecture Overview

```
┌─────────────────┐    WebSocket    ┌──────────────────┐    DDS     ┌─────────────────┐
│   Sensor Hub    │ ──────────────► │ Sensor Bridge    │ ─────────► │  ROS2 System   │
│   (JSON data)   │                 │ (Python Node)    │            │  (Navigation,  │
└─────────────────┘                 └──────────────────┘            │   SLAM, etc.)  │
                                                                    └─────────────────┘
```

### **Current Implementation (WebSocket Bridge)**
- **ROS2 Timer-Based Architecture**: No threading, uses ROS2 timers for reliability
- **Comprehensive Data Validation**: Type checking, range validation, sanity checks
- **Adaptive Reconnection**: Exponential backoff with configurable limits
- **Health Monitoring**: Connection status, performance metrics, diagnostics
- **Graceful Degradation**: Fallback modes during connection issues
- **ROS2 Publishers**: Optimized QoS settings for real-time sensor data

### **Future Implementation (Direct ROS2)**
- **ROS2 Nodes**: Deployed on sensor devices
- **DDS Transport**: Direct peer-to-peer communication
- **Zero Latency**: No JSON/WebSocket overhead
- **1000Hz+ Rates**: Hardware-limited sensor updates

---

## 📡 Sensor Data Flow

### **WebSocket Message Format**
```json
{
  "timestamp": 1234567890.123,
  "sensors": {
    "imu": {
      "accel": {"x": 0.1, "y": 0.2, "z": 9.8},
      "gyro": {"x": 0.01, "y": 0.02, "z": 0.03},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
      "accel_covariance": [0.01, 0.01, 0.01],
      "gyro_covariance": [0.01, 0.01, 0.01],
      "orientation_covariance": [0.01, 0.01, 0.01, 0.01]
    },
    "gps": {
      "lat": 37.7749,
      "lon": -122.4194,
      "altitude": 10.5,
      "position_covariance": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
    },
    "battery": {
      "voltage": 12.3,
      "current": 2.1,
      "percentage": 85.0,
      "temperature": 35.0,
      "capacity": 100.0
    },
    "wheel_odom": {
      "x": 1.5, "y": 2.3, "z": 0.0,
      "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
      "vx": 0.5, "vy": 0.0, "vz": 0.0,
      "wx": 0.0, "wy": 0.0, "wz": 0.1,
      "pose_covariance": [0.1] * 36,
      "twist_covariance": [0.1] * 36
    },
    "temperature": {
      "temperature": 35.0,
      "variance": 0.1
    }
  }
}
```

### **ROS2 Topic Mapping**
| Sensor | WebSocket Key | ROS2 Topic | Message Type | QoS | Rate |
|--------|---------------|------------|--------------|-----|------|
| IMU | `imu` | `/imu/data` | `sensor_msgs/Imu` | BEST_EFFORT | 100Hz |
| GPS | `gps` | `/gps/fix` | `sensor_msgs/NavSatFix` | RELIABLE | 10Hz |
| Battery | `battery` | `/battery/status` | `sensor_msgs/BatteryState` | RELIABLE | 1Hz |
| Wheel Odom | `wheel_odom` | `/wheel/odom` | `nav_msgs/Odometry` | BEST_EFFORT | 50Hz |
| Temperature | `temperature` | `/temperature/data` | `sensor_msgs/Temperature` | RELIABLE | 5Hz |

---

## 🚀 Quick Start

### **1. Install Dependencies**
```bash
# WebSocket client library
pip install websockets

# ROS2 dependencies (already included)
# rclpy, sensor_msgs, nav_msgs, etc.
```

### **2. Configure WebSocket Endpoint**
Edit `config/sensor_bridge.yaml`:
```yaml
sensor_bridge:
  websocket_url: "ws://your-sensor-hub:8080"  # Change this
```

### **3. Launch the Bridge**
```bash
# From your ROS2 workspace
cd ~/urc-machiato-2026/Autonomy/ros2_ws
source install/setup.bash

# Launch sensor bridge
ros2 launch sensor_bridge sensor_bridge.launch.py

# Or run directly
ros2 run sensor_bridge sensor_bridge_node
```

### **4. Verify Data Flow**
```bash
# Check topics are being published
ros2 topic list | grep -E "(imu|gps|battery)"

# Monitor IMU data
ros2 topic echo /imu/data

# Monitor GPS data
ros2 topic echo /gps/fix
```

---

## ✅ **Recent Improvements**

### **Version 0.1.0 - Production Ready**
- **ROS2 Timer Architecture**: Replaced threading with ROS2 timers for reliability
- **Comprehensive Error Handling**: Try/catch blocks throughout with graceful degradation
- **Data Validation**: Type checking, range validation, and sanity checks for all sensors
- **Adaptive Reconnection**: Exponential backoff with configurable limits
- **Health Monitoring**: Real-time connection status and performance metrics
- **QoS Optimization**: Improved settings for high-frequency vs standard sensors
- **Graceful Degradation**: Fallback modes during connection failures
- **Proper Cleanup**: Resource cleanup on shutdown prevents memory leaks

### **Key Safety Features**
- **Input Validation**: All sensor data validated before processing
- **Range Checking**: Physical limits enforced (e.g., GPS coordinates, voltages)
- **Sanity Checks**: Unusual values logged as warnings
- **Graceful Failures**: Invalid data doesn't crash the system
- **Resource Management**: Timers and connections properly cleaned up

## ⚙️ Configuration

### **Connection Parameters**
```yaml
sensor_bridge:
  # WebSocket connection settings
  websocket_url: "ws://localhost:8080"
  reconnect_interval: 3.0                    # Base reconnection delay (seconds)
  max_reconnect_attempts: 10                 # Maximum reconnection attempts
  connection_timeout: 5.0                    # Connection timeout (seconds)
  max_reconnect_interval: 60.0              # Maximum backoff delay (seconds)
  reconnect_backoff_multiplier: 1.5         # Exponential backoff multiplier

  # Monitoring settings
  health_check_interval: 1.0                # Health check frequency (seconds)
  message_timeout: 10.0                     # Message timeout threshold (seconds)
  enable_graceful_degradation: true         # Enable fallback modes
```

### **Sensor Configuration**
Edit `config/sensor_bridge.yaml` to enable/disable sensors and adjust QoS:

```yaml
sensor_bridge:
  sensors:
    imu:
      enabled: true
      topic_name: "/imu/data"
      update_rate: 100.0

    gps:
      enabled: true
      topic_name: "/gps/fix"
      update_rate: 10.0
```

### **QoS Profiles**
- **High-Frequency Sensors** (IMU, wheel odometry): `BEST_EFFORT` with `KEEP_LAST(10)`
- **Standard Sensors** (GPS, temperature): `RELIABLE` with `KEEP_LAST(5)`
- **Critical Sensors** (battery): `RELIABLE` with `TRANSIENT_LOCAL`

---

## 🔄 Migration Path to Direct ROS2

### **Phase 1: Current WebSocket Bridge (✅ Complete)**
- WebSocket JSON → ROS2 message conversion
- Centralized sensor data aggregation
- Easy debugging and monitoring

### **Phase 2: Hybrid Mode (In Progress)**
- Deploy ROS2 nodes on high-performance sensors
- Keep WebSocket for legacy/low-power sensors
- Gradual migration based on sensor capabilities

### **Phase 3: Full Direct ROS2 (Future)**
- All sensors publish directly via DDS
- Remove WebSocket/JSON overhead
- Enable 1000Hz+ sensor fusion

### **Migration Benefits**
| Aspect | WebSocket Bridge | Direct ROS2 |
|--------|------------------|-------------|
| **Latency** | 2-5ms | 0.1-1ms |
| **CPU Usage** | Medium | Low |
| **Bandwidth** | JSON verbose | Binary efficient |
| **Real-time** | Good (100Hz) | Excellent (1000Hz+) |
| **Deployment** | Simple | Complex |
| **Debugging** | Easy | Moderate |

### **Migration Strategy**

#### **1. Identify Sensor Priorities**
```
High Priority (Migrate First):
├── IMU (100Hz fusion-critical)
├── Wheel Odometry (50Hz navigation)
└── GPS (10Hz global reference)

Medium Priority:
├── LiDAR data
└── Camera metadata

Low Priority:
├── Battery status
├── Temperature sensors
└── Diagnostic data
```

#### **2. Hardware Assessment**
For each sensor device:
- **Compute capability**: Can it run ROS2?
- **Network**: DDS multicast support?
- **Power**: ROS2 overhead acceptable?
- **Update rate**: Does it need 100Hz+?

#### **3. Gradual Migration Steps**
```python
# Phase 1: WebSocket Bridge (Current)
class WebSocketSensorBridge:
    def get_sensor_data(self):
        return websocket_client.receive_json()

# Phase 2: Hybrid Bridge
class HybridSensorBridge:
    def get_imu_data(self):
        # Direct ROS2 subscription
        return ros2_subscriber.receive()

    def get_gps_data(self):
        # Still WebSocket for now
        return websocket_client.receive_gps()

# Phase 3: Direct ROS2 (Future)
class DirectROSSensorBridge:
    def get_sensor_data(self):
        # Pure ROS2 - no WebSocket
        return ros2_subscriber.receive()
```

#### **4. Implementation Template**
```python
# Future ROS2 sensor node template
class ImuSensorNode(Node):
    def __init__(self):
        super().__init__('imu_sensor')

        # Direct hardware interface (no WebSocket)
        self.imu = MPU6050(i2c_bus=1)
        self.publisher = self.create_publisher(Imu, '/imu/data', qos_profile_sensor)

        # High-frequency timer
        self.create_timer(0.01, self.publish_imu)  # 100Hz

    def publish_imu(self):
        # Direct sensor reading to ROS2 message
        accel, gyro = self.imu.read()
        msg = self.create_imu_message(accel, gyro)
        self.publisher.publish(msg)
```

---

## 🔧 Development & Testing

### **Running Tests**
```bash
# Build the package
cd ~/urc-machiato-2026/Autonomy/ros2_ws
colcon build --packages-select sensor_bridge

# Run tests
colcon test --packages-select sensor_bridge
```

### **Debugging WebSocket Connection**
```bash
# Monitor connection status
ros2 topic echo /sensor_bridge/status

# Check WebSocket logs
ros2 run sensor_bridge sensor_bridge_node --log-level debug

# Test with mock WebSocket server
python3 test/mock_websocket_server.py
```

### **Performance Monitoring**
```bash
# Monitor topic frequencies
ros2 topic hz /imu/data
ros2 topic hz /gps/fix

# Check CPU usage
ros2 run sensor_bridge sensor_bridge_node &
top -p $!
```

---

## 📚 API Reference

### **WebSocketSensorBridgeNode Class**

#### **Parameters**
- `websocket_url` (string): WebSocket endpoint URL
- `reconnect_interval` (float): Seconds between reconnection attempts
- `max_reconnect_attempts` (int): Maximum reconnection attempts
- `connection_timeout` (float): Connection timeout in seconds

#### **Published Topics**
- `/imu/data` - IMU sensor data
- `/gps/fix` - GPS position data
- `/battery/status` - Battery status
- `/wheel/odom` - Wheel odometry
- `/temperature/data` - Temperature readings

#### **Services**
None (publisher-only node)

#### **Subscribed Topics**
None (WebSocket client only)

---

## 🚨 Troubleshooting

### **WebSocket Connection Issues**
```bash
# Check if WebSocket server is running
curl -I http://sensor-hub:8080

# Test WebSocket connection manually
websocat ws://sensor-hub:8080

# Check firewall settings
sudo ufw status
```

### **ROS2 Topic Issues**
```bash
# Verify topics are being published
ros2 topic list | grep sensor

# Check topic message types
ros2 topic info /imu/data

# Monitor topic frequency
ros2 topic hz /imu/data
```

### **Performance Issues**
```bash
# Check CPU usage
ros2 run rqt_top rqt_top

# Monitor network traffic
sudo iftop -i wlan0

# Profile the bridge node
ros2 run sensor_bridge sensor_bridge_node --log-level debug
```

---

## 🤝 Contributing

### **Adding New Sensors**
1. Add sensor configuration to `config/sensor_bridge.yaml`
2. Implement message converter in `websocket_bridge.py`
3. Add ROS2 publisher setup
4. Update documentation

### **Modifying QoS Settings**
```python
# Example: Custom QoS for high-reliability sensor
qos_reliable_sensor = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=50
)
```

---

## 📋 Future Enhancements

- [ ] **Message Compression**: Reduce WebSocket bandwidth usage
- [ ] **Data Validation**: Schema validation for incoming sensor data
- [ ] **Health Monitoring**: Automatic sensor health detection
- [ ] **Load Balancing**: Multiple WebSocket endpoints
- [ ] **Security**: Authentication and encryption for WebSocket
- [ ] **Configuration UI**: Web-based configuration interface

---

*"The WebSocket bridge is a pragmatic solution for rapid sensor integration, designed with direct ROS2 migration as the ultimate goal."*
