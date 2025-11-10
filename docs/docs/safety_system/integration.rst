.. _safety_integration:

Safety System Integration
==========================

This guide provides comprehensive instructions for integrating the safety system into the URC 2026 Mars Rover autonomy stack and development environment.

System Integration Architecture
===============================

Safety System Integration Points
---------------------------------

The safety system integrates with the rover autonomy stack at multiple levels:

.. code-block:: none

   +-------------------+     +-------------------+
   | Application Layer | --> | Safety Monitoring |
   | - State Machine    |     | - State validation|
   | - Navigation       |     | - Transition      |
   | - Computer Vision  |     |   monitoring     |
   +-------------------+     +-------------------+
           |                           |
           v                           v
   +-------------------+     +-------------------+
   | System Layer      | --> | Safety Validation |
   | - Sensor Bridge    |     | - Sensor integrity|
   | - Communication    |     | - Data validation|
   | - Power Management |     | - Health monitoring|
   +-------------------+     +-------------------+
           |                           |
           v                           v
   +-------------------+     +-------------------+
   | Hardware Layer    | --> | Safety Control    |
   | - Motor Controllers|     | - Emergency stop  |
   | - Power Systems    |     | - Relay control   |
   | - Sensors          |     | - Actuator cutoff |
   +-------------------+     +-------------------+

Integration Layers
------------------

**Application Layer Integration**
   - State machine safety monitoring
   - Mission phase safety validation
   - Autonomous operation safety checks
   - Operator command safety validation

**System Layer Integration**
   - Sensor data safety validation
   - Communication system monitoring
   - Power management safety checks
   - Subsystem health monitoring

**Hardware Layer Integration**
   - Emergency stop relay control
   - Motor power cutoff systems
   - Sensor power management
   - Hardware watchdog integration

ROS2 Integration
================

Package Dependencies
--------------------

The safety system requires the following ROS2 packages:

**Core Dependencies**::

   <depend>rclpy</depend>
   <depend>std_msgs</depend>
   <depend>sensor_msgs</depend>
   <depend>nav_msgs</depend>
   <depend>diagnostic_msgs</depend>
   <depend>autonomy_interfaces</depend>

**Development Dependencies**::

   <test_depend>pytest</test_depend>
   <test_depend>ament_copyright</test_depend>
   <test_depend>ament_flake8</test_depend>
   <test_depend>ament_pep257</test_depend>
   <test_depend>ament_xmllint</test_depend>

Workspace Integration
----------------------

**Package Structure**::

   autonomy_ws/
   ├── src/
   │   ├── autonomy_interfaces/     # Custom message/service definitions
   │   ├── autonomy_state_machine/  # State machine with safety integration
   │   ├── autonomy_safety_system/  # Safety system components
   │   ├── autonomy_sensor_bridge/  # Sensor data with safety validation
   │   └── ...                      # Other autonomy packages

**Build Integration**::

   # Build safety system with dependencies
   colcon build --packages-select \
     autonomy_interfaces \
     autonomy_state_machine \
     autonomy_safety_system \
     autonomy_sensor_bridge

Launch File Integration
-----------------------

**Complete System Launch**::

   <!-- Complete rover system with safety -->
   <launch>
     <!-- Safety system (start first for monitoring) -->
     <include file="$(find-pkg-share autonomy_safety_system)/launch/safety_system.launch.py">
       <arg name="enable_safety_watchdog" value="true"/>
       <arg name="enable_redundant_monitor" value="true"/>
       <arg name="enable_emergency_coordinator" value="true"/>
       <arg name="enable_safety_dashboard" value="true"/>
       <arg name="log_level" value="info"/>
     </include>

     <!-- State machine with safety integration -->
     <include file="$(find-pkg-share autonomy_state_machine)/launch/state_machine.launch.py">
       <arg name="enable_safety_integration" value="true"/>
     </include>

     <!-- Sensor bridge with safety validation -->
     <include file="$(find-pkg-share autonomy_sensor_bridge)/launch/sensor_bridge.launch.py">
       <arg name="enable_safety_validation" value="true"/>
     </include>

     <!-- Other autonomy subsystems -->
     <include file="$(find-pkg-share autonomy_navigation)/launch/navigation.launch.py"/>
     <include file="$(find-pkg-share autonomy_computer_vision)/launch/computer_vision.launch.py"/>
   </launch>

**Safety-Only Launch**::

   <!-- Safety system testing and validation -->
   <launch>
     <include file="$(find-pkg-share autonomy_safety_system)/launch/safety_system.launch.py">
       <arg name="enable_integration_tester" value="true"/>
       <arg name="log_level" value="debug"/>
     </include>
   </launch>

Component Integration
=====================

State Machine Integration
-------------------------

**Safety State Integration**::

   from autonomy_interfaces.msg import SystemState

   class SafeStateMachine(Node):
       def __init__(self):
           super().__init__('safe_state_machine')

           # Safety-aware state transitions
           self.create_subscription(
               SafetyStatus, '/safety/violations',
               self.safety_violation_callback, 10)

           # Publish state with safety context
           self.state_pub = self.create_publisher(
               SystemState, '/state_machine/current_state', 10)

       def safety_violation_callback(self, msg: SafetyStatus):
           """Handle safety violations in state transitions"""
           if msg.safety_level in ['CRITICAL', 'EMERGENCY']:
               self.transition_to_safety_state()

       def transition_to_safety_state(self):
           """Safe state transition with safety monitoring"""
           # Validate transition safety
           # Execute transition
           # Publish new state
           pass

**Heartbeat Integration**::

   class StateMachineWithHeartbeat(Node):
       def __init__(self):
           super().__init__('state_machine')

           # Heartbeat publisher for safety monitoring
           self.heartbeat_pub = self.create_publisher(
               String, '/state_machine/heartbeat', 1)

           # Heartbeat timer
           self.heartbeat_timer = self.create_timer(
               1.0, self.publish_heartbeat)  # 1Hz heartbeat

       def publish_heartbeat(self):
           """Publish state machine heartbeat"""
           heartbeat = {
               'timestamp': time.time(),
               'state': self.current_state,
               'healthy': self.is_healthy(),
               'active_mission': self.current_mission
           }

           msg = String()
           msg.data = json.dumps(heartbeat)
           self.heartbeat_pub.publish(msg)

Subsystem Integration
---------------------

**Subsystem Health Reporting**::

   class SafeSubsystem(Node):
       def __init__(self, subsystem_name: str):
           super().__init__(f'{subsystem_name}_node')

           # Health status publisher
           self.health_pub = self.create_publisher(
               String, '/state_machine/subsystem_status', 10)

           # Emergency stop subscriber
           self.create_subscription(
               Bool, f'/{subsystem_name}/emergency_stop',
               self.emergency_stop_callback, 10)

           # Health monitoring timer
           self.health_timer = self.create_timer(
               2.0, self.publish_health_status)

       def publish_health_status(self):
           """Publish subsystem health status"""
           health_data = {
               'subsystem': self.subsystem_name,
               'timestamp': time.time(),
               'healthy': self.is_healthy(),
               'status': self.get_status(),
               'error_count': self.error_count,
               'last_error': self.last_error
           }

           msg = String()
           msg.data = json.dumps(health_data)
           self.health_pub.publish(msg)

       def emergency_stop_callback(self, msg: Bool):
           """Handle emergency stop command"""
           if msg.data:
               self.execute_emergency_stop()

**Emergency Stop Implementation**::

   class SubsystemWithEmergencyStop(Node):
       def __init__(self):
           super().__init__('subsystem_node')

           # Emergency stop service
           self.emergency_stop_service = self.create_service(
               Empty, '/subsystem/emergency_stop',
               self.emergency_stop_service_callback)

       def emergency_stop_service_callback(self, request, response):
           """Handle emergency stop service call"""
           self.logger.warning('EMERGENCY STOP received - halting operations')

           # Execute emergency stop sequence
           self.stop_all_operations()
           self.cut_power_to_actuators()
           self.publish_emergency_status()

           return response

Sensor Integration
------------------

**Sensor Safety Validation**::

   class SafeSensorNode(Node):
       def __init__(self):
           super().__init__('safe_sensor_node')

           # Sensor data publisher
           self.data_pub = self.create_publisher(
               Imu, '/imu/data', 10)

           # Safety validation of sensor data
           self.safety_validator = SensorSafetyValidator()

           # Sensor reading timer
           self.sensor_timer = self.create_timer(
               0.01, self.publish_sensor_data)  # 100Hz

       def publish_sensor_data(self):
           """Publish sensor data with safety validation"""
           # Read raw sensor data
           raw_data = self.read_sensor()

           # Validate data safety
           if self.safety_validator.is_safe(raw_data):
               # Convert and publish
               ros_msg = self.convert_to_ros_message(raw_data)
               self.data_pub.publish(ros_msg)
           else:
               # Handle unsafe data
               self.handle_unsafe_sensor_data(raw_data)

**Sensor Health Monitoring**::

   class SensorHealthMonitor(Node):
       def __init__(self):
           super().__init__('sensor_health_monitor')

           # Monitor multiple sensors
           self.sensor_subscriptions = {}
           self.sensor_health = {}

           # Setup sensor monitoring
           self.setup_sensor_monitoring()

       def setup_sensor_monitoring(self):
           """Setup monitoring for all sensors"""
           sensors = ['imu', 'gps', 'battery', 'temperature']

           for sensor in sensors:
               # Subscribe to sensor data
               self.sensor_subscriptions[sensor] = self.create_subscription(
                   self.get_sensor_message_type(sensor),
                   f'/{sensor}/data',
                   lambda msg, s=sensor: self.sensor_callback(msg, s),
                   10)

               # Initialize health tracking
               self.sensor_health[sensor] = {
                   'last_update': 0.0,
                   'update_count': 0,
                   'error_count': 0,
                   'healthy': True
               }

       def sensor_callback(self, msg, sensor_name: str):
           """Handle sensor data updates"""
           health = self.sensor_health[sensor_name]
           health['last_update'] = time.time()
           health['update_count'] += 1

           # Validate sensor data
           if self.validate_sensor_data(msg, sensor_name):
               health['healthy'] = True
           else:
               health['error_count'] += 1
               health['healthy'] = False

Dashboard Integration
---------------------

**Web Interface Safety Integration**::

   # React component for safety monitoring
   class SafetyDashboard extends React.Component {
       componentDidMount() {
           // Connect to ROS2 safety topics
           this.ros = new ROSLIB.Ros({
               url: 'ws://localhost:9090'
           });

           // Subscribe to safety status
           this.safetyStatus = new ROSLIB.Topic({
               ros: this.ros,
               name: '/safety/dashboard_status',
               messageType: 'std_msgs/String'
           });

           this.safetyStatus.subscribe((message) => {
               const status = JSON.parse(message.data);
               this.setState({ safetyStatus: status });
           });
       }

       render() {
           return (
               <div className="safety-dashboard">
                   <SafetyStatusIndicator status={this.state.safetyStatus} />
                   <ActiveAlerts alerts={this.state.activeAlerts} />
                   <SystemHealth health={this.state.systemHealth} />
               </div>
           );
       }
   }

**Command Line Safety Monitoring**::

   #!/bin/bash
   # Safety monitoring script

   echo "=== Safety System Monitor ==="

   # Monitor safety dashboard status
   echo "Safety Status:"
   timeout 5 ros2 topic echo /safety/dashboard_status --once

   # Check for active alerts
   echo -e "\nActive Alerts:"
   timeout 5 ros2 topic echo /safety/active_alerts --once

   # Monitor system health
   echo -e "\nSystem Health:"
   timeout 5 ros2 topic echo /safety/system_health --once

   # Check emergency status
   echo -e "\nEmergency Status:"
   timeout 5 ros2 topic echo /safety/emergency_status --once

Configuration Integration
==========================

Parameter Server Integration
----------------------------

**Runtime Parameter Configuration**::

   # Configure safety thresholds
   ros2 param set /safety_watchdog battery_critical_threshold 12.0
   ros2 param set /safety_watchdog temperature_warning_threshold 75.0

   # Adjust monitoring levels
   ros2 param set /safety_watchdog watchdog_levels '["HEARTBEAT", "SUBSYSTEM_HEALTH"]'

   # Configure redundant monitor
   ros2 param set /redundant_safety_monitor imu_accel_max 60.0
   ros2 param set /redundant_safety_monitor communication_timeout 10.0

**YAML Configuration Files**::

   # Complete safety system configuration
   safety_system:
     watchdog:
       levels: ["HEARTBEAT", "STATE_TRANSITIONS", "SUBSYSTEM_HEALTH", "SENSOR_INTEGRITY"]
       heartbeat_timeout: 5.0
       battery_critical_threshold: 10.0
       temperature_critical_threshold: 85.0

     redundant_monitor:
       imu_accel_max: 50.0
       battery_critical: 10.0
       temperature_critical: 85.0

     dashboard:
       max_active_alerts: 10
       health_score_threshold: 0.7

Launch File Parameter Override
------------------------------

**Environment-Specific Configuration**::

   <!-- Development environment -->
   <launch>
     <include file="$(find-pkg-share autonomy_safety_system)/launch/safety_system.launch.py">
       <arg name="log_level" value="debug"/>
       <!-- Relaxed thresholds for development -->
       <arg name="battery_critical_threshold" value="5.0"/>
       <arg name="enable_integration_tester" value="true"/>
     </include>
   </launch>

   <!-- Competition environment -->
   <launch>
     <include file="$(find-pkg-share autonomy_safety_system)/launch/safety_system.launch.py">
       <arg name="log_level" value="info"/>
       <!-- Strict thresholds for competition -->
       <arg name="battery_critical_threshold" value="15.0"/>
       <arg name="enable_automatic_recovery" value="false"/>
     </include>
   </launch>

Testing Integration
===================

Automated Test Integration
---------------------------

**Continuous Integration Testing**::

   # Add to CI pipeline
   - name: Safety System Tests
     run: |
       # Build safety system
       colcon build --packages-select autonomy_safety_system

       # Run safety integration tests
       ros2 launch autonomy_safety_system safety_system.launch.py enable_integration_tester:=true &
       sleep 10

       # Execute test scenarios
       ros2 run autonomy_safety_system safety_integration_tester --scenario BATTERY_CRITICAL
       ros2 run autonomy_safety_system safety_integration_tester --scenario EMERGENCY_STOP

       # Verify test results
       ros2 topic echo /safety/test_status --filter "success == true"

**Regression Testing**::

   # Daily safety regression tests
   #!/bin/bash
   echo "Running safety system regression tests..."

   # Start safety system
   ros2 launch autonomy_safety_system safety_system.launch.py &

   # Wait for system startup
   sleep 15

   # Run all test scenarios
   scenarios=("BATTERY_CRITICAL" "THERMAL_WARNING" "EMERGENCY_STOP" "SENSOR_FAILURE")
   for scenario in "${scenarios[@]}"; do
       echo "Testing scenario: $scenario"
       ros2 run autonomy_safety_system safety_integration_tester --scenario $scenario

       # Check result
       result=$(ros2 topic echo /safety/test_status --filter "scenario == '$scenario'" --once 2>/dev/null)
       if [[ $result == *"success.*true"* ]]; then
           echo "✓ $scenario passed"
       else
           echo "✗ $scenario failed"
           exit 1
       fi
   done

   echo "All regression tests passed"

Manual Testing Integration
--------------------------

**Development Testing Workflow**::

   # 1. Start safety system
   ros2 launch autonomy_safety_system safety_system.launch.py

   # 2. Monitor safety status
   ros2 topic hz /safety/dashboard_status

   # 3. Test emergency stop
   ros2 service call /state_machine/emergency_stop std_srvs/srv/Empty

   # 4. Verify emergency response
   ros2 topic echo /safety/emergency_status --once

   # 5. Test recovery
   ros2 service call /state_machine/recover_from_safety autonomy_interfaces/srv/RecoverFromSafety "{
     recovery_method: 'AUTO',
     operator_id: 'test_operator',
     acknowledge_risks: true
   }"

**Competition Readiness Testing**::

   # Complete competition safety validation
   #!/bin/bash

   echo "=== Competition Safety Validation ==="

   # 1. Emergency stop response time test
   echo "Testing emergency stop response time..."
   start_time=$(date +%s.%N)
   ros2 service call /state_machine/emergency_stop std_srvs/srv/Empty
   end_time=$(date +%s.%N)
   response_time=$(echo "$end_time - $start_time" | bc)
   echo "Response time: ${response_time}s (URC limit: 3.0s)"

   # 2. Safety system health check
   echo "Checking safety system health..."
   health=$(ros2 topic echo /safety/dashboard_status --once --timeout 5)
   if [[ $health == *"overall_status.*NORMAL"* ]]; then
       echo "✓ Safety system healthy"
   else
       echo "✗ Safety system issues detected"
       exit 1
   fi

   # 3. Recovery procedure validation
   echo "Testing recovery procedures..."
   ros2 service call /state_machine/recover_from_safety autonomy_interfaces/srv/RecoverFromSafety "{
     recovery_method: 'AUTO',
     operator_id: 'competition_test',
     acknowledge_risks: true
   }"

   echo "Competition safety validation complete"

Deployment Integration
======================

Development Environment Setup
------------------------------

**Local Development**::

   # Clone and setup
   git clone <repository>
   cd urc-machiato-2026

   # Install dependencies
   pip install -r requirements.txt

   # Build ROS2 packages
   cd Autonomy/ros2_ws
   colcon build --packages-select autonomy_safety_system

   # Launch safety system
   ros2 launch autonomy_safety_system safety_system.launch.py

**Docker Development**::

   # Build safety system container
   docker build -t safety-system -f docker/safety.Dockerfile .

   # Run with safety testing
   docker run -it safety-system \
     ros2 launch autonomy_safety_system safety_system.launch.py \
     enable_integration_tester:=true

Competition Deployment
----------------------

**Pre-Competition Setup**::

   # 1. Deploy to rover
   scp -r Autonomy/code/safety_system rover:/home/rover/autonomy_ws/src/

   # 2. Configure for competition
   ssh rover "cd autonomy_ws && colcon build --packages-select autonomy_safety_system"

   # 3. Update configuration for competition thresholds
   ssh rover "ros2 param set /safety_watchdog battery_critical_threshold 15.0"

   # 4. Test emergency stop response
   ssh rover "ros2 service call /state_machine/emergency_stop std_srvs/srv/Empty"

**Competition Launch**::

   # Complete system launch with safety
   ros2 launch rover_system competition.launch.py

   # Safety monitoring
   ros2 run autonomy_safety_system safety_dashboard

Troubleshooting Integration
===========================

Common Integration Issues
--------------------------

**Communication Failures**::

   # Check ROS2 network
   ros2 node list

   # Verify topic publication
   ros2 topic list | grep safety

   # Check parameter values
   ros2 param list /safety_watchdog

**Timing Issues**::

   # Monitor topic frequencies
   ros2 topic hz /safety/dashboard_status

   # Check system load
   top -p $(pgrep -f safety)

   # Adjust timer periods
   ros2 param set /safety_watchdog heartbeat_timeout 10.0

**Resource Conflicts**::

   # Monitor memory usage
   ps aux | grep safety

   # Check CPU utilization
   top -H -p $(pgrep -f safety)

   # Reduce monitoring frequency
   ros2 param set /safety_watchdog health_check_interval 5.0

**Configuration Errors**::

   # Validate YAML syntax
   python3 -c "import yaml; yaml.safe_load(open('config/safety_system.yaml'))"

   # Check parameter loading
   ros2 param get /safety_watchdog battery_critical_threshold

   # Reload configuration
   ros2 service call /safety_watchdog reload_config std_srvs/srv/Empty

Integration Validation Checklist
===============================

**System Integration**:
- [ ] ROS2 packages build successfully
- [ ] Launch files execute without errors
- [ ] All safety topics are published
- [ ] Parameter server loads configuration
- [ ] Component communication established

**Safety Functionality**:
- [ ] Heartbeat monitoring active
- [ ] Emergency stop responds within 500ms
- [ ] Safety violations detected and reported
- [ ] Recovery procedures functional
- [ ] Dashboard displays correct status

**Testing Integration**:
- [ ] Automated tests execute successfully
- [ ] Manual test procedures documented
- [ ] Test results properly logged
- [ ] Regression tests integrated into CI/CD

**Deployment Readiness**:
- [ ] Configuration files deployed
- [ ] Launch files tested on target hardware
- [ ] Monitoring scripts functional
- [ ] Emergency procedures practiced by team

This integration guide ensures the safety system is properly integrated into the URC 2026 Mars Rover autonomy stack, providing robust protection during development and competition operations.
