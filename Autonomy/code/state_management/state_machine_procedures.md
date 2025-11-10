# 📖 State Machine Procedures - Complete Operational Guide

**Step-by-step procedures for state machine operation, mission execution, and system management.**

---

## 📋 Table of Contents

- [System Startup & Initialization](#system-startup--initialization)
- [State Transitions](#state-transitions)
- [Mission Execution](#mission-execution)
- [Emergency Procedures](#emergency-procedures)
- [Subsystem Management](#subsystem-management)
- [Configuration Management](#configuration-management)

---

## 🚀 System Startup & Initialization

### Phase 1: Environment Setup

#### Prerequisites Check
```bash
# 1. Verify ROS2 environment
echo "ROS2 Distribution: $ROS_DISTRO"
ros2 --version

# 2. Check workspace setup
ls -la ~/ros2_ws/src/
# Should contain: autonomy_interfaces, autonomy_state_machine

# 3. Verify Python environment
python3 --version  # Should be 3.8+
pip list | grep rclpy

# 4. Check system resources
free -h  # At least 1GB free RAM
df -h .  # At least 2GB free disk space
```

#### Network Configuration
```bash
# 5. Set up ROS2 networking (if using multiple machines)
export ROS_DOMAIN_ID=42  # Use unique ID for your team
export ROS_LOCALHOST_ONLY=0  # Allow network communication

# 6. Verify network connectivity
ros2 multicast receive  # Should see discovery traffic
ros2 multicast send "test"  # Should be visible on other machines
```

### Phase 2: Build & Launch

#### Build System
```bash
# 1. Navigate to workspace
cd ~/ros2_ws

# 2. Build state management package
colcon build --packages-select autonomy_interfaces autonomy_state_machine --symlink-install

# 3. Source environment
source install/setup.bash

# 4. Verify packages
ros2 pkg list | grep autonomy
# Should show: autonomy_interfaces, autonomy_state_machine
```

#### Launch State Machine
```bash
# 1. Launch with default configuration
ros2 launch autonomy_state_machine state_machine.launch.py

# 2. Verify launch success
ros2 node list
# Should show: /state_machine_director

# 3. Check initial state
ros2 topic echo /state_machine/current_state --once
# Should show: current_state: BOOT
```

### Phase 3: Boot Sequence Validation

#### Monitor Boot Progress
```bash
# Watch boot sequence
ros2 topic hz /state_machine/current_state

# Monitor subsystem initialization
ros2 topic echo /state_machine/subsystem_status

# Check for boot completion
timeout 30 ros2 topic echo /state_machine/transitions | head -5
```

#### Expected Boot Sequence
```
Time 0s:   State: BOOT, Substate: NONE
Time 5s:   Subsystem Coordinator: INITIALIZING
Time 10s:  Safety Manager: ACTIVE
Time 15s:  LED Publisher: ACTIVE
Time 20s:  Transition Validator: READY
Time 25s:  BOOT → IDLE (Auto-transition)
Time 30s:  State: IDLE, Ready for commands
```

---

## 🔄 State Transitions

### Basic State Transitions

#### BOOT → IDLE (Automatic)
**Duration:** 25-35 seconds
**Trigger:** System initialization complete
**Validation:** All core subsystems operational

```bash
# Monitor automatic transition
ros2 topic echo /state_machine/transitions --filter "yaml msg.data =~ '*BOOT*IDLE*'"
```

#### IDLE → CALIBRATION (Manual)
**Purpose:** Perform sensor calibration
**Requirements:** System booted, sensors available

```bash
# Request calibration state
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'CALIBRATION',
  reason: 'Starting sensor calibration',
  operator_id: 'operator1'
}"

# Monitor calibration progress
ros2 topic echo /state_machine/current_state
```

#### CALIBRATION → IDLE (Automatic/Manual)
**Trigger:** Calibration complete or manual abort

```bash
# Manual completion (if calibration successful)
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'IDLE',
  reason: 'Calibration completed successfully',
  operator_id: 'operator1'
}"
```

### Autonomous Mission Transitions

#### IDLE → AUTONOMOUS (Mission Start)
**Requirements:**
- All required subsystems active
- No safety conditions
- Mission parameters valid

```bash
# Start science mission
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS',
  desired_substate: 'SCIENCE',
  reason: 'Starting science mission',
  operator_id: 'operator1'
}"

# Start delivery mission
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS',
  desired_substate: 'DELIVERY',
  reason: 'Starting delivery mission',
  operator_id: 'operator1'
}"

# Start equipment servicing
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS',
  desired_substate: 'EQUIPMENT_SERVICING',
  reason: 'Starting equipment servicing',
  operator_id: 'operator1'
}"
```

#### AUTONOMOUS → IDLE (Mission Complete)
**Triggers:**
- Mission objectives achieved
- Timeout reached
- Manual abort requested

```bash
# Manual mission abort
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'IDLE',
  reason: 'Mission abort requested',
  operator_id: 'operator1'
}"
```

### Teleoperation Transitions

#### IDLE → TELEOPERATION (Manual Control)
**Requirements:** Communication link established

```bash
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'TELEOPERATION',
  reason: 'Switching to manual control',
  operator_id: 'operator1'
}"
```

#### TELEOPERATION ↔ AUTONOMOUS (Mode Switching)
```bash
# Autonomous → Teleoperation
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'TELEOPERATION',
  reason: 'Manual intervention required',
  operator_id: 'operator1'
}"

# Teleoperation → Autonomous
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS',
  reason: 'Resuming autonomous operation',
  operator_id: 'operator1'
}"
```

---

## 🎯 Mission Execution

### Science Mission Execution

#### Mission Phases
```
1. APPROACH: Navigate to sample location
2. SAMPLE: Collect soil/rock sample
3. ANALYZE: Perform on-board analysis
4. STORE: Secure sample for transport
5. RETURN: Navigate back to base
```

#### State Flow
```
IDLE → AUTONOMOUS_SCIENCE → SCIENCE substates → IDLE
```

#### Execution Commands
```bash
# Start science mission
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS',
  desired_substate: 'SCIENCE',
  reason: 'Science mission execution',
  operator_id: 'operator1'
}"

# Monitor mission progress
ros2 topic echo /state_machine/mission_status
ros2 topic echo /state_machine/current_state
```

#### Success Criteria
- Sample successfully collected
- Analysis completed
- Sample properly stored
- Return navigation successful
- LED shows green flash on completion

### Delivery Mission Execution

#### Mission Phases
```
1. APPROACH: Navigate to pickup location
2. ACQUIRE: Pick up delivery object
3. TRANSPORT: Navigate to delivery location
4. DELIVER: Deposit object
5. VERIFY: Confirm successful delivery
6. RETURN: Navigate back to base
```

#### State Flow
```
IDLE → AUTONOMOUS_DELIVERY → DELIVERY substates → IDLE
```

### Equipment Servicing Mission

#### Mission Types
- **Solar Panel Servicing**: Clean/adjust panels
- **Cable Management**: Organize/connect cables
- **Sensor Maintenance**: Clean/replace sensors
- **Sample Return**: Load samples for return

#### Equipment Servicing Substates
```
TRAVELING → SAMPLE_DELIVERY → PANEL_OPERATIONS → AUTONOMOUS_TYPING → USB_CONNECTION → FUEL_CONNECTION → BUTTON_OPERATIONS
```

#### Execution Flow
```bash
# Start equipment servicing
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS',
  desired_substate: 'EQUIPMENT_SERVICING',
  reason: 'Equipment maintenance mission',
  operator_id: 'operator1'
}"

# Monitor substate progression
ros2 topic echo /state_machine/current_state --filter "yaml msg.sub_substate"
```

---

## 🚨 Emergency Procedures

### Emergency State Transitions

#### Any State → SAFETY (Emergency Trigger)
**Triggers:**
- Battery critical (< 10%)
- Motor overheat (> 80°C)
- Obstacle collision detected
- Communication loss
- Manual emergency stop

```bash
# Automatic emergency transition (happens automatically)
# Or manual emergency trigger
ros2 service call /state_machine/emergency_stop std_srvs/srv/Empty
```

#### SAFETY State Behavior
- **LED Status:** Red fast blink
- **All autonomous operations:** Stopped
- **Safety systems:** Active monitoring
- **Subsystem status:** Logged and preserved
- **Recovery options:** Manual or automatic

### Safety Recovery Procedures

#### Option 1: Automatic Recovery
```bash
# Request automatic recovery
ros2 service call /state_machine/recover_from_safety autonomy_interfaces/srv/RecoverFromSafety "{
  recovery_method: 'AUTO',
  operator_id: 'operator1',
  acknowledge_risks: true,
  notes: 'Battery recharged, ready to resume'
}"

# Monitor recovery progress
ros2 topic echo /state_machine/safety_status
```

#### Option 2: Manual Guided Recovery
```bash
# Request manual recovery
ros2 service call /state_machine/recover_from_safety autonomy_interfaces/srv/RecoverFromSafety "{
  recovery_method: 'MANUAL_GUIDED',
  operator_id: 'operator1',
  acknowledge_risks: true,
  notes: 'Manual intervention completed'
}"

# Transition to teleoperation for manual control
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'TELEOPERATION',
  reason: 'Manual recovery from safety state',
  operator_id: 'operator1'
}"
```

#### Option 3: Full System Reset
```bash
# Full reset (use only as last resort)
ros2 service call /state_machine/recover_from_safety autonomy_interfaces/srv/RecoverFromSafety "{
  recovery_method: 'FULL_RESET',
  operator_id: 'operator1',
  acknowledge_risks: true,
  notes: 'Complete system reset required'
}"

# System will restart boot sequence
ros2 topic echo /state_machine/current_state  # Should show BOOT
```

### Emergency Response Validation

#### Post-Recovery Checklist
```bash
# 1. Verify system state
ros2 service call /state_machine/get_system_state autonomy_interfaces/srv/GetSystemState "{}"

# 2. Check subsystem health
ros2 topic echo /state_machine/subsystem_status --once

# 3. Verify LED status
ros2 topic echo /state_machine/led_info --once

# 4. Test basic functionality
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'IDLE',
  reason: 'Emergency recovery validation',
  operator_id: 'operator1'
}"
```

---

## 🔧 Subsystem Management

### Subsystem Lifecycle

#### Activation Sequence
```bash
# Monitor subsystem activation during boot
ros2 topic echo /state_machine/subsystem_status

# Expected sequence:
# 1. subsystem_coordinator: INITIALIZING
# 2. safety_manager: ACTIVE
# 3. transition_validator: ACTIVE
# 4. led_state_publisher: ACTIVE
# 5. All subsystems: READY
```

#### Readiness Validation
```python
# Check subsystem readiness programmatically
import rclpy
from autonomy_interfaces.srv import GetSystemState

def check_subsystem_readiness():
    node = rclpy.create_node('readiness_checker')
    client = node.create_client(GetSystemState, '/state_machine/get_system_state')

    request = GetSystemState.Request()
    request.include_subsystems = True

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response = future.result()
    for subsystem in response.active_subsystems:
        print(f"✅ {subsystem}: ACTIVE")

    node.destroy_node()
```

### Subsystem Health Monitoring

#### Continuous Monitoring
```bash
# Monitor subsystem health in real-time
ros2 topic hz /state_machine/subsystem_status

# Check for subsystem failures
ros2 topic echo /state_machine/subsystem_status --filter "yaml msg.status != 'ACTIVE'"
```

#### Health Check Commands
```bash
# Manual health check
ros2 service call /state_machine/get_system_state autonomy_interfaces/srv/GetSystemState "{
  include_subsystems: true
}"

# Check specific subsystem
ros2 topic echo /state_machine/subsystem_status --filter "yaml msg.name == 'navigation'"
```

### Subsystem Recovery

#### Automatic Recovery
- State machine automatically attempts recovery
- Failed subsystems are restarted
- Alternative configurations used if available

#### Manual Recovery
```bash
# Force subsystem restart
ros2 service call /state_machine/restart_subsystem autonomy_interfaces/srv/RestartSubsystem "{
  subsystem_name: 'navigation',
  restart_type: 'SOFT',
  operator_id: 'operator1'
}"
```

---

## ⚙️ Configuration Management

### Configuration File Structure

#### Main Configuration File
```yaml
# config/state_machine_config.yaml
state_machine_director:
  ros__parameters:

    # Timing parameters
    update_rate: 10.0                    # State update frequency (Hz)
    boot_timeout: 30.0                   # Boot completion timeout (s)
    transition_timeout: 5.0              # State transition timeout (s)

    # Safety parameters
    battery_critical_threshold: 10.0     # Battery emergency threshold (%)
    temperature_warning_threshold: 70.0  # Temperature warning (°C)
    temperature_critical_threshold: 85.0 # Temperature emergency (°C)
    obstacle_stop_distance: 0.3          # Emergency stop distance (m)

    # Mission timeouts
    mission_timeouts:
      autonomous_navigation: 1800.0      # 30 minutes
      science: 900.0                     # 15 minutes
      delivery: 1200.0                   # 20 minutes
      equipment_servicing: 2400.0        # 40 minutes

    # Subsystem requirements
    required_subsystems:
      autonomous: ["navigation", "computer_vision", "slam"]
      teleoperation: ["navigation"]
      calibration: []

    # LED configuration
    led_enabled: true
    led_update_rate: 5.0                 # LED update frequency (Hz)

    # Logging configuration
    log_level: "INFO"                    # ROS2 log level
    enable_state_history: true
    max_history_entries: 100
```

### Dynamic Reconfiguration

#### Runtime Parameter Changes
```bash
# Update safety thresholds
ros2 param set /state_machine_director battery_critical_threshold 15.0

# Change mission timeout
ros2 param set /state_machine_director mission_timeouts.autonomous_navigation 2400.0

# Enable/disable LED system
ros2 param set /state_machine_director led_enabled false
```

#### Configuration Validation
```bash
# Validate current configuration
ros2 param dump /state_machine_director > current_config.yaml

# Check for invalid parameters
ros2 param list /state_machine_director | grep -v "^/" | sort
```

### Configuration Backup & Restore

#### Backup Current Configuration
```bash
# Create configuration backup
ros2 param dump /state_machine_director > config_backup_$(date +%Y%m%d_%H%M%S).yaml

# Backup all parameters
ros2 param dump > full_system_backup.yaml
```

#### Restore Configuration
```bash
# Load configuration from file
ros2 param load /state_machine_director config_backup_20241201_143000.yaml

# Verify loaded parameters
ros2 param get /state_machine_director battery_critical_threshold
```

---

## 📊 Advanced Operations

### State Machine Analysis

#### Transition History Analysis
```bash
# Get recent transition history
ros2 service call /state_machine/get_transition_history autonomy_interfaces/srv/GetTransitionHistory "{
  limit: 20,
  include_timestamps: true
}"

# Analyze transition patterns
python3 -c "
import subprocess
import json

# Get transition history
result = subprocess.run(['ros2', 'service', 'call', '/state_machine/get_transition_history',
                        'autonomy_interfaces/srv/GetTransitionHistory', '{limit: 50}'],
                       capture_output=True, text=True)

# Parse and analyze
# (Add your analysis logic here)
"
```

#### Performance Profiling
```bash
# Profile state transition performance
ros2 topic hz /state_machine/transitions

# Monitor CPU/memory usage
top -p \$(pgrep -f state_machine_director)

# Check for memory leaks
ros2 topic echo /state_machine/current_state | head -1000 | wc -l
```

### Custom Mission Integration

#### Adding Custom Mission States
```python
# Extend state definitions
from autonomy_state_machine.states import AutonomousSubstate

class CustomExplorationMission(AutonomousSubstate):
    def __init__(self):
        super().__init__()
        self.name = "EXPLORATION"
        self.required_subsystems = ["navigation", "mapping", "exploration_planner"]
        self.max_duration = 3600.0  # 1 hour

    def execute_mission(self):
        # Custom mission logic
        self.navigate_to_exploration_area()
        self.perform_systematic_search()
        self.return_to_base()

# Register with state machine
state_machine.register_mission_state(CustomExplorationMission())
```

#### Mission Parameter Configuration
```yaml
# Custom mission parameters
custom_missions:
  exploration:
    search_pattern: "spiral"
    exploration_radius: 50.0  # meters
    data_collection_interval: 5.0  # seconds
    return_threshold: 0.8  # battery level

  survey:
    waypoints: [[0,0], [10,0], [10,10], [0,10]]
    sensor_config: "high_resolution"
    data_rate: 10.0  # Hz
```

---

## ✅ Operational Checklists

### Pre-Mission Checklist
- [ ] System booted successfully (BOOT → IDLE)
- [ ] All required subsystems active
- [ ] Safety systems operational
- [ ] LED status functioning
- [ ] Communication links established
- [ ] Battery level adequate
- [ ] Calibration data current
- [ ] Mission parameters configured
- [ ] Emergency procedures reviewed

### Mission Execution Checklist
- [ ] State transition to AUTONOMOUS successful
- [ ] Mission substate correct
- [ ] LED status shows red (autonomous)
- [ ] Subsystem coordination active
- [ ] Safety monitoring active
- [ ] Progress monitoring operational
- [ ] Emergency stop accessible
- [ ] Communication maintained

### Post-Mission Checklist
- [ ] Mission completed successfully
- [ ] State returned to IDLE
- [ ] LED status shows green/idle
- [ ] System logs reviewed
- [ ] Data collected and stored
- [ ] Subsystems properly deactivated
- [ ] Configuration backed up
- [ ] System ready for next mission

---

*"The state machine is the conductor of the rover orchestra - ensuring every component plays its part in perfect harmony."*
