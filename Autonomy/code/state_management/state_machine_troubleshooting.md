# 🔍 State Machine Troubleshooting - Issue Resolution Guide

**Comprehensive troubleshooting for state machine issues with diagnostic procedures and solutions.**

---

## 📋 Quick Reference

| Issue | Symptom | Quick Fix | Section |
|-------|---------|-----------|---------|
| Won't start | Node not found | Check build/installation | [System Startup](#system-startup-issues) |
| Stuck in BOOT | No transition to IDLE | Check subsystem status | [Boot Sequence](#boot-sequence-problems) |
| Transition rejected | Service returns false | Check preconditions | [State Transitions](#state-transition-issues) |
| LED not updating | Wrong color/pattern | Check LED integration | [LED Status](#led-status-issues) |
| Safety triggered | Unexpected SAFETY state | Check safety conditions | [Safety System](#safety-system-issues) |
| Performance slow | High latency/delays | Check resource usage | [Performance](#performance-issues) |

---

## 🚀 System Startup Issues

### Issue: State Machine Node Won't Start

#### Symptoms
```
ros2 node list
# state_machine_director NOT listed

ros2 launch autonomy_state_machine state_machine.launch.py
# ERROR: Package 'autonomy_state_machine' not found
```

#### Root Causes & Solutions

##### **Cause 1: Package Not Built**
```bash
# Check if package exists
ros2 pkg list | grep autonomy_state_machine
# If not found:

# Build the package
cd ~/ros2_ws
colcon build --packages-select autonomy_interfaces autonomy_state_machine

# Source environment
source install/setup.bash

# Verify package
ros2 pkg list | grep autonomy
```

##### **Cause 2: Missing Dependencies**
```bash
# Check for missing interfaces
ros2 interface list | grep autonomy_interfaces
# If not found:

# Build interfaces first
colcon build --packages-select autonomy_interfaces
source install/setup.bash

# Then build state machine
colcon build --packages-select autonomy_state_machine
```

##### **Cause 3: ROS2 Environment Issues**
```bash
# Check ROS2 setup
echo $ROS_DISTRO  # Should be 'humble' or similar
echo $AMENT_PREFIX_PATH  # Should point to install directory

# Source setup again
source ~/ros2_ws/install/setup.bash

# Check Python path
python3 -c "import rclpy; print('ROS2 Python OK')"
```

##### **Cause 4: Permission Issues**
```bash
# Check file permissions
ls -la ~/ros2_ws/src/autonomy_state_machine/

# Fix permissions if needed
chmod +x ~/ros2_ws/src/autonomy_state_machine/autonomy_state_machine/*.py
chmod +x ~/ros2_ws/src/autonomy_state_machine/launch/*.py
```

### Issue: Launch File Fails

#### Symptoms
```
ros2 launch autonomy_state_machine state_machine.launch.py
[ERROR] [launch]: process[state_machine_director-1] failed to start
```

#### Solutions

##### **Check Launch File Syntax**
```bash
# Validate launch file
python3 ~/ros2_ws/src/autonomy_state_machine/launch/state_machine.launch.py --check

# Check for Python syntax errors
python3 -m py_compile ~/ros2_ws/src/autonomy_state_machine/launch/state_machine.launch.py
```

##### **Verify Entry Points**
```bash
# Check setup.py entry points
grep "entry_points" ~/ros2_ws/src/autonomy_state_machine/setup.py

# Verify executable exists
ls -la ~/ros2_ws/install/autonomy_state_machine/lib/autonomy_state_machine/
```

##### **Check Launch Arguments**
```bash
# Launch with debug output
ros2 launch autonomy_state_machine state_machine.launch.py --debug

# Check parameter files
ls -la ~/ros2_ws/src/autonomy_state_machine/config/
```

---

## 🔄 Boot Sequence Problems

### Issue: Stuck in BOOT State

#### Symptoms
```
ros2 topic echo /state_machine/current_state
# current_state: BOOT (doesn't change)
```

#### Diagnostic Steps

##### **Check Subsystem Status**
```bash
# Monitor subsystem initialization
ros2 topic echo /state_machine/subsystem_status

# Check for failed subsystems
ros2 topic echo /state_machine/subsystem_status --filter "yaml msg.status != 'ACTIVE'"
```

##### **Verify Boot Timeout**
```bash
# Check boot timeout parameter
ros2 param get /state_machine_director boot_timeout

# Monitor time in BOOT state
ros2 topic echo /state_machine/current_state --field time_in_state
```

##### **Check System Resources**
```bash
# Check memory/CPU
free -h
top -p $(pgrep -f state_machine) -n 1

# Check disk space
df -h ~/ros2_ws
```

### Issue: Boot Timeout Exceeded

#### Symptoms
```
[WARN] Boot timeout exceeded (30.0s)
[ERROR] Failed to complete boot sequence
```

#### Common Causes

##### **Subsystem Initialization Failure**
```bash
# Check which subsystems failed
ros2 service call /state_machine/get_system_state autonomy_interfaces/srv/GetSystemState "{
  include_subsystems: true
}"

# Restart failed subsystems
ros2 service call /state_machine/restart_subsystem autonomy_interfaces/srv/RestartSubsystem "{
  subsystem_name: 'navigation',
  restart_type: 'HARD'
}"
```

##### **Configuration Issues**
```bash
# Check boot configuration
ros2 param get /state_machine_director boot_timeout

# Increase timeout if needed
ros2 param set /state_machine_director boot_timeout 60.0
```

##### **Network/Communication Issues**
```bash
# Check ROS2 discovery
ros2 node list
ros2 topic list | grep state_machine

# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

---

## 🔄 State Transition Issues

### Issue: State Change Request Rejected

#### Symptoms
```
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{desired_state: 'AUTONOMOUS'}"
# success: false
# message: "Precondition check failed"
```

#### Diagnostic Procedures

##### **Check Preconditions**
```bash
# Get detailed failure information
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS',
  desired_substate: 'SCIENCE'
}" --response-only

# Check failed preconditions
# Look for: failed_preconditions field in response
```

##### **Verify Subsystem Readiness**
```bash
# Check subsystem status
ros2 service call /state_machine/get_system_state autonomy_interfaces/srv/GetSystemState "{
  include_subsystems: true
}"

# Required subsystems for AUTONOMOUS:
# - navigation: ACTIVE
# - computer_vision: ACTIVE
# - slam: ACTIVE (optional)
```

##### **Check Safety Conditions**
```bash
# Check safety status
ros2 topic echo /state_machine/safety_status --once

# Look for active safety triggers
# Battery level, temperature, obstacle detection, etc.
```

##### **Validate Transition Matrix**
```bash
# Check allowed transitions from current state
ros2 service call /state_machine/get_transition_matrix autonomy_interfaces/srv/GetTransitionMatrix "{}"

# Current state must allow transition to desired state
```

### Issue: Transition Takes Too Long

#### Symptoms
```
State transition initiated...
# Waits > 5 seconds
State transition completed
```

#### Performance Issues

##### **Subsystem Startup Delays**
```bash
# Monitor subsystem activation
ros2 topic hz /state_machine/subsystem_status

# Check startup times
ros2 topic echo /state_machine/subsystem_status --field startup_time
```

##### **Configuration Timeouts**
```bash
# Check transition timeout
ros2 param get /state_machine_director transition_timeout

# Increase if needed
ros2 param set /state_machine_director transition_timeout 10.0
```

##### **Resource Contention**
```bash
# Check system load
uptime
top -n 1 | head -10

# Check memory usage
free -h
```

---

## 🚨 Safety System Issues

### Issue: Unexpected Safety State Entry

#### Symptoms
```
ros2 topic echo /state_machine/current_state
# current_state: SAFETY (unexpected)
```

#### Safety Trigger Investigation

##### **Check Active Safety Triggers**
```bash
# Get safety status details
ros2 topic echo /state_machine/safety_status

# Look for:
# - battery_level < threshold
# - temperature > threshold
# - obstacle_distance < threshold
# - communication_lost: true
```

##### **Review Safety Logs**
```bash
# Check ROS2 logs for safety events
ros2 log info /state_machine_director | grep -i safety

# Look for trigger sources
ros2 log info /state_machine_director | grep -A5 -B5 "SAFETY"
```

##### **False Positive Investigation**
```bash
# Monitor sensor values
ros2 topic echo /battery/status
ros2 topic echo /temperature/status
ros2 topic echo /obstacle/distance

# Check sensor calibration
# Verify threshold parameters
ros2 param get /state_machine_director battery_critical_threshold
```

### Issue: Safety Recovery Fails

#### Symptoms
```
ros2 service call /state_machine/recover_from_safety ...
# success: false
# message: "Recovery conditions not met"
```

#### Recovery Diagnosis

##### **Check Recovery Preconditions**
```bash
# Verify safety conditions resolved
ros2 topic echo /state_machine/safety_status

# Ensure all safety triggers cleared
# Battery charged, temperature normal, obstacles clear
```

##### **Validate Recovery Method**
```bash
# Check available recovery options
ros2 service call /state_machine/get_recovery_options autonomy_interfaces/srv/GetRecoveryOptions "{}"

# AUTO: Automatic recovery
# MANUAL_GUIDED: Operator-guided recovery
# FULL_RESET: Complete system reset
```

##### **Manual Recovery Steps**
```bash
# Use MANUAL_GUIDED recovery
ros2 service call /state_machine/recover_from_safety autonomy_interfaces/srv/RecoverFromSafety "{
  recovery_method: 'MANUAL_GUIDED',
  operator_id: 'operator1',
  acknowledge_risks: true
}"

# Then transition to safe state
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'TELEOPERATION',
  reason: 'Manual recovery completed'
}"
```

---

## 💡 LED Status Issues

### Issue: LED Status Not Updating

#### Symptoms
```
ros2 topic echo /state_machine/led_info
# No output or wrong values
```

#### LED Integration Diagnosis

##### **Check LED Publisher Status**
```bash
# Verify LED publisher is active
ros2 topic info /state_machine/led_info

# Check publisher rate
ros2 topic hz /state_machine/led_info
```

##### **Verify State-LED Mapping**
```bash
# Check current state and LED info together
ros2 topic echo /state_machine/current_state &
ros2 topic echo /state_machine/led_info &
# Kill with Ctrl+C

# Should show corresponding LED states:
# IDLE → GREEN_SOLID
# AUTONOMOUS → RED_SOLID
# SAFETY → RED_FAST_BLINK
```

##### **LED System Integration**
```bash
# Check if LED system is running
ros2 node list | grep led

# If not running, launch it
ros2 launch autonomy_led_status led_status.launch.py

# Check LED system logs
ros2 log info /led_status_controller
```

### Issue: Wrong LED Colors/Patterns

#### Symptoms
- LED shows blue during autonomous operation
- LED shows red during teleoperation
- LED not flashing when expected

#### State Mapping Issues

##### **Check State-LED Configuration**
```bash
# Verify state mapping
ros2 param get /state_machine_director led_state_mapping

# Should match URC requirements:
# AUTONOMOUS_* → RED_SOLID
# TELEOPERATION → BLUE_SOLID
# SUCCESS → GREEN_BLINK
# SAFETY → RED_FAST_BLINK
```

##### **Manual LED Testing**
```bash
# Test LED patterns directly
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"

# Check LED system response
ros2 log info /led_status_controller | tail -10
```

##### **URC Compliance Verification**
```bash
# Verify URC color requirements
# Red = Autonomous operation
# Blue = Teleoperation
# Green flash = Success
# Red flash = Emergency
```

---

## 📊 Performance Issues

### Issue: High State Transition Latency

#### Symptoms
```
State transition: 500ms (should be < 100ms)
```

#### Performance Diagnosis

##### **Profile Transition Time**
```bash
# Enable performance logging
ros2 param set /state_machine_director enable_performance_logging true

# Monitor transition performance
ros2 topic echo /state_machine/performance_metrics
```

##### **Subsystem Startup Bottlenecks**
```bash
# Check subsystem startup times
ros2 topic echo /state_machine/subsystem_status --field startup_time

# Identify slow subsystems
# navigation startup: 2000ms (should be < 500ms)
```

##### **Resource Constraints**
```bash
# Check system resources
free -h
top -p $(pgrep -f state_machine)

# Check ROS2 performance
ros2 topic hz /state_machine/current_state  # Should be 10Hz
```

### Issue: Memory Leaks or High CPU Usage

#### Symptoms
```
Memory usage: 500MB (growing over time)
CPU usage: 80% (consistently high)
```

#### Resource Analysis

##### **Memory Leak Investigation**
```bash
# Monitor memory usage over time
while true; do
  ps aux | grep state_machine | grep -v grep
  sleep 10
done

# Check for message queue buildup
ros2 topic echo /state_machine/current_state | head -100 | wc -l
```

##### **CPU Profiling**
```bash
# Enable CPU profiling
ros2 param set /state_machine_director enable_cpu_profiling true

# Check thread activity
top -H -p $(pgrep -f state_machine)
```

##### **Optimization Steps**
```bash
# Reduce update rate if needed
ros2 param set /state_machine_director update_rate 5.0

# Disable non-essential features
ros2 param set /state_machine_director enable_state_history false
```

---

## 🔧 Advanced Diagnostics

### State Machine Internal Analysis

#### Transition Matrix Validation
```python
#!/usr/bin/env python3
"""
State Machine Transition Matrix Validator
"""

import rclpy
from rclpy.node import Node
from autonomy_interfaces.srv import GetTransitionMatrix, ChangeState

class TransitionValidator(Node):
    def __init__(self):
        super().__init__('transition_validator')

        # Get transition matrix
        self.cli = self.create_client(GetTransitionMatrix, '/state_machine/get_transition_matrix')
        self.change_cli = self.create_client(ChangeState, '/state_machine/change_state')

    def validate_transitions(self):
        # Get allowed transitions
        req = GetTransitionMatrix.Request()
        future = self.cli.call_async(req)

        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        # Test each transition
        for from_state, to_states in response.transition_matrix.items():
            for to_state in to_states:
                self.test_transition(from_state, to_state)

    def test_transition(self, from_state, to_state):
        # Change to from_state first (if needed)
        # Then attempt transition to to_state
        # Log results

        self.get_logger().info(f'Testing {from_state} -> {to_state}')

def main():
    rclpy.init()
    validator = TransitionValidator()
    validator.validate_transitions()
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subsystem Health Monitoring
```bash
#!/bin/bash
# Subsystem Health Monitor Script

echo "=== State Machine Subsystem Health Monitor ==="

# Monitor subsystem status continuously
while true; do
    echo "$(date): Checking subsystem health..."

    # Get subsystem status
    STATUS=$(ros2 topic echo /state_machine/subsystem_status --once --field status 2>/dev/null)

    if [[ $STATUS == *"FAILED"* ]]; then
        echo "WARNING: Subsystem failure detected!"
        # Send alert or take action
    fi

    sleep 5
done
```

### Log Analysis Tools

#### State Machine Log Parser
```python
#!/usr/bin/env python3
"""
State Machine Log Analyzer
"""

import re
from collections import defaultdict

def analyze_state_machine_logs(log_file):
    """Analyze ROS2 logs for state machine patterns"""

    patterns = {
        'state_changes': r'State changed: (\w+) -> (\w+)',
        'transition_failures': r'Transition failed: (.+)',
        'safety_triggers': r'Safety triggered: (.+)',
        'performance_warnings': r'Performance warning: (.+)'
    }

    stats = defaultdict(int)

    with open(log_file, 'r') as f:
        for line in f:
            for category, pattern in patterns.items():
                if re.search(pattern, line):
                    stats[category] += 1

    print("=== State Machine Log Analysis ===")
    for category, count in stats.items():
        print(f"{category}: {count}")

    # Identify common issues
    if stats['transition_failures'] > stats['state_changes'] * 0.1:
        print("WARNING: High transition failure rate")
    if stats['safety_triggers'] > 5:
        print("WARNING: Frequent safety triggers")

if __name__ == '__main__':
    import sys
    analyze_state_machine_logs(sys.argv[1])
```

---

## 🚑 Emergency Recovery

### Complete State Machine Reset

#### Nuclear Option (Use Only as Last Resort)
```bash
# 1. Stop all related processes
pkill -f "state_machine"
pkill -f "led_status"
pkill -f "autonomy_"

# 2. Clear ROS2 state
ros2 daemon stop
ros2 daemon start

# 3. Reset parameters
ros2 param delete /state_machine_director
ros2 param delete /led_status_controller

# 4. Clean rebuild
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select autonomy_interfaces autonomy_state_machine autonomy_led_status

# 5. Fresh launch
source install/setup.bash
ros2 launch autonomy_state_machine state_machine.launch.py
```

### Partial Recovery Options

#### Subsystem-Specific Restart
```bash
# Restart only failed subsystem
ros2 service call /state_machine/restart_subsystem autonomy_interfaces/srv/RestartSubsystem "{
  subsystem_name: 'navigation',
  restart_type: 'SOFT'
}"

# Force state machine reconfiguration
ros2 service call /state_machine/reconfigure autonomy_interfaces/srv/Reconfigure "{}"
```

#### Configuration Rollback
```bash
# Backup current config
ros2 param dump /state_machine_director > emergency_backup.yaml

# Load known good configuration
ros2 param load /state_machine_director known_good_config.yaml

# Restart state machine
ros2 service call /state_machine/restart std_srvs/srv/Empty
```

---

## 📞 Getting Expert Help

### Diagnostic Information to Collect

Before contacting support, gather this information:

```bash
# System information
uname -a
lsb_release -a
python3 --version

# ROS2 status
echo $ROS_DISTRO
ros2 --version
ros2 pkg list | grep autonomy

# State machine status
ros2 node list
ros2 topic list | grep state_machine
ros2 service list | grep state_machine

# Current state
ros2 service call /state_machine/get_system_state autonomy_interfaces/srv/GetSystemState "{}"

# Recent logs
ros2 log info /state_machine_director | tail -50

# Configuration
ros2 param dump /state_machine_director

# Performance metrics
top -p $(pgrep -f state_machine) -n 1
free -h
```

### Common Recovery Scenarios

| Scenario | Symptoms | Primary Solution | Backup Solution |
|----------|----------|------------------|-----------------|
| Boot hang | Stuck in BOOT > 60s | Check subsystems | Increase boot_timeout |
| Transition failures | Preconditions fail | Fix subsystem issues | Use force transition |
| Safety loops | Constant SAFETY entry | Clear safety triggers | Manual recovery |
| LED problems | Wrong colors/patterns | Check LED integration | Restart LED system |
| Performance issues | High latency | Profile and optimize | Reduce update rate |

---

*"Debugging state machines requires understanding both the forest (system behavior) and the trees (individual transitions)."*
