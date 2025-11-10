# 🔍 LED Status Troubleshooting - Issue Resolution Guide

**Comprehensive troubleshooting for LED status system issues with diagnostic procedures and solutions.**

---

## 📋 Quick Reference

| Issue | Symptom | Quick Fix | Section |
|-------|---------|-----------|---------|
| No LED output | LEDs don't light up | Check GPIO connections | [Hardware Issues](#hardware-connection-issues) |
| Wrong colors | LED shows unexpected color | Verify GPIO pin mapping | [Color Problems](#color-accuracy-issues) |
| No pattern changes | LED stuck on one color | Check ROS2 topics | [Communication Issues](#ros2-communication-issues) |
| Dim LEDs | LEDs too faint to see | Adjust brightness/PWM | [Brightness Issues](#brightness-and-visibility-issues) |
| Slow response | LED changes delayed | Check system performance | [Performance Issues](#performance-problems) |
| Competition visibility | Can't see from 50m | Hardware/replacement needed | [URC Compliance](#urc-competition-issues) |

---

## 🔌 Hardware Connection Issues

### Issue: LEDs Not Lighting Up

#### Symptoms
```
LEDs remain dark regardless of commands
No visible light output from any LED
```

#### Root Causes & Solutions

##### **Cause 1: GPIO Pin Misconfiguration**
```bash
# Check GPIO pin modes
sudo raspi-gpio get
# Look for GPIO 18, 19, 20 - should be ALT5 (PWM) or OUTPUT

# Reset GPIO pins
sudo raspi-gpio set 18 ip  # Set as input (safe state)
sudo raspi-gpio set 19 ip
sudo raspi-gpio set 20 ip

# Restart LED system
ros2 launch autonomy_led_status led_status.launch.py
```

##### **Cause 2: Power Supply Issues**
```bash
# Check Raspberry Pi power
vcgencmd get_throttled
# Should show "0x0" (no throttling)

# Verify LED power connections
# Check voltage at LED pins with multimeter
# Expected: 3.3V when LED should be on

# Test power with simple GPIO command
sudo raspi-gpio set 18 op dh  # Set GPIO 18 high
# LED should light if hardware is correct
```

##### **Cause 3: Wiring Problems**
```
LED Connection Checklist:
□ GPIO 18 connected to Red LED anode
□ GPIO 19 connected to Green LED anode  
□ GPIO 20 connected to Blue LED anode
□ GND connected to LED cathode
□ Current limiting resistors installed (330Ω recommended)
□ No shorts between pins
□ LED polarity correct (anode to GPIO, cathode to GND)
```

##### **Cause 4: LED Hardware Failure**
```bash
# Test individual LEDs with direct GPIO control
sudo raspi-gpio set 18 op  # Configure pin
sudo raspi-gpio set 18 dh  # Turn on
# If LED doesn't light, hardware fault

# Test each color separately
for pin in 18 19 20; do
    echo "Testing GPIO $pin"
    sudo raspi-gpio set $pin op dh
    sleep 2
    sudo raspi-gpio set $pin dl
done
```

### Issue: Intermittent LED Operation

#### Symptoms
```
LEDs work sometimes but not consistently
Flickering or random color changes
```

#### Solutions

##### **Power Supply Stability**
```bash
# Monitor power fluctuations
watch -n 1 'vcgencmd measure_volts core'

# Check for voltage drops under load
stress --cpu 4 --timeout 10
# Monitor LED behavior during CPU stress test
```

##### **PWM Interference**
```bash
# Check PWM frequency conflicts
sudo pigpiod -v  # Check daemon status

# Restart PWM daemon
sudo systemctl restart pigpiod

# Test PWM functionality
pigs p 18 128  # Set 50% duty cycle
pigs p 19 128
pigs p 20 128
```

##### **Grounding Issues**
```bash
# Verify common ground
# Check for ground loops between Raspberry Pi and LED power supply
# Ensure LED power supply shares ground with Raspberry Pi
```

---

## 🎨 Color Accuracy Issues

### Issue: Wrong LED Colors

#### Symptoms
```
Red commands show blue light
Colors appear washed out or incorrect
```

#### Diagnostic Steps

##### **GPIO Pin Mapping Verification**
```bash
# Verify pin assignments in configuration
cat ~/ros2_ws/src/autonomy_led_status/config/led_status_config.yaml
# Should show:
# red_pin: 18
# green_pin: 19
# blue_pin: 20

# Test individual pins
sudo raspi-gpio set 18 op dh  # Should be red
sudo raspi-gpio set 19 op dh  # Should be green
sudo raspi-gpio set 20 op dh  # Should be blue
```

##### **RGB Mixing Problems**
```bash
# Test primary colors
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"
# Should show pure red (255, 0, 0)

ros2 topic pub /state_machine/led_info std_msgs/String "data: 'TELEOPERATION_BLUE'"
# Should show pure blue (0, 0, 255)

ros2 topic pub /state_machine/led_info std_msgs/String "data: 'IDLE_GREEN'"
# Should show pure green (0, 255, 0)
```

##### **PWM Duty Cycle Issues**
```bash
# Check PWM values
pigs p 18 255  # Red full on
pigs p 19 0    # Green off
pigs p 20 0    # Blue off
# Should be pure red

# Test intermediate values
pigs p 18 128  # 50% red
pigs p 19 128  # 50% green
pigs p 20 0    # Blue off
# Should be yellow (red + green)
```

### Issue: Color Inconsistency

#### Symptoms
```
Same command shows different colors at different times
Colors drift over time
```

#### Root Causes

##### **Temperature Effects**
```bash
# Monitor temperature
watch -n 5 'vcgencmd measure_temp'

# LEDs may change color characteristics with temperature
# RGB LEDs typically have different temperature coefficients
```

##### **PWM Resolution Issues**
```bash
# Test PWM resolution
for duty in 0 64 128 192 255; do
    pigs p 18 $duty
    echo "Duty cycle: $duty"
    sleep 2
done

# Look for non-linear brightness changes
# 8-bit PWM should give smooth transitions
```

##### **Software Color Mapping**
```bash
# Check color definitions in code
grep -r "LEDColor" ~/ros2_ws/src/autonomy_led_status/
# Verify RGB values are correct:
# RED = (255, 0, 0)
# GREEN = (0, 255, 0)
# BLUE = (0, 0, 255)
```

---

## 📡 ROS2 Communication Issues

### Issue: LED Commands Not Received

#### Symptoms
```
ros2 topic pub commands don't change LED state
LED stuck on previous pattern
```

#### Diagnostic Procedures

##### **Topic Connection Verification**
```bash
# Check topic existence
ros2 topic list | grep led_info
# Should show: /state_machine/led_info

# Check topic info
ros2 topic info /state_machine/led_info
# Should show publishers and subscribers

# Check topic bandwidth
ros2 topic hz /state_machine/led_info
# Should show ~5Hz updates when LED system is running
```

##### **Node Status Check**
```bash
# Verify LED status node is running
ros2 node list | grep led_status
# Should show: /led_status_controller

# Check node info
ros2 node info /led_status_controller
# Should show subscriptions to /state_machine/led_info
```

##### **Message Format Verification**
```bash
# Test direct message format
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"

# Check message receipt
ros2 topic echo /state_machine/led_info --once
# Should show the published message
```

### Issue: State Machine Integration Problems

#### Symptoms
```
State changes don't update LED
LED status out of sync with rover state
```

#### Integration Diagnosis

##### **State Publisher Verification**
```bash
# Check state machine LED publisher
ros2 node list | grep state_machine
# Should show: /state_machine_director

# Verify LED topic publishing
ros2 topic echo /state_machine/led_info &
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS'
}"
# Should see LED info updates
```

##### **State-LED Mapping Issues**
```bash
# Check state mapping configuration
grep -r "AUTONOMOUS_RED" ~/ros2_ws/src/autonomy_state_machine/
# Should map AUTONOMOUS state to AUTONOMOUS_RED pattern

# Test manual state-LED mapping
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS'
}"
sleep 1
ros2 topic echo /state_machine/led_info --once
# Should show "AUTONOMOUS_RED"
```

##### **Timing Synchronization**
```bash
# Check for timing issues between state changes and LED updates
ros2 topic echo /state_machine/current_state &
ros2 topic echo /state_machine/led_info &
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'TELEOPERATION'
}"
# LED should update within 20ms of state change
```

---

## 💡 Brightness and Visibility Issues

### Issue: LEDs Too Dim

#### Symptoms
```
LEDs visible close up but not at distance
Hard to see in bright lighting
```

#### Brightness Solutions

##### **PWM Duty Cycle Adjustment**
```bash
# Increase brightness in configuration
ros2 param set /led_status_controller brightness_max 100

# Test different brightness levels
for brightness in 60 70 80 90 100; do
    ros2 param set /led_status_controller brightness_max $brightness
    ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"
    echo "Brightness: $brightness%"
    sleep 3
done
```

##### **Hardware Brightness Issues**
```bash
# Check current limiting resistors
# Too high resistance reduces brightness
# Recommended: 330Ω for standard LEDs

# Verify LED specifications
# Use high-brightness LEDs for better visibility
# Check forward voltage and current requirements
```

##### **Power Supply Verification**
```bash
# Ensure adequate current capacity
# RGB LED at full brightness: ~60mA (20mA per color)
# Raspberry Pi GPIO max: 16mA per pin (may need external driver)

# Consider external LED driver circuit
# Use transistors or dedicated LED driver IC
```

### Issue: LEDs Too Bright/Overdriven

#### Symptoms
```
LEDs painfully bright
Excessive current draw
LED heating up
```

#### Overdrive Solutions

##### **Reduce PWM Duty Cycle**
```bash
# Lower maximum brightness
ros2 param set /led_status_controller brightness_max 70

# Test visibility at reduced brightness
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"
```

##### **Increase Current Limiting**
```bash
# Increase resistor values
# 330Ω → 470Ω → 680Ω (reduces current and brightness)

# Or use PWM control instead of resistors
# PWM provides better brightness control
```

---

## ⚡ Performance Problems

### Issue: Slow LED Response

#### Symptoms
```
>100ms delay between command and LED change
LED updates lag behind state changes
```

#### Performance Diagnosis

##### **System Load Check**
```bash
# Monitor system resources
top -p $(pgrep -f led_status) -n 1

# Check CPU usage
# LED system should use < 2% CPU

# Check memory usage
free -h
# LED system should use < 15MB RAM
```

##### **PWM Performance Issues**
```bash
# Check PWM frequency
sudo pigpiod -s 1  # 1ms sampling (1000Hz PWM)

# Test PWM response time
time pigs p 18 255
# Should be near instantaneous
```

##### **ROS2 Communication Latency**
```bash
# Measure topic latency
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'" --times 10

# Check DDS configuration
# Ensure reliable transport is enabled
```

### Issue: Pattern Timing Inaccuracy

#### Symptoms
```
Blink patterns not at correct frequency
Fade times wrong duration
```

#### Timing Solutions

##### **PWM Frequency Verification**
```bash
# Check PWM base frequency
sudo pigpiod -f 1000  # Set 1kHz PWM frequency

# Test blink frequency
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'SAFETY_RED_BLINK'"
# Should blink at 5Hz (5 times per second)

# Measure actual frequency with oscilloscope or logic analyzer
```

##### **Software Timing Issues**
```bash
# Check update rate configuration
ros2 param get /led_status_controller update_rate
# Should be 50.0 for smooth fades (50Hz)

# Verify timer implementation
grep -r "Timer" ~/ros2_ws/src/autonomy_led_status/
# Should use ROS2 timers, not Python time.sleep
```

---

## 🏆 URC Competition Issues

### Issue: LED Not Visible from 50m

#### Symptoms
```
LED meets requirements close up but fails 50m visibility test
```

#### Competition Visibility Solutions

##### **LED Specification Upgrade**
```bash
# Use high-brightness LEDs
# Standard LEDs: 200-500 mcd
# High-brightness LEDs: 5000-10000+ mcd

# Check LED datasheet specifications
# Verify luminous intensity in candela (cd)
# Higher cd = better visibility at distance
```

##### **Optical Enhancements**
```bash
# Use LED with built-in lens/reflector
# Add external lens for better beam control
# Consider LED array instead of single LEDs

# Test viewing angle
# LEDs should be visible from wide angles (±60°)
```

##### **Environmental Factors**
```bash
# Test in actual competition lighting
# Consider daylight visibility requirements
# Check for interference from other light sources

# Use appropriate filters if needed
# Red gel filters can improve contrast in bright light
```

### Issue: URC Pattern Non-Compliance

#### Symptoms
```
Judges report incorrect LED patterns
Colors don't match URC specifications
```

#### Compliance Verification

##### **URC Specification Review**
```
URC 2026 LED Requirements:
🔴 Red = Autonomous operation (solid)
🔵 Blue = Teleoperation (solid)  
🟢 Green = Target arrival (flashing)
🔴 Red = Emergency (flashing)

Judge Visibility: 50m distance
Color Accuracy: Unambiguous identification
```

##### **Pattern Testing Against URC Rules**
```bash
# Test each required pattern
patterns=(
    "AUTONOMOUS_RED:Red solid for autonomous"
    "TELEOPERATION_BLUE:Blue solid for teleop"
    "WAYPOINT_SUCCESS:Green flash for success"
    "SAFETY_RED_BLINK:Red blink for emergency"
)

for pattern_info in "${patterns[@]}"; do
    pattern="${pattern_info%%:*}"
    description="${pattern_info##*:}"
    
    echo "Testing URC requirement: $description"
    ros2 topic pub /state_machine/led_info std_msgs/String "data: '$pattern'"
    
    read -p "Does this meet URC specifications? (y/n): " response
    if [[ "$response" != "y" ]]; then
        echo "❌ URC COMPLIANCE ISSUE: $pattern"
    fi
done
```

---

## 🔧 Advanced Diagnostics

### Hardware Oscilloscope Analysis

#### PWM Signal Verification
```
Oscilloscope Setup for LED Diagnosis:
1. Connect probe to GPIO pin (18, 19, or 20)
2. Set trigger to rising edge
3. Measure frequency and duty cycle

Expected PWM Characteristics:
- Frequency: 1000Hz (1kHz)
- Duty Cycle: 0-100% (0-255 in 8-bit)
- Rise/Fall Time: < 100ns
- Noise: < 50mV peak-to-peak
```

#### LED Current Measurement
```
Current Measurement Setup:
1. Insert ammeter in series with LED anode
2. Measure current for each color at 100% PWM
3. Check for current limiting compliance

Expected Current Values:
- Red LED: 15-20mA
- Green LED: 15-20mA  
- Blue LED: 15-20mA
- Total RGB: < 60mA
```

### Software Debugging Tools

#### LED State Machine Logging
```python
#!/usr/bin/env python3
"""
LED State Machine Debugger
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LEDDebugger(Node):
    def __init__(self):
        super().__init__('led_debugger')
        
        # Subscribe to LED commands
        self.led_sub = self.create_subscription(
            String,
            '/state_machine/led_info',
            self.led_callback,
            10
        )
        
        # Track command history
        self.command_history = []
        self.last_command_time = self.get_clock().now()
        
    def led_callback(self, msg):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_command_time).nanoseconds / 1e6  # ms
        
        self.get_logger().info(f"LED Command: {msg.data} (delay: {time_diff:.1f}ms)")
        
        self.command_history.append({
            'command': msg.data,
            'timestamp': current_time,
            'delay': time_diff
        })
        
        self.last_command_time = current_time
        
        # Keep only recent history
        if len(self.command_history) > 100:
            self.command_history.pop(0)

def main():
    rclpy.init()
    debugger = LEDDebugger()
    rclpy.spin(debugger)
    debugger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### GPIO Hardware Monitor
```bash
#!/bin/bash
# GPIO Hardware Monitor Script

echo "=== GPIO LED Hardware Monitor ==="

# Monitor GPIO pin states
while true; do
    echo "$(date): GPIO Status"
    
    # Check LED pins
    for pin in 18 19 20; do
        status=$(sudo raspi-gpio get $pin)
        echo "GPIO $pin: $status"
    done
    
    # Check system voltage
    core_volt=$(vcgencmd measure_volts core)
    echo "Core Voltage: $core_volt"
    
    echo "---"
    sleep 1
done
```

---

## 🚑 Emergency Recovery

### Complete LED System Reset

#### Nuclear Reset (Use Only as Last Resort)
```bash
# 1. Kill all LED-related processes
pkill -f "led_status"
pkill -f "pigpio"

# 2. Reset GPIO pins to safe state
for pin in 18 19 20; do
    sudo raspi-gpio set $pin ip  # Input mode (safe)
done

# 3. Restart PWM daemon
sudo systemctl restart pigpiod

# 4. Clean rebuild LED package
cd ~/ros2_ws
rm -rf build/autonomy_led_status install/autonomy_led_status
colcon build --packages-select autonomy_led_status --symlink-install

# 5. Launch with minimal configuration
source install/setup.bash
ros2 launch autonomy_led_status led_status.launch.py \
    hardware_mode:=real \
    log_level:=debug \
    brightness_max:=50

# 6. Test basic functionality
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"
```

### Component-Level Recovery

#### PWM System Recovery
```bash
# Restart PWM system
sudo systemctl stop pigpiod
sudo systemctl start pigpiod

# Test PWM functionality
pigs p 18 128  # Should not error
pigs p 19 128
pigs p 20 128
```

#### ROS2 Node Recovery
```bash
# Restart LED node only
ros2 lifecycle set /led_status_controller shutdown
ros2 lifecycle set /led_status_controller configure
ros2 lifecycle set /led_status_controller activate

# Check node status
ros2 node info /led_status_controller
```

#### Hardware Recovery
```bash
# Power cycle LED hardware
sudo raspi-gpio set 18 dl  # Ensure off
sudo raspi-gpio set 19 dl
sudo raspi-gpio set 20 dl

# Brief delay for discharge
sleep 2

# Test hardware
sudo raspi-gpio set 18 op dh  # Should light LED
```

---

## 📞 Getting Expert Help

### Diagnostic Information to Collect

Before contacting support, gather this information:

```bash
# System information
uname -a
cat /proc/cpuinfo | grep "Model"

# ROS2 status
ros2 --version
ros2 pkg list | grep autonomy

# LED hardware status
sudo raspi-gpio get 18 19 20

# LED software status
ros2 node list | grep led
ros2 topic list | grep led
ros2 param dump /led_status_controller

# Recent logs
ros2 log info /led_status_controller | tail -50

# Configuration
cat ~/ros2_ws/src/autonomy_led_status/config/led_status_config.yaml

# Test results
./test_led_patterns.sh 2>&1
```

### Common Recovery Workflows

| Issue Category | Primary Diagnostic | Quick Fix | Full Resolution |
|----------------|-------------------|-----------|-----------------|
| **No LED output** | GPIO pin status | Check wiring | Hardware replacement |
| **Wrong colors** | Pin mapping | Verify config | GPIO reassignment |
| **Communication** | ROS2 topics | Restart nodes | Network reconfiguration |
| **Brightness** | PWM duty cycle | Adjust config | Hardware upgrade |
| **Performance** | System load | Reduce rate | Code optimization |
| **URC compliance** | Pattern testing | Update mapping | Hardware replacement |

---

*"LED troubleshooting is part detective work, part electrical engineering - systematic diagnosis always finds the root cause."*
