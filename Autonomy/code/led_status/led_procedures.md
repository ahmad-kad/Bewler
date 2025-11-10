# 📖 LED Status Procedures - Complete Operational Guide

**Step-by-step procedures for LED status system operation, configuration, and testing.**

---

## 📋 Table of Contents

- [System Setup & Configuration](#system-setup--configuration)
- [LED Pattern Testing](#led-pattern-testing)
- [Hardware Integration](#hardware-integration)
- [State Machine Integration](#state-machine-integration)
- [Competition Preparation](#competition-preparation)
- [Maintenance Procedures](#maintenance-procedures)

---

## 🔧 System Setup & Configuration

### Prerequisites Check

#### Software Requirements
```bash
# Check ROS2 installation
echo "ROS2 Distribution: $ROS_DISTRO"
ros2 --version

# Verify Python GPIO libraries
python3 -c "import RPi.GPIO; print('RPi.GPIO available')"
python3 -c "import pigpio; print('pigpio available')"

# Check system resources
free -h  # At least 100MB free RAM
df -h .  # At least 500MB free disk space
```

#### Hardware Verification
```bash
# Check Raspberry Pi model and GPIO
cat /proc/cpuinfo | grep "Model"
sudo raspi-gpio get  # Show all GPIO pin status

# Verify LED hardware connections
# GPIO 18 (Red) - Physical Pin 12
# GPIO 19 (Green) - Physical Pin 35
# GPIO 20 (Blue) - Physical Pin 38

# Test individual pins
sudo raspi-gpio set 18 op  # Set red pin as output
sudo raspi-gpio set 18 dh  # Set red pin high
sudo raspi-gpio set 18 dl  # Set red pin low
```

### Package Installation

#### Build LED Status Package
```bash
# Navigate to workspace
cd ~/ros2_ws

# Build the LED status package
colcon build --packages-select autonomy_led_status --symlink-install

# Source environment
source install/setup.bash

# Verify package
ros2 pkg list | grep autonomy_led_status
```

#### Configuration Setup
```bash
# Create configuration directory
mkdir -p ~/ros2_ws/src/autonomy_led_status/config

# Copy default configuration
cp ~/ros2_ws/src/autonomy_led_status/config/led_status_config.yaml.example \
   ~/ros2_ws/src/autonomy_led_status/config/led_status_config.yaml

# Edit configuration for your hardware
nano ~/ros2_ws/src/autonomy_led_status/config/led_status_config.yaml
```

### Launch Configuration

#### Basic Launch
```bash
# Launch with default configuration
ros2 launch autonomy_led_status led_status.launch.py

# Launch with custom config
ros2 launch autonomy_led_status led_status.launch.py \
    config_file:=/path/to/custom_config.yaml

# Launch with debug logging
ros2 launch autonomy_led_status led_status.launch.py \
    log_level:=debug

# Launch in simulation mode (no hardware)
ros2 launch autonomy_led_status led_status.launch.py \
    hardware_mode:=simulation
```

#### Parameter Override
```bash
# Override parameters at launch
ros2 launch autonomy_led_status led_status.launch.py \
    hardware_mode:=real \
    log_level:=info \
    update_rate:=5.0 \
    brightness_max:=80
```

---

## 🧪 LED Pattern Testing

### Individual Pattern Testing

#### Core URC Patterns
```bash
# Test autonomous mode (RED)
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"
# Expected: Solid red LED

# Test teleoperation mode (BLUE)
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'TELEOPERATION_BLUE'"
# Expected: Solid blue LED

# Test mission success (GREEN FLASH)
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'WAYPOINT_SUCCESS'"
# Expected: Green LED blinking for 3 seconds

# Test emergency state (RED BLINK)
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'SAFETY_RED_BLINK'"
# Expected: Red LED fast blinking (5Hz)
```

#### Extended Patterns
```bash
# Test boot sequence (YELLOW BLINK)
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'BOOT_YELLOW_BLINK'"
# Expected: Yellow LED slow blinking (1Hz)

# Test calibration (YELLOW SOLID)
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'CALIBRATION_YELLOW'"
# Expected: Solid yellow LED

# Test idle state (GREEN SOLID)
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'IDLE_GREEN'"
# Expected: Solid green LED

# Test shutdown (RED FADE)
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'SHUTDOWN_RED_FADE'"
# Expected: Red LED fading to off
```

### Pattern Sequence Testing

#### Complete State Machine Sequence
```bash
# Simulate full rover lifecycle
echo "Starting LED pattern sequence test..."

# 1. Boot sequence
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'BOOT_YELLOW_BLINK'"
sleep 3

# 2. Calibration
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'CALIBRATION_YELLOW'"
sleep 3

# 3. Ready state
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'IDLE_GREEN'"
sleep 3

# 4. Teleoperation
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'TELEOPERATION_BLUE'"
sleep 3

# 5. Autonomous mission
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"
sleep 5

# 6. Mission success
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'WAYPOINT_SUCCESS'"
sleep 4

# 7. Back to ready
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'IDLE_GREEN'"
```

#### Emergency Sequence Testing
```bash
# Normal operation
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"
sleep 2

# Emergency trigger
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'SAFETY_RED_BLINK'"
sleep 5

# Recovery
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'IDLE_GREEN'"
```

### Performance Testing

#### Response Time Testing
```bash
# Test pattern change response time
time ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"

# Expected: < 20ms total response time
# Use oscilloscope on GPIO pins for precise measurement
```

#### Pattern Accuracy Testing
```bash
# Test blink frequency accuracy
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'SAFETY_RED_BLINK'"

# Measure blink frequency with oscilloscope or logic analyzer
# Expected: 5Hz ± 0.1Hz (200ms period)

# Test fade duration
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'SHUTDOWN_RED_FADE'"

# Measure fade time from 100% to 0% brightness
# Expected: 2.0 seconds ± 0.1 seconds
```

---

## 🔌 Hardware Integration

### GPIO Hardware Setup

#### Raspberry Pi GPIO Configuration
```bash
# Enable I2C and SPI (if needed for advanced LED controllers)
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0

# Install GPIO libraries
sudo apt update
sudo apt install python3-rpi.gpio python3-pigpio

# Start PWM daemon
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Test PWM functionality
pigs p 18 128  # Set GPIO 18 to 50% duty cycle
```

#### LED Circuit Assembly
```
LED Connection Schematic:
┌─────────────────┐
│   Raspberry Pi  │
│                 │
│ GPIO 18 ────────┼───[330Ω]───▶ LED Red Anode
│ GPIO 19 ────────┼───[330Ω]───▶ LED Green Anode
│ GPIO 20 ────────┼───[330Ω]───▶ LED Blue Anode
│ GND ────────────┼───────────── LED Cathode
└─────────────────┘

Current Limiting Resistors:
- 330Ω for standard LEDs (20mA max)
- 1kΩ for high-brightness LEDs
- Adjust based on LED specifications

Power Supply:
- Use 5V rail for LED power (if separate)
- Ensure common ground with Raspberry Pi
- Add decoupling capacitor (10µF) across LED power
```

### Hardware Testing

#### Basic GPIO Testing
```bash
# Test individual LED colors
sudo raspi-gpio set 18 op  # Red pin as output
sudo raspi-gpio set 19 op  # Green pin as output
sudo raspi-gpio set 20 op  # Blue pin as output

# Test red LED
sudo raspi-gpio set 18 dh  # Red on
sleep 1
sudo raspi-gpio set 18 dl  # Red off

# Test green LED
sudo raspi-gpio set 19 dh  # Green on
sleep 1
sudo raspi-gpio set 19 dl  # Green off

# Test blue LED
sudo raspi-gpio set 20 dh  # Blue on
sleep 1
sudo raspi-gpio set 20 dl  # Blue off

# Test color combinations
sudo raspi-gpio set 18 dh  # Red + Green = Yellow
sudo raspi-gpio set 19 dh
sleep 2
sudo raspi-gpio set 18 dl
sudo raspi-gpio set 19 dl
```

#### PWM Functionality Test
```bash
# Start PWM daemon
sudo pigpiod

# Test PWM on red channel
pigs p 18 64   # 25% brightness
sleep 2
pigs p 18 128  # 50% brightness
sleep 2
pigs p 18 192  # 75% brightness
sleep 2
pigs p 18 255  # 100% brightness
sleep 2
pigs p 18 0    # Off

# Test simultaneous PWM
pigs p 18 255  # Red full
pigs p 19 128  # Green half
pigs p 20 64   # Blue quarter
# Should show orange color
```

### Environmental Testing

#### Brightness Testing
```bash
# Test visibility at different distances
# Place LED array at 1m, 10m, 25m, 50m distances

for brightness in 25 50 75 100; do
    echo "Testing brightness: $brightness%"
    pigs p 18 $((brightness * 255 / 100))
    pigs p 19 $((brightness * 255 / 100))
    pigs p 20 $((brightness * 255 / 100))
    read -p "Press Enter to continue..."
done
```

#### Temperature Testing
```bash
# Monitor temperature during operation
watch -n 5 'vcgencmd measure_temp'

# Test LED operation at different temperatures
# Use heat gun or environmental chamber if available
# Verify color accuracy and brightness stability
```

---

## 🔗 State Machine Integration

### Integration Testing

#### Basic Integration Test
```bash
# Launch both systems
ros2 launch autonomy_state_machine state_machine.launch.py &
ros2 launch autonomy_led_status led_status.launch.py &

# Wait for startup
sleep 5

# Monitor both topics
ros2 topic echo /state_machine/current_state &
ros2 topic echo /state_machine/led_info &

# Test state change
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS'
}"

# Verify LED updates with state changes
```

#### Mission Simulation Test
```bash
#!/bin/bash
# LED State Machine Integration Test

echo "=== LED State Machine Integration Test ==="

# Test state transitions and LED responses
transitions=(
    "BOOT:BOOT_YELLOW_BLINK"
    "CALIBRATION:CALIBRATION_YELLOW"
    "IDLE:IDLE_GREEN"
    "TELEOPERATION:TELEOPERATION_BLUE"
    "AUTONOMOUS:AUTONOMOUS_RED"
    "SAFETY:SAFETY_RED_BLINK"
    "IDLE:IDLE_GREEN"
)

for transition in "${transitions[@]}"; do
    state="${transition%%:*}"
    expected_led="${transition##*:}"

    echo "Testing $state -> $expected_led"

    # Change state
    ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
      desired_state: '$state'
    }" >/dev/null 2>&1

    # Check LED status
    led_status=$(ros2 topic echo /state_machine/led_info --once --field data 2>/dev/null | tr -d '"')

    if [[ "$led_status" == "$expected_led" ]]; then
        echo "✅ PASS: $state -> $expected_led"
    else
        echo "❌ FAIL: $state -> $led_status (expected $expected_led)"
    fi

    sleep 2
done

echo "Integration test complete."
```

### Advanced Integration

#### Custom State Mapping
```python
# Custom LED mapping for specialized states
custom_led_mapping = {
    "CUSTOM_MISSION_START": "TRANSITION_WHITE",
    "CUSTOM_MISSION_ACTIVE": "AUTONOMOUS_RED",
    "CUSTOM_MISSION_SUCCESS": "WAYPOINT_SUCCESS",
    "CUSTOM_ERROR": "ERROR_RED_BLINK"
}

# Integration with LED publisher
from autonomy_led_status.led_state_publisher import LEDStatePublisher

led_publisher = LEDStatePublisher()
led_publisher.set_custom_mapping(custom_led_mapping)
```

#### Mission-Specific Patterns
```python
# Mission phase LED indicators
mission_patterns = {
    "APPROACH": "TRANSITION_WHITE",      # White pulse during approach
    "COLLECT": "AUTONOMOUS_RED",         # Red during collection
    "ANALYZE": "CALIBRATION_YELLOW",     # Yellow during analysis
    "RETURN": "TELEOPERATION_BLUE",      # Blue during return
    "SUCCESS": "WAYPOINT_SUCCESS"        # Green flash on success
}

# Apply mission patterns
def update_mission_led(phase):
    pattern = mission_patterns.get(phase, "IDLE_GREEN")
    led_publisher.publish_led_state(pattern)
```

---

## 🏆 Competition Preparation

### URC Compliance Verification

#### Required Pattern Testing
```bash
#!/bin/bash
# URC LED Compliance Test

echo "=== URC 2026 LED Compliance Verification ==="

# Test each required pattern
patterns=(
    "AUTONOMOUS_RED:Solid red for autonomous operation"
    "TELEOPERATION_BLUE:Solid blue for teleoperation"
    "WAYPOINT_SUCCESS:Green flash for target arrival"
    "SAFETY_RED_BLINK:Red blink for emergency"
)

all_passed=true

for pattern_info in "${patterns[@]}"; do
    pattern="${pattern_info%%:*}"
    description="${pattern_info##*:}"

    echo "Testing: $description"

    # Send pattern
    ros2 topic pub /state_machine/led_info std_msgs/String "data: '$pattern'" >/dev/null 2>&1

    # Wait for visual verification
    read -p "Verify LED shows correct pattern (y/n): " response

    if [[ "$response" == "y" || "$response" == "Y" ]]; then
        echo "✅ PASS: $pattern"
    else
        echo "❌ FAIL: $pattern"
        all_passed=false
    fi
done

# Distance visibility test
echo "Testing 50m visibility requirement..."
read -p "Can LED be seen from 50m distance in daylight? (y/n): " distance_test

if [[ "$distance_test" == "y" || "$distance_test" == "Y" ]]; then
    echo "✅ PASS: 50m visibility requirement met"
else
    echo "❌ FAIL: 50m visibility requirement NOT met"
    all_passed=false
fi

# Final result
if $all_passed; then
    echo ""
    echo "🎉 ALL URC COMPLIANCE TESTS PASSED!"
    echo "LED system is ready for competition."
else
    echo ""
    echo "⚠️  SOME COMPLIANCE TESTS FAILED!"
    echo "Address issues before competition."
fi
```

### Judge Visibility Testing

#### Competition Environment Simulation
```bash
# Test in various lighting conditions
echo "Testing LED visibility in different conditions..."

# Bright sunlight simulation (reduce brightness if needed)
ros2 param set /led_status_controller brightness_max 100

# Indoor arena lighting
ros2 param set /led_status_controller brightness_max 80

# Evening/low light conditions
ros2 param set /led_status_controller brightness_max 60

# Test pattern recognition at distance
# Use binoculars or zoom camera to verify from 50m
```

#### Backup LED System
```bash
# Configure redundant LED arrays
led_configs=(
    "primary:red_pin=18,green_pin=19,blue_pin=20"
    "backup:red_pin=21,green_pin=22,blue_pin=23"
)

# Test failover
echo "Testing LED redundancy..."
ros2 launch autonomy_led_status led_status.launch.py \
    primary_config:=${led_configs[0]} \
    backup_config:=${led_configs[1]} \
    redundancy:=true
```

### Competition Day Procedures

#### Pre-Competition Setup
```bash
# 1. Final hardware check
sudo raspi-gpio get 18 19 20  # Verify GPIO status

# 2. Launch systems
ros2 launch autonomy_state_machine state_machine.launch.py &
ros2 launch autonomy_led_status led_status.launch.py &

# 3. Verify initial state
ros2 topic echo /state_machine/current_state --once
# Should show BOOT or IDLE

# 4. Test LED response
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'IDLE_GREEN'"
# LED should show green
```

#### Competition Execution
```bash
# Monitor LED status throughout competition
ros2 topic hz /state_machine/led_info  # Should be 5Hz updates

# Quick LED test during pit stops
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'WAYPOINT_SUCCESS'"

# Emergency LED verification
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'SAFETY_RED_BLINK'"
```

---

## 🔧 Maintenance Procedures

### Regular Maintenance

#### Daily Checks
```bash
# 1. Hardware inspection
# Check LED brightness and color accuracy
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'TELEOPERATION_BLUE'"
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'WAYPOINT_SUCCESS'"

# 2. GPIO pin status
sudo raspi-gpio get 18 19 20

# 3. System resource check
ros2 topic echo /state_machine/led_info --once --field data
```

#### Weekly Maintenance
```bash
# 1. Full pattern test suite
./test_led_patterns.sh

# 2. Performance monitoring
ros2 topic hz /state_machine/led_info --window 60

# 3. Log review
ros2 log info /led_status_controller | tail -100

# 4. Configuration backup
cp ~/ros2_ws/src/autonomy_led_status/config/led_status_config.yaml \
   ~/backups/led_config_$(date +%Y%m%d).yaml
```

### Troubleshooting Maintenance

#### LED Brightness Calibration
```bash
# Calibrate brightness levels
echo "Calibrating LED brightness..."

for level in 25 50 75 100; do
    echo "Testing brightness level: $level%"
    ros2 param set /led_status_controller brightness_max $level
    ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"

    read -p "Is brightness acceptable at $level%? (y/n): " response
    if [[ "$response" == "y" ]]; then
        echo "✅ Selected brightness: $level%"
        break
    fi
done
```

#### Color Accuracy Adjustment
```bash
# Adjust RGB color mixing for accurate colors
echo "Adjusting color accuracy..."

# Test primary colors
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'AUTONOMOUS_RED'"   # Should be pure red
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'TELEOPERATION_BLUE'" # Should be pure blue
ros2 topic pub /state_machine/led_info std_msgs/String "data: 'IDLE_GREEN'"       # Should be pure green

# Adjust PWM values in configuration if colors are off
# Use color meter or camera for precise calibration
```

### Hardware Replacement

#### LED Array Replacement
```bash
# 1. Power down system
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'SHUTDOWN'
}"

# 2. Physically replace LED array
# Maintain same GPIO pin connections
# Ensure proper current limiting resistors

# 3. Test new hardware
sudo raspi-gpio set 18 op dh  # Test red
sudo raspi-gpio set 19 op dh  # Test green
sudo raspi-gpio set 20 op dh  # Test blue

# 4. Restart system
ros2 launch autonomy_led_status led_status.launch.py hardware_mode:=real

# 5. Run full pattern test
./test_led_patterns.sh
```

#### Raspberry Pi Replacement
```bash
# 1. Backup configuration
cp ~/ros2_ws/src/autonomy_led_status/config/led_status_config.yaml /tmp/

# 2. Shutdown and replace Raspberry Pi
# Ensure new Pi has same GPIO pin mappings

# 3. Restore configuration
cp /tmp/led_status_config.yaml ~/ros2_ws/src/autonomy_led_status/config/

# 4. Rebuild and test
colcon build --packages-select autonomy_led_status
source install/setup.bash
ros2 launch autonomy_led_status led_status.launch.py

# 5. Verify functionality
./competition_compliance_test.sh
```

---

## 📊 Performance Validation

### Benchmark Testing

#### Response Time Benchmark
```bash
#!/bin/bash
# LED Response Time Benchmark

echo "=== LED Response Time Benchmark ==="

# Test multiple pattern changes
patterns=("AUTONOMOUS_RED" "TELEOPERATION_BLUE" "WAYPOINT_SUCCESS" "SAFETY_RED_BLINK")

for pattern in "${patterns[@]}"; do
    echo "Testing $pattern response time..."

    # Measure round-trip time
    start_time=$(date +%s%N)
    ros2 topic pub /state_machine/led_info std_msgs/String "data: '$pattern'" >/dev/null 2>&1

    # Wait for processing (adjust based on your system)
    sleep 0.1

    end_time=$(date +%s%N)
    response_time=$(( (end_time - start_time) / 1000000 ))  # Convert to milliseconds

    echo "Response time: ${response_time}ms"

    if (( response_time > 50 )); then
        echo "⚠️  Slow response detected"
    else
        echo "✅ Response time acceptable"
    fi

    sleep 2  # Allow visual verification
done
```

#### Reliability Testing
```bash
#!/bin/bash
# LED System Reliability Test

echo "=== LED System Reliability Test ==="

test_duration=3600  # 1 hour in seconds
pattern_changes=0
failures=0

echo "Running reliability test for $((test_duration/60)) minutes..."

start_time=$(date +%s)

while (( $(date +%s) - start_time < test_duration )); do
    # Random pattern selection
    patterns=("AUTONOMOUS_RED" "TELEOPERATION_BLUE" "IDLE_GREEN" "SAFETY_RED_BLINK")
    random_pattern=${patterns[$RANDOM % ${#patterns[@]}]}

    # Send pattern
    if ros2 topic pub /state_machine/led_info std_msgs/String "data: '$random_pattern'" 2>/dev/null; then
        ((pattern_changes++))
    else
        ((failures++))
        echo "Pattern change failure detected"
    fi

    # Random delay 1-5 seconds
    sleep $((1 + RANDOM % 5))
done

# Calculate results
total_attempts=$((pattern_changes + failures))
success_rate=$((pattern_changes * 100 / total_attempts))

echo ""
echo "=== Reliability Test Results ==="
echo "Total pattern changes: $pattern_changes"
echo "Failures: $failures"
echo "Success rate: ${success_rate}%"
echo "Test duration: $((test_duration/60)) minutes"

if (( success_rate >= 99 )); then
    echo "✅ Reliability test PASSED"
else
    echo "❌ Reliability test FAILED"
fi
```

---

*"LED procedures ensure your rover speaks the universal language of status indication - clearly, reliably, and visibly from afar."*
