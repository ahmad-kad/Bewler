# Follow Me Feature Documentation

## Overview

The Follow Me feature allows the URC 2026 rover to autonomously follow a person carrying an ArUco tag at a safe distance. This feature integrates with the hierarchical state machine and provides safe, configurable following behavior.

## Architecture

### State Machine Integration

The Follow Me feature is implemented as a new autonomous substate:

```
SystemState.AUTONOMOUS
└── AutonomousSubstate.FOLLOW_ME
```

### Key Components

1. **Follow Me Behavior** (`follow_me_behavior.py`)
   - Core following logic using ArUco tag detection
   - Safety distance management
   - Speed control and obstacle avoidance
   - ROS2 service integration

2. **Frontend Interface** (`follow_me_frontend.py`)
   - High-level API for operators
   - State transition management
   - Status monitoring and callbacks

3. **ROS2 Interfaces**
   - `DetectAruco.srv` - ArUco tag detection service
   - `FollowMeControl.srv` - Follow me control service
   - `FollowMeStatus.msg` - Status information

## Usage

### Starting Follow Me Mode

```python
from autonomy_state_machine.follow_me_frontend import FollowMeFrontend

# Initialize frontend
frontend = FollowMeFrontend(node)

# Start following ArUco tag ID 42
success = frontend.start_follow_me(
    target_tag_id=42,
    safety_distance=2.0,  # meters
    max_speed=1.0,        # m/s
    operator_id="operator1"
)
```

### Stopping Follow Me Mode

```python
# Stop follow me mode
success = frontend.stop_follow_me(operator_id="operator1")
```

### Monitoring Status

```python
# Check if follow me is active
if frontend.is_follow_me_active():
    print("Follow me is active")

# Get detailed status
status = frontend.get_follow_me_status()
if status:
    print(f"Target distance: {status['target_distance']:.2f}m")
    print(f"Safety violation: {status['safety_violation']}")
```

## Configuration

### Safety Parameters

- **Safety Distance**: 2.0 meters (configurable)
- **Maximum Speed**: 1.0 m/s (configurable)
- **Detection Timeout**: 100ms
- **Control Loop Rate**: 10Hz
- **Status Update Rate**: 2Hz

### ArUco Tag Requirements

- Tag must be visible to rover's camera
- Recommended tag size: 10cm x 10cm
- Tag ID must be known and configured
- Maximum detection distance: 10 meters

## Safety Features

### Distance Management

- **Safety Distance Violation**: Robot stops immediately if too close
- **Deadband**: 20cm buffer around safety distance
- **Proportional Control**: Smooth approach to target distance

### Speed Control

- **Maximum Speed Limit**: Configurable upper bound
- **Angular Velocity Limit**: ±1.0 rad/s
- **Emergency Stop**: Immediate halt on safety violation

### State Integration

- **Safety State Override**: Follow me stops if safety state triggered
- **State Validation**: ArUco detection must be available
- **Graceful Shutdown**: Proper cleanup on state exit

## ROS2 Topics and Services

### Topics

- `/follow_me/status` - Follow me status updates (2Hz)
- `/cmd_vel` - Velocity commands to rover
- `/state_machine/current_state` - System state updates

### Services

- `/aruco_detection/detect` - ArUco tag detection
- `/state_machine/follow_me_control` - Follow me control
- `/state_machine/change_state` - State transitions

## Testing

### Unit Tests

Run the follow me test script:

```bash
cd /path/to/state_management
python3 test_follow_me.py
```

### Integration Tests

1. **State Transition Test**: Verify proper state changes
2. **ArUco Detection Test**: Test tag detection and tracking
3. **Safety Distance Test**: Verify safety distance enforcement
4. **Frontend Interface Test**: Test operator interface

## Troubleshooting

### Common Issues

1. **ArUco Detection Not Available**
   - Check camera calibration
   - Verify ArUco detection service is running
   - Ensure proper lighting conditions

2. **State Transition Fails**
   - Check system state preconditions
   - Verify ArUco detection is available
   - Check for safety state conflicts

3. **Follow Me Behavior Not Starting**
   - Verify state machine is in AUTONOMOUS/FOLLOW_ME
   - Check ArUco tag visibility
   - Verify service connections

### Debug Commands

```bash
# Check state machine status
ros2 service call /state_machine/get_system_state autonomy_interfaces/srv/GetSystemState

# Test ArUco detection
ros2 service call /aruco_detection/detect autonomy_interfaces/srv/DetectAruco "{target_tag_ids: [42], detection_timeout: 1.0}"

# Monitor follow me status
ros2 topic echo /follow_me/status
```

## Implementation Details

### State Machine Integration

The follow me feature integrates with the state machine through:

1. **State Entry Actions**: Enable follow me mode when entering AUTONOMOUS/FOLLOW_ME
2. **State Exit Actions**: Stop follow me behavior when exiting state
3. **Precondition Validation**: Check ArUco detection availability
4. **Safety Integration**: Respond to safety triggers appropriately

### Control Algorithm

The follow me behavior uses a simple proportional control algorithm:

```python
# Distance control
distance_error = current_distance - desired_distance
linear_velocity = kp_distance * distance_error

# Angle control  
angular_velocity = kp_angle * angle_to_target
```

### Safety Monitoring

Continuous safety monitoring includes:

- Distance violation detection
- Speed limit enforcement
- Target visibility monitoring
- Emergency stop conditions

## Future Enhancements

### Planned Features

1. **Dynamic Safety Distance**: Adjust based on speed and terrain
2. **Obstacle Avoidance**: Integrate with navigation system
3. **Multi-Target Following**: Follow multiple tags
4. **Path Planning**: Smooth trajectory generation
5. **Learning Behavior**: Adapt to operator preferences

### Performance Optimizations

1. **Predictive Tracking**: Kalman filter for target prediction
2. **Adaptive Control**: PID controller with auto-tuning
3. **Efficient Detection**: Optimized ArUco detection pipeline
4. **Battery Management**: Power-aware following behavior

## API Reference

### FollowMeFrontend Class

#### Methods

- `start_follow_me(target_tag_id, safety_distance, max_speed, operator_id)` - Start following
- `stop_follow_me(operator_id)` - Stop following
- `get_follow_me_status()` - Get current status
- `is_follow_me_active()` - Check if active
- `set_status_callback(callback)` - Set status callback
- `set_state_callback(callback)` - Set state callback

### FollowMeBehavior Class

#### Methods

- `start_following(target_tag_id, safety_distance, max_speed, operator_id)` - Start behavior
- `stop_following()` - Stop behavior
- `get_status()` - Get behavior status

## Conclusion

The Follow Me feature provides a robust, safe, and configurable way for the URC 2026 rover to follow a person with an ArUco tag. The implementation integrates seamlessly with the existing state machine architecture and provides comprehensive safety features and monitoring capabilities.
