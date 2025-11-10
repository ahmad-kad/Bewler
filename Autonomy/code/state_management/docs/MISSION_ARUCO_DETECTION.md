# Mission-Specific ArUco Detection System

## Overview

The Mission-Specific ArUco Detection System provides enhanced ArUco tag detection and auto-alignment capabilities for precise robotic operations. This system is specifically designed for missions requiring coplanar alignment with ArUco tags, such as autonomous typing and USB connection operations.

## Architecture

### Core Components

1. **ArUco Alignment Calculator** (`aruco_alignment_calculator.py`)
   - Calculates optimal arm positioning for coplanar alignment
   - Supports rectangular and linear tag layouts
   - Provides alignment quality assessment
   - Generates mission-specific recommendations

2. **Mission ArUco Detector** (`mission_aruco_detector.py`)
   - Mission-specific detection service
   - Integration with alignment calculator
   - Arm alignment command generation
   - ROS2 service interface

3. **Enhanced ArUco Detection Service** (`DetectAruco.srv`)
   - Multi-tag detection capabilities
   - Alignment calculation integration
   - Mission-specific parameters

4. **Mission Detection Service** (`DetectMissionAruco.srv`)
   - Specialized service for mission operations
   - Comprehensive alignment information
   - Mission readiness assessment

## Supported Missions

### 1. Autonomous Typing Mission

**Configuration:**
- **Required Tags**: 4 corner tags (keyboard layout)
- **Tag Layout**: Rectangular
- **Minimum Tags**: 3 (for alignment)
- **Target Depth**: 30cm from keyboard
- **Alignment Tolerance**: 5cm position, 6° orientation

**Use Case:**
- Detect keyboard corners using 4 ArUco tags
- Calculate optimal arm position for typing
- Ensure coplanar alignment with keyboard surface

### 2. USB Connection Mission

**Configuration:**
- **Required Tags**: 2 tags (USB port alignment)
- **Tag Layout**: Linear
- **Minimum Tags**: 2
- **Target Depth**: 20cm from USB port
- **Alignment Tolerance**: 2cm position, 3° orientation

**Use Case:**
- Detect USB port location using 2 ArUco tags
- Calculate precise arm position for USB insertion
- Ensure proper alignment for connection

### 3. Panel Operations Mission

**Configuration:**
- **Required Tags**: 4 corner tags (panel layout)
- **Tag Layout**: Rectangular
- **Minimum Tags**: 3
- **Target Depth**: 40cm from panel
- **Alignment Tolerance**: 8cm position, 9° orientation

**Use Case:**
- Detect panel corners for operations
- Calculate arm position for panel manipulation
- Ensure safe distance for operations

## Usage Examples

### Basic Mission Detection

```python
from autonomy_state_machine.mission_aruco_detector import MissionArUcoDetector

# Initialize detector
detector = MissionArUcoDetector(node)

# Detect tags for autonomous typing
result = detector.detect_mission_tags(
    mission_type="AUTONOMOUS_TYPING",
    required_tag_ids=[1, 2, 3, 4],  # 4 corner tags
    optional_tag_ids=[5, 6],        # Additional tags
    target_depth=0.3,               # 30cm from target
    timeout=5.0
)

if result["success"]:
    print(f"Alignment quality: {result['alignment_quality']:.2f}")
    print(f"Mission ready: {result['mission_ready']}")
    print(f"Arm target position: {result['arm_target_position']}")
```

### Service-Based Detection

```python
# Using ROS2 service directly
from autonomy_interfaces.srv import DetectMissionAruco

# Create service client
client = node.create_client(DetectMissionAruco, "/mission_aruco/detect")

# Create request
request = DetectMissionAruco.Request()
request.mission_type = "USB_CONNECTION"
request.required_tag_ids = [10, 11]
request.optional_tag_ids = [12]
request.detection_timeout = 3.0
request.target_depth = 0.2
request.max_detection_distance = 5.0
request.require_all_tags = False
request.min_alignment_quality = 0.7

# Call service
future = client.call_async(request)
response = future.result()

if response.success:
    print(f"Alignment center: {response.alignment_center}")
    print(f"Arm target: {response.arm_target_position}")
```

### Arm Alignment Command Generation

```python
# Generate arm alignment command
cmd = detector.create_arm_alignment_command(result)

if cmd:
    # Publish command to arm controller
    detector.publish_alignment_command(result)
    print("Arm alignment command published")
```

## Alignment Algorithm

### Rectangular Layout (Typing, Panels)

1. **Center Point Calculation**: Average of all detected tag positions
2. **Plane Normal Calculation**: Using Principal Component Analysis (PCA)
3. **Target Position**: Center point offset by target depth along normal
4. **Quality Assessment**: Based on plane fit and tag count

### Linear Layout (USB Connection)

1. **Center Point Calculation**: Average of detected tag positions
2. **Direction Vector**: Normalized vector between tags
3. **Target Position**: Center point offset by target depth along direction
4. **Quality Assessment**: Based on line fit and tag count

### Quality Scoring

The alignment quality score (0.0-1.0) is calculated based on:

- **Plane/Line Fit**: How well tags fit the expected layout
- **Tag Count**: Penalty for missing required tags
- **Position Accuracy**: Distance from ideal tag positions
- **Mission Requirements**: Specific requirements for each mission type

## Configuration Parameters

### Mission-Specific Settings

```python
mission_configs = {
    "AUTONOMOUS_TYPING": {
        "required_tags": 4,
        "tag_layout": "rectangular",
        "min_tags": 3,
        "target_depth": 0.3,
        "alignment_tolerance": 0.05,
        "orientation_tolerance": 0.1,
    },
    "USB_CONNECTION": {
        "required_tags": 2,
        "tag_layout": "linear",
        "min_tags": 2,
        "target_depth": 0.2,
        "alignment_tolerance": 0.02,
        "orientation_tolerance": 0.05,
    },
    "PANEL_OPERATIONS": {
        "required_tags": 4,
        "tag_layout": "rectangular",
        "min_tags": 3,
        "target_depth": 0.4,
        "alignment_tolerance": 0.08,
        "orientation_tolerance": 0.15,
    }
}
```

### Safety Parameters

- **Maximum Detection Distance**: 5.0 meters
- **Minimum Alignment Quality**: 0.7 (70%)
- **Position Tolerance**: Mission-specific (2-8cm)
- **Orientation Tolerance**: Mission-specific (3-9°)
- **Detection Timeout**: 3-5 seconds

## ROS2 Interfaces

### Topics

- `/arm/alignment_command` - Arm alignment commands
- `/aruco_detection/detect` - Basic ArUco detection

### Services

- `/mission_aruco/detect` - Mission-specific detection
- `/aruco_detection/detect` - Enhanced ArUco detection

### Messages

- `ArmAlignmentCommand` - Arm positioning commands
- `DetectMissionAruco` - Mission detection service
- `DetectAruco` - Enhanced detection service

## Error Handling

### Common Error Scenarios

1. **Insufficient Tags**: Not enough required tags detected
2. **Poor Alignment**: Alignment quality below threshold
3. **Service Unavailable**: ArUco detection service not running
4. **Timeout**: Detection request timed out
5. **Invalid Mission Type**: Unknown mission type specified

### Error Recovery

- **Retry Logic**: Automatic retry with exponential backoff
- **Fallback Detection**: Use optional tags if required tags missing
- **Quality Degradation**: Accept lower quality if mission critical
- **Manual Override**: Allow manual positioning if auto-alignment fails

## Testing

### Unit Tests

```bash
cd /path/to/state_management
python3 test_mission_aruco.py
```

### Test Scenarios

1. **Perfect Alignment**: All required tags detected
2. **Good Alignment**: Most required tags detected
3. **Poor Alignment**: Minimum required tags detected
4. **Service Integration**: ROS2 service calls
5. **Error Handling**: Various error conditions

## Integration with State Machine

### Mission State Integration

The ArUco detection system integrates with the state machine through:

1. **Mission Entry**: Trigger detection when entering mission states
2. **Alignment Validation**: Check alignment quality before proceeding
3. **Error Handling**: Transition to safe state if alignment fails
4. **Status Monitoring**: Continuous alignment monitoring during missions

### State Machine Integration Example

```python
# In state machine director
if substate == AutonomousSubstate.AUTONOMOUS_TYPING:
    # Detect keyboard tags
    result = self.mission_detector.detect_mission_tags(
        mission_type="AUTONOMOUS_TYPING",
        required_tag_ids=[1, 2, 3, 4]
    )
    
    if result["mission_ready"]:
        # Proceed with typing mission
        self.start_typing_mission(result)
    else:
        # Handle alignment failure
        self.handle_alignment_failure(result)
```

## Performance Considerations

### Optimization Strategies

1. **Tag Preprocessing**: Cache tag positions for repeated use
2. **Quality Thresholds**: Adjust thresholds based on mission criticality
3. **Detection Frequency**: Optimize detection rate for mission needs
4. **Memory Management**: Efficient handling of detection results

### Performance Metrics

- **Detection Time**: < 1 second for 4 tags
- **Alignment Calculation**: < 100ms
- **Memory Usage**: < 10MB for typical operations
- **CPU Usage**: < 5% during detection

## Troubleshooting

### Common Issues

1. **Tags Not Detected**
   - Check camera calibration
   - Verify lighting conditions
   - Ensure tags are within detection range
   - Check for obstructions

2. **Poor Alignment Quality**
   - Improve tag positioning
   - Add more optional tags
   - Adjust detection parameters
   - Check camera angle

3. **Service Not Available**
   - Verify ArUco detection service is running
   - Check service dependencies
   - Restart detection services

### Debug Commands

```bash
# Check service availability
ros2 service list | grep aruco

# Test detection service
ros2 service call /aruco_detection/detect autonomy_interfaces/srv/DetectAruco

# Monitor alignment commands
ros2 topic echo /arm/alignment_command

# Check mission detection
ros2 service call /mission_aruco/detect autonomy_interfaces/srv/DetectMissionAruco
```

## Future Enhancements

### Planned Features

1. **Dynamic Tag Layouts**: Support for custom tag arrangements
2. **Multi-Camera Support**: Detection from multiple camera angles
3. **Real-Time Tracking**: Continuous tag tracking during operations
4. **Machine Learning**: Improved alignment quality assessment
5. **Haptic Feedback**: Force feedback during alignment

### Performance Improvements

1. **GPU Acceleration**: Use GPU for faster detection
2. **Parallel Processing**: Concurrent detection and alignment
3. **Predictive Alignment**: Anticipate tag movements
4. **Adaptive Parameters**: Self-tuning detection parameters

## Conclusion

The Mission-Specific ArUco Detection System provides a robust, flexible solution for precise robotic operations requiring coplanar alignment with ArUco tags. The system supports multiple mission types with configurable parameters and comprehensive error handling, making it suitable for complex autonomous operations in the URC 2026 competition.
