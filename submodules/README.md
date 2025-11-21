# Submodules

This directory contains git submodules for external components that are integrated into the URC 2026 system.

## Teleoperation (`teleoperation/`)
- **Repository**: [SJSURoboticsTeam/urc-teleoperation-2026](https://github.com/SJSURoboticsTeam/urc-teleoperation-2026.git)
- **Purpose**: Web-based teleoperation interface for remote rover control
- **Status**: Readonly - No changes should be made to this submodule from this repository
- **Integration**: Provides the main user interface for teleoperation, communicates via WebSocket/ROS bridge

## Control Systems (`control-systems/`)
- **Repository**: [SJSURoboticsTeam/urc-control-systems-2024](https://github.com/SJSURoboticsTeam/urc-control-systems-2024.git)
- **Purpose**: Low-level control systems for rover drive and arm motion (STM32-based)
- **Status**: Readonly - No changes should be made to this submodule from this repository
- **Integration**: Handles real-time motor control, CAN bus communication, and hardware interfaces

## Guidelines

### Readonly Policy
- **DO NOT** make changes to files within these submodules
- **DO NOT** commit changes to submodule contents from this parent repository
- **DO** update submodule references only when pulling new versions from upstream

### Updating Submodules
```bash
# Update all submodules to latest upstream
git submodule update --remote

# Update specific submodule
git submodule update --remote submodules/teleoperation

# Check submodule status
git submodule status
```

### Development Workflow
1. Make changes in the respective upstream repositories
2. Pull updates here using `git submodule update --remote`
3. Test integration in this repository
4. Commit the submodule reference updates

## Integration Points

### Communication Architecture
```
Teleoperation Frontend (React/WebSocket)
    ↕️ WebSocket/HTTP
ROS Bridge Server (rosbridge_suite)
    ↕️ ROS2 Topics/Services
Autonomy System (Navigation, Planning, Control)
    ↕️ ROS2/CAN Bus
Control Systems (STM32 - Drive/Arm Control)
```

### Key Interfaces
- **WebSocket Bridge**: `/bridges/websocket_*_bridge.py`
- **ROS Bridge**: `rosbridge_server` package
- **CAN Bus Bridge**: Control systems integration
- **Launch Files**: Integration launch configurations

## Contact
For changes to submodule contents, contact the respective repository maintainers:
- Teleoperation: SJSU Robotics Team - Teleoperation subgroup
- Control Systems: SJSU Robotics Team - Control Systems subgroup
