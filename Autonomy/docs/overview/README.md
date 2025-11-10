# URC 2026 Autonomy System Documentation - 40 DAYS TO COMPETITION!

## CRITICAL TIME ALERT: 40 Days Remaining

### AGGRESSIVE DEVELOPMENT SPRINT (40 Days Total)
- **Days 1-8**: Core Infrastructure & Basic Functionality
- **Days 9-16**: Feature Integration & Testing
- **Days 17-24**: Performance Optimization
- **Days 25-32**: Environmental Adaptation & Robustness
- **Days 33-40**: Competition Preparation & Finalization

### IMMEDIATE PRIORITIES (Next 7 Days):
- [ ] All ROS 2 packages created and compiling
- [ ] Basic sensor integration operational
- [ ] Fundamental algorithms implemented
- [ ] Core subsystem communication established

---

## Overview
Comprehensive documentation for the University Rover Challenge 2026 autonomy system, covering development, deployment, and operation of autonomous rover capabilities.

## 📚 **Documentation Structure (Organized by Purpose)**

```
🎯 docs/ (Documentation Hub)
├── 📖 overview/ (High-level Understanding & Architecture)
│   ├── README.md (Main overview - Start Here!)
│   ├── TechnicalOverview.md (PRIMARY TOP-DOWN ENTRY POINT - Complete system overview)
│   ├── SystemArchitecture.md (Complete technical architecture)
│   ├── InterfaceContract.md (ROS2 interface specifications)
│   ├── GoalsAndSuccessMetrics.md (Competition requirements & success criteria)
│   ├── UniversityRoverChallenge2026.md (Official URC 2026 competition rules)
│   ├── DistributedArchitecture.md (Distributed systems design)
│   ├── ExternalSystemsIntegration.md (External system integration)
│   ├── ChartsOverview.md (System visualization & diagrams)
│   └── Robotics_Engineering_Concepts_Guide.md (Core robotics concepts)
│
├── 🚀 onboard/ (Setup, Installation & Getting Started)
│   ├── PROJECT_SETUP_GUIDE.md (Project setup instructions)
│   ├── fundamental_setup_requirements.md (Fundamental requirements)
│   ├── Documentation_Navigation_Guide.md (Navigation guide)
│   ├── LearningTracks.md (Structured learning paths)
│   ├── TeamHandbook.md (Team operations guide)
│   ├── TeamIntegration.md (Team integration procedures)
│   ├── guides/ (Development guides)
│   │   ├── DevelopmentPipeline.md (Testing & CI/CD pipeline)
│   │   └── ProjectStatus.md (Project status tracking)
│   └── reference/ (Technical reference guides)
│       ├── SensorGuide.md (Sensor specifications)
│       ├── CalibrationGuide.md (Setup procedures)
│       ├── CameraCalibrationGuide.md (Camera calibration)
│       ├── IMUCalibrationGuide.md (IMU calibration)
│       ├── TFTreeSetupGuide.md (TF tree configuration)
│       ├── TimeSynchronizationGuide.md (Time sync setup)
│       ├── HealthMonitoringGuide.md (System monitoring)
│       ├── ExtrinsicCalibrationGuide.md (Multi-sensor calibration)
│       └── [8 more specialized guides...]
│
└── 📋 tasks/ (Task Management & Development Workflow)
    ├── TaskBasedHandbook.md (Task management methodology)
    ├── DevelopmentWorkflow.md (Code development processes)
    └── PID_Control_Diagrams.md (Control system diagrams)

🔧 code/ (Implementation with TODO roadmaps)
├── navigation/ (Path planning & GPS navigation)
├── slam/ (Localization & mapping)
├── computer_vision/ (Object detection & tracking)
├── autonomous_typing/ (Arm control & typing)
├── state_management/ (Mission coordination)
└── led_status/ (Status signaling)

📋 subsystems/ (Requirements & specifications)
├── navigation/Navigation_PathPlanning.md
├── slam/SLAM.md
├── computer_vision/ComputerVision_ObjectClassification.md
├── autonomous_typing/AutonomousTyping.md
├── state_management/StateManagement_ModeControl.md
└── led_status/LED_StatusSignaling.md

🚀 development/ (Environment setup)
├── QUICKSTART.md (5-minute setup)
├── ONBOARDING.md (Detailed checklist)
├── docker/ (Container environment)
└── SimulationSetup.md (Gazebo testing)

📦 deployment/ (Hardware deployment)
├── RaspberryPiDeployment.md
└── ClientServerArchitecture.md
```

## ⚠️ **CRITICAL CALIBRATION & SYSTEM GUIDES - COMPETITION READINESS**

**🚨 HIGH PRIORITY: These critical guides were previously missing and are essential for competition success:**

### **🔧 Critical Calibration Guides (Newly Created)**
| Guide | Purpose | Competition Impact | Status |
|-------|---------|-------------------|---------|
| [**IMU Calibration Guide**](../onboard/reference/IMUCalibrationGuide.md) | Accelerometer/gyroscope calibration for navigation accuracy | Prevents 5-15° heading drift, enables <3m waypoint accuracy | ✅ **Complete** |
| [**TF Tree Setup Guide**](../onboard/reference/TFTreeSetupGuide.md) | Complete transform tree configuration for multi-sensor fusion | Enables accurate coordinate transformations for SLAM/navigation | ✅ **Complete** |
| [**Time Synchronization Guide**](../onboard/reference/TimeSynchronizationGuide.md) | NTP/PTP/ROS2 clock sync for sensor fusion | Prevents temporal misalignment causing localization failures | ✅ **Complete** |
| [**Health Monitoring Guide**](../onboard/reference/HealthMonitoringGuide.md) | System diagnostics and failure detection/recovery | Enables autonomous operation through failure detection and recovery | ✅ **Complete** |
| [**Extrinsic Calibration Guide**](../onboard/reference/ExtrinsicCalibrationGuide.md) | Camera-LiDAR calibration and hand-eye calibration | Critical for SLAM accuracy and autonomous manipulation | ✅ **Complete** |

### **🎯 Competition Readiness Checklist**
- [ ] **IMU calibrated** - Bias <0.1 m/s², drift <0.1°/minute
- [ ] **TF tree configured** - All sensor transforms accurate (<1cm, <1°)
- [ ] **Time sync established** - <10ms inter-sensor timestamp alignment
- [ ] **Health monitoring active** - All critical subsystems monitored
- [ ] **Extrinsic calibration complete** - Camera-LiDAR error <5 pixels, hand-eye <5mm

**📖 Start with:** [`TechnicalOverview.md`](TechnicalOverview.md) for system understanding, then follow the critical calibration guides above.

### **🚨 Critical Path for Competition Success**
1. **[`TechnicalOverview.md`](TechnicalOverview.md)** (60-90 min) - System understanding
2. **[`../onboard/reference/IMUCalibrationGuide.md`](../onboard/reference/IMUCalibrationGuide.md)** (45-60 min) - Navigation foundation
3. **[`../onboard/reference/TFTreeSetupGuide.md`](../onboard/reference/TFTreeSetupGuide.md)** (45-60 min) - Sensor integration
4. **[`../onboard/reference/TimeSynchronizationGuide.md`](../onboard/reference/TimeSynchronizationGuide.md)** (30-45 min) - Data fusion
5. **[`../onboard/reference/ExtrinsicCalibrationGuide.md`](../onboard/reference/ExtrinsicCalibrationGuide.md)** (60-90 min) - Multi-sensor calibration
6. **[`../onboard/reference/HealthMonitoringGuide.md`](../onboard/reference/HealthMonitoringGuide.md)** (45-60 min) - System reliability

---

## 📢 **Recent Documentation Updates**

**🔄 Documentation Reorganization (November 2024):** TechnicalOverview.md has been streamlined by moving detailed technical content to specialized documents. This improves navigation and reduces redundancy while maintaining comprehensive coverage.

- **See:** This README has been updated to reflect the new organization structure
- **Main document** is now more focused on high-level understanding and navigation
- **Technical details** moved to appropriate specialized guides

**🆕 Critical Calibration Guides Added (October 2025):** Previously missing calibration and system guides have been created to ensure competition readiness. These cover IMU calibration, TF tree setup, time synchronization, health monitoring, and extrinsic calibration - all critical for autonomous operation.

## ⚠️ Competition Compliance Requirements

All autonomy development must comply with **URC 2026 Competition Rules**. Key constraints:

### **Mission 1.e: Equipment Servicing**
- **Autonomous Typing**: 3-6 letter launch key input with backspace/delete correction capability
- **ArUco Markers**: 2x2cm markers at keyboard corners, 1x1cm at USB slot corners
- **Operator Intervention**: Only allowed to exit autonomous mode or abort/restart tasks
- **USB Reading**: GNSS coordinates from memory card in USB slot

### **Mission 1.f: Autonomous Navigation**
- **LED Status Indicators**: 🔴 Red (autonomous), 🔵 Blue (teleoperation), 🟢 Flashing Green (success)
- **Target Specifications**:
  - 2 GNSS-only locations (3m accuracy)
  - 2 posts with 3-sided visual markers (20 x 20 cm faces, 2.5 cm cells, 2m accuracy)
  - 3 ground objects (10m accuracy): mallet, rock pick hammer, water bottle
- **Visit Order**: Flexible (any sequence allowed)
- **Object Interaction**: Detection only (no manipulation required)
- **C2 Display**: Must highlight detected objects, one at a time
- **Abort Protocol**: Return to previous location with 20% teleoperation penalty

### **General Limitations**
- **No Antenna Camera**: Cameras cannot be mounted on antenna (Section 3.b.iii)

---

## 🎯 **Top-Down Documentation Access**

### **🗺️ Primary Navigation Guide**
**New to the project?** Use our comprehensive navigation guide:
- **[`../onboard/Documentation_Navigation_Guide.md`](../onboard/Documentation_Navigation_Guide.md)** - Complete guide to accessing docs top-down

### **🚀 Quick Start by Role**

#### **For New Team Members (Recommended Path):**
1. **[`TechnicalOverview.md`](TechnicalOverview.md)** (60-90 min) - **Start Here!** Complete top-down understanding
2. **[`../onboard/LearningTracks.md`](../onboard/LearningTracks.md)** (45-60 min) - Choose your learning path
3. **Setup Environment**: `../QUICKSTART.md` or `../ONBOARDING.md`
4. **Your Subsystem**: Choose from `../subsystems/` directory

#### **For Developers:**
1. **[`TechnicalOverview.md`](TechnicalOverview.md)** - System understanding
2. **[`SystemArchitecture.md`](SystemArchitecture.md)** - Technical architecture
3. **[`InterfaceContract.md`](InterfaceContract.md)** - ROS2 interfaces
4. **Subsystem TODOs** - Implementation details

#### **For Competition Teams:**
1. **[`GoalsAndSuccessMetrics.md`](GoalsAndSuccessMetrics.md)** - Success criteria
2. **[`TechnicalOverview.md`](TechnicalOverview.md)** - Technical requirements
3. **Subsystem Docs** - Competition specifications
4. **[`../onboard/reference/EnvironmentalChallenges.md`](../onboard/reference/EnvironmentalChallenges.md)** - Desert conditions

### **📚 Learning & Reference Materials**
- **Complete Learning Paths**: [`../onboard/LearningTracks.md`](../onboard/LearningTracks.md) - Structured skill development
- **Technical Deep-Dive**: [`TechnicalOverview.md`](TechnicalOverview.md) - ELI5 to expert level
- **Success Metrics**: [`GoalsAndSuccessMetrics.md`](GoalsAndSuccessMetrics.md) - Competition requirements
- **Quick Setup**: `../QUICKSTART.md` - 5-minute environment setup
- **Detailed Onboarding**: `../ONBOARDING.md` - Step-by-step checklist

### For Development:
- **Simulation Testing**: `../development/SimulationSetup.md`
- **Hardware Deployment**: `../deployment/RaspberryPiDeployment.md`
- **Distributed Systems**: `../deployment/ClientServerArchitecture.md`
- **Environmental Considerations**: `../onboard/reference/EnvironmentalChallenges.md`

### **Quick Task Access - Jump to TODOs:**
| **Subsystem** | **Subsystem TODO** | **Status** | **Key Deliverables** |
|---------------|-------------|------------|-------------------|
| **Navigation** | [`../code/navigation/navigation_TODO.md`](../code/navigation/navigation_TODO.md) | ✅ Ready | Navigation system status |
| **SLAM** | [`../code/slam/slam_TODO.md`](../code/slam/slam_TODO.md) | 🔄 Active | Pose estimation (<0.5m accuracy), 2D occupancy mapping, sensor fusion (LIDAR+IMU), loop closure detection |
| **Computer Vision** | [`../code/computer_vision/computer_vision_TODO.md`](../code/computer_vision/computer_vision_TODO.md) | 🔄 Active | Mallet hammer/water bottle detection, AprilTag localization, depth sensing, real-time processing (10Hz) |
| **Autonomous Typing** | [`../code/autonomous_typing/autonomous_typing_TODO.md`](../code/autonomous_typing/autonomous_typing_TODO.md) | ✅ Ready | Autonomous typing system status |
| **State Management** | [`../code/state_management/state_management_TODO.md`](../code/state_management/state_management_TODO.md) | 🔄 Active | Teleop↔autonomous mode switching (<1s), subsystem health monitoring, mission sequencing, error handling |
| **LED Status** | [`../code/led_status/led_status_TODO.md`](../code/led_status/led_status_TODO.md) | 🔄 Active | Judge-visible status signaling (50m), mode indication (red/green/blue), error pattern display, PWM brightness control |

### For Technical Reference:
- **🔴 IMU Calibration**: `../onboard/reference/IMUCalibrationGuide.md` *(Critical for navigation)*
- **🔴 TF Tree Setup**: `../onboard/reference/TFTreeSetupGuide.md` *(Critical for sensor fusion)*
- **🔴 Time Synchronization**: `../onboard/reference/TimeSynchronizationGuide.md` *(Critical for multi-sensor fusion)*
- **🔴 Health Monitoring**: `../onboard/reference/HealthMonitoringGuide.md` *(Critical for reliability)*
- **🔴 Extrinsic Calibration**: `../onboard/reference/ExtrinsicCalibrationGuide.md` *(Critical for SLAM/manipulation)*
- **Camera Calibration**: `../onboard/reference/CalibrationGuide.md`
- **Docker Analysis**: `../onboard/reference/DockerSuitabilityAnalysis.md`
- **Hybrid Development**: `../onboard/reference/HybridDevelopmentGuide.md`
- **Sensors & Hardware**: `../onboard/reference/SensorGuide.md`
- **Software Libraries**: `../onboard/reference/LibrariesGuide.md`
- **Distributed Systems**: `../onboard/reference/DistributedSystemsGuide.md`
- **Subsystem APIs**: `../onboard/reference/APIGuide.md`

## MCP Servers for Enhanced Development

This project includes specialized **Model Context Protocol (MCP) servers** that provide AI assistants with advanced capabilities for robotics development:

### Available MCP Servers:
- **FileSystem Server**: Advanced file operations and code management
- **Git Server**: Complete version control operations
- **Terminal Server**: Shell command execution with process management
- **Python Server**: Python development, testing, and analysis tools
- **Docker Server**: Container lifecycle and orchestration management
- **ROS2 Server**: ROS2-specific operations and introspection
- **Lint Server**: Code quality analysis and formatting
- **Docs Server**: Documentation generation and validation

### Setup Instructions:
```bash
# Navigate to MCP servers directory
cd mcp-servers

# Install dependencies
npm install

# Test a server
npm run start:filesystem
```

**📖 Full documentation**:
- **Setup Guide**: See `mcp-servers/README.md` for detailed setup and usage instructions
- **Justification Guide**: See `mcp-servers/MCP_SERVERS_GUIDE.md` for comprehensive explanation of why each server is needed and its benefits

## Mission Overview

The autonomy system enables the rover to complete two primary missions:

### Autonomous Navigation Mission (1.f)
- Navigate to 7 targets across 2km terrain in 30 minutes
- Targets: 2 GNSS-only locations, 2 AR-tagged posts, 3 ground objects
- Requires real-time coordination of all subsystems

### Equipment Servicing Mission (1.e)
- Interact with keyboard to input 3-6 letter launch codes
- Precision manipulation with error correction capabilities

## Development Workflow

### Phase 1: Environment Setup
```bash
# Choose your development approach:
# Option A: Docker (Recommended)
cd development/docker
./scripts/docker-dev.sh build
./scripts/docker-dev.sh start

# Option B: Cross-platform native
# Follow development/CrossPlatformDevelopment.md
```

### Phase 2: Subsystem Development
```bash
# Choose your subsystem
cd subsystems/[navigation|slam|computer_vision|etc.]

# Read requirements
cat *_PathPlanning.md  # Technical requirements
cat *Guide.md          # Implementation guide

# Start coding
cd ../../code/[subsystem]/
# Implement following TODO.md phases
```

### Phase 3: Integration & Testing
```bash
# Simulation testing
cd development/
ros2 launch mars_simulation simulation.launch.xml

# Hardware deployment
cd ../deployment/
# Follow RaspberryPiDeployment.md or ClientServerArchitecture.md
```

## Team Organization

### Subsystem Responsibilities

#### Navigation & Path Planning
- **Lead**: Path planning algorithms, terrain classification
- **Files**: `subsystems/navigation/`, `code/navigation/`
- **Hardware**: GPS, IMU, wheel encoders
- **Key Deliverables**: Autonomous waypoint navigation, obstacle avoidance

#### SLAM (Localization & Mapping)
- **Lead**: Sensor fusion, pose estimation, mapping
- **Files**: `subsystems/slam/`, `code/slam/`
- **Hardware**: Lidar, RGB-D camera, IMU
- **Key Deliverables**: Real-time localization, occupancy mapping

#### Computer Vision & Object Classification
- **Lead**: Object detection, ArUco tracking, visual servoing
- **Files**: `subsystems/computer_vision/`, `code/computer_vision/`
- **Hardware**: RGB-D cameras, GPU acceleration
- **Key Deliverables**: Ground object detection, C2 visualization

#### Autonomous Typing System
- **Lead**: Robotic manipulation, keyboard interaction
- **Files**: `subsystems/autonomous_typing/`, `code/autonomous_typing/`
- **Hardware**: Robotic arm, force sensors
- **Key Deliverables**: Precision manipulation, error correction

#### State Management & Mode Control
- **Lead**: Mission coordination, fault handling
- **Files**: `subsystems/state_management/`, `code/state_management/`
- **Hardware**: Central processing, communication
- **Key Deliverables**: Autonomous/teleop switching, health monitoring

#### LED Status Signaling
- **Lead**: Visual status indication, hardware control
- **Files**: `subsystems/led_status/`, `code/led_status/`
- **Hardware**: LED arrays, GPIO control
- **Key Deliverables**: Judge-visible status signaling

### Development Timeline
- **Week 1-4**: Environment setup, basic subsystem implementation
- **Week 5-8**: Core algorithm development, integration testing
- **Week 9-12**: System integration, performance optimization
- **Week 13-16**: Competition preparation, documentation

## Integration Architecture

```
State Management (Central Coordinator)
    ├── Navigation & Path Planning
    ├── SLAM (Localization & Mapping)
    ├── Computer Vision & Classification
    ├── Autonomous Typing System
    └── LED Status Signaling
```

### Communication Patterns
- **ROS 2 DDS**: Real-time, reliable communication
- **Topic-based**: Sensor data streaming, status updates
- **Service-based**: Command execution, configuration
- **Action-based**: Long-running tasks (navigation, manipulation)

### Data Flow
1. **Sensors** → **Preprocessing** → **Algorithm Processing** → **Control Commands**
2. **State Management** coordinates all subsystems
3. **SLAM** provides localization for navigation
4. **Computer Vision** supports navigation and manipulation
5. **LED System** provides status feedback

## Success Metrics

### Technical Requirements
- **Navigation**: 100% target reach within 3m accuracy (GNSS), 2m (AR tags)
- **SLAM**: <1m drift over 30 minutes, >95% loop closure detection
- **Computer Vision**: >90% object detection, reliable ArUco tracking
- **Autonomous Typing**: >90% sequence completion, <2mm positioning
- **State Management**: <100ms mode switching, >95% uptime

### Development Milestones
- **Week 4**: All subsystems with basic functionality
- **Week 8**: Integrated system running in simulation
- **Week 12**: Hardware integration completed
- **Week 16**: Competition-ready system validated

## Development Tools

### Required Software
- **ROS 2 Humble**: Robotics middleware
- **Docker**: Containerized development
- **Gazebo**: Simulation environment
- **Python 3.10+**: Primary development language
- **OpenCV/PyTorch**: Computer vision and ML

### Recommended Tools
- **VS Code**: Development environment with ROS extension
- **Git**: Version control with LFS for large files
- **RViz**: ROS visualization tool
- **PlotJuggler**: Data analysis and plotting

## Support & Resources

### Getting Help
1. Check the relevant subsystem documentation
2. Review the development pipeline guide
3. Check existing issues and solutions
4. Contact subsystem leads for specific questions

### Key Resources
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **OpenCV Tutorials**: Computer vision fundamentals
- **PyTorch Documentation**: Deep learning development
- **Gazebo Tutorials**: Simulation environment

### Communication
- **Team Meetings**: Weekly progress updates
- **Documentation**: Keep guides updated with new findings
- **Code Reviews**: Required for all subsystem changes
- **Issue Tracking**: Use repository issues for bugs and features

## Competition Preparation

### Pre-Competition Checklist
- [ ] All subsystems integrated and tested
- [ ] Simulation validation completed
- [ ] Hardware deployment verified
- [ ] Environmental testing passed
- [ ] Documentation finalized
- [ ] Backup procedures established

### Competition Day
- [ ] System calibration completed
- [ ] Health checks passed
- [ ] Communication links verified
- [ ] Emergency procedures reviewed
- [ ] Team roles assigned and confirmed

---

**This documentation provides the foundation for successful development of the URC 2026 autonomy system. Follow the guides in order and maintain regular communication across subsystems for optimal results.**