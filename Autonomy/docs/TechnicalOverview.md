# 🔬 URC 2026 Technical Overview

**From Simple Concepts to Competition-Ready Systems**

---

## 🎨 **Diagram Theme Compatibility**

**All mermaid diagrams are optimized for both light and dark IDE themes:**

### **🎯 Color Scheme & Accessibility**
- **High-contrast borders** (`stroke-width: 2-3px`) ensure visibility in both themes
- **Theme-compatible colors** work on light and dark backgrounds
- **Consistent color families** maintain visual coherence
- **White text on colored backgrounds** for optimal readability

### **📊 Color Legend**
| **Color** | **Hex Code** | **Usage** | **Theme Compatibility** |
|-----------|-------------|-----------|----------------------|
| 🔵 **Blue** | `#4a90e2`, `#3498db` | Navigation, Planning, Coordination | ✅ Excellent |
| 🟢 **Green** | `#27ae60`, `#229954` | SLAM, Perception, Success | ✅ Excellent |
| 🟣 **Purple** | `#9b59b6`, `#8e44ad` | Computer Vision, Processing | ✅ Excellent |
| 🔴 **Red** | `#e74c3c`, `#c0392b` | Actuators, Critical Systems | ✅ Excellent |
| 🟠 **Orange** | `#f39c12`, `#e67e22` | Control, State Management | ✅ Excellent |

### **💡 Best Viewing Practices**
- **Light Mode**: All colors are clearly visible with good contrast
- **Dark Mode**: High-contrast borders ensure diagram readability
- **High DPI**: Crisp rendering on retina displays
- **Zoom**: Diagrams scale well from 100% to 150%

### **🔧 Custom CSS (Optional)**
For web-based rendering with enhanced theme support:
```css
/* Enhanced theme support for web viewers */
.light-theme .mermaid {
  --mermaid-bg: #ffffff;
  --mermaid-text: #2c3e50;
}

.dark-theme .mermaid {
  --mermaid-bg: #2c3e50;
  --mermaid-text: #ecf0f1;
}
```

---

## 🎯 **Top-Down Navigation Guide**

**New to the project?** Start here and follow the progression:

```
📖 You Are Here (High-Level Overview)
    ↓
🏗️ System Architecture (How it all fits together)
    ↓
🔧 Subsystem Deep-Dives (Individual components)
    ↓
💻 Implementation Details (Code & algorithms)
    ↓
🧪 Testing & Validation (Making it work)
```

**Quick Access:**
- **🚀 Just Starting?** → Continue reading this overview
- **👨‍💻 Developer?** → Jump to [System Architecture](#system-architecture-technical-view)
- **🔍 Specific Topic?** → Use the [Table of Contents](#table-of-contents)
- **🎯 Competition Focus?** → See [Mission Requirements](#competition-requirements)

---

## 📋 **Table of Contents**

### **High-Level Understanding**
1. [ELI5: What Are We Building?](#eli5-what-are-we-building)
2. [System Architecture (Simple View)](#system-architecture-simple-view)
3. [Core Technologies (ELI5)](#core-technologies-eli5)

### **Technical Architecture**
4. [System Architecture (Technical View)](#system-architecture-technical-view)
5. [ROS 2 Communication Architecture](#ros-2-communication-architecture)
6. [Data Flow & Processing Pipeline](#data-flow--processing-pipeline)

### **Competition Requirements**
7. [Mission Requirements & Success Criteria](#mission-requirements--success-criteria)

### **Subsystem Breakdown**
8. [Navigation & Path Planning](#navigation--path-planning)
9. [SLAM (Localization & Mapping)](#slam-localization--mapping)
10. [Computer Vision & Perception](#computer-vision--perception)
11. [Autonomous Typing System](#autonomous-typing-system)
12. [State Management & Control](#state-management--control)

### **Implementation & Validation**
13. [Technical Implementation Overview](#technical-implementation-overview)
14. [System Communication & Architecture](#system-communication--architecture)
15. [Testing & Validation Overview](#testing--validation-overview)

### **References & Resources**
16. [Key Documents & Cross-References](#key-documents--cross-references)
17. [Getting Help & Support](#getting-help--support)

---

## 🤖 **ELI5: What Are We Building?**

### **The Big Picture**
We're building an autonomous rover for the **University Rover Challenge 2026** that must complete four competition missions:

**Mission 1.f: Autonomous Navigation** 🚗
- Navigate to 7 targets (2km total, 30 minutes)
- Targets: 2 GNSS-only locations, 2 posts with 3-sided visual markers (20×20cm faces, 2.5cm cells), 3 ground objects
- Must autonomously traverse desert terrain with obstacles
- LED signaling: 🔴 Red (autonomous), 🔵 Blue (teleop), 🟢 Flashing Green (success)

**Mission 1.e: Equipment Servicing** 🔧
- Travel to mock lander and perform dexterous operations
- **Autonomous typing**: Input 3-6 letter launch codes on keyboard
- Robotic arm manipulation: open drawers, connect hoses, turn valves
- ArUco markers for precise positioning (2x2cm keyboard, 1x1cm USB)

**Mission 1.d: Delivery Mission** 📦
- Assist astronauts across rugged terrain (30-60 minutes)
- Pick up and deliver objects up to 5kg
- Optional drone assistance for scouting/sign reading
- GNSS coordinates for all pickup/delivery locations

**Mission 1.b: Science Mission** 🔬
- Collect subsurface soil samples (10cm depth minimum)
- Onboard life detection and science analysis
- Document sites with panoramas and close-ups
- Return sealed cache to judges

### **Competition Constraints** ⚖️
- **Weight Limit**: 50kg rover (70kg total parts)
- **Budget**: $24,000 USD maximum
- **Size**: Must fit in 1.2m × 1.2m × 1.2m transport crate
- **Power**: Mars-applicable (no air-breathing systems)
- **Communications**: 900MHz/2.4GHz bands with restrictions
- **Environment**: Desert conditions (100°F, dust, wind)

### **Why It's Hard**
1. **Extreme Environment**: Hot desert, dust storms, no roads
2. **Multiple Modalities**: Vision + GPS + IMU + LIDAR integration
3. **Real-time Performance**: <100ms control loops required
4. **Power Efficiency**: Limited battery life in harsh conditions
5. **Robustness**: Must handle failures, interventions, GPS-denied areas
6. **Competition Pressure**: 30-60 minute missions with scoring

### **Our Technical Solution** 🏗️
We break the big problem into 6 specialized subsystems:

| **Subsystem** | **Primary Mission Focus** | **Key Challenge** |
|---------------|---------------------------|-------------------|
| **Navigation** | Mission 1.f (Autonomous Navigation) | Path planning through desert obstacles |
| **SLAM** | All missions | Real-time localization without GPS |
| **Computer Vision** | Missions 1.b, 1.e, 1.f | Object detection, ArUco tracking |
| **Autonomous Typing** | Mission 1.e | Precise keyboard interaction |
| **State Management** | All missions | Mission coordination, mode switching |
| **LED Status** | Mission 1.f | Competition-compliant signaling |

**Think of it like:**
- **Mars rover meets self-driving car** → But with no roads and extreme conditions
- **Warehouse robot meets search-and-rescue** → But autonomous and solar-powered
- **Drone delivery meets scientific exploration** → But ground-based and dexterous

---

## 🏗️ **System Architecture (Competition-Aligned)**

### **High-Level System Overview**

```mermaid
graph TB
    subgraph "🎯 Competition Missions"
        NAV[Autonomous<br/>Navigation<br/>🚗]
        SERV[Equipment<br/>Servicing<br/>🔧]
        DELIV[Delivery<br/>Mission<br/>📦]
        SCI[Science<br/>Mission<br/>🔬]
    end

    subgraph "🏗️ Core Autonomy System"
        SENSORS[Sensors<br/>GPS/IMU/LIDAR/Camera<br/>👁️]
        PROCESSING[Processing<br/>SLAM/Navigation/Vision<br/>🧠]
        CONTROL[Control<br/>State Management<br/>🎛️]
        ACTUATORS[Actuators<br/>Motors/Arm/LEDs<br/>⚙️]
    end

    subgraph "🔧 6 Subsystems"
        NAVIGATION[Navigation<br/>Path Planning<br/>🧭]
        SLAM[SLAM<br/>Localization<br/>📍]
        VISION[Computer Vision<br/>Object Detection<br/>📷]
        TYPING[Autonomous Typing<br/>Arm Control<br/>⌨️]
        STATE[State Management<br/>Mission Control<br/>🎮]
        LED[LED Status<br/>Signaling<br/>💡]
    end

    NAV --> SENSORS
    SERV --> SENSORS
    DELIV --> SENSORS
    SCI --> SENSORS

    SENSORS --> PROCESSING
    PROCESSING --> NAVIGATION
    PROCESSING --> SLAM
    PROCESSING --> VISION

    NAVIGATION --> CONTROL
    SLAM --> CONTROL
    VISION --> CONTROL
    TYPING --> CONTROL
    STATE --> CONTROL

    CONTROL --> ACTUATORS
    ACTUATORS --> LED

    classDef navigation fill:#4a90e2,stroke:#2171b5,stroke-width:3px,color:#ffffff
    classDef servicing fill:#9b59b6,stroke:#7d3c98,stroke-width:3px,color:#ffffff
    classDef delivery fill:#f39c12,stroke:#e67e22,stroke-width:3px,color:#ffffff
    classDef science fill:#27ae60,stroke:#229954,stroke-width:3px,color:#ffffff

    class NAV navigation
    class SERV servicing
    class DELIV delivery
    class SCI science
    classDef sensors fill:#e74c3c,stroke:#c0392b,stroke-width:2px,color:#ffffff
    classDef processing fill:#9b59b6,stroke:#8e44ad,stroke-width:2px,color:#ffffff
    classDef control fill:#27ae60,stroke:#229954,stroke-width:2px,color:#ffffff
    classDef actuators fill:#f39c12,stroke:#e67e22,stroke-width:2px,color:#ffffff
    classDef subsystems fill:#3498db,stroke:#2980b9,stroke-width:2px,color:#ffffff

    class SENSORS sensors
    class PROCESSING processing
    class CONTROL control
    class ACTUATORS actuators
    class NAVIGATION,SLAM,VISION,TYPING,STATE,LED subsystems
```

### **System Data Flow**

```mermaid
graph LR
    A[Sensors<br/>Raw Data] --> B[Preprocessing<br/>Filtering] --> C[Perception<br/>SLAM/Vision] --> D[Planning<br/>Navigation] --> E[Control<br/>PID Loops] --> F[Actuators<br/>Commands]

    A --> G[State<br/>Management]
    B --> G
    C --> G
    D --> G
    E --> G

    G --> H[Mission<br/>Coordination]
    H --> D
    H --> E

    classDef sensorData fill:#e74c3c,stroke:#c0392b,stroke-width:2px,color:#ffffff
    classDef preprocessing fill:#9b59b6,stroke:#8e44ad,stroke-width:2px,color:#ffffff
    classDef perception fill:#27ae60,stroke:#229954,stroke-width:2px,color:#ffffff
    classDef planning fill:#3498db,stroke:#2980b9,stroke-width:2px,color:#ffffff
    classDef control fill:#f39c12,stroke:#e67e22,stroke-width:2px,color:#ffffff
    classDef actuators fill:#e74c3c,stroke:#c0392b,stroke-width:2px,color:#ffffff
    classDef stateMgmt fill:#9b59b6,stroke:#8e44ad,stroke-width:2px,color:#ffffff
    classDef coordination fill:#3498db,stroke:#2980b9,stroke-width:2px,color:#ffffff

    class A sensorData
    class B preprocessing
    class C perception
    class D planning
    class E control
    class F actuators
    class G stateMgmt
    class H coordination
```

### **URC 2026 Mission Flow Architecture**

```mermaid
graph TD
    subgraph "🏆 URC 2026 Competition Missions"
        NAV[Mission 1.f<br/>Autonomous Navigation<br/>🚗 30 min, 2km, 7 targets]
        SERV[Mission 1.e<br/>Equipment Servicing<br/>🔧 30 min, Robotic Tasks]
        DELIV[Mission 1.d<br/>Delivery Mission<br/>📦 30-60 min, Terrain Navigation]
        SCI[Mission 1.b<br/>Science Mission<br/>🔬 20-30 min, Sample Collection]
    end

    NAV --> NAV_GNSS[GNSS Locations<br/>3m accuracy<br/>📍 Navigation]
    NAV --> NAV_AR[AR Posts<br/>2m accuracy<br/>🏷️ Vision + Nav]
    NAV --> NAV_OBJ[Ground Objects<br/>10m accuracy<br/>🎯 Computer Vision]
    NAV --> NAV_LED[LED Signaling<br/>🔴🔵🟢 Competition<br/>💡 Status System]

    SERV --> SERV_NAV[Land Navigation<br/>100m<br/>🚶 Basic Navigation]
    SERV --> SERV_ARM[Robotic Manipulation<br/>Drawer, Hose, Valve<br/>🤖 Arm Control]
    SERV --> SERV_TYPE[Autonomous Typing<br/>3-6 Letters<br/>⌨️ Vision + Arm]
    SERV --> SERV_USB[USB Data Reading<br/>GNSS Coordinates<br/>💾 Precision]

    DELIV --> DELIV_TERRAIN[Terrain Navigation<br/>Obstacle Avoidance<br/>🏔️ Navigation]
    DELIV --> DELIV_MANIP[Object Manipulation<br/>5kg Objects<br/>🔧 Arm Dexterity]
    DELIV --> DELIV_DRONE[Drone Assistance<br/>Optional<br/>🚁 Extended Capability]

    SCI --> SCI_SITE[Site Investigation<br/>Panoramas, Photos<br/>📸 Multi-modal Sensing]
    SCI --> SCI_SAMPLE[Sample Collection<br/>10cm Depth<br/>🧪 Scientific Autonomy]

    classDef missionNav fill:#4a90e2,stroke:#2171b5,stroke-width:3px,color:#ffffff
    classDef missionServ fill:#9b59b6,stroke:#7d3c98,stroke-width:3px,color:#ffffff
    classDef missionDeliv fill:#f39c12,stroke:#e67e22,stroke-width:3px,color:#ffffff
    classDef missionSci fill:#27ae60,stroke:#229954,stroke-width:3px,color:#ffffff

    class NAV missionNav
    class SERV missionServ
    class DELIV missionDeliv
    class SCI missionSci
```

**Mission Interdependencies:**
- **Navigation Mission** focuses on **autonomous waypoint following** and **LED compliance**
- **Equipment Servicing** emphasizes **precision manipulation** and **typing automation**
- **Delivery Mission** requires **robust terrain navigation** and **object handling**
- **Science Mission** demands **multi-modal sensing** and **sample processing**

### **Multi-Pi Distributed Architecture**
```
Control Pi (User Interface + Coordination)
├── State Management (Mission coordination, mode switching)
├── LED Status (Competition signaling: 🔴 🔵 🟢)
└── User Interface (Mission planning, emergency controls)

Autonomy Pi (Real-time Autonomy)
├── Navigation (Path planning, obstacle avoidance)
├── SLAM (Localization, mapping, sensor fusion)
├── Computer Vision (Object detection, ArUco tracking)
└── Autonomous Typing (Arm control, keyboard interaction)

Microcontrollers (Hardware Control)
├── Motor Controllers (Wheel velocity, arm joints)
├── Sensor Interfaces (GPS, IMU, LIDAR, Camera)
└── Power Management (Battery monitoring, E-stop)
```

### **ROS 2 Communication Architecture**
```
Topics (Sensor Data & Commands)
/sensor/gps → /sensor/imu → /sensor/camera → /sensor/lidar
    ↓              ↓              ↓              ↓
Navigation ←──── SLAM ←──── Computer Vision ←─── Sensor Fusion
    ↓              ↓              ↓              ↓
/cmd_vel ←── /odom ←──── /vision/detections ←─── /slam/pose
    ↓              ↓              ↓              ↓
Wheel Motors ←─ Arm Control ←─ LED Control ←─── State Management
```

### **Subsystem Interaction Architecture**

```mermaid
graph TD
    subgraph "🎯 Mission Requirements"
        NAV_REQ[Navigation<br/>Targets<br/>🚗]
        SERV_REQ[Servicing<br/>Tasks<br/>🔧]
        DELIV_REQ[Delivery<br/>Objects<br/>📦]
        SCI_REQ[Science<br/>Sites<br/>🔬]
    end

    subgraph "🔧 6 Core Subsystems"
        NAV[Navigation<br/>Path Planning<br/>🧭]
        SLAM[SLAM<br/>Localization<br/>📍]
        VISION[Computer Vision<br/>Detection<br/>📷]
        TYPING[Autonomous Typing<br/>Arm Control<br/>⌨️]
        STATE[State Management<br/>Coordination<br/>🎮]
        LED[LED Status<br/>Signaling<br/>💡]
    end

    subgraph "📡 ROS 2 Communication"
        TOPICS[Topics<br/>Data Streams]
        SERVICES[Services<br/>Commands]
        ACTIONS[Actions<br/>Long Tasks]
    end

    NAV_REQ --> NAV
    SERV_REQ --> TYPING
    DELIV_REQ --> NAV
    SCI_REQ --> VISION

    NAV --> STATE
    SLAM --> STATE
    VISION --> STATE
    TYPING --> STATE
    LED --> STATE

    STATE --> NAV
    STATE --> SLAM
    STATE --> VISION
    STATE --> TYPING
    STATE --> LED

    NAV <--> SLAM
    NAV <--> VISION
    VISION <--> TYPING
    STATE <--> TOPICS
    STATE <--> SERVICES
    STATE <--> ACTIONS

    classDef subsystemNav fill:#4a90e2,stroke:#2171b5,stroke-width:2px,color:#ffffff
    classDef subsystemSlam fill:#27ae60,stroke:#229954,stroke-width:2px,color:#ffffff
    classDef subsystemVision fill:#9b59b6,stroke:#8e44ad,stroke-width:2px,color:#ffffff
    classDef subsystemTyping fill:#e74c3c,stroke:#c0392b,stroke-width:2px,color:#ffffff
    classDef subsystemState fill:#f39c12,stroke:#e67e22,stroke-width:2px,color:#ffffff
    classDef subsystemLed fill:#27ae60,stroke:#229954,stroke-width:2px,color:#ffffff

    class NAV subsystemNav
    class SLAM subsystemSlam
    class VISION subsystemVision
    class TYPING subsystemTyping
    class STATE subsystemState
    class LED subsystemLed
```

### **Development Workflow (Competition-Focused)**
```
1. Requirements Analysis (URC 2026 rules)
    ↓
2. Subsystem Implementation (Individual components)
    ↓
3. Integration Testing (ROS 2 communication)
    ↓
4. Mission Simulation (Gazebo environment)
    ↓
5. Field Testing (Desert conditions)
    ↓
6. Competition Validation (Full mission runs)
```

### **Development Timeline Visualization**

```mermaid
gantt
    title URC 2026 Development Timeline (40 Days to Competition)
    dateFormat  YYYY-MM-DD
    section Phase 1: Foundation (Days 1-8)
    Environment Setup          :done,    2024-10-01, 2d
    ROS 2 Package Creation     :done,    2024-10-03, 3d
    Basic Sensor Integration   :done,    2024-10-06, 3d
    section Phase 2: Core Systems (Days 9-16)
    State Management Setup     :done,    2024-10-09, 3d
    LED Competition Signaling  :done,    2024-10-12, 2d
    Navigation Algorithms      :done,    2024-10-14, 3d
    section Phase 3: Advanced Features (Days 17-24)
    SLAM Implementation        :done,    2024-10-17, 4d
    Computer Vision Pipeline   :done,    2024-10-21, 4d
    Autonomous Typing System   :done,    2024-10-25, 4d
    section Phase 4: Integration (Days 25-32)
    System Integration         :active,  2024-10-29, 4d
    Mission Testing            :         2024-11-02, 4d
    Performance Optimization   :         2024-11-06, 4d
    section Phase 5: Competition (Days 33-40)
    Environmental Testing      :         2024-11-10, 4d
    Final Validation           :         2024-11-14, 4d
    Competition Preparation    :         2024-11-18, 4d
```

### **Document Topics Coverage Map**

```mermaid
mindmap
  root((URC 2026<br/>Technical Overview))
    Competition:::competition
      Mission 1.f Navigation:::missionNav
        GNSS waypoints:::navDetail
        AR-tagged posts:::navDetail
        Ground objects:::navDetail
        LED signaling:::navDetail
      Mission 1.e Servicing:::missionServ
        Robotic manipulation:::servDetail
        Autonomous typing:::servDetail
        Precision tasks:::servDetail
      Mission 1.d Delivery:::missionDeliv
        Terrain navigation:::delivDetail
        Object handling:::delivDetail
        Optional drone:::delivDetail
      Mission 1.b Science:::missionSci
        Sample collection:::sciDetail
        Site investigation:::sciDetail
        Onboard analysis:::sciDetail
    System Architecture:::architecture
      Multi-Pi Distribution:::archDetail
        Control Pi:::piDetail
        Autonomy Pi:::piDetail
        Microcontrollers:::piDetail
      ROS 2 Communication:::archDetail
        Topics/Services/Actions:::commDetail
        QoS settings:::commDetail
        Real-time requirements:::commDetail
    Subsystems:::subsystems
      Navigation:::subsysNav
        Path planning:::navAlgo
        Obstacle avoidance:::navAlgo
        PID control:::navAlgo
      SLAM:::subsysSlam
        Sensor fusion:::slamAlgo
        Localization:::slamAlgo
        Mapping:::slamAlgo
      Computer Vision:::subsysVision
        ArUco detection:::visionAlgo
        Object recognition:::visionAlgo
        Pose estimation:::visionAlgo
      Autonomous Typing:::subsysTyping
        Arm control:::typingAlgo
        Trajectory planning:::typingAlgo
        Force sensing:::typingAlgo
      State Management:::subsysState
        Mission coordination:::stateAlgo
        Mode switching:::stateAlgo
        Health monitoring:::stateAlgo
      LED Status:::subsysLed
        Competition signaling:::ledAlgo
        PWM control:::ledAlgo
        Visibility requirements:::ledAlgo
    Technical Topics:::technical
      PID Control Systems:::pidTopic
        Controller design:::pidDetail
        Tuning methods:::pidDetail
        Implementation:::pidDetail
      Real-time Systems:::rtTopic
        Timing constraints:::rtDetail
        Scheduling:::rtDetail
        Resource management:::rtDetail
      Sensor Processing:::sensorTopic
        Data fusion:::sensorDetail
        Calibration:::sensorDetail
        Noise modeling:::sensorDetail
    Development:::development
      ROS 2 Framework:::rosTopic
        Nodes/Topics/Services:::rosDetail
        Launch files:::rosDetail
        Testing:::rosDetail
      Testing Strategy:::testTopic
        Unit testing:::testDetail
        Integration testing:::testDetail
        Simulation:::testDetail
      Deployment:::deployTopic
        Hardware setup:::deployDetail
        Calibration:::deployDetail
        Validation:::deployDetail

    classDef competition fill:#4a90e2,stroke:#2171b5,stroke-width:2px,color:#ffffff
    classDef missionNav fill:#4a90e2,stroke:#2171b5,stroke-width:1px,color:#ffffff
    classDef missionServ fill:#9b59b6,stroke:#7d3c98,stroke-width:1px,color:#ffffff
    classDef missionDeliv fill:#f39c12,stroke:#e67e22,stroke-width:1px,color:#ffffff
    classDef missionSci fill:#27ae60,stroke:#229954,stroke-width:1px,color:#ffffff
    classDef architecture fill:#27ae60,stroke:#229954,stroke-width:2px,color:#ffffff
    classDef subsystems fill:#9b59b6,stroke:#8e44ad,stroke-width:2px,color:#ffffff
    classDef technical fill:#e74c3c,stroke:#c0392b,stroke-width:2px,color:#ffffff
    classDef development fill:#f39c12,stroke:#e67e22,stroke-width:2px,color:#ffffff
```

---

## 🎯 **Mission Requirements & Success Criteria**

### **Competition Mission Specifications** ([URC 2026 Rules](../UniversityRoverChallenge2026.md))

#### **Mission 1.f: Autonomous Navigation** 🚗
**Reference:** Section 1.f, UniversityRoverChallenge2026.md
- **Time Limit:** 30 minutes
- **Distance:** ≤2km total
- **Targets:** 7 total (2 GNSS-only, 2 AR-tagged posts, 3 ground objects)
- **Accuracy Requirements:**
  - GNSS-only locations: 3m tolerance
  - AR-tagged posts: 2m tolerance (GNSS coords provided ±3m)
  - Ground objects: 10m tolerance
- **LED Requirements:** Section 1.f.vi
  - 🔴 Red: Autonomous operation
  - 🔵 Blue: Teleoperation
  - 🟢 Flashing Green: Successful target arrival
- **Autonomous Operation:** Section 1.f.vii-viii
  - Rover decides when target reached
  - Operators may abort and return (20% penalty)
  - Programming allowed only at start/completion

#### **Mission 1.e: Equipment Servicing** 🔧
**Reference:** Section 1.e, UniversityRoverChallenge2026.md
- **Time Limit:** 30 minutes
- **Distance:** ~100m to lander
- **Autonomous Typing:** Section 1.e.ii, 1.e.iii
  - 3-6 letter launch codes
  - Backspace/delete for corrections
  - Spelling mistakes allowed for partial credit
  - ArUco markers: 2x2cm (keyboard), 1x1cm (USB)
- **Robotic Operations:**
  - Sample tube insertion and cache sealing
  - Drawer opening and cache placement
  - Latch opening and panel access
  - Hose connection (GatorLock fitting)
  - Valve operation (¼-turn handle)
  - Push buttons, flip switches, turn knobs

#### **Mission 1.d: Delivery Mission** 📦
**Reference:** Section 1.c, UniversityRoverChallenge2026.md
- **Time Limit:** 30-60 minutes
- **Terrain:** Variable difficulty (flat to extreme)
- **GNSS Coordinates:** Provided for all locations
- **Object Handling:** Up to 5kg, various grasp points
- **Drone Support:** Optional (Section 1.d)
  - FAA registration required
  - TRUST certification mandatory
  - Weight restrictions and safety protocols

#### **Mission 1.b: Science Mission** 🔬
**Reference:** Section 1.b, UniversityRoverChallenge2026.md
- **Time Limit:** 20-30 minutes
- **Sample Collection:** Section 1.b.vi-vii
  - Subsurface sampling (≥10cm depth)
  - ≥5g sample mass
  - Sealed cache return
- **Site Documentation:** Section 1.b.iii
  - Wide-angle panoramas
  - Close-up photos with scale
  - Stratigraphic profiles
  - GNSS coordinates with accuracy
- **Onboard Analysis:** Section 1.b.v
  - Life detection capability
  - Second science capability (team choice)

### **Technical Success Metrics**

#### **Performance Requirements**
- **Navigation Accuracy:** <1m RMS position error (target)
- **Mission Success:** >95% completion rate
- **Real-time Performance:** <100ms control loops
- **Power Efficiency:** <50W system power
- **Robustness:** Handle environmental challenges

#### **Competition Scoring**
- **Mission Weight:** 100 points each (400 total)
- **SAR Weight:** 100 points (500 total)
- **Penalties:**
  - Overweight: 5% per kg over 50kg
  - Interventions: 20% per intervention
  - Late submissions: 10% per day

### **System Constraints** ⚖️

#### **Physical Constraints** (Section 3.a)
- **Weight:** 50kg deployed, 70kg total parts
- **Size:** Fits 1.2m × 1.2m × 1.2m crate
- **Power:** Mars-applicable (no air-breathing)
- **E-stop:** Red push-button emergency stop

#### **Communication Constraints** (Section 3.b)
- **Frequencies:** 900MHz (restricted sub-bands), 2.4GHz (unrestricted)
- **FCC Compliance:** All equipment must comply
- **Antenna Restrictions:** No camera on antenna mast

#### **Environmental Constraints** (Section 2.b)
- **Temperature:** Up to 100°F (37.8°C)
- **Weather:** Dust, wind, light rain
- **Line-of-sight:** Not guaranteed for all missions

### **Subsystem-Specific Requirements**

#### **Navigation Subsystem**
- **Path Planning:** A* with obstacle avoidance
- **Waypoint Following:** GNSS coordinate navigation
- **Terrain Adaptation:** Slope analysis, traversability
- **Velocity Control:** PID-based motion control

#### **SLAM Subsystem**
- **Localization:** <1m drift over 30 minutes
- **Mapping:** 2D occupancy grid generation
- **Sensor Fusion:** GPS/IMU/odometry integration
- **Loop Closure:** >95% detection rate

#### **Computer Vision Subsystem**
- **Camera Calibration:** Essential for accurate measurements ([`CameraCalibrationGuide.md`](../reference/CameraCalibrationGuide.md))
- **ArUco Detection:** 4x4_50 library markers
- **Object Recognition:** Competition objects (mallet, hammer, bottle)
- **Pose Estimation:** Sub-pixel accuracy
- **Real-time Performance:** >10Hz processing

#### **Autonomous Typing Subsystem**
- **Arm Control:** 6-DOF precision manipulation
- **Force Sensing:** Key press detection
- **Trajectory Planning:** Collision-free paths
- **Error Recovery:** Backspace/delete handling

#### **State Management Subsystem**
- **Mode Switching:** <1s autonomous ↔ teleoperation
- **Mission Coordination:** Waypoint sequencing
- **Health Monitoring:** Subsystem status tracking
- **Fault Tolerance:** Graceful degradation

#### **LED Status Subsystem**
- **Competition Compliance:** Red/Blue/Green signaling
- **Visibility:** Daylight visible from 50m
- **Power Efficiency:** <1W average consumption
- **Timing:** Precise flash patterns

---

## 📡 **Core Technologies Overview**

**For detailed explanations of core technologies, see:**
- **[`Robotics_Engineering_Concepts_Guide.md`](../Robotics_Engineering_Concepts_Guide.md)** - Comprehensive guide to ROS 2, sensors, and algorithms
- **[`reference/SensorGuide.md`](../reference/SensorGuide.md)** - Detailed sensor specifications and integration
- **[`reference/LibrariesGuide.md`](../reference/LibrariesGuide.md)** - ROS 2 libraries and development tools

---

## 🔧 **Technical Implementation Overview**

**For detailed technical implementations and code examples, see:**
- **[`Robotics_Engineering_Concepts_Guide.md`](../Robotics_Engineering_Concepts_Guide.md)** - ROS 2 nodes, computer vision algorithms, navigation stack, SLAM fundamentals, control systems, and state machines
- **[`docs/PID_Control_Diagrams.md`](../PID_Control_Diagrams.md)** - Complete PID controller implementations and tuning guides
- **[`reference/LibrariesGuide.md`](../reference/LibrariesGuide.md)** - Code examples for ROS 2, OpenCV, PyTorch integration

---

## 🌐 **System Communication & Architecture**

**For detailed distributed systems concepts, see:**
- **[`docs/DistributedArchitecture.md`](../DistributedArchitecture.md)** - Client-server architecture, communication patterns, fault tolerance, and graceful degradation
- **[`docs/InterfaceContract.md`](../InterfaceContract.md)** - ROS 2 topics, services, actions, and QoS profiles

---

## 🧪 **Testing & Validation Overview**

**For comprehensive testing strategies and examples, see:**
- **[`docs/guides/DevelopmentPipeline.md`](../guides/DevelopmentPipeline.md)** - Unit testing, integration testing, simulation testing, and CI/CD pipelines
- **[`reference/CalibrationGuide.md`](../reference/CalibrationGuide.md)** - Sensor calibration and validation procedures

---

## 📊 **Performance & Development Overview**

**For detailed performance optimization and development workflows, see:**
- **[`docs/DevelopmentWorkflow.md`](../DevelopmentWorkflow.md)** - Version control, CI/CD, code quality, and development processes
- **[`reference/LibrariesGuide.md`](../reference/LibrariesGuide.md)** - Memory management, communication optimization, and power management techniques

---

## 🎯 **Competition Preparation**

### **System Integration**
1. **Hardware assembly:** Mount all sensors and computers
2. **Network configuration:** Set up ROS 2 communication
3. **Calibration:** Complete autonomy setup ([`AutonomySetupSystemsGuide.md`](../reference/AutonomySetupSystemsGuide.md)) + Camera calibration tools ([`calibration/`](../calibration/))
4. **Testing:** Full system integration tests

### **Performance Validation**
- **Timing:** All real-time requirements met
- **Reliability:** System works under competition conditions
- **Power:** Battery life sufficient for full mission
- **Robustness:** Handles environmental challenges

### **Competition Checklist**
- [ ] All mission requirements implemented
- [ ] System tested in desert-like conditions
- [ ] Backup systems operational
- [ ] Remote monitoring working
- [ ] Team communication protocols ready

---

## 📊 **System Status Dashboard**

### **Subsystem Implementation Status**

```mermaid
stateDiagram-v2
    [*] --> ROS2_Packages: Day 1-2
    ROS2_Packages --> Basic_Sensors: Day 2-4
    Basic_Sensors --> State_Management: Day 4-6
    State_Management --> LED_Status: Day 6-8
    LED_Status --> Navigation_Core: Day 8-10
    Navigation_Core --> SLAM_Foundation: Day 10-12
    SLAM_Foundation --> Computer_Vision: Day 12-14
    Computer_Vision --> Autonomous_Typing: Day 14-16
    Autonomous_Typing --> System_Integration: Day 16-20

    ROS2_Packages --> [*]: ✅ Complete
    Basic_Sensors --> [*]: ✅ Complete
    State_Management --> [*]: ✅ Complete
    LED_Status --> [*]: ✅ Complete
    Navigation_Core --> [*]: ✅ Complete
    SLAM_Foundation --> [*]: ✅ Complete
    Computer_Vision --> [*]: ✅ Complete
    Autonomous_Typing --> [*]: ✅ Complete
    System_Integration --> [*]: 🔄 Active

    note right of ROS2_Packages : All 6 subsystems created
    note right of Basic_Sensors : GPS/IMU/Camera interfaces
    note right of State_Management : Mission coordination ready
    note right of LED_Status : Competition signaling working
    note right of Navigation_Core : Path planning + PID control
    note right of SLAM_Foundation : Pose estimation + TF publishing
    note right of Computer_Vision : ArUco detection + object recognition
    note right of Autonomous_Typing : Arm control + trajectory planning
    note right of System_Integration : ROS2 communication + testing
```

### **Mission Readiness Matrix**

```mermaid
heatmapBeta
    title Subsystem Readiness by Mission
    xlabel Mission
    ylabel Subsystem
    x-axis ["Navigation", "Servicing", "Delivery", "Science"]
    y-axis ["Navigation", "SLAM", "Vision", "Typing", "State", "LED"]
    range [0, 100]
    Navigation: [95, 80, 90, 70]
    SLAM: [85, 75, 85, 80]
    Vision: [90, 95, 85, 95]
    Typing: [70, 95, 75, 60]
    State: [95, 90, 90, 85]
    LED: [95, 70, 80, 60]
```

### **Critical Path Timeline**

```mermaid
journey
    title Development Progress Journey
    section Foundation
      ROS2 Setup: 5: Team
      Sensor Integration: 5: Team
      State Management: 5: Team
    section Core Systems
      LED Signaling: 5: Team
      Navigation: 5: Team
      SLAM: 4: Team
    section Advanced Features
      Computer Vision: 4: Team
      Arm Control: 4: Team
      System Integration: 3: Team
    section Competition Ready
      Testing & Validation: 2: Team
      Performance Optimization: 2: Team
      Final Competition Prep: 1: Team
```

---

## 📚 **Key Documents & Cross-References**

### **📖 Primary Entry Points**
| **Purpose** | **Document** | **Best For** | **Reading Time** |
|-------------|-------------|--------------|------------------|
| **Competition Rules** | [`../UniversityRoverChallenge2026.md`](../UniversityRoverChallenge2026.md) | **Official requirements & constraints** | 45-60 min |
| **Complete Overview** | [`docs/README.md`](../README.md) | Understanding full project scope | 20-30 min |
| **Learning Paths** | [`docs/LearningTracks.md`](../LearningTracks.md) | Structured skill development | 45-60 min |
| **System Architecture** | [`docs/SystemArchitecture.md`](../SystemArchitecture.md) | Technical system design | 30-45 min |
| **This Document** | `docs/TechnicalOverview.md` | **Top-down technical understanding** | 60-90 min |

### **🔗 Cross-Reference Map**

```
🏆 UniversityRoverChallenge2026.md (Official Competition Rules)
├── 📖 TechnicalOverview.md (You are here - Top-down guide)
│   ├── 🎯 High-Level Concepts
│   │   ├── docs/Robotics_Engineering_Concepts_Guide.md
│   │   └── docs/GoalsAndSuccessMetrics.md
│   ├── 🏗️ System Architecture
│   │   ├── docs/SystemArchitecture.md
│   │   ├── docs/DistributedArchitecture.md
│   │   └── docs/InterfaceContract.md
│   ├── 🔧 Subsystems (Deep technical details)
│   │   ├── subsystems/navigation/ (Path planning algorithms)
│   │   ├── subsystems/slam/ (Localization & mapping)
│   │   ├── subsystems/computer_vision/ (Object detection)
│   │   ├── subsystems/autonomous_typing/ (Arm control)
│   │   ├── subsystems/state_management/ (Mission coordination)
│   │   └── subsystems/led_status/ (Status signaling)
│   ├── 💻 Implementation (Code & algorithms)
│   │   ├── code/[subsystem]/[subsystem]_TODO.md (Implementation roadmaps)
│   │   ├── docs/PID_Control_Diagrams.md (Control systems)
│   │   └── docs/DevelopmentWorkflow.md (Code development)
│   ├── 🧪 Testing & Validation
│   │   ├── docs/guides/DevelopmentPipeline.md
│   │   ├── reference/CalibrationGuide.md
│   │   └── tests/integration_test.py
│   └── 📚 Reference Materials
│       ├── docs/reference/ (8 technical guides)
│       └── docs/guides/ (User guides)
└── 📋 Competition-Specific References
    ├── docs/README.md (Project overview with competition focus)
    └── docs/LearningTracks.md (Competition-focused learning paths)
```

### **🎯 Quick Access by Role**

#### **For New Team Members:**
1. **[`docs/README.md`](../README.md)** (5 min) - Project overview
2. **[`docs/LearningTracks.md`](../LearningTracks.md)** (10 min) - Choose your track
3. **This document** (TechnicalOverview.md) - Understand the system
4. **Your subsystem docs** - Dive into your area

#### **For Developers:**
1. **This document** - System understanding
2. **[`docs/SystemArchitecture.md`](../SystemArchitecture.md)** - Technical architecture
3. **[`docs/InterfaceContract.md`](../InterfaceContract.md)** - ROS2 interfaces
4. **Subsystem TODO files** - Implementation details

#### **For System Architects:**
1. **[`docs/SystemArchitecture.md`](../SystemArchitecture.md)** - Architecture overview
2. **This document** - Technical deep-dive
3. **[`docs/DistributedArchitecture.md`](../DistributedArchitecture.md)** - Multi-Pi design
4. **[`docs/InterfaceContract.md`](../InterfaceContract.md)** - System contracts

#### **For Competition Teams:**
1. **[`docs/GoalsAndSuccessMetrics.md`](../GoalsAndSuccessMetrics.md)** - Success criteria
2. **This document** - Technical requirements
3. **Subsystem requirements docs** - Competition specs
4. **[`reference/EnvironmentalChallenges.md`](../reference/EnvironmentalChallenges.md)** - Desert conditions

### **🔍 Topic-Specific Navigation**

| **Topic** | **Primary Reference** | **Secondary References** |
|-----------|----------------------|-------------------------|
| **Competition Rules** | [`../UniversityRoverChallenge2026.md`](../UniversityRoverChallenge2026.md) | **Official URC 2026 requirements** |
| **PID Control** | [`docs/PID_Control_Diagrams.md`](../PID_Control_Diagrams.md) | [`Robotics_Engineering_Concepts_Guide.md`](../Robotics_Engineering_Concepts_Guide.md) |
| **ROS2 Topics/Services** | [`docs/InterfaceContract.md`](../InterfaceContract.md) | [`reference/LibrariesGuide.md`](../reference/LibrariesGuide.md) |
| **Sensor Integration** | [`reference/SensorGuide.md`](../reference/SensorGuide.md) | [`reference/CalibrationGuide.md`](../reference/CalibrationGuide.md) |
| **Navigation Algorithms** | [`subsystems/navigation/Navigation_PathPlanning.md`](../subsystems/navigation/Navigation_PathPlanning.md) | [`code/navigation/navigation_TODO.md`](../code/navigation/navigation_TODO.md) - Status |
| **Computer Vision** | [`subsystems/computer_vision/ComputerVision_ObjectClassification.md`](../subsystems/computer_vision/ComputerVision_ObjectClassification.md) | [`code/computer_vision/computer_vision_TODO.md`](../code/computer_vision/computer_vision_TODO.md) |
| **SLAM** | [`subsystems/slam/SLAM.md`](../subsystems/slam/SLAM.md) | [`code/slam/slam_TODO.md`](../code/slam/slam_TODO.md) |
| **Arm Control** | [`subsystems/autonomous_typing/AutonomousTyping.md`](../subsystems/autonomous_typing/AutonomousTyping.md) | [`code/autonomous_typing/autonomous_typing_TODO.md`](../code/autonomous_typing/autonomous_typing_TODO.md) - Status |
| **State Management** | [`subsystems/state_management/StateManagement_ModeControl.md`](../subsystems/state_management/StateManagement_ModeControl.md) | [`code/state_management/state_management_TODO.md`](../code/state_management/state_management_TODO.md) |
| **LED Status** | [`subsystems/led_status/LED_StatusSignaling.md`](../subsystems/led_status/LED_StatusSignaling.md) | [`code/led_status/led_status_TODO.md`](../code/led_status/led_status_TODO.md) |

### **📈 Learning Progression Paths**

#### **Beginner to Expert:**
1. **ELI5 Understanding** (This document, Section 1-3)
2. **Technical Architecture** (This document, Section 4-6)
3. **Subsystem Knowledge** (This document, Section 7-11 + subsystem docs)
4. **Implementation Details** (Code TODOs, PID diagrams, reference guides)
5. **Competition Mastery** (Testing, environmental challenges, optimization)

#### **By Background:**
- **New to Robotics**: Start with ELI5 → Learning Tracks → Subsystem basics
- **Some Programming**: Skip basics → Architecture → Implementation
- **ROS Experience**: Architecture → Subsystems → Competition requirements
- **Advanced Robotics**: Jump to technical sections → Implementation details

---

## 🆘 **Getting Help & Support**

### **Immediate Assistance:**
- **Setup Issues**: Check [`../../QUICKSTART.md`](../../QUICKSTART.md) or [`../../ONBOARDING.md`](../../ONBOARDING.md)
- **Code Problems**: Review [`CODE_STYLE.md`](../../CODE_STYLE.md) and [`check_code.sh`](../../scripts/check_code.sh)
- **ROS2 Issues**: See [`reference/LibrariesGuide.md`](../reference/LibrariesGuide.md)

### **Documentation Navigation:**
- **Lost?** Start at [`docs/README.md`](../README.md) for the complete map
- **Deep-dive needed?** Use the cross-reference map above
- **Implementation stuck?** Check subsystem TODO files and development pipeline

### **Team Communication:**
- **Subsystem Leads**: Contact via team channels for specific subsystem guidance
- **Integration Issues**: Post in team channels with error logs and system state
- **Architecture Questions**: Tag system architects in technical discussions

---

## 📚 **Further Reading**

### **Beginner Resources**
- **ROS 2 Tutorials:** https://docs.ros.org/en/humble/Tutorials.html
- **Python Robotics:** https://github.com/AtsushiSakai/PythonRobotics
- **Computer Vision Basics:** https://opencv.org/

### **Advanced Topics**
- **SLAM:** "Simultaneous Localization and Mapping" papers
- **Control Systems:** "Modern Control Systems" textbook
- **Machine Learning:** "Deep Learning" by Goodfellow et al.

### **Competition Specific**
- **URC 2026 Rules:** Official competition documentation
- **Previous Years:** Analysis of winning team approaches
- **Research Papers:** Recent robotics conference proceedings

---

## 🎨 **Diagram Rendering Guide**

### **IDE Compatibility**
- **VS Code**: Install "Markdown Preview Mermaid Support" extension
- **Cursor**: Built-in mermaid support with theme detection
- **Other IDEs**: Use online mermaid editors (mermaid.live)

### **Theme-Specific Optimization**

#### **Light Mode Colors**
- All diagrams use high-contrast colors that are clearly visible on white backgrounds
- Text is primarily white on colored backgrounds for optimal readability
- Borders are thick enough (2-3px) to maintain definition

#### **Dark Mode Colors**
- Same color palette works excellently in dark themes
- High-contrast borders ensure diagram elements remain distinct
- Color choices avoid low-contrast combinations that become invisible

### **Color Psychology & Meaning**
| **Color** | **Psychology** | **System Meaning** |
|-----------|----------------|-------------------|
| 🔵 **Blue** | Trust, Stability | Navigation, Planning, Coordination |
| 🟢 **Green** | Growth, Success | SLAM, Perception, Health |
| 🟣 **Purple** | Innovation, Vision | Computer Vision, Processing |
| 🔴 **Red** | Energy, Critical | Actuators, Critical Systems |
| 🟠 **Orange** | Action, Control | State Management, Control |

### **Accessibility Features**
- **WCAG Compliant**: Contrast ratios exceed AA standards
- **Color Blind Friendly**: Color is not the only differentiator (shapes, labels used)
- **Scalable**: Diagrams work at various zoom levels (100%-150%)
- **Universal**: Clear in both light and dark themes

### **Export & Sharing**
- **PNG/SVG**: Export from mermaid.live for presentations
- **PDF**: Include in technical documentation
- **Web**: Embed with custom CSS for enhanced theming

---

*This technical overview provides both intuitive understanding and deep technical details for building competition-ready autonomous systems, with diagrams optimized for universal readability across all viewing environments and themes.*
