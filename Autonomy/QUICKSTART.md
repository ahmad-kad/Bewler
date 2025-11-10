# 🚀 URC 2026 Autonomy - Quick Start Guide

Welcome! This guide gets you coding in **5 minutes**. No experience needed.

## ⚡ 3-Step Setup

### 1. Start Environment (2 min)
```bash
# Open terminal in Autonomy folder
cd Autonomy

# Start Docker (downloads everything automatically)
./development/docker/scripts/docker-dev.sh build
./development/docker/scripts/docker-dev.sh start
./development/docker/scripts/docker-dev.sh enter
```

### 2. Pick Your Task (1 min)
Choose what interests you:

| **Navigation** | **SLAM** | **Computer Vision** | **Autonomous Typing** |
|---|---|---|---|---|
| Drive the robot | Map the world | See and recognize | Type on keyboard |
| `cd code/navigation` | `cd code/slam` | `cd code/computer_vision` | `cd code/autonomous_typing` |
| ⭐ **Beginner Friendly** | 🔧 **Tech Heavy** | 🤖 **AI Focus** | ⚙️ **Hardware Focus** |

### 3. Start Coding (2 min)
```bash
# 📁 DEVELOPMENT: Edit in code/ folder (your workspace)
cd code/navigation

# Open TODO list to see what to implement
cat code/navigation/navigation_TODO.md  # Navigation system status

# Edit your code (this is where you write and modify code)
code autonomy_navigation/navigation_node.py

# 🔗 BUILDING: Switch to ROS2 workspace for building/running
cd ../../ros2_ws

# Build your changes
colcon build --packages-select autonomy_navigation

# Run your code
source install/setup.bash
ros2 run autonomy_navigation navigation_node
```

## 🎯 What You'll Build

**Week 1 Goal**: Working ROS 2 node that publishes/subscribes to messages.

**Week 2 Goal**: Integration with other subsystems.

**Week 3 Goal**: Real hardware testing.

**Competition Goal**: Complete autonomous rover.

## 👥 **Team Structure & Your Role**

Your **Autonomy Team** is one piece of a larger robot system:

### **🤖 Your Autonomy Pi (SLAM, Navigation, Computer Vision)**
- **SLAM**: Maps the environment and tracks robot position
- **Path Planning**: Finds optimal routes to goals
- **Computer Vision**: Detects objects and reads markers
- **GPS Processing**: Provides accurate positioning
- **Mission Execution**: Runs autonomous tasks (navigation, typing)

### **🎮 Control Team (User Interface & System Coordination)**
- **User Controls**: Joystick, keyboard, touchscreen interfaces
- **System Status**: Displays robot status and telemetry
- **Mission Planning**: Sets waypoints and objectives
- **State Management**: Coordinates overall system behavior

### **⚙️ Hardware Team (Microcontrollers & Actuators)**
- **Wheel Control**: Motor drivers and odometry sensors
- **Robotic Arm**: Joint control and force sensing
- **Mast Camera**: Pan-tilt mechanisms and stabilization
- **Sensor Hub**: Data collection from various sensors

### **🔗 Integration Points**
- **ROS2 Topics**: Standardized message passing between all components
- **Service Calls**: Configuration and status queries
- **Action Interfaces**: Complex operations with progress feedback
- **Emergency Systems**: Coordinated safety responses

**You focus on autonomy algorithms** - the other teams handle user interaction and hardware control!

## 📁 **Development Workflow: Code vs ROS2 Workspace**

### **The Two-Workspace System**

This project uses a **two-workspace system** for clean development:

```
📁 Autonomy/code/           ← Your development workspace (edit here!)
├── navigation/
├── slam/
├── computer_vision/
└── state_management/

🔗 Autonomy/ros2_ws/        ← ROS2 build workspace (build/run here)
├── src/ (symlinks to code/)
├── build/ (generated)
├── install/ (generated)
└── log/ (generated)
```

### **Daily Development Workflow**

```bash
# 🛠️ DEVELOPMENT: Edit code in Autonomy/code/
cd Autonomy/code/navigation
code autonomy_navigation/navigation_node.py  # Edit in your IDE

# 🔨 BUILDING: Build in ROS2 workspace
cd Autonomy/ros2_ws
colcon build --packages-select autonomy_navigation

# ▶️ RUNNING: Run from ROS2 workspace
source install/setup.bash
ros2 run autonomy_navigation navigation_node

# 🔄 ITERATION: Edit → Build → Run → Repeat
```

### **Why Two Workspaces?**

- **`code/`**: Clean development, familiar file structure, version control
- **`ros2_ws/`**: ROS2 build system, generated files, runtime environment
- **Symlinks**: Connect the two seamlessly

**Edit in `code/`, build/run in `ros2_ws/`!** 🎯

## 🆘 Need Help?

### **Stuck on Setup?**
```bash
# Check Docker status
docker ps

# Restart environment
./development/docker/scripts/docker-dev.sh restart
./development/docker/scripts/docker-dev.sh enter
```

### **Code Not Working?**
```bash
# Check for errors
python3 -m py_compile src/your_file.py

# Run tests
python3 -m pytest test/ -v
```

### **Lost? Get Help**
```bash
# Show all options
./scripts/navigate.sh help

# Find documentation
./scripts/navigate.sh docs

# See project status
./scripts/navigate.sh status
```

## 📚 Learn More

- **📖 Full Docs**: `docs/README.md`
- **📊 Charts Overview**: `docs/ChartsOverview.md` (VISUAL SYSTEM DIAGRAMS!)
- **📖 Team Handbook**: `docs/TeamHandbook.md` (YOUR COMPLETE GUIDE!)
- **📋 Task-Based Guide**: `docs/TaskBasedHandbook.md` (STEP-BY-STEP TASKS!)
- **🤝 Team Integration**: `docs/TeamIntegration.md` (HOW TO WORK WITH OTHER TEAMS!)
- **📋 Interface Contract**: `docs/InterfaceContract.md` (ROS2 Interface Specifications!)
- **🏗️ System Architecture**: `docs/SystemArchitecture.md` (Complete Robot Overview)
- **🔄 Development Workflow**: `docs/DevelopmentWorkflow.md` (IMPORTANT!)
- **🌐 Distributed Architecture**: `docs/DistributedArchitecture.md` (Multi-Pi System)
- **🔌 External Integration**: `docs/ExternalSystemsIntegration.md`
- **🔧 Code Standards**: `code/templates/README.md`
- **🧪 Testing**: Follow `TODO.md` in your subsystem
- **🎮 Simulation**: `development/SimulationSetup.md`

## 🎉 You're Done!

**You've successfully set up the development environment!**

Next: Pick a subsystem, read its TODO.md, and start building. The templates and documentation will guide you every step.

**Questions?** Ask in Slack or check the docs. Happy coding! 🤖
