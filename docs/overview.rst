========
Overview
========

The URC 2026 Mars Rover Autonomy System is a comprehensive autonomous robotics platform designed for the University Rover Challenge competition. This system integrates multiple subsystems to create a fully autonomous rover capable of navigation, object detection, and task execution in challenging environments.

System Architecture
===================

The rover system follows a modular, layered architecture:

.. figure:: _static/architecture_diagram.png
   :alt: System Architecture Diagram
   :align: center
   :width: 100%

   High-level system architecture showing major components and data flow.

Core Components
===============

🤖 **Autonomy Stack**
   - **State Machine**: Hierarchical state management for mission execution
   - **Navigation**: Path planning and obstacle avoidance using SLAM
   - **Computer Vision**: Object detection, terrain analysis, and target identification
   - **Sensor Fusion**: Integration of IMU, GPS, and camera data

📷 **Perception System**
   - **Camera Calibration**: Multi-camera intrinsic and extrinsic calibration
   - **Hand-Eye Calibration**: Coordinate frame alignment between sensors and actuators
   - **ArUco Marker Detection**: Fiducial markers for pose estimation and calibration

🌐 **User Interface**
   - **Web Dashboard**: Real-time monitoring and control interface
   - **3D Visualization**: Interactive rover and environment visualization
   - **ROS2 Integration**: WebSocket-based communication with ROS2 backend

🔧 **Hardware Integration**
   - **ROS2 Middleware**: Message passing and service architecture
   - **Motor Controllers**: Wheel and arm actuation systems
   - **Sensor Interfaces**: Camera, IMU, GPS, and encoder integration

Key Features
============

🔄 **Modular Design**
   Each subsystem operates independently with well-defined interfaces, allowing for easy testing, development, and replacement.

🛡️ **Robust Error Handling**
   Comprehensive error detection and recovery mechanisms ensure system reliability in harsh environments.

📊 **Real-time Performance**
   Optimized algorithms and efficient data structures enable real-time operation on embedded hardware.

🔍 **Extensive Testing**
   Automated testing framework with unit tests, integration tests, and simulation validation.

Technology Stack
================

+----------------+---------------------+---------------------+
| Category       | Technology          | Purpose             |
+================+=====================+=====================+
| **Backend**    | Python 3.10+        | Core autonomy logic |
|                +---------------------+---------------------+
|                | ROS2 Humble         | Middleware & comms  |
|                +---------------------+---------------------+
|                | OpenCV 4.8+         | Computer vision     |
+----------------+---------------------+---------------------+
| **Frontend**   | React 18+           | Web interface       |
|                +---------------------+---------------------+
|                | Three.js            | 3D visualization    |
|                +---------------------+---------------------+
|                | ROSLIB.js           | ROS2 web bridge     |
+----------------+---------------------+---------------------+
| **Hardware**   | Raspberry Pi 4B+    | Main computer       |
|                +---------------------+---------------------+
|                | Arduino Mega        | Motor control       |
|                +---------------------+---------------------+
|                | Intel RealSense     | Depth sensing       |
+----------------+---------------------+---------------------+
| **Development**| Docker              | Containerization    |
|                +---------------------+---------------------+
|                | Sphinx              | Documentation       |
+----------------+---------------------+---------------------+

Mission Scenarios
=================

The system is designed to handle various URC mission scenarios:

**Equipment Servicing**
   - Autonomous navigation to equipment locations
   - Precise manipulation using computer vision guidance
   - Tool deployment and operation

**Terrain Traversability**
   - Real-time terrain analysis and path planning
   - Obstacle detection and avoidance
   - Slope and roughness assessment

**Science Operations**
   - Sample collection and analysis
   - Autonomous science target identification
   - Data collection and transmission

**Autonomous Navigation**
   - GPS-denied navigation using SLAM
   - Multi-modal sensor fusion
   - Long-range path planning and execution

Performance Metrics
===================

The system is benchmarked against key performance indicators:

- **Navigation Accuracy**: ±5cm position accuracy in known environments
- **Detection Range**: Object detection up to 10m with 95% accuracy
- **Response Time**: <100ms control loop execution
- **Uptime**: >99.5% operational availability
- **Power Efficiency**: <50W average consumption during autonomous operation

Development Philosophy
======================

**Quality First**
   Rigorous testing, code review, and documentation standards ensure maintainable, reliable code.

**Modular Architecture**
   Clean interfaces and separation of concerns enable parallel development and easy maintenance.

**Performance Optimization**
   Efficient algorithms and data structures optimized for embedded deployment.

**Open Standards**
   Use of ROS2, OpenCV, and web standards ensures compatibility and community support.
