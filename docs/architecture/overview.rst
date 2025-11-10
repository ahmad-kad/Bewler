========
Overview
========

High-level architecture of the URC 2026 Mars Rover Autonomy System.

System Architecture
===================

.. graphviz::

   digraph system_architecture {
      rankdir=TB;
      node [shape=box, style=rounded, fontsize=10];

      // User Interface Layer
      subgraph cluster_ui {
         label="User Interface Layer";
         color=lightblue;
         WebDashboard [label="Web Dashboard"];
         ThreeDVisualization [label="3D Visualization"];
         ConfigTools [label="Configuration Tools"];
      }

      // ROS2 Middleware
      ROS2 [label="ROS2 Middleware", shape=ellipse, color=blue];

      // Autonomy Stack
      subgraph cluster_autonomy {
         label="Autonomy Stack";
         color=lightgreen;
         StateMachine [label="State Machine"];
         Navigation [label="Navigation"];
         ComputerVision [label="Computer Vision"];
         SLAM [label="SLAM"];
         Calibration [label="Calibration"];
      }

      // Hardware Interfaces
      subgraph cluster_hardware {
         label="Hardware Interfaces";
         color=orange;
         MotorControllers [label="Motor Controllers"];
         Sensors [label="Sensors"];
         Communication [label="Communication"];
      }

      // External Systems
      Simulation [label="Gazebo Simulation", shape=hexagon];
      ExternalSystems [label="External Systems", shape=hexagon];

      // Connections
      WebDashboard -> ROS2;
      ThreeDVisualization -> ROS2;
      ConfigTools -> ROS2;

      ROS2 -> StateMachine;
      ROS2 -> Navigation;
      ROS2 -> ComputerVision;
      ROS2 -> SLAM;
      ROS2 -> Calibration;

      StateMachine -> MotorControllers;
      Navigation -> MotorControllers;
      ComputerVision -> Sensors;
      SLAM -> Sensors;
      Calibration -> Sensors;

      MotorControllers -> Communication;
      Sensors -> Communication;

      // Bidirectional connections
      ROS2 -> Simulation [dir=both];
      ROS2 -> ExternalSystems [dir=both];
   }

System Components
=================

The system is organized into several key components:

Core Autonomy Stack
-------------------

- **State Machine**: Hierarchical state management
- **Navigation**: Path planning and execution
- **Computer Vision**: Perception and object detection
- **SLAM**: Simultaneous Localization and Mapping

Hardware Interfaces
-------------------

- **Motor Controllers**: Wheel and arm actuation
- **Sensors**: Cameras, IMU, GPS, encoders
- **Communication**: ROS2 middleware and WebSocket bridge

User Interface
--------------

- **Web Dashboard**: Real-time monitoring and control
- **3D Visualization**: Interactive rover model
- **Configuration Tools**: Calibration and setup wizards

Data Flow Architecture
=====================

.. graphviz::

   digraph dataflow {
      rankdir=LR;
      node [shape=box, style=rounded, fontsize=10];
      edge [fontsize=9];

      // Data sources
      Sensors [label="Sensors\n(Camera, IMU, GPS)"];
      Commands [label="User Commands"];

      // Processing layers
      Perception [label="Perception Layer\n(OpenCV, ArUco)"];
      Planning [label="Planning Layer\n(Path Planning,\nState Machine)"];
      Control [label="Control Layer\n(Motion Control,\nArm Control)"];

      // Data sinks
      Actuators [label="Actuators\n(Motors, Servos)"];
      Feedback [label="User Feedback\n(Web UI, Logs)"];

      // Data flow
      Sensors -> Perception [label="Raw Data"];
      Commands -> Planning [label="Goals"];

      Perception -> Planning [label="Processed\nFeatures"];
      Planning -> Control [label="Commands"];

      Control -> Actuators [label="Control\nSignals"];
      Control -> Feedback [label="Status"];

      Planning -> Feedback [label="Progress"];
      Perception -> Feedback [label="Diagnostics"];

      // Feedback loops
      Feedback -> Planning [label="Adjustments", style=dashed];
   }

The system follows a layered architecture:

1. **Sensor Layer**: Raw sensor data acquisition (cameras, IMU, GPS, encoders)
2. **Perception Layer**: Computer vision processing and feature extraction
3. **Planning Layer**: State machine logic and mission planning algorithms
4. **Control Layer**: Motion control and actuator command execution
5. **Feedback Layer**: User interface updates and system monitoring

Communication Architecture
==========================

All components communicate through ROS2 topics and services:

- **Topics**: Asynchronous data streams (sensor data, status updates)
- **Services**: Synchronous request-response (calibration, configuration)
- **Actions**: Long-running tasks with feedback (navigation, typing)

Safety and Reliability
======================

The system includes multiple safety mechanisms:

- **Watchdog timers** for critical processes
- **Graceful degradation** on subsystem failures
- **Emergency stop** capabilities
- **State validation** before transitions
