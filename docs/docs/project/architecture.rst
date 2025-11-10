.. _project_architecture:

================
System Architecture
================

The URC Machiato 2026 system follows a modular, distributed architecture designed
for autonomous rover operations in challenging environments. This document outlines
the high-level system design, component interactions, and architectural principles.

.. toctree::
   :maxdepth: 2

   integrated/overview/SystemArchitecture
   integrated/overview/DistributedArchitecture
   integrated/overview/InterfaceContract
   integrated/overview/ExternalSystemsIntegration

Architectural Overview
======================

The system is built on a layered architecture with clear separation of concerns:

.. figure:: ../_static/architecture_diagram.png
   :alt: System Architecture Diagram
   :align: center
   :scale: 75%

Core Principles
===============

**Modularity**
   Each subsystem operates independently with well-defined interfaces,
   enabling parallel development and testing.

**Fault Tolerance**
   The system is designed to gracefully handle component failures
   and maintain operational capability.

**Scalability**
   Architecture supports addition of new sensors, actuators, and
   computational nodes without major redesign.

**Real-time Performance**
   Critical components operate with deterministic timing constraints
   suitable for autonomous navigation.

Component Architecture
=======================

Perception Layer
----------------

**Computer Vision Subsystem**

- Multi-camera processing with stereo vision
- Real-time object detection and tracking
- Terrain classification and hazard detection
- AR tag recognition for precision navigation

**Sensor Integration**

- GNSS/IMU fusion for localization
- LiDAR for obstacle detection
- Environmental sensors (temperature, radiation, etc.)

Planning Layer
--------------

**Path Planning**

- Global route optimization
- Local obstacle avoidance
- Terrain-adaptive trajectory generation
- Mission-level task sequencing

**State Management**

- Hierarchical state machine
- Mode transitions (manual/autonomous)
- Safety state monitoring
- System health assessment

Control Layer
-------------

**Motion Control**

- Differential drive kinematics
- Velocity control with feedback
- Precision positioning for tasks
- Emergency stop handling

**Actuator Control**

- Servo motor control for manipulation
- LED status indication
- Power management

Communication Architecture
===========================

ROS2 Middleware
---------------

The system uses ROS2 as the primary communication middleware:

- **Topics**: Asynchronous data streaming (sensor data, status updates)
- **Services**: Synchronous request-response (configuration, commands)
- **Actions**: Long-running tasks with feedback (navigation, manipulation)
- **Parameters**: Dynamic configuration management

Interface Definitions
---------------------

All inter-component communication is defined through ROS2 interfaces:

.. code-block:: yaml

   # Example: Navigation action interface
   action: NavigateToPose
   request:
     pose:
       position: [x, y, z]
       orientation: [x, y, z, w]
     tolerance: float
   feedback:
     distance_remaining: float
     eta: duration
   result:
     success: bool
     final_pose: pose

Web Interface Integration
=========================

The system includes a web-based monitoring and control interface:

**Real-time Visualization**
   - 3D environment rendering
   - Rover position and orientation
   - Sensor data visualization
   - Mission progress tracking

**Remote Control**
   - Manual driving interface
   - Mission planning tools
   - System configuration
   - Diagnostic monitoring

Deployment Architecture
========================

The system supports multiple deployment configurations:

**Development Environment**
   - Local simulation with Gazebo
   - Mock hardware interfaces
   - Full debugging capabilities

**Field Deployment**
   - Embedded systems on rover
   - Real hardware interfaces
   - Minimal debugging overhead

**Testing Environment**
   - Hardware-in-the-loop simulation
   - Automated test suites
   - Performance benchmarking

Data Flow Architecture
======================

.. graphviz::

   digraph dataflow {
      rankdir=LR;
      node [shape=box, style=rounded];

      Sensors -> Perception [label="raw data"];
      Perception -> Planning [label="processed data"];
      Planning -> Control [label="commands"];
      Control -> Actuators [label="control signals"];

      Planning -> WebInterface [label="status"];
      Perception -> WebInterface [label="visualization"];
      Control -> WebInterface [label="telemetry"];

      WebInterface -> Planning [label="commands"];
   }

Quality Assurance
=================

**Testing Strategy**
   - Unit tests for individual components
   - Integration tests for subsystem interaction
   - System-level validation in simulation
   - Hardware-in-the-loop testing

**Code Quality**
   - Type hints and documentation
   - Static analysis tools
   - Code review requirements
   - Continuous integration

**Performance Monitoring**
   - Real-time performance metrics
   - Resource usage tracking
   - Error rate monitoring
   - System health indicators

Future Extensions
=================

The architecture is designed to accommodate future enhancements:

- **Multi-rover coordination**
- **Advanced AI planning**
- **Cloud connectivity**
- **Extended sensor suites**
- **Manipulator arm integration**

This modular design ensures that new capabilities can be added
without disrupting existing functionality.
