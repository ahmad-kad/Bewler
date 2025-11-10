.. _getting_started_overview:

============
Project Overview
============

The URC Machiato 2026 project is an autonomous rover system designed to compete in the University Rover Challenge (URC).
This comprehensive robotics platform integrates multiple subsystems to create a fully autonomous exploration vehicle.

System Architecture
===================

The system follows a modular architecture with clear separation of concerns:

.. figure:: ../_static/system_architecture.png
   :alt: System Architecture Diagram
   :align: center
   :scale: 75%

   High-level system architecture showing major components and data flow.

Core Components
---------------

Autonomy Stack
^^^^^^^^^^^^^^

The autonomy stack is built on ROS2 and provides the core intelligence for the rover:

- **State Management**: Centralized coordination of all subsystems
- **Navigation**: Path planning and motion control
- **Computer Vision**: Real-time perception and object detection
- **SLAM**: Simultaneous Localization and Mapping
- **LED Status**: Visual feedback and system health indicators

Simulation Environment
^^^^^^^^^^^^^^^^^^^^^

A complete Gazebo-based simulation environment for:

- Algorithm development and testing
- System integration verification
- Mission scenario rehearsal
- Performance benchmarking

Web Interface
^^^^^^^^^^^^^

A React-based monitoring and control interface providing:

- Real-time system status visualization
- Mission planning and execution
- Telemetry data display
- System configuration management

Key Technologies
================

+----------------+-----------------------------+------------------+
| Component      | Technology                  | Purpose          |
+================+=============================+==================+
| Framework      | ROS2 (Robot Operating System)| Robotics middleware|
+----------------+-----------------------------+------------------+
| Simulation     | Gazebo                      | Physics simulation|
+----------------+-----------------------------+------------------+
| Computer Vision| OpenCV + Custom Algorithms  | Perception       |
+----------------+-----------------------------+------------------+
| Frontend       | React + Three.js            | User interface   |
+----------------+-----------------------------+------------------+
| Navigation     | Custom path planning        | Motion control   |
+----------------+-----------------------------+------------------+
| Communication  | ROS2 Topics/Services        | Inter-process comm|
+----------------+-----------------------------+------------------+

Project Goals
=============

The primary objectives of URC Machiato 2026 are:

1. **Autonomous Navigation**: Develop robust autonomous navigation capabilities
2. **Multi-modal Perception**: Integrate multiple sensor modalities for reliable perception
3. **Mission Autonomy**: Execute complex mission scenarios with minimal human intervention
4. **System Reliability**: Ensure robust operation in harsh environmental conditions
5. **Extensibility**: Create a modular platform for future enhancements

Competition Requirements
========================

The system is designed to meet URC 2026 competition requirements including:

- Autonomous traverse of varied terrain
- Equipment servicing tasks
- Science cache operations
- Extreme delivery challenges
- Virtual competition elements

Development Philosophy
======================

The project follows modern software engineering practices:

- **Modular Design**: Clean separation between subsystems
- **Test-Driven Development**: Comprehensive testing at all levels
- **Continuous Integration**: Automated build and test pipelines
- **Documentation-Driven**: Extensive documentation for maintainability
- **Open Source**: Community collaboration and knowledge sharing

Next Steps
==========

Ready to get started? Continue to:

- :doc:`installation` - Set up your development environment
- :doc:`quick_start` - Run your first autonomous mission
- :doc:`development_setup` - Configure for development
