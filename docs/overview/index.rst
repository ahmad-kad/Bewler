========
Overview
========

Welcome to the URC Machiato 2026 project documentation. This section provides a comprehensive overview of the autonomous rover system designed for the University Rover Challenge 2026.

Project Vision
==============

URC Machiato 2026 aims to create a robust, autonomous rover capable of navigating complex terrain, performing scientific tasks, and operating with minimal human intervention. The system integrates cutting-edge robotics technologies with practical engineering solutions.

System Architecture
===================

The rover system is built around a modular architecture that separates concerns while maintaining tight integration between components.

.. figure:: ../_static/architecture_diagram.png
   :alt: System Architecture Diagram
   :align: center
   :width: 100%

   High-level system architecture showing major components and data flow.

Core Components
===============

Autonomy Stack
--------------

The autonomy stack provides the brain of the rover, handling:

* **State Management**: Hierarchical state machine controlling mission phases
* **Navigation**: Path planning, obstacle avoidance, and localization
* **Computer Vision**: Object detection, marker tracking, and scene understanding
* **SLAM**: Simultaneous Localization and Mapping for environment mapping

.. seealso::
   :doc:`../autonomy/index`
      Detailed documentation of the autonomy systems.

Frontend Interface
------------------

A modern web-based interface provides:

* **Real-time Monitoring**: Live telemetry and system status
* **3D Visualization**: Interactive rover and environment visualization
* **Mission Control**: Remote operation and mission planning
* **System Diagnostics**: Health monitoring and troubleshooting tools

.. seealso::
   :doc:`../frontend/index`
      Frontend documentation and component reference.

Calibration Systems
-------------------

Automated calibration ensures accurate sensor operation:

* **Camera Calibration**: Intrinsic and extrinsic camera parameter estimation
* **Hand-Eye Calibration**: Coordinate frame alignment between sensors
* **Multi-Camera Systems**: Synchronization and calibration of multiple cameras

.. seealso::
   :doc:`../calibration/index`
      Calibration procedures and algorithms.

Key Technologies
================

ROS2 (Robot Operating System 2)
--------------------------------

The entire autonomy stack is built on ROS2, providing:

* **Distributed Computing**: Node-based architecture for modularity
* **Quality of Service**: Configurable communication reliability
* **Security**: Built-in security features for robust operation
* **Cross-Platform**: Support for various hardware platforms

Computer Vision Pipeline
-------------------------

Advanced computer vision capabilities using:

* **OpenCV**: Core computer vision library
* **ArUco Markers**: Fiducial marker detection and tracking
* **Deep Learning**: Neural network-based object detection
* **Real-time Processing**: Optimized algorithms for embedded systems

Web Technologies
-----------------

Modern web stack for user interface:

* **React**: Component-based frontend framework
* **Three.js**: 3D visualization and rendering
* **ROS Bridge**: Real-time communication with ROS2 systems
* **Responsive Design**: Works across desktop and mobile devices

Mission Scenarios
=================

The rover is designed to handle various URC mission scenarios:

Science Operations
------------------

* Autonomous sample collection and analysis
* Terrain mapping and geological surveying
* Environmental monitoring and data collection

Navigation Challenges
---------------------

* Rough terrain traversal
* Obstacle detection and avoidance
* GPS-denied navigation using visual odometry
* Multi-modal localization (GPS + visual + IMU)

Equipment Servicing
-------------------

* Autonomous equipment deployment and retrieval
* Precision manipulation tasks
* Tool calibration and maintenance

Safety and Reliability
======================

The system incorporates multiple layers of safety:

* **Fail-safe States**: Automatic transition to safe modes on errors
* **Health Monitoring**: Continuous system health assessment
* **Redundant Systems**: Backup sensors and communication paths
* **Operator Override**: Manual control capabilities

.. warning::
   Safety systems are critical for field operations. Always test safety mechanisms before deployment.

Performance Metrics
===================

The system is designed to meet URC performance requirements:

.. list-table:: Key Performance Metrics
   :header-rows: 1
   :widths: 30 20 50

   * - Metric
     - Target
     - Description
   * - Navigation Accuracy
     - ±10cm
     - Position accuracy during autonomous navigation
   * - Detection Range
     - 5-10m
     - Maximum reliable object detection distance
   * - Processing Latency
     - <100ms
     - End-to-end processing delay for control loops
   * - Mission Autonomy
     - 30+ minutes
     - Continuous autonomous operation time
   * - Communication Range
     - 1km+
     - Reliable telemetry and control range

Development Roadmap
===================

Phase 1: Core Systems (Current)
--------------------------------

* Basic autonomy stack implementation
* Fundamental computer vision capabilities
* Web interface development
* Simulation environment setup

Phase 2: Integration
--------------------

* Multi-subsystem integration
* Field testing and validation
* Performance optimization
* Safety system implementation

Phase 3: Competition Preparation
---------------------------------

* Mission-specific feature development
* Extensive testing and validation
* Documentation completion
* Competition readiness assessment

.. toctree::
   :maxdepth: 2
   :caption: Overview Topics:

   goals_success_metrics
   system_architecture
   technical_overview
   university_rover_challenge
