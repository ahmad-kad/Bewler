=============
API Reference
=============

This section contains the complete API reference for all components of the URC Machiato 2026 system.

.. toctree::
   :maxdepth: 2
   :caption: API Reference:

   ros2_interfaces/index
   python/index
   javascript/index

ROS2 Interfaces
===============

The ROS2 interfaces define the communication contracts between system components.

.. doxygenindex::
   :project: autonomy

Python API
==========

Python modules and utilities used throughout the autonomy stack.

.. autosummary::
   :toctree: python/

   scripts.extract_todos_to_issues
   Autonomy.calibration.camera.calibrate_from_markers

JavaScript/TypeScript API
=========================

Frontend components and utilities.

.. js:autosummary::
   :toctree: javascript/

Frontend Components
-------------------

.. js:module:: App

   Main application component providing the primary user interface.

.. js:module:: ThreeDVisualization

   3D visualization component using React Three Fiber.

.. js:module:: StateTree

   State machine visualization and control component.

ROS2 Services and Topics
========================

Core ROS2 interfaces for system communication.

Services
--------

.. list-table:: ROS2 Services
   :header-rows: 1
   :widths: 30 70

   * - Service
     - Description
   * - ``/switch_mode``
     - Switch between different operational modes
   * - ``/get_subsystem_status``
     - Get status of all subsystems
   * - ``/configure_mission``
     - Configure mission parameters
   * - ``/calibrate_camera``
     - Trigger camera calibration
   * - ``/change_state``
     - Change system state
   * - ``/recover_from_safety``
     - Recover from safety state

Topics
------

.. list-table:: ROS2 Topics
   :header-rows: 1
   :widths: 30 70

   * - Topic
     - Description
   * - ``/navigation_status``
     - Current navigation status and pose
   * - ``/vision_detection``
     - Object detection results
   * - ``/slam_status``
     - SLAM system status
   * - ``/system_state``
     - Current system state
   * - ``/safety_status``
     - Safety system status
   * - ``/led_command``
     - LED status commands
