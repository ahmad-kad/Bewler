================
C++ API Reference
================

This section contains the API documentation for all C++ ROS2 packages and interfaces in the URC 2026 autonomy system.

Full API Documentation
======================

.. doxygenindex::
   :project: autonomy

Class Hierarchy
===============

.. doxygenclass:: autonomy_state_machine::state_machine_core::RoverStateMachine
   :members:
   :private-members:
   :protected-members:

.. doxygenclass:: autonomy_navigation::navigation_node::NavigationNode
   :members:
   :private-members:
   :protected-members:

.. doxygenclass:: autonomy_computer_vision::computer_vision_node::ComputerVisionNode
   :members:
   :private-members:
   :protected-members:

ROS2 Packages
=============

.. toctree::
   :maxdepth: 2
   :caption: Core Packages

   packages/autonomy_interfaces
   packages/navigation
   packages/computer_vision
   packages/slam
   packages/state_management

Message Types
=============

System State Messages
---------------------

* **SystemState** - Comprehensive state information for the rover state machine
* **StateTransition** - State transition events and metadata

Navigation Messages
-------------------

* **NavigationStatus** - Current navigation state and progress
* **NavigationGoal** - Target pose for navigation tasks

Vision Messages
---------------

* **VisionDetection** - Object detection results
* **CameraCommand** - Camera control commands

Service Interfaces
==================

Calibration Services
---------------------

* **CalibrateCamera** - Camera intrinsic calibration service
* **LoadCalibrationParameters** - Load calibration data service
* **ValidateCalibration** - Validate calibration quality service

State Management Services
-------------------------

* **ChangeState** - Request state transitions
* **GetSystemState** - Query current system state

Action Interfaces
=================

Navigation Actions
------------------

* **NavigateToPose** - Navigate to a specific pose

Typing Actions
--------------

* **PerformTyping** - Execute autonomous typing tasks

Node Classes
============

* **NavigationNode** - ROS2 navigation node implementation
* **VisionNode** - Computer vision processing node
* **SLAMNode** - Simultaneous Localization and Mapping node
* **StateManagerNode** - System state management node

Utility Classes
===============

* **CameraCalibrator** - Camera calibration utilities
* **HandEyeCalibrator** - Hand-eye calibration utilities

Exception Classes
=================

* **ServiceException** - Service call exceptions
* **NavigationException** - Navigation-related exceptions

Indices and Tables
==================

* :ref:`genindex`
* :ref:`search`
