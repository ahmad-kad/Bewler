.. _python_api:

==============
Python API Reference
==============

This section contains the API documentation for all Python modules in the URC Machiato 2026 project.
The codebase is organized into ROS2 packages with clear module boundaries.

.. note::
   All Python code follows Google-style docstrings and type hints for better IDE support and documentation generation.

Core Modules
============

The Python codebase is organized as ROS2 packages. Due to the complex dependency management
required for ROS2 packages, the API documentation is provided through manual documentation
of key components rather than automated extraction.

Documented Components
=====================

The following components have been documented with comprehensive docstrings:

LED Status System
-----------------

The LED status system provides visual feedback about rover operational state.

**Key Files:**
- ``Autonomy/code/led_status/autonomy_led_status/led_controller.py`` - Main LED controller
- ``Autonomy/code/led_status/autonomy_led_status/led_status_node.py`` - ROS2 node wrapper

**Features:**
- Competition-compliant color coding (Red, Blue, Green, Yellow, White)
- Multiple pattern support (solid, blink, fade, pulse)
- State machine integration
- Hardware abstraction for different LED implementations

Navigation System
-----------------

The navigation system handles autonomous waypoint navigation and obstacle avoidance.

**Key Files:**
- ``Autonomy/code/navigation/autonomy_navigation/navigation_node.py`` - Main navigation node
- ``Autonomy/code/navigation/autonomy_navigation/path_planner.py`` - Path planning algorithms
- ``Autonomy/code/navigation/autonomy_navigation/motion_controller.py`` - Low-level motion control

**Features:**
- GNSS waypoint navigation
- Terrain-adaptive path planning
- AR tag precision approaches
- Real-time obstacle avoidance
- Multi-waypoint mission support

State Management
----------------

The state management system coordinates all rover subsystems and manages operational modes.

**Key Files:**
- ``Autonomy/code/state_management/autonomy_state_machine/state_machine_core.py`` - Core state machine
- ``Autonomy/code/state_management/autonomy_state_machine/states.py`` - State definitions
- ``Autonomy/code/state_management/autonomy_state_machine/transition_validator.py`` - State transitions

**Features:**
- Hierarchical state machine
- Safety state monitoring
- Mode transitions (manual/autonomous)
- System health assessment
- Mission progress tracking

Utility Modules
===============

**Key Utility Scripts:**
- ``scripts/extract-todos-to-issues.py`` - Extract TODOs from code and create GitHub issues
- ``scripts/velocity_tools.py`` - Development velocity tracking and analysis
- ``scripts/convert_md_to_rst.py`` - Convert markdown documentation to RST format

Configuration Files
===================

Configuration files use YAML format with comprehensive validation:

- ``Autonomy/config/calibration_manager.yaml`` - Camera calibration parameters
- ``Autonomy/code/led_status/config/led_status_config.yaml`` - LED system configuration
- ``Autonomy/code/navigation/config/`` - Navigation parameters
- ``Autonomy/code/slam/config/`` - SLAM configuration files

Exception Hierarchy
===================

All custom exceptions inherit from appropriate base classes:

.. code-block:: python

   class AutonomyError(Exception):
       """Base exception for autonomy system errors."""
       pass

   class NavigationError(AutonomyError):
       """Raised when navigation operations fail."""
       pass

   class VisionError(AutonomyError):
       """Raised when computer vision operations fail."""
       pass

Type Definitions
================

Common type definitions used throughout the codebase:

.. code-block:: python

   from typing import Dict, List, Optional, Tuple, Union
   import numpy as np
   from geometry_msgs.msg import Pose, PoseStamped

   # Common types
   Pose2D = Tuple[float, float, float]  # (x, y, theta)
   Point3D = Tuple[float, float, float]  # (x, y, z)
   ImageArray = np.ndarray  # OpenCV image array
   DetectionResult = Dict[str, Union[str, float, List[Point3D]]]

Logging Configuration
=====================

All modules use structured logging with correlation IDs:

.. code-block:: python

   import logging
   import uuid

   logger = logging.getLogger(__name__)

   def operation_with_logging():
       correlation_id = str(uuid.uuid4())
       logger.info("Starting operation", correlation_id=correlation_id)
       try:
           # Operation logic
           result = perform_operation()
           logger.info("Operation completed", correlation_id=correlation_id)
           return result
       except Exception as e:
           logger.error("Operation failed", correlation_id=correlation_id, error=str(e))
           raise
