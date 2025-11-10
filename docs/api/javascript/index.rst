==================================
JavaScript/TypeScript API Reference
==================================

This section contains the API documentation for the React-based frontend interface of the URC 2026 autonomy system.

.. toctree::
   :maxdepth: 2
   :caption: Components

   components/App
   components/StateTree
   components/ThreeDVisualization

.. toctree::
   :maxdepth: 2
   :caption: Hooks

   hooks/useROS
   hooks/useStateMachine

.. toctree::
   :maxdepth: 2
   :caption: Utilities

   utils/rosbridge
   utils/stateDefinitions

Main Application Component
==========================

**App** - Main React component for rover control interface

React Components
================

State Tree Component
---------------------

* **StateTree** - Hierarchical state visualization component
* **StateControls** - State transition control panel

3D Visualization Component
--------------------------

* **ThreeDVisualization** - 3D rover model and environment renderer

Custom Hooks
============

ROS Hook
--------

* **useROS** - ROS WebSocket connection management hook

State Machine Hook
------------------

* **useStateMachine** - State machine state and controls hook

Utility Functions
=================

ROS Bridge Utilities
--------------------

* **getStatusText()** - Convert status codes to human-readable text
* **connectToROS()** - Establish ROS bridge connection

State Definitions
-----------------

* **StateTransitions** - State transition configuration
* **SystemState** - System state enumeration

Type Definitions
================

* **ROSConnection** - ROS connection state interface
* **StateMachineState** - State machine state interface
* **NavigationGoal** - Navigation goal interface

Error Classes
=============

* **ROSConnectionError** - ROS connection failures
* **StateTransitionError** - State transition validation errors

Indices and Tables
==================

* :ref:`genindex`
* :ref:`search`
