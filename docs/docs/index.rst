<<<<<<< Current (Your changes)
=======
.. URC Machiato 2026 documentation master file

==========================================
URC Machiato 2026 - University Rover Challenge
==========================================

.. image:: _static/urc_logo.png
   :alt: URC Machiato 2026 Logo
   :align: center
   :scale: 50%

The URC Machiato 2026 project is a comprehensive autonomous robotics system designed for the University Rover Challenge.
This documentation covers the complete system including autonomy software, computer vision, navigation, simulation,
and web interface components.

.. note::
   This documentation is automatically generated from code comments and markdown files throughout the project.

Overview
========

The URC Machiato 2026 system consists of:

- **Autonomy Stack**: ROS2-based autonomy software with multiple subsystems
- **Computer Vision**: Real-time perception and object detection
- **Navigation**: Path planning and motion control
- **Simulation**: Gazebo-based simulation environment
- **Web Interface**: React-based monitoring and control interface
- **State Management**: Centralized system coordination

Quick Start
===========

.. tabs::

   .. tab:: Development Setup

      .. code-block:: bash

         # Clone and setup the project
         git clone <repository-url>
         cd urc-machiato-2026

         # Install dependencies
         ./scripts/setup_development.sh

         # Build documentation
         cd docs/docs && make html

   .. tab:: Running the System

      .. code-block:: bash

         # Launch the complete system
         ros2 launch autonomy_system system.launch.py

         # Or run individual components
         ros2 run autonomy_navigation navigation_node
         ros2 run autonomy_computer_vision computer_vision_node

   .. tab:: Web Interface

      .. code-block:: bash

         cd frontend
         npm install
         npm run dev

.. toctree::
   :maxdepth: 2
   :caption: 🚀 Getting Started

   getting_started/overview
   getting_started/installation
   getting_started/quick_start
   getting_started/development_setup

.. toctree::
   :maxdepth: 2
   :caption: 🤖 Autonomy System

   autonomy/overview
   autonomy/architecture
   autonomy/state_management
   autonomy/subsystems/index

.. toctree::
   :maxdepth: 3
   :caption: 🔧 API Reference

   api/python/index
   api/cpp/index
   api/javascript/index
   api/ros_interfaces

.. toctree::
   :maxdepth: 2
   :caption: 🖥️ User Interface

   frontend/overview
   frontend/components
   frontend/api

.. toctree::
   :maxdepth: 2
   :caption: 🔬 Simulation & Testing

   simulation/overview
   simulation/worlds
   simulation/testing

.. toctree::
   :maxdepth: 2
   :caption: 🛡️ Safety System

   safety_system/index
   safety_system/architecture
   safety_system/components
   safety_system/procedures
   safety_system/testing
   safety_system/integration
   safety_system/configuration

.. toctree::
   :maxdepth: 2
   :caption: 🌐 Sensor Bridge

   sensor_bridge/index
   sensor_bridge/integration
   sensor_bridge/usage
   sensor_bridge/configuration
   sensor_bridge/api
   sensor_bridge/examples
   sensor_bridge/migration

.. toctree::
   :maxdepth: 2
   :caption: 📚 Guides & Tutorials

   guides/calibration
   guides/troubleshooting
   guides/contributing
   guides/best_practices

.. toctree::
   :maxdepth: 1
   :caption: 📋 Project Documentation

   project/requirements
   project/architecture
   project/roadmap
   project/team

Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Project Links
=============

* `GitHub Repository <https://github.com/your-org/urc-machiato-2026>`_
* `Issue Tracker <https://github.com/your-org/urc-machiato-2026/issues>`_
* `Project Wiki <https://github.com/your-org/urc-machiato-2026/wiki>`_
* `URC Competition <https://urc.marssociety.org/>`_

>>>>>>> Incoming (Background Agent changes)
