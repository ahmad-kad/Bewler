.. URC 2026 - Mars Rover Autonomy System documentation master file, created by
   sphinx-quickstart on Wed Nov  5 12:00:00 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

==========================================
URC 2026 - Mars Rover Autonomy System
==========================================

.. image:: _static/urc_logo.png
   :alt: URC 2026 Logo
   :align: center
   :width: 200px

Welcome to the official documentation for the University Rover Challenge 2026 Mars Rover Autonomy System.

This comprehensive documentation covers all aspects of our autonomous rover platform, including:

- 🤖 **Autonomy Software**: State machines, navigation, computer vision, and SLAM
- 📷 **Calibration Systems**: Camera calibration, hand-eye calibration, and sensor fusion
- 🌐 **Frontend Interface**: React-based web interface for rover control and monitoring
- 🔧 **System Integration**: ROS2 middleware, hardware interfaces, and deployment

.. toctree::
   :maxdepth: 2
   :caption: 🚀 Getting Started

   overview
   quickstart
   installation

.. toctree::
   :maxdepth: 2
   :caption: 📚 System Architecture

   architecture/overview
   architecture/state_machine
   architecture/ros2_interfaces
   architecture/hardware

.. toctree::
   :maxdepth: 2
   :caption: 🔧 Subsystems

   subsystems/navigation
   subsystems/computer_vision
   subsystems/slam
   subsystems/autonomous_typing
   subsystems/led_status
   subsystems/state_management

.. toctree::
   :maxdepth: 2
   :caption: 📐 Calibration

   calibration/camera_calibration
   calibration/hand_eye_calibration
   calibration/extrinsics
   calibration/aruco_tags

.. toctree::
   :maxdepth: 2
   :caption: 💻 API Reference

   api/python/index
   api/cpp/index
   api/javascript/index

.. toctree::
   :maxdepth: 2
   :caption: 🔨 Development

   development/contributing
   development/testing
   development/deployment
   development/docker

.. toctree::
   :maxdepth: 2
   :caption: 📖 Reference

   reference/glossary
   reference/troubleshooting
   reference/faq

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Project Information
===================

.. list-table:: Project Details
   :header-rows: 1
   :widths: 30 70

   * - **Project Name**
     - URC 2026 Mars Rover Autonomy System
   * - **Version**
     - 1.0.0
   * - **License**
     - MIT
   * - **Repository**
     - `GitHub Repository <https://github.com/your-org/urc-2026>`_
   * - **Documentation**
     - `Read the Docs <https://urc-2026.readthedocs.io/>`_
   * - **Contact**
     - urc2026@your-university.edu

Quick Links
===========

🏃‍♂️ **Quick Start**
   Get up and running with the rover system in minutes.

📚 **API Reference**
   Complete API documentation for all components. See :doc:`api/python/index` for Python APIs.

🔧 **Development**
   Learn how to contribute to the project.

📐 **Calibration**
   Camera and sensor calibration procedures.
