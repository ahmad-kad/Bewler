==========
Quick Start
==========

This guide will help you get up and running with the URC 2026 Mars Rover Autonomy System quickly.

Prerequisites
=============

Before starting, ensure you have:

- Python 3.10+
- ROS2 Humble (Ubuntu 22.04 recommended)
- Git
- Basic understanding of robotics and autonomous systems

Installation
============

1. Clone the repository:

   .. code-block:: bash

      git clone https://github.com/your-org/urc-2026.git
      cd urc-2026

2. Install Python dependencies:

   .. code-block:: bash

      pip install -r requirements.txt

3. Set up ROS2 workspace:

   .. code-block:: bash

      cd Autonomy/ros2_ws
      colcon build

First Run
=========

1. Start the ROS2 system:

   .. code-block:: bash

      ros2 launch autonomy_system system.launch.py

2. Open the web interface:

   Open your browser to ``http://localhost:3000``

3. Begin calibration:

   Follow the on-screen calibration wizard for camera setup.

Next Steps
==========

- Read the :doc:`overview` for system architecture
- Check the :doc:`api/python/index` for API documentation
- Join our Discord/Slack for community support
