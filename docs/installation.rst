============
Installation
============

Complete installation guide for the URC 2026 Mars Rover Autonomy System.

System Requirements
===================

**Hardware Requirements:**

- CPU: Quad-core 2.5GHz or better
- RAM: 8GB minimum, 16GB recommended
- Storage: 50GB free space
- GPU: NVIDIA GPU with CUDA support (optional, for computer vision acceleration)

**Software Requirements:**

- Ubuntu 22.04 LTS (recommended) or 20.04 LTS
- ROS2 Humble Hawksbill
- Python 3.10+
- Node.js 18+ (for frontend)

Dependencies Installation
=========================

1. **ROS2 Installation:**

   .. code-block:: bash

      # Follow official ROS2 documentation
      # https://docs.ros2.org/humble/installation/

2. **Python Dependencies:**

   .. code-block:: bash

      pip install -r requirements.txt

3. **Frontend Dependencies:**

   .. code-block:: bash

      cd frontend
      npm install

4. **Documentation Tools (Optional):**

   .. code-block:: bash

      pip install sphinx breathe sphinx-js
      npm install -g jsdoc
      sudo apt-get install doxygen

Project Setup
=============

1. **Clone Repository:**

   .. code-block:: bash

      git clone https://github.com/your-org/urc-2026.git
      cd urc-2026

2. **Build ROS2 Packages:**

   .. code-block:: bash

      cd Autonomy/ros2_ws
      colcon build

3. **Set Python Path:**

   .. code-block:: bash

      export PYTHONPATH="${PYTHONPATH}:$(pwd)"

Verification
============

Run the system health check:

.. code-block:: bash

   python -m Autonomy.scripts.validate_infrastructure

Expected output should show all systems healthy.

Troubleshooting
===============

**Common Issues:**

- **Import errors**: Check PYTHONPATH is set correctly
- **ROS2 build fails**: Ensure all dependencies are installed
- **Web interface not loading**: Check Node.js version and npm install

**Getting Help:**

- Check the :doc:`reference/troubleshooting` guide
- Search existing issues on GitHub
- Contact the development team
