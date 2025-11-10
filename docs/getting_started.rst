Getting Started
===============

This guide will help you set up your development environment and get started with the URC 2026 Robotics Project.

Prerequisites
-------------

Before setting up the project, ensure you have the following prerequisites installed:

System Requirements
~~~~~~~~~~~~~~~~~~~

- **Operating System**: Ubuntu 22.04 LTS (recommended) or Ubuntu 20.04 LTS
- **RAM**: Minimum 16GB, Recommended 32GB+
- **Storage**: 50GB+ free space
- **GPU**: NVIDIA GPU with CUDA support (optional, for accelerated computer vision)

Required Software
~~~~~~~~~~~~~~~~~

.. list-table:: Required Software
   :header-rows: 1
   :widths: 20 40 40

   * - **Software**
     - **Version**
     - **Installation Command**
   * - **Git**
     - Latest
     - ``sudo apt install git``
   * - **Python**
     - 3.10+
     - ``sudo apt install python3 python3-pip python3-venv``
   * - **Node.js**
     - 18+
     - ``curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash - && sudo apt-get install -y nodejs``
   * - **Docker**
     - Latest
     - ``curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh``
   * - **VS Code**
     - Latest
     - Download from `https://code.visualstudio.com/`_

Recommended Extensions
~~~~~~~~~~~~~~~~~~~~~~

For the best development experience, install these VS Code extensions:

- **C/C++** (ms-vscode.cpptools) - C++ development support
- **Python** (ms-python.python) - Python development support
- **ROS** (ms-iot.vscode-ros) - ROS2 development support
- **Docker** (ms-azuretools.vscode-docker) - Docker integration
- **GitLens** (eamodio.gitlens) - Enhanced Git capabilities

Project Setup
-------------

Clone the Repository
~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Clone the repository
   git clone https://github.com/your-org/urc-2026-machiato.git
   cd urc-2026-machiato

   # Initialize submodules if any
   git submodule update --init --recursive

Environment Setup
~~~~~~~~~~~~~~~~~

1. **Python Virtual Environment**

   .. code-block:: bash

      # Create virtual environment
      python3 -m venv venv
      source venv/bin/activate

      # Install Python dependencies
      pip install -r requirements.txt

2. **ROS2 Installation**

   .. code-block:: bash

      # Add ROS2 repository
      sudo apt update && sudo apt install locales
      sudo locale-gen en_US en_US.UTF-8
      sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
      export LANG=en_US.UTF-8

      sudo apt install software-properties-common
      sudo add-apt-repository universe

      sudo apt update && sudo apt install curl -y
      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

      sudo apt update
      sudo apt upgrade

      # Install ROS2 Humble
      sudo apt install ros-humble-desktop

      # Source ROS2 setup
      source /opt/ros/humble/setup.bash

3. **Frontend Setup**

   .. code-block:: bash

      cd frontend

      # Install Node.js dependencies
      npm install

      # Start development server
      npm run dev

4. **Build ROS2 Packages**

   .. code-block:: bash

      cd Autonomy

      # Install ROS2 dependencies
      sudo apt install python3-colcon-common-extensions

      # Build packages
      colcon build --symlink-install

      # Source workspace
      source install/setup.bash

Development Workflow
-------------------

Daily Development Cycle
~~~~~~~~~~~~~~~~~~~~~~~

1. **Pull Latest Changes**

   .. code-block:: bash

      git pull origin main
      git submodule update --recursive

2. **Activate Environment**

   .. code-block:: bash

      # Python virtual environment
      source venv/bin/activate

      # ROS2 workspace
      cd Autonomy
      source /opt/ros/humble/setup.bash
      source install/setup.bash

3. **Run Tests**

   .. code-block:: bash

      # Python tests
      python -m pytest

      # ROS2 tests
      colcon test

4. **Start Development**

   .. code-block:: bash

      # Frontend development
      cd ../frontend && npm run dev

      # ROS2 development
      cd ../Autonomy && ros2 launch system_integration.launch.py

Building Documentation
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   cd docs

   # Build HTML documentation
   make html

   # Open documentation in browser
   xdg-open _build/html/index.html

Code Quality Checks
~~~~~~~~~~~~~~~~~~~

Before committing code, run these quality checks:

.. code-block:: bash

   # Python linting and formatting
   black .
   flake8 .
   mypy .

   # C++ linting
   cppcheck --enable=all --std=c++17 --language=c++ .

   # Frontend linting
   cd frontend && npm run lint

Running Tests
-------------

Unit Tests
~~~~~~~~~~

.. code-block:: bash

   # Python unit tests
   python -m pytest tests/ -v

   # ROS2 unit tests
   cd Autonomy && colcon test --packages-select YOUR_PACKAGE

Integration Tests
~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Run integration test suite
   python scripts/run_integration_tests.py

   # ROS2 integration tests
   ros2 launch integration_tests.launch.py

Simulation Testing
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start Gazebo simulation
   ros2 launch simulation full_simulation.launch.py

   # Run automated simulation tests
   python scripts/simulation_validation.py

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**ROS2 Build Failures**

.. code-block:: bash

   # Clean and rebuild
   cd Autonomy
   rm -rf build/ install/ log/
   colcon build --symlink-install

**Python Import Errors**

.. code-block:: bash

   # Ensure virtual environment is activated
   source venv/bin/activate

   # Reinstall dependencies
   pip install -r requirements.txt --force-reinstall

**Frontend Build Issues**

.. code-block:: bash

   cd frontend

   # Clear node_modules and reinstall
   rm -rf node_modules package-lock.json
   npm install

Getting Help
------------

- **Documentation**: Check the full documentation at ``docs/_build/html/index.html``
- **Issues**: Report bugs and request features on GitHub Issues
- **Discussions**: Join development discussions on GitHub Discussions
- **Team Chat**: Connect with the team on Slack/Discord

Next Steps
----------

Now that your environment is set up:

1. Read the :doc:`architecture` documentation to understand the system design
2. Explore the :doc:`development/workflow` guide for coding standards
3. Check the :doc:`api/index` for available interfaces
4. Join a development team and start contributing!

Happy coding! 🚀
