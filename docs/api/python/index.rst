==================
Python API Reference
==================

This section contains the API documentation for all Python modules in the URC 2026 autonomy system.

.. toctree::
   :maxdepth: 2
   :caption: Calibration

   calibration/camera
   calibration/hand_eye
   calibration/extrinsics

.. toctree::
   :maxdepth: 2
   :caption: Autonomy Subsystems

   autonomy/navigation
   autonomy/computer_vision
   autonomy/slam
   autonomy/state_management
   autonomy/autonomous_typing

.. toctree::
   :maxdepth: 2
   :caption: Utilities

   utils/rosbridge
   utils/calibration_tools
   utils/validation

Core Modules
============

# Note: Uncomment these when the Python modules are properly set up
# .. automodule:: Autonomy.calibration.camera.calibrate_from_markers
#    :members:
#    :undoc-members:
#    :show-inheritance:

# .. automodule:: Autonomy.calibration.hand_eye.hand_eye_calibration
#    :members:
#    :undoc-members:
#    :show-inheritance:

# .. automodule:: Autonomy.code.state_management.state_machine
#    :members:
#    :undoc-members:
#    :show-inheritance:

Class Inheritance Diagrams
=========================

State Management Classes
-------------------------

.. inheritance-diagram:: Autonomy.code.state_management.autonomy_state_machine.state_machine_core.RoverStateMachine
   :parts: 1

.. inheritance-diagram:: Autonomy.code.state_management.autonomy_state_machine.states.SystemState
   :parts: 1

Navigation Classes
------------------

.. inheritance-diagram:: Autonomy.code.navigation.autonomy_navigation.path_planner.PathPlanner
   :parts: 1

Computer Vision Classes
-----------------------

.. inheritance-diagram:: Autonomy.code.computer_vision.autonomy_computer_vision.computer_vision_node.ComputerVisionNode
   :parts: 1

Calibration Classes
-------------------

.. inheritance-diagram:: Autonomy.calibration.camera.calibrate_from_markers
   :parts: 1

Utility Functions
=================

.. autofunction:: Autonomy.scripts.velocity_tools.calculate_velocity_metrics

.. autofunction:: Autonomy.scripts.validate_infrastructure.check_system_health

Calibration Classes
===================

.. autoclass:: Autonomy.calibration.camera.CameraCalibrator
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: Autonomy.calibration.extrinsics.HandEyeCalibrator
   :members:
   :undoc-members:
   :show-inheritance:

Exception Classes
=================

.. autoexception:: Autonomy.code.state_management.exceptions.StateTransitionError

.. autoexception:: Autonomy.calibration.camera.CalibrationError

Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
