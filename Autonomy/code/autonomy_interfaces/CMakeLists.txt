cmake_minimum_required(VERSION 3.8)
project(autonomy_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define the interface files
set(msg_files
  "msg/NavigationStatus.msg"
  "msg/VisionDetection.msg"
  "msg/SlamStatus.msg"
  "msg/TypingGoal.msg"
  "msg/LedCommand.msg"
  "msg/CameraCommand.msg"
  "msg/SystemState.msg"
  "msg/StateTransition.msg"
  "msg/SafetyStatus.msg"
  "msg/FollowMeStatus.msg"
  "msg/ArmAlignmentCommand.msg"
)

set(srv_files
  "srv/SwitchMode.srv"
  "srv/GetSubsystemStatus.srv"
  "srv/ConfigureMission.srv"
  "srv/CalibrateCamera.srv"
  "srv/LoadCalibrationParameters.srv"
  "srv/ValidateCalibration.srv"
  "srv/ChangeState.srv"
  "srv/GetSystemState.srv"
  "srv/RecoverFromSafety.srv"
  "srv/DetectAruco.srv"
  "srv/FollowMeControl.srv"
  "srv/DetectMissionAruco.srv"
  "srv/SafestopControl.srv"
  "srv/SoftwareEstop.srv"
  "srv/NavigationIntegrityCheck.srv"
  "srv/TimingSafetyCheck.srv"
)

set(action_files
  "action/NavigateToPose.action"
  "action/PerformTyping.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
