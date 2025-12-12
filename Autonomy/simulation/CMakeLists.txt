cmake_minimum_required(VERSION 3.8)
project(autonomy_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install model files
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install test scenarios
install(DIRECTORY test_scenarios
  DESTINATION share/${PROJECT_NAME}/
)

# Install tools
install(DIRECTORY tools
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  test_scenarios/autonomous_waypoint_navigation.py
  test_scenarios/gps_denied_slam.py
  test_scenarios/dynamic_obstacle_avoidance.py
  test_scenarios/endurance_slam_test.py
  test_scenarios/sensor_failure_recovery.py
  tools/sensor_quality_analyzer.py
  tools/generate_simulation_report.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install URDF files
install(FILES
  models/rover2025_gazebo.urdf
  DESTINATION share/${PROJECT_NAME}/models
)

# Install world files
install(FILES
  worlds/urc_desert_terrain.world
  worlds/urc_obstacles.world
  worlds/gps_denied_area.world
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install config files
install(FILES
  config/sensor_parameters.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(FILES
  launch/rover_gazebo.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install documentation
install(FILES
  test_scenarios/README.md
  SIMULATION_VALIDATION_CHECKLIST.md
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Add tests here if needed
endif()

ament_package()
