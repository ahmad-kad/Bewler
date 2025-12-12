cmake_minimum_required(VERSION 3.8)
project(autonomy_state_management)

# Default to C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autonomy_interfaces REQUIRED)

# Install Python modules
ament_python_install_package(autonomy_state_machine)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install resource marker file
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  autonomy_state_machine/state_machine_director.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Linting
  ament_lint_auto_find_test_dependencies()

  # Python tests
  ament_add_pytest_test(state_management_tests tests/
    TIMEOUT 60
  )
endif()

ament_package()
