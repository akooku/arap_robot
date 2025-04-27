cmake_minimum_required(VERSION 3.8)
project(arap_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# Dependencies
# ========================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)

# ========================
# Python Installation
# ========================
# Ensure Python package is installed
ament_python_install_package(${PROJECT_NAME})

# Install Python node
install(PROGRAMS
  ${PROJECT_NAME}/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========================
# C++ Executables
# ========================
install(
  DIRECTORY config maps rviz launch
  DESTINATION share/${PROJECT_NAME}
)

# ========================
# Tests and Linting
# ========================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ========================
# Finalize Package
# ========================
ament_package()
