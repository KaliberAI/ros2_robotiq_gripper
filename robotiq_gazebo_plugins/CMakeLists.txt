cmake_minimum_required(VERSION 3.8)
project(robotiq_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

# Include directories
include_directories(
  include
)

# Build the mimic joint plugin
add_library(mimic_joint_plugin SHARED src/mimic_joint_plugin.cpp)
ament_target_dependencies(mimic_joint_plugin rclcpp)
target_link_libraries(mimic_joint_plugin
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
)

# Install plugin library
install(
  TARGETS mimic_joint_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Export plugin path so Gazebo can find it
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

ament_package()