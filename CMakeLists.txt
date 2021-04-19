cmake_minimum_required(VERSION 3.5)
project(stewart_actuator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(stewart_interfaces REQUIRED)

# Add executables
add_executable(actuator src/actuator_node.cpp src/canbus.cpp)
ament_target_dependencies(actuator rclcpp stewart_interfaces)

# Include the directories where header files in found
include_directories(include/stewart_actuator)

install(TARGETS
  actuator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
