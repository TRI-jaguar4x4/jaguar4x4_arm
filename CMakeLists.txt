cmake_minimum_required(VERSION 3.5)
project(jaguar4x4_arm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

###add_executable(jaguar4x4_node src/jaguar4x4.cpp src/DrRobotMotionSensorDriver.cpp)
add_library(jaguar4x4_arm_communication src/Communication.cpp src/ArmCommand.cpp)
###ament_target_dependencies(jaguar4x4_node rclcpp std_msgs sensor_msgs geometry_msgs jaguar4x4_msgs Boost)

###install(TARGETS
###  jaguar4x4_node
###  DESTINATION lib/${PROJECT_NAME}
###)

ament_package()
