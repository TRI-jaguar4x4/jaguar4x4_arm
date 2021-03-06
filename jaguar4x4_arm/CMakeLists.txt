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
find_package(geometry_msgs REQUIRED)
find_package(jaguar4x4_arm_msgs REQUIRED)
find_package(jaguar4x4_comms REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(jaguar4x4_arm_node src/jaguar4x4_arm.cpp src/ArmCommand.cpp src/ArmReceive.cpp src/HandCommand.cpp)
ament_target_dependencies(jaguar4x4_arm_node
  geometry_msgs
  jaguar4x4_arm_msgs
  jaguar4x4_comms
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
)

install(TARGETS
  jaguar4x4_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
