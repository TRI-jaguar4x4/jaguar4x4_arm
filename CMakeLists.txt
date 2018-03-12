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
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(jaguar4x4_arm_communication src/Communication.cpp src/ArmCommand.cpp src/ArmReceive.cpp)
add_executable(jaguar4x4_arm_node src/jaguar4x4_arm.cpp)
target_link_libraries(jaguar4x4_arm_node jaguar4x4_arm_communication)
ament_target_dependencies(jaguar4x4_arm_node rclcpp geometry_msgs)

install(TARGETS
  jaguar4x4_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  jaguar4x4_arm_communication
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_package()
