cmake_minimum_required(VERSION 3.5)
project(dds_offboard_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(dds_offboard_control_node src/dds_offboard_control_node.cpp)

ament_target_dependencies(dds_offboard_control_node
  rclcpp
  px4_msgs
)

install(TARGETS
  dds_offboard_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

