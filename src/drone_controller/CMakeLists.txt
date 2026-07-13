cmake_minimum_required(VERSION 3.8)
project(drone_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(drone_core REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(position_controller_node src/position_controller_node.cpp)
ament_target_dependencies(position_controller_node
  drone_msgs
  geometry_msgs
  nav_msgs
  rclcpp
)
target_link_libraries(position_controller_node drone_core::drone_core)
target_compile_options(position_controller_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS position_controller_node DESTINATION lib/${PROJECT_NAME})

ament_package()
