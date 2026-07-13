cmake_minimum_required(VERSION 3.8)
project(drone_dynamics)

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
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(quadrotor_dynamics_node src/quadrotor_dynamics_node.cpp)
ament_target_dependencies(quadrotor_dynamics_node
  drone_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
)
target_link_libraries(quadrotor_dynamics_node drone_core::drone_core)
target_compile_options(quadrotor_dynamics_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS quadrotor_dynamics_node DESTINATION lib/${PROJECT_NAME})

ament_package()

