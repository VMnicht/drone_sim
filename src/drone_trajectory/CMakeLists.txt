cmake_minimum_required(VERSION 3.8)
project(drone_trajectory)

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

add_executable(analytic_trajectory_node src/analytic_trajectory_node.cpp)
ament_target_dependencies(analytic_trajectory_node
  drone_msgs geometry_msgs nav_msgs rclcpp
)
target_link_libraries(analytic_trajectory_node drone_core::drone_core)
target_compile_options(analytic_trajectory_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS analytic_trajectory_node DESTINATION lib/${PROJECT_NAME})

ament_package()
