cmake_minimum_required(VERSION 3.8)
project(drone_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(drone_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(sensor_simulator_node src/sensor_simulator_node.cpp)
ament_target_dependencies(sensor_simulator_node nav_msgs rclcpp sensor_msgs std_srvs)
target_link_libraries(sensor_simulator_node drone_core::drone_core)
target_compile_options(sensor_simulator_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS sensor_simulator_node DESTINATION lib/${PROJECT_NAME})

ament_package()
