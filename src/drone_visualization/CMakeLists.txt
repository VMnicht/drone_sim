cmake_minimum_required(VERSION 3.8)
project(drone_visualization)
find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/drone_marker_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
