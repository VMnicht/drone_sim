cmake_minimum_required(VERSION 3.8)
project(drone_perception)

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/local_perception_node.py DESTINATION lib/${PROJECT_NAME})
ament_package()
