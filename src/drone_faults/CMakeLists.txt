cmake_minimum_required(VERSION 3.8)
project(drone_faults)

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/fault_injector_node.py DESTINATION lib/${PROJECT_NAME})
ament_package()
