cmake_minimum_required(VERSION 3.8)
project(drone_fleet)

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/fleet_monitor_node.py DESTINATION lib/${PROJECT_NAME})
ament_package()
