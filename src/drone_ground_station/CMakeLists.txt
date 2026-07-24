cmake_minimum_required(VERSION 3.8)
project(drone_ground_station)

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/web_ground_station_node.py DESTINATION lib/${PROJECT_NAME})
ament_package()
