cmake_minimum_required(VERSION 3.8)
project(drone_tools)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/experiment_recorder.py
    scripts/waypoint_mission_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

