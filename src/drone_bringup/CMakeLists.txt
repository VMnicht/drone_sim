cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_yaml_parameters test/test_yaml_parameters.py)
  ament_add_pytest_test(test_mode_panel test/test_mode_panel.py TIMEOUT 45)
  add_launch_test(test/test_hover_launch.py TIMEOUT 30)
endif()

ament_package()
