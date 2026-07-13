cmake_minimum_required(VERSION 3.8)
project(drone_core)

option(
  DRONE_CORE_STANDALONE
  "Build drone_core and its tests without ROS2/ament"
  OFF
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
if(DRONE_CORE_STANDALONE)
  include(CTest)
else()
  find_package(ament_cmake REQUIRED)
endif()

add_library(${PROJECT_NAME} SHARED
  src/model_based_controller.cpp
  src/motor_mixer.cpp
  src/quadrotor_model.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(NOT DRONE_CORE_STANDALONE)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE drone_core::)
  ament_export_dependencies(Eigen3)
endif()

if(BUILD_TESTING AND NOT DRONE_CORE_STANDALONE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_quadrotor_model test/test_quadrotor_model.cpp)
  if(TARGET test_quadrotor_model)
    target_link_libraries(test_quadrotor_model ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_model_based_controller test/test_model_based_controller.cpp)
  if(TARGET test_model_based_controller)
    target_link_libraries(test_model_based_controller ${PROJECT_NAME})
  endif()
endif()

if(BUILD_TESTING AND DRONE_CORE_STANDALONE)
  find_package(GTest REQUIRED)
  add_executable(test_quadrotor_model test/test_quadrotor_model.cpp)
  target_link_libraries(test_quadrotor_model PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  add_test(NAME test_quadrotor_model COMMAND test_quadrotor_model)

  add_executable(test_model_based_controller test/test_model_based_controller.cpp)
  target_link_libraries(
    test_model_based_controller PRIVATE ${PROJECT_NAME} GTest::gtest_main
  )
  add_test(NAME test_model_based_controller COMMAND test_model_based_controller)
endif()

if(NOT DRONE_CORE_STANDALONE)
  ament_package()
endif()
