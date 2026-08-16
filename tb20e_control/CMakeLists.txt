# Copyright 2026 tb20e_ros2 contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(tb20e_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/tb20e_lever_hardware.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

pluginlib_export_plugin_description_file(
  hardware_interface tb20e_control_plugins.xml
)

add_executable(tb20e_gamepad_node
  src/tb20e_gamepad_node.cpp
)
target_compile_features(tb20e_gamepad_node PUBLIC cxx_std_17)
target_include_directories(tb20e_gamepad_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tb20e_gamepad_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS tb20e_gamepad_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_math_utils
    test/test_math_utils.cpp
  )
  target_compile_features(test_math_utils PUBLIC cxx_std_17)
  target_include_directories(test_math_utils PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_gamepad_mapping
    test/test_gamepad_mapping.cpp
  )
  target_compile_features(test_gamepad_mapping PUBLIC cxx_std_17)
  target_include_directories(test_gamepad_mapping PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_position_command
    test/test_position_command.cpp
  )
  target_compile_features(test_position_command PUBLIC cxx_std_17)
  target_include_directories(test_position_command PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

ament_package()
