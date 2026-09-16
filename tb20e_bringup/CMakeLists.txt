# Copyright 2026 tb20e_bringup contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(tb20e_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
