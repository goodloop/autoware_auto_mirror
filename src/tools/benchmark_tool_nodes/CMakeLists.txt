#
# Copyright (c) 2020-2021, Arm Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.5)
project(benchmark_tool_nodes)

### Dependencies

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build

add_custom_command(
  OUTPUT perfscript
  COMMAND
    curl -L -s -o cyclonedds.tar.gz https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/0.8.0beta1.tar.gz &&
    tar --strip-components=2 -xzf cyclonedds.tar.gz cyclonedds-0.8.0beta1/examples/perfscript &&
    patch perfscript/perftest ${CMAKE_CURRENT_SOURCE_DIR}/perftest.diff
)

add_custom_target(perftest ALL DEPENDS perfscript)

### Install

install(
  PROGRAMS
    nodes/benchmark_tool_node.py
    nodes/ros_info_node.py
    nodes/sys_info_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/perfscript/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE launch/
)
