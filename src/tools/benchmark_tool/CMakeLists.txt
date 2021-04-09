#
# Copyright (c) 2020-2021, Arm Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.5)
project(benchmark_tool)

### Dependencies

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build

# Compile Python module kittiobjeval
add_custom_target(kittiobjeval.so ALL
  COMMAND
    ${PYTHON_EXECUTABLE} setup.py build_ext
      --build-lib ${CMAKE_CURRENT_BINARY_DIR}/kittiobjdetsdk/lib
      --build-temp ${CMAKE_CURRENT_BINARY_DIR}/kittiobjdetsdk
  DEPENDS
    ${PROJECT_NAME}/kittiobjdetsdk/src/kittiobjevalmodule.cpp
    ${PROJECT_NAME}/kittiobjdetsdk/src/evaluate_object.cpp
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/kittiobjdetsdk
)

### Install

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/kittiobjdetsdk/lib/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/kittiobjdetsdk/
  FILES_MATCHING PATTERN "*.so"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
