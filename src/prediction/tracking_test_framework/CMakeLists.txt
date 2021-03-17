# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(tracking_test_framework)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

# Disable warnings due to external dependencies (Eigen)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

set(TRACKING_TEST_FRAMEWORK_LIB_SRC
  src/tracking_test_framework.cpp
)

set(TRACKING_TEST_FRAMEWORK_LIB_HEADERS
  include/tracking_test_framework/tracking_test_framework.hpp
  include/tracking_test_framework/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${TRACKING_TEST_FRAMEWORK_LIB_SRC}
  ${TRACKING_TEST_FRAMEWORK_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(TRACKING_TEST_FRAMEWORK_NODE_SRC
  src/tracking_test_framework_node.cpp
)

set(TRACKING_TEST_FRAMEWORK_NODE_HEADERS
  include/tracking_test_framework/tracking_test_framework_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${TRACKING_TEST_FRAMEWORK_NODE_SRC}
  ${TRACKING_TEST_FRAMEWORK_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::tracking_test_framework::TrackingTestFrameworkNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_tracking_test_framework.cpp)
  set(TEST_TRACKING_TEST_FRAMEWORK_EXE test_tracking_test_framework)
  ament_add_gtest(${TEST_TRACKING_TEST_FRAMEWORK_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_TRACKING_TEST_FRAMEWORK_EXE})
  target_link_libraries(${TEST_TRACKING_TEST_FRAMEWORK_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
)
