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

project(outlier_filter_nodes)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(OUTLIER_FILTER_NODES_SRC
  src/outlier_filter_node.cpp
)

set(OUTLIER_FILTER_NODES_HEADERS
  include/outlier_filter_nodes/outlier_filter_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${OUTLIER_FILTER_NODES_SRC}
  ${OUTLIER_FILTER_NODES_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::outlier_filter_nodes::OutlierFilterNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_outlier_filter_nodes.cpp)
  set(TEST_OUTLIER_FILTER_NODES_EXE test_outlier_filter_nodes)
  ament_add_gtest(${TEST_OUTLIER_FILTER_NODES_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_OUTLIER_FILTER_NODES_EXE})
  target_link_libraries(${TEST_OUTLIER_FILTER_NODES_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
)
