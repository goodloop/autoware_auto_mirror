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

project(filter_node_base)

# require that dependencies from package.xml be available
### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(FILTER_NODE_BASE_LIB_SRC
  src/filter_node_base.cpp
)

set(FILTER_NODE_BASE_LIB_HEADERS
  include/filter_node_base/filter_node_base.hpp
  include/filter_node_base/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${FILTER_NODE_BASE_LIB_SRC}
  ${FILTER_NODE_BASE_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_filter_node_base.cpp)
  set(TEST_FILTER_NODE_BASE_EXE test_filter_node_base)
  ament_add_gtest(${TEST_FILTER_NODE_BASE_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_FILTER_NODE_BASE_EXE})
  target_link_libraries(${TEST_FILTER_NODE_BASE_EXE} ${PROJECT_NAME})
  ament_target_dependencies(${TEST_FILTER_NODE_BASE_EXE} "lidar_utils")
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
)
