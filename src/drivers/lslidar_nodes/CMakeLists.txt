# Copyright 2018 the Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)

### Build the nodes
project(lslidar_nodes)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# The library that generates PointCloud2 is separate so that we don't have to lug around unused code
set(CLOUD_LIB lslidar_cloud_node)
ament_auto_add_library(${CLOUD_LIB} SHARED
  include/lslidar_nodes/lslidar_cloud_node.hpp
  include/lslidar_nodes/visibility_control.hpp
  src/lslidar_cloud_node.cpp)
autoware_set_compile_options(${CLOUD_LIB})

# generate executable for ros1-style standalone nodes
set(CLOUD_EXEC "lslidar_cloud_node_exe")
ament_auto_add_executable(${CLOUD_EXEC} src/lslidar_cloud_node_main.cpp)
autoware_set_compile_options(${CLOUD_EXEC})

## install stuff
ament_auto_package(INSTALL_TO_SHARE param launch)
