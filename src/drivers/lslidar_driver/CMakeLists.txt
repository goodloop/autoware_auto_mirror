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
project(lslidar_driver)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build driver library
ament_auto_add_library(${PROJECT_NAME} SHARED
        "include/lslidar_driver/lslidar_translator.hpp"
        "include/lslidar_driver/ls16_data.hpp"
		"include/lslidar_driver/ch64w_data.hpp"
        "include/lslidar_driver/ch128_data.hpp"
        "include/lslidar_driver/ls32_data.hpp"
        "include/lslidar_driver/common.hpp"
        "src/ls16_data.cpp"
		"src/ch64w_data.cpp"
        "src/ch128_data.cpp"
        "src/ls32_data.cpp"
        "src/lslidar_translator.cpp")

autoware_set_compile_options(${PROJECT_NAME})


# ament package generation and installing
ament_auto_package()
