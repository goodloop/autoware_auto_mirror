// Copyright 2018 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef LSLIDAR_NODES__VISIBILITY_CONTROL_HPP_
#define LSLIDAR_NODES__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(LSLIDAR_NODES_BUILDING_DLL) || defined(LSLIDAR_NODES_EXPORTS)
    #define LSLIDAR_NODES_PUBLIC __declspec(dllexport)
    #define LSLIDAR_NODES_LOCAL
  #else  // defined(LSLIDAR_NODES_BUILDING_DLL) || defined(LSLIDAR_NODES_EXPORTS)
    #define LSLIDAR_NODES_PUBLIC __declspec(dllimport)
    #define LSLIDAR_NODES_LOCAL
  #endif  // defined(LSLIDAR_NODES_BUILDING_DLL) || defined(LSLIDAR_NODES_EXPORTS)
#elif defined(__linux__)
  #define LSLIDAR_NODES_PUBLIC __attribute__((visibility("default")))
  #define LSLIDAR_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define LSLIDAR_NODES_PUBLIC __attribute__((visibility("default")))
  #define LSLIDAR_NODES_LOCAL __attribute__((visibility("hidden")))
#else  // defined(LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(WINDOWS)

#endif  // LSLIDAR_NODES__VISIBILITY_CONTROL_HPP_
