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

#ifndef VELODYNE_NODES__VISIBILITY_CONTROL_HPP_
#define VELODYNE_NODES__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(VELODYNE_NODES_BUILDING_DLL) || defined(VELODYNE_NODES_EXPORTS)
    #define VELODYNE_NODES_PUBLIC __declspec(dllexport)
    #define VELODYNE_NODES_LOCAL
  #else  // defined(VELODYNE_NODES_BUILDING_DLL) || defined(VELODYNE_NODES_EXPORTS)
    #define VELODYNE_NODES_PUBLIC __declspec(dllimport)
    #define VELODYNE_NODES_LOCAL
  #endif  // defined(VELODYNE_NODES_BUILDING_DLL) || defined(VELODYNE_NODES_EXPORTS)
#elif defined(__linux__)
  #define VELODYNE_NODES_PUBLIC __attribute__((visibility("default")))
  #define VELODYNE_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define VELODYNE_NODES_PUBLIC __attribute__((visibility("default")))
  #define VELODYNE_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__QNXNTO__)
  #define VELODYNE_NODES_PUBLIC __attribute__((visibility("default")))
  #define VELODYNE_NODES_LOCAL __attribute__((visibility("hidden")))
#else  // defined(LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(WINDOWS)

#endif  // VELODYNE_NODES__VISIBILITY_CONTROL_HPP_
