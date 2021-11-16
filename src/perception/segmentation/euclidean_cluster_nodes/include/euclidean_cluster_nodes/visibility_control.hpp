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

#ifndef EUCLIDEAN_CLUSTER_NODES__VISIBILITY_CONTROL_HPP_
#define EUCLIDEAN_CLUSTER_NODES__VISIBILITY_CONTROL_HPP_


////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(EUCLIDEAN_CLUSTER_NODES_BUILDING_DLL) || defined(EUCLIDEAN_CLUSTER_NODES_EXPORTS)
    #define EUCLIDEAN_CLUSTER_NODES_PUBLIC __declspec(dllexport)
    #define EUCLIDEAN_CLUSTER_NODES_LOCAL
  #else
// defined(EUCLIDEAN_CLUSTER_NODES_BUILDING_DLL) || defined(EUCLIDEAN_CLUSTER_NODES_EXPORTS)
    #define EUCLIDEAN_CLUSTER_NODES_PUBLIC __declspec(dllimport)
    #define EUCLIDEAN_CLUSTER_NODES_LOCAL
  #endif
// defined(EUCLIDEAN_CLUSTER_NODES_BUILDING_DLL) || defined(EUCLIDEAN_CLUSTER_NODES_EXPORTS)
#elif defined(__linux__)
  #define EUCLIDEAN_CLUSTER_NODES_PUBLIC __attribute__((visibility("default")))
  #define EUCLIDEAN_CLUSTER_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define EUCLIDEAN_CLUSTER_NODES_PUBLIC __attribute__((visibility("default")))
  #define EUCLIDEAN_CLUSTER_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__QNXNTO__)
  #define EUCLIDEAN_CLUSTER_NODES_PUBLIC __attribute__((visibility("default")))
  #define EUCLIDEAN_CLUSTER_NODES_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // EUCLIDEAN_CLUSTER_NODES__VISIBILITY_CONTROL_HPP_
