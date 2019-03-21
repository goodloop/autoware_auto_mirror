// Copyright 2017-2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#ifndef RAY_GROUND_CLASSIFIER_NODES__VISIBILITY_CONTROL_HPP_
#define RAY_GROUND_CLASSIFIER_NODES__VISIBILITY_CONTROL_HPP_

#include <apexutils/apexdef.h>

////////////////////////////////////////////////////////////////////////////////
#if defined(APEX_WINDOWS)
  #if defined(RAY_GROUND_CLASSIFIER_NODES_BUILDING_DLL) || \
  defined(RAY_GROUND_CLASSIFIER_NODES_EXPORTS)
    #define RAY_GROUND_CLASSIFIER_NODES_PUBLIC __declspec(dllexport)
    #define RAY_GROUND_CLASSIFIER_NODES_LOCAL
  #else  // defined(RAY_GROUND_CLASSIFIER_NODES_BUILDING_DLL) ||
         // defined(RAY_GROUND_CLASSIFIER_NODES_EXPORTS)
    #define RAY_GROUND_CLASSIFIER_NODES_PUBLIC __declspec(dllimport)
    #define RAY_GROUND_CLASSIFIER_NODES_LOCAL
  #endif  // defined(RAY_GROUND_CLASSIFIER_NODES_BUILDING_DLL) ||
          // defined(RAY_GROUND_CLASSIFIER_NODES_EXPORTS)
#elif defined(APEX_LINUX)
  #define RAY_GROUND_CLASSIFIER_NODES_PUBLIC __attribute__((visibility("default")))
  #define RAY_GROUND_CLASSIFIER_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(APEX_OSX)
  #define RAY_GROUND_CLASSIFIER_NODES_PUBLIC __attribute__((visibility("default")))
  #define RAY_GROUND_CLASSIFIER_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(APEX_QNX)
  #define RAY_GROUND_CLASSIFIER_NODES_PUBLIC __attribute__((visibility("default")))
  #define RAY_GROUND_CLASSIFIER_NODES_LOCAL __attribute__((visibility("hidden")))
#else  // defined(APEX_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(APEX_WINDOWS)

#endif  // RAY_GROUND_CLASSIFIER_NODES__VISIBILITY_CONTROL_HPP_
