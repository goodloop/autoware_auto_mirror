// Copyright 2021 The Autoware Foundation
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

#ifndef MEASUREMENT_CONVERSION__VISIBILITY_CONTROL_HPP_
#define MEASUREMENT_CONVERSION__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(MEASUREMENT_CONVERSION_BUILDING_DLL) || defined(MEASUREMENT_CONVERSION_EXPORTS)
    #define MEASUREMENT_CONVERSION_PUBLIC __declspec(dllexport)
    #define MEASUREMENT_CONVERSION_LOCAL
  #else  // defined(MEASUREMENT_CONVERSION_BUILDING_DLL) || defined(MEASUREMENT_CONVERSION_EXPORTS)
    #define MEASUREMENT_CONVERSION_PUBLIC __declspec(dllimport)
    #define MEASUREMENT_CONVERSION_LOCAL
  #endif  // defined(MEASUREMENT_CONVERSION_BUILDING_DLL) || defined(MEASUREMENT_CONVERSION_EXPORTS)
#elif defined(__linux__)
  #define MEASUREMENT_CONVERSION_PUBLIC __attribute__((visibility("default")))
  #define MEASUREMENT_CONVERSION_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define MEASUREMENT_CONVERSION_PUBLIC __attribute__((visibility("default")))
  #define MEASUREMENT_CONVERSION_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // MEASUREMENT_CONVERSION__VISIBILITY_CONTROL_HPP_
