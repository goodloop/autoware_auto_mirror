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

#ifndef CLOUD_WRAPPER__VISIBILITY_CONTROL_HPP_
#define CLOUD_WRAPPER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(CLOUD_WRAPPER_BUILDING_DLL) || defined(CLOUD_WRAPPER_EXPORTS)
    #define CLOUD_WRAPPER_PUBLIC __declspec(dllexport)
    #define CLOUD_WRAPPER_LOCAL
  #else  // defined(CLOUD_WRAPPER_BUILDING_DLL) || defined(CLOUD_WRAPPER_EXPORTS)
    #define CLOUD_WRAPPER_PUBLIC __declspec(dllimport)
    #define CLOUD_WRAPPER_LOCAL
  #endif  // defined(CLOUD_WRAPPER_BUILDING_DLL) || defined(CLOUD_WRAPPER_EXPORTS)
#elif defined(__linux__)
  #define CLOUD_WRAPPER_PUBLIC __attribute__((visibility("default")))
  #define CLOUD_WRAPPER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define CLOUD_WRAPPER_PUBLIC __attribute__((visibility("default")))
  #define CLOUD_WRAPPER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // CLOUD_WRAPPER__VISIBILITY_CONTROL_HPP_
