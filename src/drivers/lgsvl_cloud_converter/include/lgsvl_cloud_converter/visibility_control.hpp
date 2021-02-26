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

#ifndef LGSVL_CLOUD_CONVERTER__VISIBILITY_CONTROL_HPP_
#define LGSVL_CLOUD_CONVERTER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(LGSVL_CLOUD_CONVERTER_BUILDING_DLL) || defined(LGSVL_CLOUD_CONVERTER_EXPORTS)
    #define LGSVL_CLOUD_CONVERTER_PUBLIC __declspec(dllexport)
    #define LGSVL_CLOUD_CONVERTER_LOCAL
  #else  // defined(LGSVL_CLOUD_CONVERTER_BUILDING_DLL) || defined(LGSVL_CLOUD_CONVERTER_EXPORTS)
    #define LGSVL_CLOUD_CONVERTER_PUBLIC __declspec(dllimport)
    #define LGSVL_CLOUD_CONVERTER_LOCAL
  #endif  // defined(LGSVL_CLOUD_CONVERTER_BUILDING_DLL) || defined(LGSVL_CLOUD_CONVERTER_EXPORTS)
#elif defined(__linux__)
  #define LGSVL_CLOUD_CONVERTER_PUBLIC __attribute__((visibility("default")))
  #define LGSVL_CLOUD_CONVERTER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define LGSVL_CLOUD_CONVERTER_PUBLIC __attribute__((visibility("default")))
  #define LGSVL_CLOUD_CONVERTER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // LGSVL_CLOUD_CONVERTER__VISIBILITY_CONTROL_HPP_
