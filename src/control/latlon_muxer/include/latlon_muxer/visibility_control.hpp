// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LATLON_MUXER__VISIBILITY_CONTROL_HPP_
#define LATLON_MUXER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(LATLON_MUXER_BUILDING_DLL) || defined(LATLON_MUXER_EXPORTS)
    #define LATLON_MUXER_PUBLIC __declspec(dllexport)
    #define LATLON_MUXER_LOCAL
  #else  // defined(LATLON_MUXER_BUILDING_DLL) || defined(LATLON_MUXER_EXPORTS)
    #define LATLON_MUXER_PUBLIC __declspec(dllimport)
    #define LATLON_MUXER_LOCAL
  #endif  // defined(LATLON_MUXER_BUILDING_DLL) || defined(LATLON_MUXER_EXPORTS)
#elif defined(__linux__)
  #define LATLON_MUXER_PUBLIC __attribute__((visibility("default")))
  #define LATLON_MUXER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define LATLON_MUXER_PUBLIC __attribute__((visibility("default")))
  #define LATLON_MUXER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // LATLON_MUXER__VISIBILITY_CONTROL_HPP_
