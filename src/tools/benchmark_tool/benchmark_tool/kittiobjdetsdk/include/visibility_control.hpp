// Copyright 2021 Arm Limited
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

#ifndef VISIBILITY_CONTROL_HPP_
#define VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(KITTIOBJEVALMODULE_BUILDING_DLL) || \
  defined(KITTIOBJEVALMODULE_EXPORTS)
    #define KITTIOBJEVALMODULE_PUBLIC __declspec(dllexport)
    #define KITTIOBJEVALMODULE_LOCAL
  #else  // defined(KITTIOBJEVALMODULE_BUILDING_DLL) ||
         // defined(KITTIOBJEVALMODULE_EXPORTS)
    #define KITTIOBJEVALMODULE_PUBLIC __declspec(dllimport)
    #define KITTIOBJEVALMODULE_LOCAL
  #endif  // defined(KITTIOBJEVALMODULE_BUILDING_DLL) ||
          // defined(KITTIOBJEVALMODULE_EXPORTS)
#elif defined(__linux__)
  #define KITTIOBJEVALMODULE_PUBLIC __attribute__((visibility("default")))
  #define KITTIOBJEVALMODULE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define KITTIOBJEVALMODULE_PUBLIC __attribute__((visibility("default")))
  #define KITTIOBJEVALMODULE_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // VISIBILITY_CONTROL_HPP_
