// Copyright 2021 Robotec.ai
// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_HPP_
#define AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_HPP_

#include <string>

#include "autoware_auto_msgs/msg/autoware_state.hpp"

enum class AutowareState : uint8_t
{
  InitializingVehicle = 1,
  WaitingForRoute = 2,
  Planning = 3,
  WaitingForEngage = 4,
  Driving = 5,
  ArrivedGoal = 6,
  Finalizing = 7,
};

inline std::string toString(const AutowareState& state)
{
  if (state == AutowareState::InitializingVehicle) { return "InitializingVehicle"; }
  if (state == AutowareState::WaitingForRoute) { return "WaitingForRoute"; }
  if (state == AutowareState::Planning) { return "Planning"; }
  if (state == AutowareState::WaitingForEngage) { return "WaitingForEngage"; }
  if (state == AutowareState::Driving) { return "Driving"; }
  if (state == AutowareState::ArrivedGoal) { return "ArrivedGoal"; }
  if (state == AutowareState::Finalizing) { return "Finalizing"; }

  throw std::runtime_error("invalid state");
}

#endif  // AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_HPP_
