// Copyright 2021 Robotec.ai
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

#include "autoware_state_monitor/autoware_state.hpp"

#include <memory>

#include "gtest/gtest.h"

#include "test_utils.hpp"

using autoware::state_monitor::AutowareState;

TEST(AutowareStateTest, autoware_state_to_string)
{
  EXPECT_EQ(toString(AutowareState::InitializingVehicle), "InitializingVehicle");
  EXPECT_EQ(toString(AutowareState::WaitingForRoute), "WaitingForRoute");
  EXPECT_EQ(toString(AutowareState::Planning), "Planning");
  EXPECT_EQ(toString(AutowareState::WaitingForEngage), "WaitingForEngage");
  EXPECT_EQ(toString(AutowareState::Driving), "Driving");
  EXPECT_EQ(toString(AutowareState::ArrivedGoal), "ArrivedGoal");
  EXPECT_EQ(toString(AutowareState::Finalizing), "Finalizing");
  EXPECT_ANY_THROW(toString(static_cast<AutowareState>(10)));
}
