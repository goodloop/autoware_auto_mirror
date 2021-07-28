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

#include "autoware_state_monitor/state_machine.hpp"

#include <memory>

#include "gtest/gtest.h"

using autoware::state_monitor::StateMachine;
using autoware::state_monitor::StateParam;
using autoware::state_monitor::StateInput;
using autoware::state_monitor::AutowareState;

class StateMachineTest : public ::testing::Test
{
public:
  StateMachineTest()
  {
    StateParam params;
    params.th_arrived_distance_m = 1.0;
    params.th_stopped_time_sec = 2.0;
    params.th_stopped_velocity_mps = 0.1;

    state_machine.reset(new StateMachine(params));
  }

protected:
  std::shared_ptr<StateMachine> state_machine;
};

TEST_F(StateMachineTest, create_destroy)
{
  state_machine.reset();
}

TEST_F(StateMachineTest, basic_states_sequence)
{
  StateInput input;
  EXPECT_EQ(state_machine->getCurrentState(), AutowareState::InitializingVehicle);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WaitingForRoute);
}
