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

#include "test_utils.hpp"

using autoware::state_monitor::AutowareState;
using autoware::state_monitor::StateInput;
using autoware::state_monitor::StateMachine;
using autoware::state_monitor::StateParam;

using autoware_auto_msgs::msg::Complex32;
using autoware_auto_msgs::msg::Engage;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::RoutePoint;
using autoware_auto_msgs::msg::VehicleOdometry;
using autoware_auto_msgs::msg::VehicleStateReport;

using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Quaternion;

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

TEST_F(StateMachineTest, init_states_sequence)
{
  StateInput input;
  // time: 0s, start initialization
  input.current_time = rclcpp::Time(0, 0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::InitializingVehicle);

  // time: 0s, after initialization SM waits 1s
  EXPECT_EQ(state_machine->updateState(input), AutowareState::InitializingVehicle);

  // time: 2s, initialization state should be changed after 2s
  input.current_time = rclcpp::Time(2, 0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WaitingForRoute);
}

TEST_F(StateMachineTest, basic_full_states_sequence)
{
  // sequence: InitializingVehicle -> WaitingForRoute -> Planning
  //           -> WaitingForEngage -> Driving -> ArrivedGoal
  StateInput input;
  // time: 0s, start initialization
  input.current_time = rclcpp::Time(0, 0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::InitializingVehicle);

  // time: 2s, initialization state should be changed after 2s
  input.current_time = rclcpp::Time(2, 0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WaitingForRoute);

  // time: 2s, when new route is received then system starts planning
  input.route = std::make_shared<HADMapRoute>();
  EXPECT_EQ(state_machine->updateState(input), AutowareState::Planning);

  // time: 3s, if planning completed, the system waits 3s before state transition
  input.current_time = rclcpp::Time(3, 0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::Planning);

  // time: 7s
  input.current_time = rclcpp::Time(7, 0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WaitingForEngage);

  // time 7s, engage set to true and start driving
  input.engage = prepareEngageMsg(true);
  input.vehicle_state_report = prepareVehicleStateReportMsg(true);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::Driving);

  // time 7s, current pose equal to goal pose -> vehicle arrived goal
  input.current_pose = preparePoseStampedMsg(getPoint(1,1,0));
  input.goal_pose = prepareRoutePointMsg(getPoint(1,1,0));
  EXPECT_EQ(state_machine->updateState(input), AutowareState::ArrivedGoal);
}