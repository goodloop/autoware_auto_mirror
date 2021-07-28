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

#include "autoware_state_monitor/state_machine.hpp"

#include <cmath>
#include <deque>

namespace autoware
{
namespace state_monitor
{

double calcDistance2d(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

bool isNearGoal(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_auto_msgs::msg::RoutePoint & goal_pose,
  const double th_dist)
{
  return calcDistance2d(current_pose.position, goal_pose.position) < th_dist;
}

bool isStopped(
  const std::deque<autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr> & odometry_buffer,
  const double th_stopped_velocity_mps)
{
  for (const auto & odometry : odometry_buffer) {
    if (std::abs(static_cast<double>(odometry->velocity_mps)) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}

bool StateMachine::isVehicleInitialized() const
{
  return true;
}

bool StateMachine::hasRoute() const
{
  return state_input_.route != nullptr;
}

bool StateMachine::isRouteReceived() const
{
  return state_input_.route != executing_route_;
}

bool StateMachine::isPlanningCompleted() const
{
  return true;
}

bool StateMachine::isEngaged() const
{
  using autoware_auto_msgs::msg::VehicleStateReport;

  if (!state_input_.engage) {
    return false;
  }

  if (state_input_.engage->engage != 1) {
    return false;
  }

  if (!state_input_.vehicle_state_report) {
    return false;
  }

  if (state_input_.vehicle_state_report->mode == VehicleStateReport::MODE_MANUAL) {
    return false;
  }

  return true;
}

bool StateMachine::isOverridden() const
{
  return !isEngaged();
}

bool StateMachine::hasArrivedGoal() const
{
  const auto is_near_goal = isNearGoal(
    state_input_.current_pose->pose, *state_input_.goal_pose, state_param_.th_arrived_distance_m);
  const auto is_stopped =
    isStopped(state_input_.odometry_buffer, state_param_.th_stopped_velocity_mps);

  if (is_near_goal && is_stopped) {
    return true;
  }

  return false;
}

bool StateMachine::isFinalizing() const
{
  return state_input_.is_finalizing;
}

__attribute__ ((visibility("default")))
AutowareState StateMachine::getCurrentState() const
{
  return autoware_state_;
}

__attribute__ ((visibility("default")))
AutowareState StateMachine::updateState(const StateInput & state_input)
{
  msgs_ = {};
  state_input_ = state_input;
  autoware_state_ = judgeAutowareState();
  return autoware_state_;
}

AutowareState StateMachine::judgeAutowareState() const
{
  if (isFinalizing()) {
    return AutowareState::Finalizing;
  }

  switch (autoware_state_) {
    case AutowareState::InitializingVehicle: {
        if (isVehicleInitialized()) {
          if (!flags_.waiting_after_initializing) {
            flags_.waiting_after_initializing = true;
            times_.initializing_completed = state_input_.current_time;
            break;
          }

          // Wait after initialize completed to avoid sync error
          constexpr double wait_time_after_initializing = 1.0;
          const auto time_from_initializing =
            state_input_.current_time - times_.initializing_completed;
          if (time_from_initializing.seconds() > wait_time_after_initializing) {
            flags_.waiting_after_initializing = false;
            return AutowareState::WaitingForRoute;
          }
        }

        break;
      }

    case AutowareState::WaitingForRoute: {
        if (isRouteReceived()) {
          return AutowareState::Planning;
        }

        if (hasRoute() && isEngaged() && !hasArrivedGoal()) {
          return AutowareState::Driving;
        }

        break;
      }

    case AutowareState::Planning: {
        executing_route_ = state_input_.route;

        if (isPlanningCompleted()) {
          if (!flags_.waiting_after_planning) {
            flags_.waiting_after_planning = true;
            times_.planning_completed = state_input_.current_time;
            break;
          }

          // Wait after planning completed to avoid sync error
          constexpr double wait_time_after_planning = 3.0;
          const auto time_from_planning = state_input_.current_time - times_.planning_completed;
          if (time_from_planning.seconds() > wait_time_after_planning) {
            flags_.waiting_after_planning = false;
            return AutowareState::WaitingForEngage;
          }
        }

        break;
      }

    case AutowareState::WaitingForEngage: {
        if (isRouteReceived()) {
          return AutowareState::Planning;
        }

        if (isEngaged()) {
          return AutowareState::Driving;
        }

        if (hasArrivedGoal()) {
          times_.arrived_goal = state_input_.current_time;
          return AutowareState::ArrivedGoal;
        }

        break;
      }

    case AutowareState::Driving: {
        if (isRouteReceived()) {
          return AutowareState::Planning;
        }

        if (isOverridden()) {
          return AutowareState::WaitingForEngage;
        }

        if (hasArrivedGoal()) {
          times_.arrived_goal = state_input_.current_time;
          return AutowareState::ArrivedGoal;
        }

        break;
      }

    case AutowareState::ArrivedGoal: {
        constexpr double wait_time_after_arrived_goal = 2.0;
        const auto time_from_arrived_goal = state_input_.current_time - times_.arrived_goal;
        if (time_from_arrived_goal.seconds() > wait_time_after_arrived_goal) {
          return AutowareState::WaitingForRoute;
        }

        break;
      }

    case AutowareState::Finalizing: {
        break;
      }

    default: {
        throw std::runtime_error("invalid state");
      }
  }

  // continue previous state when break
  return autoware_state_;
}

}  // namespace state_monitor
}  // namespace autoware
