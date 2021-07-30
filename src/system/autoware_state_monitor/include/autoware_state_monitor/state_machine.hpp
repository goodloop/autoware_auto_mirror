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

#ifndef AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_
#define AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_

#include <deque>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"

#include "autoware_auto_msgs/msg/autoware_state.hpp"
#include "autoware_auto_msgs/msg/engage.hpp"
#include "autoware_auto_msgs/msg/had_map_route.hpp"
#include "autoware_auto_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_msgs/msg/vehicle_state_report.hpp"

#include "autoware_state_monitor/autoware_state.hpp"

namespace autoware
{
namespace state_monitor
{

/// \brief Input state of the state machine.
struct StateInput
{
  rclcpp::Time current_time;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  autoware_auto_msgs::msg::RoutePoint::ConstSharedPtr goal_pose;

  autoware_auto_msgs::msg::Engage::ConstSharedPtr engage;
  autoware_auto_msgs::msg::VehicleStateReport::ConstSharedPtr vehicle_state_report;
  autoware_auto_msgs::msg::HADMapRoute::ConstSharedPtr route;

  using VehicleOdometry = autoware_auto_msgs::msg::VehicleOdometry;
  using OdometryBuffer = std::deque<VehicleOdometry::ConstSharedPtr>;
  OdometryBuffer odometry_buffer;

  bool is_finalizing = false;
};

/// \brief Parameters used by the state machine
struct StateParam
{
  /// Distance threshold between a current position and a goal position.
  double th_arrived_distance_m;
  /// Length of the odometry buffer used in checking if vehicle is stopped
  double th_stopped_time_sec;
  /// Velocity threshold for determining if vehicle is stopped.
  double th_stopped_velocity_mps;
  /// Delay after initialization and before transition to a next state
  double wait_time_after_initializing = 1.0;
  /// Delay after planning and before transition to a next state
  double wait_time_after_planning = 3.0;
  /// Delay after arrived goal and before transition to a next state
  double wait_time_after_arrived_goal = 2.0;
};

/// \brief State machine for determining a state of the Autoware system.
class StateMachine
{
public:
  /// \brief Construct the state machine.
  /// \param The set of parameters used by state machine.
  explicit StateMachine(const StateParam & state_param)
  : state_param_(state_param) {}

  /// \brief Get the current state of the state machine.
  /// \return The current state.
  AutowareState getCurrentState() const;

  /// \brief Update the state machine.
  /// \param Input values used during the determination of the new state.
  /// \return A state after the state machine update.
  AutowareState updateState(const StateInput & state_input);

private:
  struct Times
  {
    rclcpp::Time arrived_goal;
    rclcpp::Time initializing_completed;
    rclcpp::Time planning_completed;
  };

  struct Flags
  {
    bool waiting_after_initializing = false;
    bool waiting_after_planning = false;
  };

  AutowareState autoware_state_ = AutowareState::InitializingVehicle;
  StateInput state_input_;
  const StateParam state_param_;

  mutable Times times_;
  mutable Flags flags_;
  mutable autoware_auto_msgs::msg::HADMapRoute::ConstSharedPtr executing_route_ = nullptr;

  AutowareState judgeAutowareState() const;

  bool isVehicleInitialized() const;
  bool isRouteReceived() const;
  bool isPlanningCompleted() const;
  bool isAutonomousMode() const;
  bool isEngaged() const;
  bool isOverridden() const;
  bool hasArrivedGoal() const;
  bool isFinalizing() const;
};

}  // namespace state_monitor
}  // namespace autoware

#endif  // AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_
