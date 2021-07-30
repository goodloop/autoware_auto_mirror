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

#ifndef AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_
#define AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_

#include <memory>
#include <string>

// ROS
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"

// Autoware
#include "autoware_auto_msgs/msg/autoware_state.hpp"
#include "autoware_auto_msgs/msg/engage.hpp"
#include "autoware_auto_msgs/msg/had_map_route.hpp"
#include "autoware_auto_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_msgs/msg/vehicle_state_report.hpp"

// Local
#include "autoware_state_monitor/state_machine.hpp"
#include "autoware_state_monitor/autoware_state.hpp"

namespace autoware
{
namespace state_monitor
{

/// \brief A node for monitoring the state of Autoware system
class AutowareStateMonitorNode : public rclcpp::Node
{
public:
  AutowareStateMonitorNode();

private:
  using VehicleStateReport = autoware_auto_msgs::msg::VehicleStateReport;
  using VehicleOdometry = autoware_auto_msgs::msg::VehicleOdometry;
  using HADMapRoute = autoware_auto_msgs::msg::HADMapRoute;
  using Engage = autoware_auto_msgs::msg::Engage;

  // Parameters
  double update_rate_;
  /// Local (child) frame used during the vehicle pose estimation
  std::string local_frame_;
  /// Global (parent) frame used during the vehicle pose estimation
  std::string global_frame_;

  double th_stopped_time_sec;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // CallbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // Subscribers
  rclcpp::Subscription<Engage>::SharedPtr sub_engage_;
  rclcpp::Subscription<VehicleStateReport>::SharedPtr sub_vehicle_state_report_;
  rclcpp::Subscription<HADMapRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr sub_odometry_;

  void onAutowareEngage(const Engage::ConstSharedPtr msg);
  void onVehicleStateReport(const VehicleStateReport::ConstSharedPtr msg);
  void onRoute(const HADMapRoute::ConstSharedPtr msg);
  void onVehicleOdometry(const VehicleOdometry::ConstSharedPtr msg);

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_shutdown_;

  bool onShutdownService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Publisher
  rclcpp::Publisher<autoware_auto_msgs::msg::AutowareState>::SharedPtr pub_autoware_state_;

  geometry_msgs::msg::PoseStamped::SharedPtr getCurrentPose(
    const tf2_ros::Buffer & tf_buffer);
  bool isEngaged();
  AutowareState updateState();
  void publishAutowareState(const AutowareState & state);

  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr timer_;

  // State Machine
  std::shared_ptr<StateMachine> state_machine_;
  StateInput state_input_;
  StateParam state_param_;
};

}  // namespace state_monitor
}  // namespace autoware

#endif  // AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_
