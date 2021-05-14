// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <memory>

#include "latlon_muxer/node.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace latlon_muxer
{

LatLonMuxer::LatLonMuxer(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("latlon_muxer", node_options)
{
  control_cmd_pub_ =
    create_publisher<autoware_auto_msgs::msg::AckermannControlCommand>(
    "output/control_cmd",
    rclcpp::QoS{1}.transient_local());
  lat_control_cmd_sub_ =
    create_subscription<autoware_auto_msgs::msg::AckermannLateralCommand>(
    "input/lateral/control_cmd", rclcpp::QoS{1},
    std::bind(&LatLonMuxer::latCtrlCmdCallback, this, std::placeholders::_1));
  lon_control_cmd_sub_ =
    create_subscription<autoware_auto_msgs::msg::LongitudinalCommand>(
    "input/longitudinal/control_cmd", rclcpp::QoS{1},
    std::bind(&LatLonMuxer::lonCtrlCmdCallback, this, std::placeholders::_1));
  timeout_thr_sec_ = declare_parameter("timeout_thr_sec", 0.5);
}

bool LatLonMuxer::checkTimeout()
{
  const auto now = this->now();
  if ((now - lat_cmd_->stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "lat_cmd_ timeout failed.");
    return false;
  }
  if ((now - lon_cmd_->stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "lon_cmd_ timeout failed.");
    return false;
  }
  return true;
}

void LatLonMuxer::publishCmd()
{
  if (!lat_cmd_ || !lon_cmd_) {
    return;
  }
  if (!checkTimeout()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000 /*ms*/,
      "timeout failed. stop publish command.");
    return;
  }

  autoware_auto_msgs::msg::AckermannControlCommand out;
  out.stamp = rclcpp::Node::now();
  out.lateral = *lat_cmd_;
  out.longitudinal = *lon_cmd_;

  control_cmd_pub_->publish(out);
}

void LatLonMuxer::latCtrlCmdCallback(
  const autoware_auto_msgs::msg::AckermannLateralCommand::SharedPtr input_msg)
{
  lat_cmd_ = std::make_shared<autoware_auto_msgs::msg::AckermannLateralCommand>(*input_msg);
  publishCmd();
}

void LatLonMuxer::lonCtrlCmdCallback(
  const autoware_auto_msgs::msg::LongitudinalCommand::SharedPtr input_msg)
{
  lon_cmd_ = std::make_shared<autoware_auto_msgs::msg::LongitudinalCommand>(*input_msg);
  publishCmd();
}
}  // namespace latlon_muxer
}  // namespace control
}  // namespace motion
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::latlon_muxer::LatLonMuxer)
