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

#ifndef LATLON_MUXER__NODE_HPP_
#define LATLON_MUXER__NODE_HPP_

#include <memory>
#include <string>

#include "autoware_auto_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_msgs/msg/longitudinal_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the latlon_muxer package
namespace latlon_muxer
{
class LatLonMuxer : public rclcpp::Node
{
public:
  explicit LatLonMuxer(const rclcpp::NodeOptions & node_options);

private:
  void latCtrlCmdCallback(const autoware_auto_msgs::msg::AckermannLateralCommand::SharedPtr msg);
  void lonCtrlCmdCallback(const autoware_auto_msgs::msg::LongitudinalCommand::SharedPtr msg);
  void publishCmd();
  bool checkTimeout();

  rclcpp::Publisher<autoware_auto_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_pub_;
  rclcpp::Subscription<autoware_auto_msgs::msg::AckermannLateralCommand>::SharedPtr
    lat_control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_msgs::msg::LongitudinalCommand>::SharedPtr
    lon_control_cmd_sub_;

  std::shared_ptr<autoware_auto_msgs::msg::AckermannLateralCommand> lat_cmd_;
  std::shared_ptr<autoware_auto_msgs::msg::LongitudinalCommand> lon_cmd_;
  double timeout_thr_sec_;
};  // class LatLonMuxer
}  // namespace latlon_muxer
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // LATLON_MUXER__NODE_HPP_
