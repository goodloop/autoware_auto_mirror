// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef EMERGENCY_HANDLER__HEARTBEAT_CHECKER_HPP_
#define EMERGENCY_HANDLER__HEARTBEAT_CHECKER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "emergency_handler/visibility_control.hpp"

namespace autoware
{
namespace emergency_handler
{

template<class HeartbeatMsg>
class EMERGENCY_HANDLER_PUBLIC HeaderlessHeartbeatChecker
{
public:
  HeaderlessHeartbeatChecker(
    rclcpp::Node & node, const std::string & topic_name, const double timeout)
  : clock_(node.get_clock()),
    timeout_(timeout)
  {
    using std::placeholders::_1;
    sub_heartbeat_ = node.create_subscription<HeartbeatMsg>(
      topic_name, rclcpp::QoS{1},
      std::bind(&HeaderlessHeartbeatChecker::onHeartbeat, this, _1));
  }

  bool isTimeout()
  {
    const auto time_from_last_heartbeat = clock_->now() - last_heartbeat_time_;
    return time_from_last_heartbeat.seconds() > timeout_;
  }

private:
  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Parameter
  double timeout_;

  // Subscriber
  typename rclcpp::Subscription<HeartbeatMsg>::SharedPtr sub_heartbeat_;
  rclcpp::Time last_heartbeat_time_ = rclcpp::Time(0);

  void onHeartbeat(const typename HeartbeatMsg::ConstSharedPtr msg)
  {
    (void)msg;
    last_heartbeat_time_ = clock_->now();
  }
};

}  // namespace emergency_handler
}  // namespace autoware

#endif  // EMERGENCY_HANDLER__HEARTBEAT_CHECKER_HPP_
