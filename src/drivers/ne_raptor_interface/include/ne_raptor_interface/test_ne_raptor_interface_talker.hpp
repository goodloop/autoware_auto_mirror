// Copyright 2020 The Autoware Foundation
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

#ifndef NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_TALKER_HPP_
#define NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_TALKER_HPP_

#include <gtest/gtest.h>

#include <ne_raptor_interface/ne_raptor_interface.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>

namespace autoware
{
namespace ne_raptor_interface
{

class NERaptorInterfaceTalker
{
public:
  /// \brief Default constructor.
  /// \param[in] node Reference to node
  explicit NERaptorInterfaceTalker(
    rclcpp::Node & node
  );

  /// \brief Default destructor
  ~NERaptorInterfaceTalker() noexcept = default;

  /// \brief Try to receive data from the vehicle platform, and update StateReport and Odometry.
  ///   Exceptions may be thrown on errors
  /// \param[in] timeout The maximum amount of time to check/receive data
  /// \return True if data was received before the timeout, false otherwise
  bool8_t update(std::chrono::nanoseconds timeout);

  /// \brief Put in data to publish
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_report(const BrakeReport & msg);
  bool8_t send_report(const GearReport & msg);
  bool8_t send_report(const MiscReport & msg);
  bool8_t send_report(const OtherActuatorsReport & msg);
  bool8_t send_report(const SteeringReport & msg);
  bool8_t send_report(const WheelSpeedReport & msg);
  bool8_t send_report(const std_msgs::msg::Bool & msg);

private:
  // Publishers (from Raptor DBW)
  rclcpp::Publisher<BrakeReport>::SharedPtr t_brake_rpt_pub;
  rclcpp::Publisher<GearReport>::SharedPtr t_gear_rpt_pub;
  rclcpp::Publisher<MiscReport>::SharedPtr t_misc_rpt_pub;
  rclcpp::Publisher<OtherActuatorsReport>::SharedPtr t_other_acts_rpt_pub;
  rclcpp::Publisher<SteeringReport>::SharedPtr t_steering_rpt_pub;
  rclcpp::Publisher<WheelSpeedReport>::SharedPtr t_wheel_spd_rpt_pub;
  // Publishers (from Autoware.Auto)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr t_dbw_state_pub;
};  // class NERaptorInterfaceTalker

}  // namespace ne_raptor_interface
}  // namespace autoware

#endif  // NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_TALKER_HPP_
