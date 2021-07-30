// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the vesc_interface class.

#ifndef VESC_INTERFACE__VESC_INTERFACE_HPP_
#define VESC_INTERFACE__VESC_INTERFACE_HPP_

#include <vesc_interface/visibility_control.hpp>

#include <common/types.hpp>
#include <vehicle_interface/platform_interface.hpp>

#include <autoware_auto_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

#include <autoware_auto_msgs/srv/autonomy_mode_change.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>

using autoware_auto_msgs::msg::RawControlCommand;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;
using autoware_auto_msgs::msg::VehicleStateReport;
using autoware_auto_msgs::msg::VehicleOdometry;

namespace autoware
{
/// \brief TODO(jjj025): Document namespaces!
namespace vesc_interface
// Inherit from vehicle_interface::platform_interface
{
/// Platform interface implementation for VESC. Bridges data to and from the VESC 
/// to convert speed and wheel angle position to motor speed and servo position.
/// \brief Class for interfacing with VESC
class VESC_INTERFACE_PUBLIC VESCInterface 
    : public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
    /// \brief Default Constructor.
    /// \todo 
    VESCInterface(
        rclcpp::Node &node
    );
    /// \brief Default Destructor
    ~VESCInterface() noexcept override = default;

    /// \brief Sends True message
    bool8_t update(std::chrono::nanoseconds timeout);

// send_state_command() - Most for gears, trasmit the gear to VESC driver
// Gear equivalent of reverse/ forward 
    bool8_t send_state_command(const VehicleControlCommand &msg);

// send_control_command() - desired speed and desired tire angle
// Convert those to motorRPM, servo positions
// RawControlCommand - Log NotSupported!!
// IF mannual mode - send zeros.
    bool8_t send_control_command(const VehicleControlCommand &msg);

// handle_mode_change_request()
// Switch between autonomous, and manual mode
// maintain interal state, and only send commands when autonomous mode is active
// IF mannual mode - send zeros. 

    /// \brief Send raw control commands, currently not implemented, hence logs error.
    bool8_t send_control_command(const RawControlCommand &msg);

protected:

// state_report() -> Set the gear (forward/backward)
    VehicleStateReport & state_report();

// odometry() -> velocity_mps meters/s
//               front_wheel_angle_rad (radians, positive-to the left)
// https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/VehicleOdometry.idl
    VehicleOdometry & get_odometry();

private:


}  // namespace vesc_interface
}  // namespace autoware

#endif  // VESC_INTERFACE__VESC_INTERFACE_HPP_
