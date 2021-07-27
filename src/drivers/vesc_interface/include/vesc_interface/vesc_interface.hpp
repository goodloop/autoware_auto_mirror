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

#include <cstdint>

namespace autoware
{
/// \brief TODO(jjj025): Document namespaces!
namespace vesc_interface
// Inherit from vehicle_interface::platform_interface
{
// update() - nothing, send True

// send_state_command() - Most for gears, trasmit the gear to VESC driver
// Gear equivalent of reverse/ forward 

// send_control_command() - desired speed and desired tire angle
// Convert those to motorRPM, servo positions
// RawControlCommand - Log NotSupported!!
// IF mannual mode - send zeros.
// 

// handle_mode_change_request()
// Switch between autonomous, and manual mode
// maintain interal state, and only send commands when autonomous mode is active
// IF mannual mode - send zeros. 

// state_report() -> Set the gear (forward/backward)

// odometry() -> velocity_mps meters/s
//               front_wheel_angle_rad (radians, positive-to the left)
// https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/VehicleOdometry.idl

/// \brief TODO(jjj025): Document your functions
int32_t VESC_INTERFACE_PUBLIC print_hello();



}  // namespace vesc_interface
}  // namespace autoware

#endif  // VESC_INTERFACE__VESC_INTERFACE_HPP_
