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

#ifndef NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_LISTENER_HPP_
#define NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_LISTENER_HPP_

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

class NERaptorInterfaceListener
{
public:
  /// \brief Default constructor.
  /// \param[in] node Reference to node
  explicit NERaptorInterfaceListener(
    rclcpp::Node & node
  );

  /// \brief Default destructor
  ~NERaptorInterfaceListener() noexcept = default;

  /// \brief Try to receive data from the vehicle platform, and update StateReport and Odometry.
  ///   Exceptions may be thrown on errors
  /// \param[in] timeout The maximum amount of time to check/receive data
  /// \return True if data was received before the timeout, false otherwise
  bool8_t update(std::chrono::nanoseconds timeout);

  // Save received values
  AcceleratorPedalCmd l_accel_cmd;
  BrakeCmd l_brake_cmd;
  GearCmd l_gear_cmd;
  GlobalEnableCmd l_enable_cmd;
  MiscCmd l_misc_cmd;
  SteeringCmd l_steer_cmd;
  VehicleStateReport l_vehicle_state;
  VehicleOdometry l_vehicle_odo;
  VehicleKinematicState l_vehicle_kin_state;

private:
  // Subscribers (from Raptor DBW)
  rclcpp::SubscriptionBase::SharedPtr
    l_accel_cmd_sub,
    l_brake_cmd_sub,
    l_gear_cmd_sub,
    l_enable_cmd_sub,
    l_misc_cmd_sub,
    l_steer_cmd_sub;
  // Subscribers (from Autoware.Auto)
  rclcpp::SubscriptionBase::SharedPtr
    l_vehicle_state_sub,
    l_vehicle_odo_sub,
    l_vehicle_kin_state_sub;

  // Listener functions
  void on_accel_cmd(const AcceleratorPedalCmd::SharedPtr & msg);
  void on_brake_cmd(const BrakeCmd::SharedPtr & msg);
  void on_gear_cmd(const GearCmd::SharedPtr & msg);
  void on_enable_cmd(const GlobalEnableCmd::SharedPtr & msg);
  void on_misc_cmd(const MiscCmd::SharedPtr & msg);
  void on_steer_cmd(const SteeringCmd::SharedPtr & msg);
  void on_vehicle_state(const VehicleStateReport::SharedPtr & msg);
  void on_vehicle_odo(const VehicleOdometry::SharedPtr & msg);
  void on_vehicle_kin_state(const VehicleKinematicState::SharedPtr & msg);
};  // class NERaptorInterfaceListener

}  // namespace ne_raptor_interface
}  // namespace autoware

#endif  // NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_LISTENER_HPP_
