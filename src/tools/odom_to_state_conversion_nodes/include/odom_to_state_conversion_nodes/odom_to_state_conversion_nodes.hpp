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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the odom_to_state_conversion_nodes_node class.

#ifndef ODOM_TO_STATE_CONVERSION_NODES__ODOM_TO_STATE_CONVERSION_NODES_HPP_
#define ODOM_TO_STATE_CONVERSION_NODES__ODOM_TO_STATE_CONVERSION_NODES_HPP_

#include <memory>
#include <string>

#include "odom_to_state_conversion_nodes/visibility_control.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp"
#include "common/types.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

using autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
using autoware_auto_vehicle_msgs::msg::VehicleOdometry;
using nav_msgs::msg::Odometry;
using autoware::common::types::float32_t;

namespace odom_to_state_conversion_nodes
{
/// \class OdomToStateConversionNode
class ODOM_TO_STATE_CONVERSION_NODES_PUBLIC OdomToStateConversionNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit OdomToStateConversionNode(const rclcpp::NodeOptions & options);

protected:
  std::shared_ptr<VehicleKinematicState> m_state{};

  rclcpp::Subscription<Odometry>::SharedPtr m_odom_sub{};
  rclcpp::Subscription<VehicleOdometry>::SharedPtr m_vehicle_odom_sub{};
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr m_state_pub{};

  ODOM_TO_STATE_CONVERSION_NODES_LOCAL void on_odom(const Odometry::SharedPtr msg);
  ODOM_TO_STATE_CONVERSION_NODES_LOCAL void on_vehicle_odom(const VehicleOdometry::SharedPtr msg);

  bool m_use_vehicle_odom_for_velocity = true;
};
}  // namespace odom_to_state_conversion_nodes

#endif  // ODOM_TO_STATE_CONVERSION_NODES__ODOM_TO_STATE_CONVERSION_NODES_HPP_
