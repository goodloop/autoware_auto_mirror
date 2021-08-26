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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the vehicle_constants_manager_nodes_node class.

#ifndef VEHICLE_CONSTANTS_MANAGER_NODES__VEHICLE_CONSTANTS_MANAGER_NODE_HPP_
#define VEHICLE_CONSTANTS_MANAGER_NODES__VEHICLE_CONSTANTS_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "vehicle_constants_manager_nodes/visibility_control.hpp"


namespace autoware
{
namespace common
{
namespace vehicle_constants_manager_node
{

/// \class VehicleConstantsManagerNodesNode
/// \brief ROS 2 Node for hello world.
class VEHICLE_CONSTANTS_MANAGER_NODES_PUBLIC VehicleConstantsManagerNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit VehicleConstantsManagerNode(const rclcpp::NodeOptions & options);

private:
};
}  // namespace vehicle_constants_manager_node
}  // namespace common
}  // namespace autoware

#endif  // VEHICLE_CONSTANTS_MANAGER_NODES__VEHICLE_CONSTANTS_MANAGER_NODE_HPP_
