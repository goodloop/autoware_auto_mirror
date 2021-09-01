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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include <vehicle_constants_manager/vehicle_constants_manager.hpp>
#include <memory>
#include <stdexcept>
#include <vector>
#include "vehicle_constants_manager_nodes/vehicle_constants_manager_node.hpp"

using float64_t = autoware::common::types::float64_t;

TEST(test_vehicle_constants_manager_nodes, test_call_from_another_node) {
  rclcpp::init(0, nullptr);

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("wheel_radius", 0.37);
  params.emplace_back("wheel_width", 0.27);
  params.emplace_back("wheel_base", 2.734);
  params.emplace_back("wheel_tread", 1.571);
  params.emplace_back("overhang_front", 1.033);
  params.emplace_back("overhang_rear", 1.021);
  params.emplace_back("overhang_left", 0.3135);
  params.emplace_back("overhang_right", 0.3135);
  params.emplace_back("vehicle_height", 1.662);
  params.emplace_back("cg_to_rear", 1.367);
  params.emplace_back("tire_cornering_stiffness_front", 0.1);
  params.emplace_back("tire_cornering_stiffness_rear", 0.1);
  params.emplace_back("mass_vehicle", 2120.0);
  params.emplace_back("inertia_yaw_kg_m2", 12.0);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  auto node_vehicle_constants_manager =
    std::make_shared<autoware::common::vehicle_constants_manager_node::VehicleConstantsManagerNode>(
    node_options);

  // Create it by passing it a sub node (emulating an outer node)
  auto vc = autoware::common::vehicle_constants_manager::get_vehicle_constants(
    node_vehicle_constants_manager->create_sub_node("test"));

  using VehicleConstants = autoware::common::vehicle_constants_manager::VehicleConstants;
  using ParamsPrimary = VehicleConstants::ParamsPrimary;
  using ParamsDerived = VehicleConstants::ParamsDerived;
  // Get these parameters from the VehicleConstants object
  float64_t vehicle_width = vc.map_params_derived.at(ParamsDerived::vehicle_width);
  float64_t overhang_left = vc.map_params_primary.at(ParamsPrimary::overhang_left);
  float64_t wheel_tread = vc.map_params_primary.at(ParamsPrimary::wheel_tread);
  float64_t overhang_right = vc.map_params_primary.at(ParamsPrimary::overhang_right);
  // Perform a known check
  EXPECT_DOUBLE_EQ(vehicle_width, overhang_left + wheel_tread + overhang_right);
  rclcpp::shutdown();
}

TEST(test_vehicle_constants_manager_nodes, test_expect_exception) {
  rclcpp::init(0, nullptr);

  // Initialize it with not enough parameters
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("wheel_radius", 0.37);
  params.emplace_back("wheel_width", 0.27);
  params.emplace_back("wheel_base", 2.734);
  params.emplace_back("wheel_tread", 1.571);
  params.emplace_back("overhang_front", 1.033);
  params.emplace_back("overhang_rear", 1.021);
  params.emplace_back("overhang_left", 0.3135);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  using VehicleConstantsManagerNode =
    autoware::common::vehicle_constants_manager_node::VehicleConstantsManagerNode;

  VehicleConstantsManagerNode::SharedPtr node_vehicle_constants_manager;
  EXPECT_THROW(
    node_vehicle_constants_manager = std::make_shared<VehicleConstantsManagerNode>(node_options),
    std::runtime_error);
  rclcpp::shutdown();
}
