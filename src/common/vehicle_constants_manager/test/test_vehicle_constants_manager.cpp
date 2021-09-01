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

#include <map>
#include <memory>
#include <stdexcept>
#include <utility>

#include "gtest/gtest.h"
#include "vehicle_constants_manager/vehicle_constants_manager.hpp"


TEST(test_vehicle_constants_manager, test_initialization_constructor) {
  using float64_t = autoware::common::types::float64_t;
  using VehicleConstants = autoware::common::vehicle_constants_manager::VehicleConstants;
  using ParamsPrimary = VehicleConstants::ParamsPrimary;

  std::map<ParamsPrimary, float64_t> map_params_primary{
    std::make_pair(ParamsPrimary::wheel_radius, 0.37),
    std::make_pair(ParamsPrimary::wheel_width, 0.27),
    std::make_pair(ParamsPrimary::wheel_base, 2.734),
    std::make_pair(ParamsPrimary::wheel_tread, 1.571),
    std::make_pair(ParamsPrimary::overhang_front, 1.033),
    std::make_pair(ParamsPrimary::overhang_rear, 1.021),
    std::make_pair(ParamsPrimary::overhang_left, 0.3135),
    std::make_pair(ParamsPrimary::overhang_right, 0.3135),
    std::make_pair(ParamsPrimary::vehicle_height, 1.662),
    std::make_pair(ParamsPrimary::cg_to_rear, 1.367),
    std::make_pair(ParamsPrimary::tire_cornering_stiffness_front, 0.1),
    std::make_pair(ParamsPrimary::tire_cornering_stiffness_rear, 0.1),
    std::make_pair(ParamsPrimary::mass_vehicle, 2120.0),
    std::make_pair(ParamsPrimary::inertia_yaw_kg_m2, 12.0)
  };
  EXPECT_NO_THROW(VehicleConstants vc(map_params_primary));

  std::map<ParamsPrimary, float64_t> map_params_primary_missing{
    std::make_pair(ParamsPrimary::wheel_radius, 0.37),
    std::make_pair(ParamsPrimary::wheel_width, 0.27),
    std::make_pair(ParamsPrimary::wheel_base, 2.734)
  };
  EXPECT_THROW(VehicleConstants vc(map_params_primary_missing), std::runtime_error);

  std::map<ParamsPrimary, float64_t> map_params_primary_bad_center_of_gravity{
    std::make_pair(ParamsPrimary::wheel_radius, 0.37),
    std::make_pair(ParamsPrimary::wheel_width, 0.27),
    std::make_pair(ParamsPrimary::wheel_base, 2.734),
    std::make_pair(ParamsPrimary::wheel_tread, 1.571),
    std::make_pair(ParamsPrimary::overhang_front, 1.033),
    std::make_pair(ParamsPrimary::overhang_rear, 1.021),
    std::make_pair(ParamsPrimary::overhang_left, 0.3135),
    std::make_pair(ParamsPrimary::overhang_right, 0.3135),
    std::make_pair(ParamsPrimary::vehicle_height, 1.662),
    std::make_pair(ParamsPrimary::cg_to_rear, 3.367),
    std::make_pair(ParamsPrimary::tire_cornering_stiffness_front, 0.1),
    std::make_pair(ParamsPrimary::tire_cornering_stiffness_rear, 0.1),
    std::make_pair(ParamsPrimary::mass_vehicle, 2120.0),
    std::make_pair(ParamsPrimary::inertia_yaw_kg_m2, 12.0)
  };
  EXPECT_THROW(VehicleConstants vc(map_params_primary_bad_center_of_gravity), std::runtime_error);

  std::map<ParamsPrimary, float64_t> map_params_primary_some_negative{
    std::make_pair(ParamsPrimary::wheel_radius, -0.37),
    std::make_pair(ParamsPrimary::wheel_width, -0.27),
    std::make_pair(ParamsPrimary::wheel_base, -2.734),
    std::make_pair(ParamsPrimary::wheel_tread, -1.571),
    std::make_pair(ParamsPrimary::overhang_front, 1.033),
    std::make_pair(ParamsPrimary::overhang_rear, 1.021),
    std::make_pair(ParamsPrimary::overhang_left, 0.3135),
    std::make_pair(ParamsPrimary::overhang_right, 0.3135),
    std::make_pair(ParamsPrimary::vehicle_height, 1.662),
    std::make_pair(ParamsPrimary::cg_to_rear, 3.367),
    std::make_pair(ParamsPrimary::tire_cornering_stiffness_front, 0.1),
    std::make_pair(ParamsPrimary::tire_cornering_stiffness_rear, 0.1),
    std::make_pair(ParamsPrimary::mass_vehicle, 2120.0),
    std::make_pair(ParamsPrimary::inertia_yaw_kg_m2, 12.0)
  };
  EXPECT_THROW(VehicleConstants vc(map_params_primary_some_negative), std::runtime_error);
}

TEST(test_vehicle_constants_manager, test_get_vehicle_constants) {
  rclcpp::init(0, nullptr);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("some_node");
  using namespace std::chrono_literals;
  EXPECT_THROW(
    auto vc = autoware::common::vehicle_constants_manager::try_get_vehicle_constants(node, 300ms),
    std::runtime_error);

  rclcpp::shutdown();
}
