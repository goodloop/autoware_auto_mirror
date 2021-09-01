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

#include "vehicle_constants_manager/vehicle_constants_manager.hpp"
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware
{
namespace common
{
namespace vehicle_constants_manager
{

using float64_t = VehicleConstants::float64_t;
using ParamsPrimary = VehicleConstants::ParamsPrimary;
using ParamsDerived = VehicleConstants::ParamsDerived;

const std::map<ParamsPrimary, std::string>
VehicleConstants::map_names_primary{
  std::make_pair(ParamsPrimary::wheel_radius, "wheel_radius"),
  std::make_pair(ParamsPrimary::wheel_width, "wheel_width"),
  std::make_pair(ParamsPrimary::wheel_base, "wheel_base"),
  std::make_pair(ParamsPrimary::wheel_tread, "wheel_tread"),
  std::make_pair(ParamsPrimary::overhang_front, "overhang_front"),
  std::make_pair(ParamsPrimary::overhang_rear, "overhang_rear"),
  std::make_pair(ParamsPrimary::overhang_left, "overhang_left"),
  std::make_pair(ParamsPrimary::overhang_right, "overhang_right"),
  std::make_pair(ParamsPrimary::vehicle_height, "vehicle_height"),
  std::make_pair(ParamsPrimary::cg_to_rear, "cg_to_rear"),
  std::make_pair(ParamsPrimary::tire_cornering_stiffness_front, "tire_cornering_stiffness_front"),
  std::make_pair(ParamsPrimary::tire_cornering_stiffness_rear, "tire_cornering_stiffness_rear"),
  std::make_pair(ParamsPrimary::mass_vehicle, "mass_vehicle"),
  std::make_pair(ParamsPrimary::inertia_yaw_kg_m2, "inertia_yaw_kg_m2")
};

const std::map<ParamsDerived, std::string>
VehicleConstants::map_names_derived{
  std::make_pair(ParamsDerived::cg_to_front, "cg_to_front"),
  std::make_pair(ParamsDerived::offset_longitudinal_min, "offset_longitudinal_min"),
  std::make_pair(ParamsDerived::offset_longitudinal_max, "offset_longitudinal_max"),
  std::make_pair(ParamsDerived::offset_lateral_min, "offset_lateral_min"),
  std::make_pair(ParamsDerived::offset_lateral_max, "offset_lateral_max"),
  std::make_pair(ParamsDerived::offset_height_min, "offset_height_min"),
  std::make_pair(ParamsDerived::offset_height_max, "offset_height_max"),
  std::make_pair(ParamsDerived::vehicle_length, "vehicle_length"),
  std::make_pair(ParamsDerived::vehicle_width, "vehicle_width")
};


VehicleConstants::VehicleConstants(const std::map<ParamsPrimary, float64_t> & map_params_primary)
: map_params_primary(map_params_primary)
{
  // Sanity Checks

  // All params are provided
  for (ParamsPrimary param = ParamsPrimary::wheel_radius;
    param != ParamsPrimary::last;
    param = static_cast<ParamsPrimary>(static_cast<int>(param) + 1))
  {
    if (map_params_primary.find(param) != map_params_primary.end() ) {
      // found
      continue;
    }
    throw std::runtime_error(
            "Please provide following parameter: " +
            VehicleConstants::map_names_primary.at(param));
  }

  // Center of gravity must be between front and rear axles
  if (map_params_primary.at(ParamsPrimary::wheel_base) <
    map_params_primary.at(ParamsPrimary::cg_to_rear))
  {
    throw std::runtime_error("wheel_base must be larger than cg_to_rear");
  }

  // These values must be positive
  auto throw_if_negative = [&](const ParamsPrimary & param_enum) {
      float64_t number = map_params_primary.at(param_enum);
      const auto & name = map_names_primary.at(param_enum);
      if (number < 0.0) {
        throw std::runtime_error(
                name + " = " + std::to_string(number) +
                " shouldn't be negative.");
      }
    };
  throw_if_negative(ParamsPrimary::wheel_radius);
  throw_if_negative(ParamsPrimary::wheel_width);
  throw_if_negative(ParamsPrimary::wheel_base);
  throw_if_negative(ParamsPrimary::wheel_tread);
  throw_if_negative(ParamsPrimary::overhang_front);
  throw_if_negative(ParamsPrimary::overhang_rear);
  throw_if_negative(ParamsPrimary::overhang_left);
  throw_if_negative(ParamsPrimary::overhang_right);
  throw_if_negative(ParamsPrimary::vehicle_height);
  throw_if_negative(ParamsPrimary::cg_to_rear);
  throw_if_negative(ParamsPrimary::tire_cornering_stiffness_front);
  throw_if_negative(ParamsPrimary::tire_cornering_stiffness_rear);
  throw_if_negative(ParamsPrimary::mass_vehicle);
  throw_if_negative(ParamsPrimary::inertia_yaw_kg_m2);

  // Assign Derived Params
  map_params_derived = {
    std::make_pair(
      ParamsDerived::cg_to_front,
      map_params_primary.at(ParamsPrimary::wheel_base) -
      map_params_primary.at(ParamsPrimary::cg_to_rear)),

    std::make_pair(
      ParamsDerived::offset_longitudinal_min,
      -map_params_primary.at(ParamsPrimary::overhang_rear)),

    std::make_pair(
      ParamsDerived::offset_longitudinal_max,
      map_params_primary.at(ParamsPrimary::wheel_base) +
      map_params_primary.at(ParamsPrimary::overhang_front)),

    std::make_pair(
      ParamsDerived::offset_lateral_min,
      -map_params_primary.at(ParamsPrimary::wheel_tread) / 2.0 -
      map_params_primary.at(ParamsPrimary::overhang_right)),

    std::make_pair(
      ParamsDerived::offset_lateral_max,
      -map_params_primary.at(ParamsPrimary::wheel_tread) / 2.0 -
      map_params_primary.at(ParamsPrimary::overhang_left)),

    std::make_pair(
      ParamsDerived::offset_height_min,
      -map_params_primary.at(ParamsPrimary::wheel_radius)),

    std::make_pair(
      ParamsDerived::offset_height_max,
      map_params_primary.at(ParamsPrimary::vehicle_height) -
      map_params_primary.at(ParamsPrimary::wheel_radius)),

    std::make_pair(
      ParamsDerived::vehicle_length,
      map_params_primary.at(ParamsPrimary::overhang_front) +
      map_params_primary.at(ParamsPrimary::wheel_base) +
      map_params_primary.at(ParamsPrimary::overhang_rear)),

    std::make_pair(
      ParamsDerived::vehicle_width,
      map_params_primary.at(ParamsPrimary::overhang_left) +
      map_params_primary.at(ParamsPrimary::wheel_tread) +
      map_params_primary.at(ParamsPrimary::overhang_right))
  };
}

std::string VehicleConstants::str_pretty() const
{
  std::stringstream sstream;
  for (ParamsPrimary param = ParamsPrimary::wheel_radius;
    param != ParamsPrimary::last;
    param = static_cast<ParamsPrimary>(static_cast<int>(param) + 1))
  {
    sstream << map_names_primary.at(param) << ": " << map_params_primary.at(param) << "\n";
  }
  for (ParamsDerived param = ParamsDerived::cg_to_front;
    param != ParamsDerived::last;
    param = static_cast<ParamsDerived>(static_cast<int>(param) + 1))
  {
    sstream << map_names_derived.at(param) << ": " << map_params_derived.at(param) << "\n";
  }

  return sstream.str();
}

VehicleConstants get_vehicle_constants(const rclcpp::Node::SharedPtr & node_ptr)
{
  std::map<ParamsPrimary, float64_t> map_params_primary;
  for (ParamsPrimary param = ParamsPrimary::wheel_radius;
    param != ParamsPrimary::last;
    param = static_cast<ParamsPrimary>(static_cast<int>(param) + 1))
  {
    map_params_primary.insert(
      std::make_pair(
        param,
        node_ptr->declare_parameter(
          "/vehicle_constants_manager_node/" +
          VehicleConstants::map_names_primary.at(param)).get<float64_t>()));
  }
  return VehicleConstants(map_params_primary);
}
}  // namespace vehicle_constants_manager
}  // namespace common
}  // namespace autoware
