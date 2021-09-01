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
/// \brief This file defines the vehicle_constants_manager class.

#ifndef VEHICLE_CONSTANTS_MANAGER__VEHICLE_CONSTANTS_MANAGER_HPP_
#define VEHICLE_CONSTANTS_MANAGER__VEHICLE_CONSTANTS_MANAGER_HPP_

#include <vehicle_constants_manager/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <common/types.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>


namespace autoware
{
namespace common
{
namespace vehicle_constants_manager
{

struct VEHICLE_CONSTANTS_MANAGER_PUBLIC VehicleConstants
{
  using SharedPtr = std::shared_ptr<VehicleConstants>;
  using ConstSharedPtr = const SharedPtr;

  using float64_t = autoware::common::types::float64_t;

  enum class ParamsPrimary
  {
    // Don't assign values to specific enums!
    wheel_radius,
    wheel_width,
    wheel_base,
    wheel_tread,
    overhang_front,
    overhang_rear,
    overhang_left,
    overhang_right,
    vehicle_height,
    cg_to_rear,
    tire_cornering_stiffness_front,
    tire_cornering_stiffness_rear,
    mass_vehicle,
    inertia_yaw_kg_m2,
    last
  };

  enum class ParamsDerived
  {
    // Don't assign values to specific enums!
    cg_to_front,
    offset_longitudinal_min,
    offset_longitudinal_max,
    offset_lateral_min,
    offset_lateral_max,
    offset_height_min,
    offset_height_max,
    vehicle_length,
    vehicle_width,
    last
  };

  /*
   # Primary Constants

   wheel_radius: Radius of the wheel including the tires
   wheel_width:
   wheel_base: Absolute distance between axis centers of front and rear wheels.
   wheel_tread: Absolute distance between axis centers of left and right wheels.

   overhang_front: Absolute distance between the vertical plane passing through
   the centres of the front wheels and the foremost point of the vehicle

   overhang_rear: Absolute distance between the vertical plane passing through
   the centres of the rear wheels and the rearmost point of the vehicle

   overhang_left: Absolute distance between axis centers of left wheels
   and the leftmost point of the vehicle

   overhang_right: Absolute distance between axis centers of right wheels
   and the rightmost point of the vehicle

   vehicle_height: Absolute vertical distance between ground and topmost point
   of the vehicle including mounted sensors

   cg_to_rear: Absolute value of longitudinal distance between Center of Gravity
   and center of rear axle

   tire_cornering_stiffness_front_N_per_deg: The nominal cornering stiffness
   is equal to the side force in Newtons divided by the slip angle in Degrees for small angles
   https://www.mchenrysoftware.com/medit32/readme/msmac/default.htm?turl=examplestirecorneringstiffnesscalculation1.htm

   tire_cornering_stiffness_rear_N_per_deg:

   mass_vehicle: Total mass of the vehicle including sensors in kilograms
   inertia_yaw_kg_m2: Moment of inertia around vertical axis of the vehicle

   # Derived Constants
   cg_to_front: Absolute value of longitudinal distance between
   Center of Gravity and center of front axle

   // Offsets from base_link
   offset_longitudinal_min:
   offset_longitudinal_max:
   offset_lateral_min:
   offset_lateral_max:
   offset_height_min:
   offset_height_max:

   vehicle_length: Absolute horizontal distance between the fore-most and rear-most
   points of the vehicle

   vehicle_width: Absolute horizontal distance between left-most and right-most
   points of the vehicle
   */

  static const std::map<ParamsPrimary, std::string> map_names_primary;
  static const std::map<ParamsDerived, std::string> map_names_derived;

  explicit VehicleConstants(const std::map<ParamsPrimary, float64_t> & map_params_primary);

  // Primary Constants
  std::map<ParamsPrimary, float64_t> map_params_primary;
  std::map<ParamsDerived, float64_t> map_params_derived;

  std::string str_pretty() const;
};

VEHICLE_CONSTANTS_MANAGER_PUBLIC VehicleConstants try_get_vehicle_constants(
  const rclcpp::Node::SharedPtr & node_ptr,
  const std::chrono::nanoseconds & nanos_timeout);

}  // namespace vehicle_constants_manager
}  // namespace common
}  // namespace autoware

#endif  // VEHICLE_CONSTANTS_MANAGER__VEHICLE_CONSTANTS_MANAGER_HPP_
