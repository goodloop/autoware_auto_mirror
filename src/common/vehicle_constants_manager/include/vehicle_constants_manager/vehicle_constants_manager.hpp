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

#include <memory>
#include <string>


namespace autoware
{
namespace common
{
namespace vehicle_constants_manager
{

struct VEHICLE_CONSTANTS_MANAGER_PUBLIC VehicleConstants
{
  using SharedPtr = std::shared_ptr<VehicleConstants>;
  using ConstSharedPtr = const std::shared_ptr<VehicleConstants>;

  using float64_t = autoware::common::types::float64_t;

  VehicleConstants(
    float64_t wheel_radius,
    float64_t wheel_width,
    float64_t wheel_base,
    float64_t wheel_tread,
    float64_t overhang_front,
    float64_t overhang_rear,
    float64_t overhang_left,
    float64_t overhang_right,
    float64_t vehicle_height,
    float64_t cg_to_rear,
    float64_t tire_cornering_stiffness_front_n_per_deg,
    float64_t tire_cornering_stiffness_rear_n_per_deg,
    float64_t mass_vehicle,
    float64_t inertia_yaw_kg_m_2);

  // Primary Constants

  const float64_t wheel_radius;  // Radius of the wheel including the tires
  const float64_t wheel_width;

  const float64_t wheel_base;  // Absolute distance between axis centers of front and rear wheels.
  const float64_t wheel_tread;  // Absolute distance between axis centers of left and right wheels.

  const float64_t overhang_front;  // Absolute distance between the vertical plane passing through
  // the centres of the front wheels and the foremost point of the vehicle

  const float64_t overhang_rear;  // Absolute distance between the vertical plane passing through
  // the centres of the rear wheels and the rearmost point of the vehicle

  const float64_t overhang_left;  // Absolute distance between axis centers of left wheels
  // and the leftmost point of the vehicle

  const float64_t overhang_right;  // Absolute distance between axis centers of right wheels
  // and the rightmost point of the vehicle

  const float64_t vehicle_height;  // Absolute vertical distance between ground and topmost point
  // of the vehicle including mounted sensors

  const float64_t cg_to_rear;  // Absolute value of longitudinal distance between Center of Gravity
  // and center of rear axle

  const float64_t tire_cornering_stiffness_front_N_per_deg;  // The nominal cornering stiffness
  // is equal to the side force in Newtons divided by the slip angle in Degrees for small angles
  // https://www.mchenrysoftware.com/medit32/readme/msmac/default.htm?turl=examplestirecorneringstiffnesscalculation1.htm
  const float64_t tire_cornering_stiffness_rear_N_per_deg;

  const float64_t mass_vehicle;  // Total mass of the vehicle including sensors in kilograms
  const float64_t inertia_yaw_kg_m2;  // Moment of inertia around vertical axis of the vehicle


  // Derived Constants

  const float64_t cg_to_front;  // Absolute value of longitudinal distance between
  // Center of Gravity and center of front axle

  const float64_t vehicle_length;
  const float64_t vehicle_width;

  // Offsets from base_link
  const float64_t offset_longitudinal_min;
  const float64_t offset_longitudinal_max;
  const float64_t offset_lateral_min;
  const float64_t offset_lateral_max;
  const float64_t offset_height_min;
  const float64_t offset_height_max;

  std::string str_pretty() const;
};

}  // namespace vehicle_constants_manager
}  // namespace common
}  // namespace autoware

#endif  // VEHICLE_CONSTANTS_MANAGER__VEHICLE_CONSTANTS_MANAGER_HPP_
