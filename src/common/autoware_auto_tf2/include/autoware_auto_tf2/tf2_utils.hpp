// Copyright 2020, The Autoware Foundation.
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
/// \file
/// \brief This file includes common utilities for tf2 types

#ifndef AUTOWARE_AUTO_TF2__TF2_UTILS_HPP_
#define AUTOWARE_AUTO_TF2__TF2_UTILS_HPP_

#include <common/types.hpp>
#include <motion_common/motion_common.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#include <tf2/convert.h>
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <tf2/utils.h>
#pragma GCC diagnostic pop

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using Heading = decltype(decltype(State::state)::heading);

namespace autoware
{
namespace tf2_utils
{
/// \brief Converts any object that can be converted into a tf2::Quaternion
/// to a simple hading representation
/// \tparam QuatT A type which can be converted to a tf2::Quaternion
/// \param[in] quat The quaternion-like object
/// \returns Converted heading with real and imaginary parts
template<typename QuatT>
Heading from_quat(QuatT quat)
{
  Heading ret{};
  ret = motion::motion_common::from_angle(tf2::getYaw(quat));
  return ret;
}

/// \brief Converts a simple heading representation into a tf2::Quaternion
/// \param[in] heading The heading with real and imaginary parts to be converted
/// \returns Converted tf2::Quaternion
tf2::Quaternion to_quat(Heading heading)
{
  tf2::Quaternion quat{};
  quat.setEuler(motion::motion_common::to_angle(heading), 0.0, 0.0);
  return quat;
}
}  // namespace tf2_utils
}  // namespace autoware

#endif  // AUTOWARE_AUTO_TF2__TF2_UTILS_HPP_
