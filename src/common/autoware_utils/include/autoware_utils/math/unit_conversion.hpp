// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE_UTILS__MATH__UNIT_CONVERSION_HPP_
#define AUTOWARE_UTILS__MATH__UNIT_CONVERSION_HPP_

#include "autoware_utils/math/constants.hpp"

namespace autoware_utils
{
/// \brief Convert angle from degree to radian.
/// \param deg Angle in degree
/// \return The corresponding angle in radians.
constexpr double deg2rad(const double deg)
{
  return deg * pi / 180.0;
}
/// \brief Convert angle from radian to degree.
/// \param rad Angle in radian
/// \return The corresponding angle in radians.
constexpr double rad2deg(const double rad)
{
  return rad * 180.0 / pi;
}
/// \brief Convert speed unit from kmph to mps
/// \param kmph Speed in kmph
/// \return The corresponding speed in mps.
constexpr double kmph2mps(const double kmph)
{
  return kmph * 1000.0 / 3600.0;
}
/// \brief Convert speed unit from mps to kmph
/// \param mps Speed in mps
/// \return The corresponding speed in kmph
constexpr double mps2kmph(const double mps)
{
  return mps * 3600.0 / 1000.0;
}
}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__MATH__UNIT_CONVERSION_HPP_
