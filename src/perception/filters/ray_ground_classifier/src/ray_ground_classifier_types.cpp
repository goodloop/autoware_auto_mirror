// Copyright 2017-2020 the Autoware Foundation, Arm Limited
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "common/types.hpp"
#include "ray_ground_classifier/contract.hpp"
#include "ray_ground_classifier/ray_ground_point_classifier.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier
{

using autoware::common::types::FEPS;
using autoware::common::types::PointXYZIF;
using autoware::common::types::float32_t;

Config::Config(
  const Realf sensor_height_m,
  const AcuteDegreef max_local_slope_deg,
  const AcuteDegreef max_global_slope_deg,
  const AcuteDegreef nonground_retro_thresh_deg,
  const NonnegativeRealf min_height_thresh_m,
  const StrictlyPositiveRealf max_global_height_thresh_m,
  const Realf max_last_local_ground_thresh_m,
  const Realf max_provisional_ground_distance_m)
: m_ground_z_m(-sensor_height_m),
  m_max_local_slope(tanf(AcuteRadianf {max_local_slope_deg})),
  m_max_global_slope(tanf(AcuteRadianf {max_global_slope_deg})),
  m_nonground_retro_thresh(tanf(AcuteRadianf {nonground_retro_thresh_deg})),
  m_min_height_thresh_m(min_height_thresh_m),
  m_max_global_height_thresh_m(max_global_height_thresh_m),
  m_max_last_local_ground_thresh_m(max_last_local_ground_thresh_m),
  m_max_provisional_ground_distance_m(max_provisional_ground_distance_m)
{
  DEFAULT_ENFORCE(contract::postconditions::config::constructor(*this));
}
////////////////////////////////////////////////////////////////////////////////
Realf Config::get_sensor_height() const
{
  return -m_ground_z_m;
}
////////////////////////////////////////////////////////////////////////////////
PointXYZIFR::PointXYZIFR(const PointXYZIF * pt)
: m_point(pt),
  m_r_xy(sqrtf((pt->x * pt->x) + (pt->y * pt->y)))
{
}
////////////////////////////////////////////////////////////////////////////////
StrictlyPositiveRealf PointXYZIFR::get_r() const
{
  return m_r_xy;
}
////////////////////////////////////////////////////////////////////////////////
Realf PointXYZIFR::get_z() const
{
  return m_point->z;
}
////////////////////////////////////////////////////////////////////////////////
const PointXYZIF * PointXYZIFR::get_point_pointer() const
{
  return m_point;
}

}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware
