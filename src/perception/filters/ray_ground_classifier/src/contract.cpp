// Copyright 2020 The Autoware Foundation
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

#include <algorithm>
#include <utility>

#include "ray_ground_classifier/contract.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier
{
namespace contract
{
using autoware::common::types::POINT_BLOCK_CAPACITY;
using contracts_lite::gcc_7x_to_string_fix;

namespace preconditions
{
namespace ray_ground_classifier
{

//------------------------------------------------------------------------------

contracts_lite::ReturnStatus partition(
  const Ray & ray, PointPtrBlock & ground_block,
  PointPtrBlock & nonground_block)
{
  const auto comment = CONTRACT_COMMENT("",
      "ray ground classifier: Blocks must be able to fit parition result");
  return contracts_lite::ReturnStatus(std::move(comment),
           RayGroundClassifier::can_fit_result(ray, ground_block, nonground_block));
}

//------------------------------------------------------------------------------

}  // namespace ray_ground_classifier
}  // namespace preconditions

namespace postconditions
{
namespace config
{

//------------------------------------------------------------------------------

contracts_lite::ReturnStatus constructor(const Config & c)
{
  const auto retro_nonground_local = contracts_lite::ReturnStatus(
    "ray ground classifier: retro nonground classification (" +
    gcc_7x_to_string_fix(c.m_nonground_retro_thresh) +
    ") must be greater than local slope thresholds (" + gcc_7x_to_string_fix(c.m_max_local_slope) +
    ")", (c.m_max_local_slope < c.m_nonground_retro_thresh));

  const auto retro_nonground_global = contracts_lite::ReturnStatus(
    "ray ground classifier: retro nonground classification (" +
    gcc_7x_to_string_fix(c.m_nonground_retro_thresh) +
    ") must be greater than global slope thresholds (" +
    gcc_7x_to_string_fix(c.m_max_global_slope) + ")",
    (c.m_max_global_slope < c.m_nonground_retro_thresh));

  const auto ground_thresh_consistent = contracts_lite::ReturnStatus(
    "ray ground classifier: max local last ground thresh (" +
    gcc_7x_to_string_fix(c.m_max_last_local_ground_thresh_m) +
    ") must be greater than max_global_height_m (" +
    gcc_7x_to_string_fix(c.m_max_global_height_thresh_m) + ")",
    (c.m_max_last_local_ground_thresh_m > c.m_max_global_height_thresh_m));

  return retro_nonground_local && retro_nonground_global && ground_thresh_consistent;
}

//------------------------------------------------------------------------------

}  // namespace config
}  // namespace postconditions

}  // namespace contract
}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware
