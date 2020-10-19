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

#ifndef RAY_GROUND_CLASSIFIER__CONTRACT_HPP_
#define RAY_GROUND_CLASSIFIER__CONTRACT_HPP_

#include <utility>

#include "autoware_auto_contracts/violation_handler.hpp"
#include "ray_ground_classifier/ray_ground_classifier.hpp"
#include "ray_ground_classifier/ray_ground_point_classifier.hpp"

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

namespace preconditions
{
namespace ray_ground_classifier
{

/// @brief Enforce that partition can fit partition result.
contracts_lite::ReturnStatus partition(
  const Ray & ray, PointPtrBlock & ground_block,
  PointPtrBlock & nonground_block);

}  // namespace ray_ground_classifier
}  // namespace preconditions

namespace postconditions
{
namespace config
{

/// @brief Ensure relations between config parameters
contracts_lite::ReturnStatus constructor(const Config & c);

}  // namespace config
}  // namespace postconditions

}  // namespace contract
}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // RAY_GROUND_CLASSIFIER__CONTRACT_HPP_
