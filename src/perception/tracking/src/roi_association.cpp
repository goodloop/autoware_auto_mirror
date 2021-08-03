// Copyright 2021 Apex.AI, Inc.
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <tracking/projection.hpp>
#include <tracking/roi_association.hpp>
#include <geometry/intersection.hpp>

namespace autoware
{
namespace perception
{
namespace tracking
{
common::types::float32_t IOUHeuristic::operator()(
  const Polygon & shape1, const Polygon & shape2)
{
  return common::geometry::convex_intersection_over_union_2d(shape1, shape2);
}

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
