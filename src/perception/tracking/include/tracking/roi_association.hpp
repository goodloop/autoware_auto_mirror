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
#ifndef TRACKING__ROI_ASSOCIATION_HPP_
#define TRACKING__ROI_ASSOCIATION_HPP_

#include <tracking/visibility_control.hpp>
#include <tracking/projection.hpp>

//#include <autoware_auto_msgs/msg/detected_objects.hpp>
//#include <hungarian_assigner/hungarian_assigner.hpp>
//#include <tracking/tracker_types.hpp>
//#include <tracking/tracked_object.hpp>
//
//#include <experimental/optional>
//#include <limits>
//#include <map>
//#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

/// \brief Simple heuristic functor that returns the IoU between two shapes.
class IOUHeuristic
{
  common::types::float32_t operator()(const Polygon & shape1, const Polygon & shape2);
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__ROI_ASSOCIATION_HPP_
