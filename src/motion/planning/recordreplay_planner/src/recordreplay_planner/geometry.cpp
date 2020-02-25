// Copyright 2020 Embotech AG, Zurich, Switzerland
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

#include "recordreplay_planner/geometry.hpp"

#include <motion_common/motion_common.hpp>
#include <geometry/bounding_box/eigenbox_2d.hpp>
#include <geometry/convex_hull.hpp>
#include <geometry/common_2d.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <limits>
#include <vector>
#include <array>
#include <iostream>
#include <list>
#include <utility>
#include <type_traits>
#include <algorithm>

namespace motion
{
namespace planning
{
namespace recordreplay_planner
{
using motion::motion_common::to_angle;
using geometry_msgs::msg::Point32;
using autoware::common::geometry::bounding_box::eigenbox_2d;

BoundingBox compute_boundingbox_from_trajectorypoint(
  const TrajectoryPoint & state,
  const VehicleConfig & vehicle_param)
{
  // Shorthands to keep the formulas sane
  const auto h = to_angle(state.heading);
  const auto xcog = state.x, ycog = state.y, z = 0.0f;
  const auto lf = vehicle_param.length_cg_front_axel() + vehicle_param.front_overhang();
  const auto lr = vehicle_param.length_cg_rear_axel() + vehicle_param.rear_overhang();
  const auto wh = vehicle_param.width() * 0.5;
  const auto ch = cos(h), sh = sin(h);

  std::array<Point32, 4> vehicle_corners;

  {     // Front left
    auto p = Point32{};
    p.x = xcog + (lf * ch) - (wh * sh);
    p.y = ycog + (lf * sh) + (wh * ch);
    vehicle_corners[0] = p;
  }
  {     // Front right
    auto p = Point32{};
    p.x = xcog + (lf * ch) + (wh * sh);
    p.y = ycog + (lf * sh) - (wh * ch);
    vehicle_corners[1] = p;
  }
  {     // Rear right
    auto p = Point32{};
    p.x = xcog - (lr * ch) + (wh * sh);
    p.y = ycog - (lr * sh) - (wh * ch);
    vehicle_corners[2] = p;
  }
  {     // Rear right
    auto p = Point32{};
    p.x = xcog - (lr * ch) - (wh * sh);
    p.y = ycog - (lr * sh) + (wh * ch);
    vehicle_corners[3] = p;
  }

  return eigenbox_2d(vehicle_corners.begin(), vehicle_corners.end());
}


}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion
