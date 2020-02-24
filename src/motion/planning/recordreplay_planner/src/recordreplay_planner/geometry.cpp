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

#include <vector>
#include <iostream>
#include <list>
#include <tuple>
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
using autoware::common::geometry::convex_hull;
using autoware::common::geometry::get_normal;
using autoware::common::geometry::dot_2d;
using autoware::common::geometry::minus_2d;
using autoware::common::geometry::times_2d;
using autoware::common::geometry::norm_2d;
using autoware::common::geometry::closest_line_point_2d;


// Compute a bounding box from a vehicle state and configuration.
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

  std::vector<Point32> vehicle_corners{};

  {     // Front left
    auto p = Point32{};
    p.x = xcog + lf * ch - wh * sh;
    p.y = ycog + lf * sh + wh * ch;
    vehicle_corners.push_back(p);
  }
  {     // Front right
    auto p = Point32{};
    p.x = xcog + lf * ch + wh * sh;
    p.y = ycog + lf * sh - wh * ch;
    vehicle_corners.push_back(p);
  }
  {     // Rear right
    auto p = Point32{};
    p.x = xcog - lr * ch + wh * sh;
    p.y = ycog - lr * sh - wh * ch;
    vehicle_corners.push_back(p);
  }
  {     // Rear right
    auto p = Point32{};
    p.x = xcog - lr * ch - wh * sh;
    p.y = ycog - lr * sh + wh * ch;
    vehicle_corners.push_back(p);
  }

  return eigenbox_2d(vehicle_corners.begin(), vehicle_corners.end());
}


std::vector<std::tuple<Point32, Point32>> get_sorted_face_list(const BoundingBox & box)
{
  // First get a sorted list of points - convex_hull does that by modifying its argument
  auto corners_list = std::list<Point32>(box.corners.begin(), box.corners.end());
  auto first_interior_point = convex_hull(corners_list);

  std::vector<std::tuple<Point32, Point32>> face_list{};
  for (auto it = corners_list.begin();
    // second check appears to be needed if all the points are in the convex hull already
    (it != first_interior_point) && (it != corners_list.end());
    it++)
  {
    // Look at one point past the current one
    auto next = std::next(it, 1);

    // Deal with the case of us being at the end of the list - we still need the face
    // between the last and the first point
    if ( (next == first_interior_point) || (next == corners_list.end()) ) {
      face_list.push_back({*it, *(corners_list.begin())});
    } else {
      // Regular case in the "list interior"
      face_list.push_back({*it, *next});
    }
  }

  return face_list;
}


// check if two bounding boxes collide. This uses SAT for doing the actual checking
// (https://en.wikipedia.org/wiki/Hyperplane_separation_theorem#Use_in_collision_detection)
bool boxes_collide(const BoundingBox & box1, const BoundingBox & box2)
{
  // Obtain sorted lists of faces of both boxes, merge them into one big list of faces
  const auto faces_1 = get_sorted_face_list(box1);
  const auto faces_2 = get_sorted_face_list(box2);

  std::remove_const<decltype(faces_1)>::type all_faces{};
  all_faces.reserve(faces_1.size() + faces_2.size());
  all_faces.insert(all_faces.end(), faces_1.begin(), faces_1.end() );
  all_faces.insert(all_faces.end(), faces_2.begin(), faces_2.end() );

  // Also look at last line
  for (auto face : all_faces) {
    // Compute normal vector to the face and define a closure to get progress along it
    const auto normal = get_normal(minus_2d(std::get<1>(face), std::get<0>(face)));
    auto get_position_along_line = [&normal](auto point)
      {
        return dot_2d(normal, minus_2d(point, Point32{}) );
      };

    // Define a function to get the minimum and maximum projected position of the corners
    // of a given bounding box along the normal line of the face
    auto get_projected_min_max = [&get_position_along_line, &normal](const BoundingBox & box)
      {
        auto corners_list = std::vector<Point32>(box.corners.begin(), box.corners.end());
        auto min_corners =
          get_position_along_line(closest_line_point_2d(normal, Point32{}, corners_list[0]));
        auto max_corners = min_corners;

        for (const auto & point : corners_list) {
          const auto point_projected = closest_line_point_2d(normal, Point32{}, point);
          const auto position_along_line = get_position_along_line(point_projected);
          min_corners = std::min(min_corners, position_along_line);
          max_corners = std::max(max_corners, position_along_line);
        }
        return std::tuple<float, float>{min_corners, max_corners};
      };

    // Perform the actual computations for the extent computation
    auto minmax_1 = get_projected_min_max(box1);
    auto minmax_2 = get_projected_min_max(box2);

    // Check for any intersections
    if (std::get<0>(minmax_1) > std::get<1>(minmax_2) ||
      std::get<0>(minmax_2) > std::get<1>(minmax_1))
    {
      // Found separating hyperplane, stop
      return false;
    }
  }

  // No separating hyperplane found, boxes collide
  return true;
}


}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion
