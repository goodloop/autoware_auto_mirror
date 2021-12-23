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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef GEOMETRY__INTERSECTION_HPP_
#define GEOMETRY__INTERSECTION_HPP_

class point_xy;

#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_perception_msgs/msg/bounding_box.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry/convex_hull.hpp>
#include <geometry/common_2d.hpp>
#include <limits>
#include <vector>
#include <iostream>
#include <list>
#include <utility>
#include <type_traits>
#include <algorithm>



namespace autoware
{
namespace common
{
namespace geometry
{
namespace bg = boost::geometry;
using point_type = bg::model::point<double, 2, bg::cs::cartesian>;
using polygon_type = bg::model::ring<point_type>;

std::deque<polygon_type> convex_polygon_intersection2d(polygon_type poly1, polygon_type poly2){

    std::deque<polygon_type> output;
    bg::correct(poly1);
    bg::correct(poly2);

    bg::intersection(poly1, poly2, output);
    return output;
}

}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__INTERSECTION_HPP_
