// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the polygon_remover class.

#ifndef POLYGON_REMOVER__POLYGON_REMOVER_HPP_
#define POLYGON_REMOVER__POLYGON_REMOVER_HPP_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <polygon_remover/visibility_control.hpp>
#include <common/types.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.h>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>
#include <string>
#include <vector>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace polygon_remover
{

class POLYGON_REMOVER_PUBLIC PolygonRemover
{
public:
  using SharedPtr = std::shared_ptr<PolygonRemover>;
  using ConstSharedPtr = const std::shared_ptr<PolygonRemover>;

  using PointXYZI = common::types::PointXYZI;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Shape = autoware_auto_msgs::msg::Shape;
  using Marker = visualization_msgs::msg::Marker;
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2 Point;

  explicit PolygonRemover(bool will_visualize);

  static PointCloud2::SharedPtr  remove_shape_polygon_from_cloud(
    const PointCloud2::ConstSharedPtr & cloud_in,
    const Shape::ConstSharedPtr & shape_in);

  static PointCloud2::SharedPtr  remove_polyline_polygon_from_cloud(
    const PointCloud2::ConstSharedPtr & cloud_in_ptr,
    const std::vector<Point> & polyline_polygon);

  static std::vector<Point> shape_to_polyline_polygon(
    const Shape::ConstSharedPtr & shape_in);

  void update_polygon(const Shape::ConstSharedPtr & shape_in);

  PointCloud2::SharedPtr remove_updated_polygon_from_cloud(
    const PointCloud2::ConstSharedPtr & cloud_in);

  bool polygon_is_initialized() const;

  void set_marker_frame_id(const std::string & frame_id);

  const Marker & get_marker() const;

private:
  bool polygon_is_initialized_;
  bool will_visualize_;
  std::vector<Point> polyline_polygon_;
  Marker marker_;
};


}  // namespace polygon_remover
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // POLYGON_REMOVER__POLYGON_REMOVER_HPP_
