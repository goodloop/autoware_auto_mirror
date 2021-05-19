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

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <common/types.hpp>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <tuple>
#include <memory>
#include <string>
#include <vector>
#include "polygon_remover/polygon_remover.hpp"


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace polygon_remover
{
using common::types::PointXYZI;
using sensor_msgs::msg::PointCloud2;
using autoware_auto_msgs::msg::Shape;

PolygonRemover::PolygonRemover(bool will_visualize)
: polygon_is_initialized_{false},
  will_visualize_{will_visualize}
{
}

PointCloud2::SharedPtr PolygonRemover::remove_shape_polygon_from_cloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_in,
  const autoware_auto_msgs::msg::Shape::ConstSharedPtr & shape_in)
{
  return remove_polyline_polygon_from_cloud(
    cloud_in,
    shape_to_polyline_polygon(shape_in));
}

PointCloud2::SharedPtr PolygonRemover::remove_polyline_polygon_from_cloud(
  const PointCloud2::ConstSharedPtr & cloud_in_ptr,
  const std::vector<Point> & polyline_polygon)
{
  PointCloud2::SharedPtr cloud_filtered_ptr = std::make_shared<PointCloud2>();

  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>;
  using CloudView = point_cloud_msg_wrapper::PointCloud2View<PointXYZI>;

  CloudModifier cloud_modifier_filtered(*cloud_filtered_ptr, "");
  cloud_filtered_ptr->header = cloud_in_ptr->header;

  CloudView cloud_view_in(*cloud_in_ptr);
  cloud_modifier_filtered.resize(static_cast<uint32_t>(cloud_view_in.size()));

  auto point_is_outside_polygon = [&polyline_polygon](const PointXYZI & point) -> bool {
      auto result = CGAL::bounded_side_2(
        polyline_polygon.begin(), polyline_polygon.end(),
        Point(point.x, point.y), K());
      return result == CGAL::ON_UNBOUNDED_SIDE;  // not INSIDE or ON the polygon
    };

  auto new_end = std::copy_if(
    cloud_view_in.cbegin(),
    cloud_view_in.cend(),
    cloud_modifier_filtered.begin(),
    [&point_is_outside_polygon](const PointXYZI & point) {
      return point_is_outside_polygon(point);
    });

  cloud_modifier_filtered.resize(
    static_cast<uint32_t>(std::distance(
      cloud_modifier_filtered.begin(),
      new_end)));
  return cloud_filtered_ptr;
}

std::vector<Point> PolygonRemover::shape_to_polyline_polygon(
  const Shape::ConstSharedPtr & shape_in)
{
  std::vector<Point> polyline_polygon;
  if (shape_in->polygon.points.size() < 3) {
    throw std::length_error("Polygon vertex count should be larger than 2.");
  }
  const auto & vertices_in = shape_in->polygon.points;
  polyline_polygon.resize(vertices_in.size());
  std::transform(
    shape_in->polygon.points.begin(),
    shape_in->polygon.points.end(),
    polyline_polygon.begin(),
    [](const geometry_msgs::msg::Point32 & p_in) {
      return Point(p_in.x, p_in.y);
    });
  return polyline_polygon;
}

void PolygonRemover::update_polygon(const Shape::ConstSharedPtr & shape_in)
{
  polyline_polygon_ = shape_to_polyline_polygon(shape_in);
  if (will_visualize_) {
    marker_.ns = "ns_polygon_remover";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.2;
    marker_.color.a = 1.0;
    marker_.color.r = 1.0;
    marker_.color.g = 1.0;
    marker_.color.b = 1.0;

    auto make_point = [](float x, float y, float z) {
        geometry_msgs::msg::Point point;
        point.x = static_cast<double>(x);
        point.y = static_cast<double>(y);
        point.z = static_cast<double>(z);
        return point;
      };

    for (size_t index_cur = 0; index_cur < polyline_polygon_.size(); ++index_cur) {
      const auto & vertex = polyline_polygon_.at(index_cur);

      // Take the last segment into consideration to connect the loop
      size_t index_next = index_cur == polyline_polygon_.size() - 1 ? 0UL : index_cur + 1;
      const auto & vertex_next = polyline_polygon_.at(index_next);

      // Build upper ring
      auto vertex_up_cur = make_point(
        static_cast<float>(vertex.x()),
        static_cast<float>(vertex.y()), 5.0F);
      auto vertex_up_next =
        make_point(
        static_cast<float>(vertex_next.x()),
        static_cast<float>(vertex_next.y()), 5.0F);
      marker_.points.emplace_back(vertex_up_cur);
      marker_.points.emplace_back(vertex_up_next);

      // Build lower ring
      auto vertex_down_cur =
        make_point(static_cast<float>(vertex.x()), static_cast<float>(vertex.y()), -5.0F);
      auto vertex_down_next =
        make_point(
        static_cast<float>(vertex_next.x()),
        static_cast<float>(vertex_next.y()), -5.0F);
      marker_.points.emplace_back(vertex_down_cur);
      marker_.points.emplace_back(vertex_down_next);

      // Connect up and down vertices
      marker_.points.emplace_back(vertex_up_cur);
      marker_.points.emplace_back(vertex_down_cur);
    }
  }
  polygon_is_initialized_ = true;
}

PointCloud2::SharedPtr PolygonRemover::remove_updated_polygon_from_cloud(
  const PointCloud2::ConstSharedPtr & cloud_in)
{
  if (will_visualize_) {
    set_marker_frame_id(cloud_in->header.frame_id);
  }
  if (!polygon_is_initialized_) {
    throw std::runtime_error(
            "Shape polygon is not initialized. Please use `update_polygon` first.");
  }
  return remove_polyline_polygon_from_cloud(cloud_in, polyline_polygon_);
}

bool PolygonRemover::polygon_is_initialized() const
{
  return polygon_is_initialized_;
}

void PolygonRemover::set_marker_frame_id(const std::string & frame_id)
{
  marker_.header.frame_id = frame_id;
}

const PolygonRemover::Marker & PolygonRemover::get_marker() const
{
  return marker_;
}

}  // namespace polygon_remover
}  // namespace filters
}  // namespace perception
}  // namespace autoware
