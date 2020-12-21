// Copyright 2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief This file implements a clustering node that published colored point clouds and convex
///        hulls

#include <memory>

#include "geometry/bounding_box_2d.hpp"
#include "autoware_auto_msgs/msg/bounding_box.hpp"
#include "autoware_auto_msgs/msg/bounding_box_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "euclidean_cluster_nodes/euclidean_cluster_node.hpp"


namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster_nodes
{
namespace details
{

// reinterpret_cast is not possible here since it is undefined behavior to read or modify
// data through a pointer of another type, with a few exceptions that do not apply here.
static Points points_from_bytes(const sensor_msgs::msg::PointCloud2& pc) {
  if (pc.data.size() == 0) {
    throw std::runtime_error("Tried to compute bounding box of empty point cloud");
  }
  Points dest;
  std::size_t length = pc.data.size() / pc.point_step;
  dest.reserve(length);
  const void * const src = &pc.data[0U];
  (void)std::memcpy(&dest, src, length);
  return dest;
}

////////////////////////////////////////////////////////////////////////////////
void compute_eigenboxes(const Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (const auto & cls : clusters.clusters) {
    try {
      auto ws = points_from_bytes(cls);
      auto box = common::geometry::bounding_box::eigenbox_2d(ws.begin(), ws.end());
      boxes.boxes.push_back(box);
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void compute_eigenboxes_with_z(const Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (auto & cls : clusters.clusters) {
    try {
      auto ws = points_from_bytes(cls);
      auto box = common::geometry::bounding_box::eigenbox_2d(ws.begin(), ws.end());
      common::geometry::bounding_box::compute_height(ws.begin(), ws.end(), box);
      boxes.boxes.push_back(box);
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void compute_lfit_bounding_boxes(Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (auto & cls : clusters.clusters) {
    try {
      auto ws = points_from_bytes(cls);
      auto box = common::geometry::bounding_box::lfit_bounding_box_2d(ws.begin(), ws.end());
      boxes.boxes.push_back(box);
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void compute_lfit_bounding_boxes_with_z(Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (auto & cls : clusters.clusters) {
    try {
      auto ws = points_from_bytes(cls);
      auto box = common::geometry::bounding_box::lfit_bounding_box_2d(ws.begin(), ws.end());
      common::geometry::bounding_box::compute_height(ws.begin(), ws.end(), box);
      boxes.boxes.push_back(box);
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
}  // namespace details
}  // namespace euclidean_cluster_nodes
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
