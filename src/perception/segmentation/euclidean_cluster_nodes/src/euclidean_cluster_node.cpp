// Copyright 2019 Apex.AI, Inc.
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

#include <common/types.hpp>
#include <euclidean_cluster_nodes/euclidean_cluster_node.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <string>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster_nodes
{
////////////////////////////////////////////////////////////////////////////////
EuclideanClusterNode::EuclideanClusterNode(
  const rclcpp::NodeOptions & node_options)
: Node("euclidean_cluster_cloud_node", node_options),
  m_cloud_sub_ptr{create_subscription<PointCloud2>(
      declare_parameter("cloud_topic", "points_in"),
      rclcpp::QoS(10),
      [this](const PointCloud2::SharedPtr msg) {handle(msg);})},
  m_cluster_pub_ptr{declare_parameter("use_cluster").get<bool8_t>() ?
  create_publisher<Clusters>(
    declare_parameter("cluster_topic", "points_clustered"),
    rclcpp::QoS(10)) : nullptr},
m_box_pub_ptr{declare_parameter("use_box").get<bool8_t>() ?
  create_publisher<BoundingBoxArray>(
    declare_parameter("box_topic", "lidar_bounding_boxes"), rclcpp::QoS{10}) :
  nullptr},
m_cluster_alg{
  euclidean_cluster::Config{
    declare_parameter("cluster.frame_id").get<std::string>().c_str(),
    static_cast<std::size_t>(declare_parameter("cluster.min_cluster_size").get<std::size_t>()),
    static_cast<std::size_t>(declare_parameter("cluster.max_num_clusters").get<std::size_t>())
  },
  euclidean_cluster::HashConfig{
    static_cast<float32_t>(declare_parameter("hash.min_x").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.max_x").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.min_y").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.max_y").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.side_length").get<float32_t>()),
    static_cast<std::size_t>(declare_parameter("max_cloud_size").get<std::size_t>())
  }
},
m_clusters{},
m_boxes{},
m_voxel_ptr{nullptr},  // Because voxel config's Point types don't accept positional arguments
m_use_lfit{declare_parameter("use_lfit").get<bool8_t>()},
m_use_z{declare_parameter("use_z").get<bool8_t>()}
{
  init(m_cluster_alg.get_config());
  // Initialize voxel grid
  if (declare_parameter("downsample").get<bool8_t>()) {
    filters::voxel_grid::PointXYZ min_point;
    filters::voxel_grid::PointXYZ max_point;
    filters::voxel_grid::PointXYZ voxel_size;
    min_point.x = static_cast<float32_t>(declare_parameter("voxel.min_point.x").get<float32_t>());
    min_point.y = static_cast<float32_t>(declare_parameter("voxel.min_point.y").get<float32_t>());
    min_point.z = static_cast<float32_t>(declare_parameter("voxel.min_point.z").get<float32_t>());
    max_point.x = static_cast<float32_t>(declare_parameter("voxel.max_point.x").get<float32_t>());
    max_point.y = static_cast<float32_t>(declare_parameter("voxel.max_point.y").get<float32_t>());
    max_point.z = static_cast<float32_t>(declare_parameter("voxel.max_point.z").get<float32_t>());
    voxel_size.x = static_cast<float32_t>(declare_parameter("voxel.voxel_size.x").get<float32_t>());
    voxel_size.y = static_cast<float32_t>(declare_parameter("voxel.voxel_size.y").get<float32_t>());
    voxel_size.z = static_cast<float32_t>(declare_parameter("voxel.voxel_size.z").get<float32_t>());
    // Aggressive downsampling if not using z
    if (!m_use_z) {
      voxel_size.z = (max_point.z - min_point.z) + 1.0F;
      // Info
      RCLCPP_INFO(get_logger(), "z is not used, height aspect is fully downsampled away");
    }
    m_voxel_ptr = std::make_unique<VoxelAlgorithm>(
      filters::voxel_grid::Config{
              min_point,
              max_point,
              voxel_size,
              static_cast<std::size_t>(get_parameter("max_cloud_size").as_int())
            });
  }
}

////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::init(const euclidean_cluster::Config & cfg)
{
  // Sanity check
  if ((!m_box_pub_ptr) && (!m_cluster_pub_ptr)) {
    throw std::domain_error{"EuclideanClusterNode: No publisher topics provided"};
  }
  // Reserve
  m_clusters.clusters.reserve(cfg.max_num_clusters());
  m_boxes.header.frame_id.reserve(256U);
  m_boxes.header.frame_id = cfg.frame_id().c_str();
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::insert_plain(const PointCloud2 & cloud)
{
  using euclidean_cluster::PointXYZI;
  //lint -e{826, 9176} NOLINT I claim this is ok and tested
  const auto begin = reinterpret_cast<const PointXYZI *>(&cloud.data[0U]);
  //lint -e{826, 9176} NOLINT I claim this is ok and tested
  const auto end = reinterpret_cast<const PointXYZI *>(&cloud.data[cloud.row_step]);
  m_cluster_alg.insert(begin, end);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::insert_voxel(const PointCloud2 & cloud)
{
  m_voxel_ptr->insert(cloud);
  insert_plain(m_voxel_ptr->get());
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::insert(const PointCloud2 & cloud)
{
  if (m_voxel_ptr) {
    insert_voxel(cloud);
  } else {
    insert_plain(cloud);
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::publish_clusters(
  Clusters & clusters,
  const std_msgs::msg::Header & header)
{
  for (auto & cls : clusters.clusters) {
    cls.header.stamp = header.stamp;
    // frame id was reserved
    cls.header.frame_id = header.frame_id;
  }
  m_cluster_pub_ptr->publish(clusters);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::handle_clusters(
  Clusters & clusters,
  const std_msgs::msg::Header & header)
{
  if (m_cluster_pub_ptr) {
    publish_clusters(clusters, header);
  }
  if (m_box_pub_ptr) {
    if (m_use_lfit) {
      if (m_use_z) {
        details::compute_eigenboxes_with_z(clusters, m_boxes);
      } else {
        details::compute_eigenboxes(clusters, m_boxes);
      }
    } else {
      if (m_use_z) {
        details::compute_lfit_bounding_boxes_with_z(clusters, m_boxes);
      } else {
        details::compute_lfit_bounding_boxes(clusters, m_boxes);
      }
    }
    m_boxes.header.stamp = header.stamp;
    // Frame id was reserved
    m_boxes.header.frame_id = header.frame_id;
    m_box_pub_ptr->publish(m_boxes);
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::handle(const PointCloud2::SharedPtr msg_ptr)
{
  try {
    try {
      insert(*msg_ptr);
    } catch (const std::length_error & e) {
      // Hit limits of inserting, can still cluster, but in bad state
      RCLCPP_WARN(get_logger(), e.what());
    }
    m_cluster_alg.cluster(m_clusters);
    //lint -e{523} NOLINT empty functions to make this modular
    handle_clusters(m_clusters, msg_ptr->header);
    m_cluster_alg.cleanup(m_clusters);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), e.what());
  } catch (...) {
    RCLCPP_FATAL(get_logger(), "EuclideanClusterNode: Unexpected error occurred!");
    throw;
  }
}
}  // namespace euclidean_cluster_nodes
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode)
