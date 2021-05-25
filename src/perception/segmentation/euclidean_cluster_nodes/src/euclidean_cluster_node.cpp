// Copyright 2019-2020 the Autoware Foundation
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
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

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

static BBoxConfig bbox_config_declare(rclcpp::Node & n)
{
  using builtin_interfaces::msg::Duration;
  using std_msgs::msg::ColorRGBA;

  ColorRGBA rgba;
  rgba.r = static_cast<decltype(ColorRGBA::r)>(
    n.declare_parameter<float32_t>(ColorRParamName, 1.0f));
  rgba.g = static_cast<decltype(ColorRGBA::g)>(
    n.declare_parameter<float32_t>(ColorGParamName, 0.5f));
  rgba.b = static_cast<decltype(ColorRGBA::b)>(
    n.declare_parameter<float32_t>(ColorBParamName, 0.0f));
  rgba.a = static_cast<decltype(ColorRGBA::a)>(
    n.declare_parameter<float32_t>(ColorAParamName, 0.75f));

  Duration duration;
  duration.sec = static_cast<decltype(Duration::sec)>
    (n.declare_parameter<int>(LifetimeSecParamName, 0));
  duration.nanosec = static_cast<decltype(Duration::nanosec)>
    (n.declare_parameter<int>(LifetimeNanosecParamName, 500000000));

  return BBoxConfig{std::move(rgba), std::move(duration)};
}

////////////////////////////////////////////////////////////////////////////////
EuclideanClusterNode::EuclideanClusterNode(
  const rclcpp::NodeOptions & node_options)
: Node("euclidean_cluster_cloud_node", node_options),
  m_cloud_sub_ptr{create_subscription<PointCloud2>(
      "points_in",
      rclcpp::QoS(10),
      [this](const PointCloud2::SharedPtr msg) {handle(msg);})},
  m_cluster_pub_ptr{declare_parameter("use_cluster").get<bool8_t>() ?
  create_publisher<Clusters>(
    "points_clustered",
    rclcpp::QoS(10)) : nullptr},
m_box_pub_ptr{declare_parameter("use_box").get<bool8_t>() ?
  create_publisher<BoundingBoxArray>(
    "lidar_bounding_boxes", rclcpp::QoS{10}) :
  nullptr},
m_marker_pub_ptr{get_parameter("use_box").as_bool() ?
  create_publisher<MarkerArray>(
    "lidar_bounding_boxes_viz", rclcpp::QoS{10}) :
  nullptr},
m_parameters_client{std::make_unique<rclcpp::AsyncParametersClient>(this)},
m_parameter_event_sub{m_parameters_client->on_parameter_event(
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
    {on_parameter_event_callback(event);})},
m_cluster_alg{
  euclidean_cluster::Config{
    declare_parameter("cluster.frame_id").get<std::string>().c_str(),
    static_cast<std::size_t>(declare_parameter("cluster.min_cluster_size").get<std::size_t>()),
    static_cast<std::size_t>(declare_parameter("cluster.max_num_clusters").get<std::size_t>()),
    static_cast<float32_t>(declare_parameter("cluster.min_cluster_threshold_m").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("cluster.max_cluster_threshold_m").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("cluster.threshold_saturation_distance_m")
    .get<float32_t>())
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
m_use_z{declare_parameter("use_z").get<bool8_t>()},
m_bbox_config{bbox_config_declare(*this)}
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
  m_boxes.header.frame_id.reserve(256U);
  m_boxes.header.frame_id = cfg.frame_id().c_str();
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::insert_plain(const PointCloud2 & cloud)
{
  using euclidean_cluster::PointXYZI;
  const auto indices = common::lidar_utils::sanitize_point_cloud(cloud);
  if (indices.point_step != cloud.point_step) {
    std::cout << "Using only a subset of Point cloud fields" << std::endl;
  }
  if (indices.data_length != cloud.data.size()) {
    std::cout << "Misaligned data: Using only a subset of Point cloud data" << std::endl;
  }
  // Insert via memcpy to ensure proper aliasing
  for (auto idx = 0U; idx < indices.data_length; idx += cloud.point_step) {
    using euclidean_cluster::PointXYZI;
    PointXYZI pt;
    void * const dest = &pt;
    const void * const src = &cloud.data[idx];
    (void)std::memcpy(dest, src, indices.point_step);
    m_cluster_alg.insert(euclidean_cluster::PointXYZIR{pt});
  }
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
  m_clusters.header = header;
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
        euclidean_cluster::details::compute_eigenboxes_with_z(clusters, m_boxes);
      } else {
        euclidean_cluster::details::compute_eigenboxes(clusters, m_boxes);
      }
    } else {
      if (m_use_z) {
        euclidean_cluster::details::compute_lfit_bounding_boxes_with_z(clusters, m_boxes);
      } else {
        euclidean_cluster::details::compute_lfit_bounding_boxes(clusters, m_boxes);
      }
    }
    m_boxes.header.stamp = header.stamp;
    // Frame id was reserved
    m_boxes.header.frame_id = header.frame_id;
    m_box_pub_ptr->publish(m_boxes);

    // Also publish boxes for visualization
    uint32_t id_counter = 0;
    MarkerArray marker_array;
    for (const auto & box : m_boxes.boxes) {
      Marker m{};
      m.header.stamp = rclcpp::Time(0);
      m.header.frame_id = header.frame_id;
      m.ns = "bbox";
      m.id = static_cast<int>(id_counter);
      m.type = Marker::CUBE;
      m.action = Marker::ADD;
      m.pose.position.x = static_cast<float64_t>(box.centroid.x);
      m.pose.position.y = static_cast<float64_t>(box.centroid.y);
      m.pose.position.z = static_cast<float64_t>(box.centroid.z);
      m.pose.orientation.x = static_cast<float64_t>(box.orientation.x);
      m.pose.orientation.y = static_cast<float64_t>(box.orientation.y);
      m.pose.orientation.z = static_cast<float64_t>(box.orientation.z);
      m.pose.orientation.w = static_cast<float64_t>(box.orientation.w);
      // X and Y scale are swapped between these two message types
      m.scale.x = static_cast<float64_t>(box.size.y);
      m.scale.y = static_cast<float64_t>(box.size.x);
      m.scale.z = static_cast<float64_t>(box.size.z);
      m.color = m_bbox_config.color;
      m.lifetime = m_bbox_config.lifetime;

      marker_array.markers.push_back(m);
      id_counter++;
    }
    m_marker_pub_ptr->publish(marker_array);
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
    m_cluster_alg.throw_stored_error();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), e.what());
  } catch (...) {
    RCLCPP_FATAL(get_logger(), "EuclideanClusterNode: Unexpected error occurred!");
    throw;
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::on_parameter_event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  using rcl_interfaces::msg::ParameterType;
  using builtin_interfaces::msg::Duration;
  using std_msgs::msg::ColorRGBA;

  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == ColorRParamName) {
        RCLCPP_INFO(
          get_logger(), "%s set from %lf to %lf", name.c_str(),
          static_cast<double>(m_bbox_config.color.r), value.double_value);
        m_bbox_config.color.r = static_cast<decltype(ColorRGBA::r)>(value.double_value);
      } else if (name == ColorGParamName) {
        RCLCPP_INFO(
          get_logger(), "%s set from %lf to %lf", name.c_str(),
          static_cast<double>(m_bbox_config.color.g), value.double_value);
        m_bbox_config.color.g = static_cast<decltype(ColorRGBA::g)>(value.double_value);
      } else if (name == ColorBParamName) {
        RCLCPP_INFO(
          get_logger(), "%s set from %lf to %lf", name.c_str(),
          static_cast<double>(m_bbox_config.color.b), value.double_value);
        m_bbox_config.color.b = static_cast<decltype(ColorRGBA::b)>(value.double_value);
      } else if (name == ColorAParamName) {
        RCLCPP_INFO(
          get_logger(), "%s set from %lf to %lf", name.c_str(),
          static_cast<double>(m_bbox_config.color.a), value.double_value);
        m_bbox_config.color.a = static_cast<decltype(ColorRGBA::a)>(value.double_value);
      } else {
        RCLCPP_INFO(
          get_logger(), "Parameter %s type %s was not set",
          name.c_str(), std::to_string(type).c_str());
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == LifetimeSecParamName) {
        RCLCPP_INFO(
          get_logger(), "%s set from %d to %d",
          name.c_str(), m_bbox_config.lifetime.sec, value.integer_value);
        m_bbox_config.lifetime.sec = static_cast<decltype(Duration::sec)>(value.integer_value);
      } else if (name == LifetimeNanosecParamName) {
        RCLCPP_INFO(
          get_logger(), "%s set from %d to %d", name.c_str(),
          m_bbox_config.lifetime.nanosec, value.integer_value);
        m_bbox_config.lifetime.nanosec =
          static_cast<decltype(Duration::nanosec)>(value.integer_value);
      } else {
        RCLCPP_INFO(
          get_logger(), "Parameter %s type %s was not set",
          name.c_str(), std::to_string(type).c_str());
      }
    } else {
      RCLCPP_INFO(
        get_logger(), "Parameter %s type %s was not set",
        name.c_str(), std::to_string(type).c_str());
    }
  }
}

}  // namespace euclidean_cluster_nodes
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode)
