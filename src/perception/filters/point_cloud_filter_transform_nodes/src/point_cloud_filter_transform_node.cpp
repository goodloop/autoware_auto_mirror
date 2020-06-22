// Copyright 2017-2020 Apex.AI, Inc.
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

#include <common/types.hpp>
#include <point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp>
#include <memory>
#include <string>

namespace autoware
{
namespace perception
{
namespace filters
{
/// \brief Boilerplate Apex.OS nodes around point_cloud_filter_transform_nodes
namespace point_cloud_filter_transform_nodes
{
using autoware::common::lidar_utils::add_point_to_cloud;
using autoware::common::lidar_utils::has_intensity_and_throw_if_no_xyz;
using autoware::common::lidar_utils::reset_pcl_msg;
using autoware::common::lidar_utils::resize_pcl_msg;
using autoware::common::lidar_utils::sanitize_point_cloud;
using autoware::common::types::float64_t;
using autoware::common::types::PointXYZIF;
using geometry_msgs::msg::Transform;
using sensor_msgs::msg::PointCloud2;

Transform PointCloud2FilterTransformNode::get_transform_from_parameters(const std::string & prefix)
{
  Transform ret;
  ret.rotation.x = declare_parameter(prefix + ".quaternion.x").get<float64_t>();
  ret.rotation.y = declare_parameter(prefix + ".quaternion.y").get<float64_t>();
  ret.rotation.z = declare_parameter(prefix + ".quaternion.z").get<float64_t>();
  ret.rotation.w = declare_parameter(prefix + ".quaternion.w").get<float64_t>();
  ret.translation.x = declare_parameter(prefix + ".translation.x").get<float64_t>();
  ret.translation.y = declare_parameter(prefix + ".translation.y").get<float64_t>();
  ret.translation.z = declare_parameter(prefix + ".translation.z").get<float64_t>();
  return ret;
}

const PointCloud2 & PointCloud2FilterTransformNode::filter_and_transform(const PointCloud2 & msg)
{
  // Verify frame_id
  if (msg.header.frame_id != m_input_frame_id) {
    throw std::runtime_error("Raw topic from unexpected frame. Expected: " +
            m_input_frame_id + ", got: " + msg.header.frame_id);
  }

  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it(msg, "z");

  auto && intensity_it = intensity_iterator_wrapper(msg);

  auto point_cloud_idx = 0U;
  reset_pcl_msg(m_filtered_transformed_msg, m_pcl_size, point_cloud_idx);
  m_filtered_transformed_msg.header.stamp = msg.header.stamp;

  while (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end() &&
    !intensity_it.eof())
  {
    PointXYZIF pt;
    pt.x = *x_it;
    pt.y = *y_it;
    pt.z = *z_it;
    intensity_it.get_curent_value(pt.intensity);

    if (point_not_filtered(pt)) {
      auto transformed_point = transform_point(pt);
      transformed_point.intensity = pt.intensity;
      if (!add_point_to_cloud(
          m_filtered_transformed_msg, transformed_point, point_cloud_idx))
      {
        throw std::runtime_error(
                "Overran cloud msg point capacity");
      }
    }

    ++x_it;
    ++y_it;
    ++z_it;
    intensity_it.next();
  }
  resize_pcl_msg(m_filtered_transformed_msg, point_cloud_idx);
  return m_filtered_transformed_msg;
}

void
PointCloud2FilterTransformNode::process_filtered_transformed_message(
  const PointCloud2::SharedPtr msg)
{
  const auto filtered_transformed_msg = filter_and_transform(*msg);
  m_pub_ptr->publish(filtered_transformed_msg);
}

}  // namespace point_cloud_filter_transform_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
