// Copyright 2018 the Autoware Foundation
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

#include <string>
#include <chrono>
#include <vector>

#include "common/types.hpp"
#include "lidar_utils/point_cloud_utils.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "velodyne_nodes/velodyne_cloud_node.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace drivers
{
namespace velodyne_nodes
{
template<typename T>
VelodyneCloudNode<T>::VelodyneCloudNode(
  const std::string & node_name,
  const std::string & ip,
  const uint16_t port,
  const std::string & frame_id,
  const std::size_t cloud_size,
  const Config & config)
: UdpDriverNode(
    node_name,
    "points_xyzi",
    typename UdpDriverNode::UdpConfig{ip, port}),
  m_translator(config),
  m_published_cloud(false),
  m_remainder_start_idx(0U),
  m_point_cloud_idx(0),
  m_frame_id(frame_id),
  m_cloud_size(cloud_size)
{
  m_point_block.reserve(VelodyneTranslatorT::POINT_BLOCK_CAPACITY);
  // If your preallocated cloud size is too small, the node really won't operate well at all
  if (static_cast<uint32_t>(m_point_block.capacity()) >= cloud_size) {
    throw std::runtime_error("VelodyneCloudNode: cloud_size must be > PointBlock::CAPACITY");
  }
}

////////////////////////////////////////////////////////////////////////////////
template<typename T>
VelodyneCloudNode<T>::VelodyneCloudNode(
  const std::string & node_name,
  const std::string & node_namespace)
: UdpDriverNode(node_name, node_namespace),
  m_translator(
    Config{
        static_cast<float32_t>(this->declare_parameter("rpm").template get<int>())}),
  m_published_cloud(false),
  m_remainder_start_idx(0U),
  m_point_cloud_idx(0),
  m_frame_id(this->declare_parameter("frame_id").template get<std::string>().c_str()),
  m_cloud_size(static_cast<std::size_t>(
      this->declare_parameter("cloud_size").template get<std::size_t>()))
{
  m_point_block.reserve(autoware::drivers::velodyne_driver::Vlp16Translator::POINT_BLOCK_CAPACITY);
  // If your preallocated cloud size is too small, the node really won't operate well at all
  if (static_cast<uint32_t>(m_point_block.capacity()) >= m_cloud_size) {
    throw std::runtime_error("VelodyneCloudNode: cloud_size must be > PointBlock::CAPACITY");
  }
}
////////////////////////////////////////////////////////////////////////////////
template<typename T>
void VelodyneCloudNode<T>::init_output(sensor_msgs::msg::PointCloud2 & output)
{
  autoware::common::lidar_utils::init_pcl_msg(output, m_frame_id.c_str(), m_cloud_size);
  m_point_cloud_its.reset(output, m_point_cloud_idx);
}

////////////////////////////////////////////////////////////////////////////////
template<typename T>
bool8_t VelodyneCloudNode<T>::convert(
  const Packet & pkt,
  sensor_msgs::msg::PointCloud2 & output)
{
  // This handles the case when the below loop exited due to containing extra points
  if (m_published_cloud) {
    // reset the pointcloud
    autoware::common::lidar_utils::reset_pcl_msg(output, m_cloud_size, m_point_cloud_idx);
    m_point_cloud_its.reset(output, m_point_cloud_idx);

    // deserialize remainder into pointcloud
    m_published_cloud = false;
    for (uint32_t idx = m_remainder_start_idx; idx < m_point_block.size(); ++idx) {
      const autoware::common::types::PointXYZIF & pt = m_point_block[idx];
      (void)add_point_to_cloud(m_point_cloud_its, pt, m_point_cloud_idx);
      // Here I am ignoring the return value, because this operation should never fail.
      // In the constructor I ensure that cloud_size > PointBlock::CAPACITY. This means
      // I am guaranteed to fit at least one whole PointBlock into my PointCloud2.
      // Because just above this for loop, I reset the capacity of the pcl message,
      // I am guaranteed to have capacity for the remainder of a point block.
    }
  }
  m_translator.convert(pkt, m_point_block);
  for (uint32_t idx = 0U; idx < m_point_block.size(); ++idx) {
    const autoware::common::types::PointXYZIF & pt = m_point_block[idx];
    if (static_cast<uint16_t>(autoware::common::types::PointXYZIF::END_OF_SCAN_ID) != pt.id) {
      if (!add_point_to_cloud(m_point_cloud_its, pt, m_point_cloud_idx)) {
        m_published_cloud = true;
        m_remainder_start_idx = idx;
      }
    } else {
      m_published_cloud = true;
      m_remainder_start_idx = idx;
      break;
    }
  }
  if (m_published_cloud) {
    // resize pointcloud down to its actual size
    autoware::common::lidar_utils::resize_pcl_msg(output, m_point_cloud_idx);
    output.header.stamp = this->now();
    m_point_cloud_its.reset(output, m_point_cloud_idx);
  }

  return m_published_cloud;
}

////////////////////////////////////////////////////////////////////////////////
template<typename T>
bool8_t VelodyneCloudNode<T>::get_output_remainder(sensor_msgs::msg::PointCloud2 & output)
{
  // The assumption checked in the constructor is that the PointCloud size is bigger than
  // the PointBlocks, which can fully contain a packet. The use case of this method is in case
  // PacketT > OutputT, which is not the case here.
  (void)output;
  return false;
}

template class VelodyneCloudNode<velodyne_driver::VLP16Data>;
template class VelodyneCloudNode<velodyne_driver::VLP32CData>;
template class VelodyneCloudNode<velodyne_driver::VLS128Data>;
}  // namespace velodyne_nodes
}  // namespace drivers
}  // namespace autoware
