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
#include "lslidar_nodes/lslidar_cloud_node.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace drivers
{
namespace lslidar_nodes
{

template<typename T>
LslidarCloudNode<T>::LslidarCloudNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options),
  m_io_cxt(),
  m_udp_driver(m_io_cxt),
  m_udp_driver1(m_io_cxt),
  m_translator(Config{static_cast<float32_t>(this->declare_parameter("rpm").template get<int>())}),
  m_ip(this->declare_parameter("ip").template get<std::string>().c_str()),
  m_port(static_cast<uint16_t>(this->declare_parameter("port").template get<uint16_t>())),
  m_port1(static_cast<uint16_t>(this->declare_parameter("port_dev").template get<uint16_t>())),
  m_pc2_pub_ptr(create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter("topic").template get<std::string>(), rclcpp::QoS{10})),
  m_remainder_start_idx(0U),
  m_point_cloud_idx(0),
  m_frame_id(this->declare_parameter("frame_id").template get<std::string>().c_str())
{
  m_point_block.reserve(LslidarTranslatorT::POINT_BLOCK_CAPACITY);
  init_udp_driver();
  init_output(m_pc2_msg);
}

template<typename T>
void LslidarCloudNode<T>::init_udp_driver()
{
  m_udp_driver.init_receiver(m_ip, m_port);
  m_udp_driver.receiver()->open();
  m_udp_driver.receiver()->bind();
  m_udp_driver.receiver()->asyncReceive(
    std::bind(&LslidarCloudNode<T>::receiver_callback, this, std::placeholders::_1));
	
  m_udp_driver1.init_receiver(m_ip, m_port1);
  m_udp_driver1.receiver()->open();
  m_udp_driver1.receiver()->bind();
  m_udp_driver1.receiver()->asyncReceive(
    std::bind(&LslidarCloudNode<T>::receiver_callback_dev, this, std::placeholders::_1));
}


template<typename T>
void LslidarCloudNode<T>::receiver_callback_dev(const std::vector<uint8_t> & buffer)
{
  m_translator.unpack_dev(buffer, m_cloud_size);
  m_analysis_dev = true;
}

template<typename T>
void LslidarCloudNode<T>::receiver_callback(const std::vector<uint8_t> & buffer)
{
	if(m_analysis_dev)
	{
	  try {
		// message received, convert and publish
		if (this->convert(buffer, m_pc2_msg)) {
		  m_pc2_pub_ptr->publish(m_pc2_msg);
		}
	  } catch (const std::exception & e) {
		RCLCPP_WARN(this->get_logger(), e.what());
		// And then just continue running
	  } catch (...) {
		// Something really weird happened and I can't handle it here
		RCLCPP_WARN(this->get_logger(), "Unknown exception occured in LslidarCloudNode");
		throw;
	  }
	}
}

////////////////////////////////////////////////////////////////////////////////
template<typename T>
void LslidarCloudNode<T>::init_output(sensor_msgs::msg::PointCloud2 & output)
{
  autoware::common::lidar_utils::init_pcl_msg(output, m_frame_id.c_str(), m_cloud_size);
  m_point_cloud_its.reset(output, m_point_cloud_idx);
}

////////////////////////////////////////////////////////////////////////////////
template<typename T>
bool8_t LslidarCloudNode<T>::convert(
  const std::vector<uint8_t> & buffer,
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
  m_translator.convert(buffer, m_point_block);
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

template class LslidarCloudNode<lslidar_driver::LS16Data>;
template class LslidarCloudNode<lslidar_driver::LS32Data>;
template class LslidarCloudNode<lslidar_driver::CH128Data>;
template class LslidarCloudNode<lslidar_driver::CH64wData>;
}  // namespace lslidar_nodes
}  // namespace drivers
}  // namespace autoware
