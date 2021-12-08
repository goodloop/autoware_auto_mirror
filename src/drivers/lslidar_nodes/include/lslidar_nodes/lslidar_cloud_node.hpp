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


/// \copyright Copyright 2017-2018 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief This file defines a simple ROS 2 lslidar driver that publishes full point clouds

#ifndef LSLIDAR_NODES__LSLIDAR_CLOUD_NODE_HPP_
#define LSLIDAR_NODES__LSLIDAR_CLOUD_NODE_HPP_

#include <string>
#include <vector>
#include "common/types.hpp"
#include "lidar_utils/point_cloud_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "udp_driver/udp_driver.hpp"
#include "lslidar_driver/lslidar_translator.hpp"
#include "lslidar_nodes/visibility_control.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using autoware::common::types::bool8_t;

namespace autoware
{
namespace drivers
{
/// \brief Resources for nodes that use the `lslidar_driver`
namespace lslidar_nodes
{

/// \tparam SensorData SensorData implementation for the specific lslidar sensor model.
template<typename SensorData>
class LSLIDAR_NODES_PUBLIC LslidarCloudNode : public rclcpp::Node
{
public:
  using LslidarTranslatorT = lslidar_driver::LslidarTranslator<SensorData>;
  using Config = typename LslidarTranslatorT::Config;

  LslidarCloudNode(const std::string & node_name, const rclcpp::NodeOptions & options);

  /// Handle data packet from the udp driver
  /// \param buffer Data from the udp driver
  void receiver_callback(const std::vector<uint8_t> & buffer);
  void receiver_callback_dev(const std::vector<uint8_t> & buffer);
protected:
  void init_output(sensor_msgs::msg::PointCloud2 & output);
  bool8_t convert(
    const std::vector<uint8_t> & buffer,
    sensor_msgs::msg::PointCloud2 & output);

private:
  void init_udp_driver();

  IoContext m_io_cxt;
  ::drivers::udp_driver::UdpDriver m_udp_driver;
  ::drivers::udp_driver::UdpDriver m_udp_driver1;
  LslidarTranslatorT m_translator;
  std::vector<autoware::common::types::PointXYZIF> m_point_block;

  std::string m_ip;
  uint16_t m_port;
  uint16_t m_port1;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pc2_pub_ptr;
  sensor_msgs::msg::PointCloud2 m_pc2_msg{};
  bool m_published_cloud = false;
  bool m_analysis_dev = false;
  // Keeps track of where you left off on the converted point block in case you needed to publish
  // a point cloud in the middle of processing it
  uint32_t m_remainder_start_idx;
  // keeps track of the constructed point cloud to continue growing it with new data
  uint32_t m_point_cloud_idx;
  autoware::common::lidar_utils::PointCloudIts m_point_cloud_its;
  const std::string m_frame_id;
  std::size_t m_cloud_size;
};  // class LslidarCloudNode

using LS16DriverNode = LslidarCloudNode<lslidar_driver::LS16Data>;
using LS32DriverNode = LslidarCloudNode<lslidar_driver::LS32Data>;
using CH128DriverNode = LslidarCloudNode<lslidar_driver::CH128Data>;
using CH64wDriverNode = LslidarCloudNode<lslidar_driver::CH64wData>;
}  // namespace lslidar_nodes
}  // namespace drivers
}  // namespace autoware

#endif  // LSLIDAR_NODES__LSLIDAR_CLOUD_NODE_HPP_
