// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#include "test_map_publisher.hpp"
#include <pcl/io/pcd_io.h>
#include <gtest/gtest.h>
#include <ndt_nodes/map_publisher.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <string>
#include <limits>


namespace autoware
{
namespace localization
{
namespace ndt_nodes
{
TEST(PCDLoadTest, basics) {
  constexpr auto num_points = 5U;
  pcl::PointCloud<pcl::PointXYZI> dummy_cloud{};
  sensor_msgs::msg::PointCloud2 msg;
  common::lidar_utils::init_pcl_msg(msg, "base_link", num_points);
  const std::string test_fname = "PCDLoadTest_test_pcd_file.pcd";
  const std::string non_existing_fname = "NON_EXISTING_FILE_PCDLoadTest.XYZ";
  for (auto i = 0U; i < num_points; i++) {
    pcl::PointXYZI pt;
    pt.x = static_cast<float32_t>(i);
    pt.y = static_cast<float32_t>(i);
    pt.z = static_cast<float32_t>(i);
    pt.intensity = static_cast<float32_t>(i);
    dummy_cloud.push_back(pt);
  }
  pcl::io::savePCDFile(test_fname, dummy_cloud);

  ASSERT_TRUE(std::ifstream{test_fname}.good());
  ASSERT_FALSE(std::ifstream{non_existing_fname}.good());
  EXPECT_THROW(read_from_pcd(non_existing_fname, msg), std::runtime_error);
  EXPECT_NO_THROW(read_from_pcd(test_fname, msg));

  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it(msg, "intensity");

  auto counter = 0.0F;

  while (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end())
  {
    EXPECT_FLOAT_EQ(*x_it, counter);
    EXPECT_FLOAT_EQ(*y_it, counter);
    EXPECT_FLOAT_EQ(*z_it, counter);
    EXPECT_FLOAT_EQ(*intensity_it, counter);

    ++x_it;
    ++y_it;
    ++z_it;
    ++intensity_it;
    counter += 1.0F;
  }
  EXPECT_FLOAT_EQ(counter, num_points);
  remove(test_fname.c_str());
}

TEST_F(MapPublisherTest, core_functionality)
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  const auto grid_config =
    perception::filters::voxel_grid::Config(m_min_point, m_max_point, m_voxel_size, m_capacity);

  const auto file_name = "MapPublisherTest_test.pcd";
  // have a validation map to transform the source cloud. The publisher's output
  // should match the cells in this map.
  ndt::DynamicNDTMap dynamic_validation_map(grid_config);
  ndt::StaticNDTMap static_received_map(grid_config);
  const auto map_topic = "publisher_test_map_topic";
  const auto map_frame = "map";
  auto callback_counter = 0U;
  Cloud received_cloud_map;
  const auto listener_node = rclcpp::Node::make_shared("MapPublisherTest_listener_node");
  const auto sub =
    listener_node->create_subscription<Cloud>(map_topic,
      rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local(),
      [&received_cloud_map, &callback_counter](Cloud::ConstSharedPtr msg) {
        ++callback_counter;
        received_cloud_map = *msg;
      });

  // Create map publisher. It is given 5 seconds to discover the test subscription.
  NDTMapPublisherNode map_publisher("test_map_publisher", "", map_topic, map_frame,
    grid_config, file_name);

  // Build a dense PC that can be transformed into 125 cells. See function for details.
  build_pc(grid_config);

  // Set up the validation grid.
  EXPECT_EQ(m_voxel_centers.size(), 125U);
  dynamic_validation_map.insert(m_pc);
  EXPECT_EQ(dynamic_validation_map.size(), m_voxel_centers.size());

  // Save the dense map to be read.
  const auto pcl_source = from_pointcloud2(m_pc);
  pcl::io::savePCDFile(file_name, pcl_source);
  ASSERT_TRUE(std::ifstream{file_name}.good());

  // Read pcd, pass the cloud to the internal dynamic map.
  EXPECT_NO_THROW(map_publisher.run());


  while (callback_counter < 1U) {
    rclcpp::spin_some(listener_node);
  }

  EXPECT_EQ(callback_counter, 1U);
  // Check that received pointcloud is a valid ndt map in terms of meta information.
  EXPECT_EQ(ndt::validate_pcl_map(received_cloud_map), dynamic_validation_map.size());
  // Insert to static map for easier iteration and access.
  static_received_map.insert(received_cloud_map);
  EXPECT_EQ(static_received_map.size(), dynamic_validation_map.size());

  // m_voxel_centers contain the centroid locations in the dynamic map.
  // Iterate and lookup these centroids in the reference and the received map.
  for (const auto & expected_centroid_it : m_voxel_centers) {
    const auto expected_centroid = expected_centroid_it.second;
    const auto & received_cell = static_received_map.cell(expected_centroid)[0U];
    const auto & reference_cell = dynamic_validation_map.cell(expected_centroid)[0U];
    EXPECT_TRUE(received_cell.centroid().isApprox(reference_cell.centroid(),
      std::numeric_limits<ndt::Real>::epsilon()));
    EXPECT_TRUE(received_cell.covariance().isApprox(reference_cell.covariance(),
      std::numeric_limits<ndt::Real>::epsilon() * 1e2));
  }
  remove(file_name);
}

TEST_F(MapPublisherTest, viz_functionality)
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  const auto grid_config =
    perception::filters::voxel_grid::Config(m_min_point, m_max_point, m_voxel_size, m_capacity);

  const auto file_name = "MapPublisherTest_test.pcd";
  // have a validation source cloud. The viz cloud publisher's output
  // should match the cells in this map.
  const auto map_topic = "publisher_test_map_topic";
  const auto viz_map_topic = "viz_publisher_test_map_topic";
  const auto map_frame = "map";
  auto callback_counter = 0U;
  auto viz_callback_counter = 0U;
  Cloud received_cloud_map;
  Cloud received_viz_cloud_map;
  const auto listener_node = rclcpp::Node::make_shared("MapPublisherTest_listener_node");
  const auto sub =
    listener_node->create_subscription<Cloud>(map_topic,
      rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local(),
      [&received_cloud_map, &callback_counter](Cloud::ConstSharedPtr msg) {
        ++callback_counter;
        received_cloud_map = *msg;
      });
  const auto viz_listener_node = rclcpp::Node::make_shared("MapPublisherTest_viz_listener_node");
  const auto viz_sub =
    viz_listener_node->create_subscription<Cloud>(viz_map_topic,
      rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local(),
      [&received_viz_cloud_map, &viz_callback_counter](Cloud::ConstSharedPtr msg) {
        ++viz_callback_counter;
        received_viz_cloud_map = *msg;
      });

  const auto map_publisher_ptr = std::make_shared<NDTMapPublisherNode>("test_map_publisher", "",
      map_topic, map_frame, grid_config, file_name, true, viz_map_topic);

  // Build a dense PC that can be transformed into 125 cells. See function for details.
  build_pc(grid_config);

  // Save the dense map to be read.
  const auto pcl_source = from_pointcloud2(m_pc);
  pcl::io::savePCDFile(file_name, pcl_source);
  ASSERT_TRUE(std::ifstream{file_name}.good());

  // Read pcd, pass the cloud to the internal dynamic map.
  EXPECT_NO_THROW(map_publisher_ptr->run());


  while (viz_callback_counter < 1U) {
    rclcpp::spin_some(map_publisher_ptr);  // TODO(yunus.caliskan): Remove spinning in #380
    rclcpp::spin_some(listener_node);
    rclcpp::spin_some(viz_listener_node);
  }

  EXPECT_EQ(viz_callback_counter, 1U);
  // Check that received viz pointcloud is a valid ndt map in terms of meta information.
  EXPECT_EQ(received_viz_cloud_map.width, m_pc.width);

  remove(file_name);
}

}  // namespace ndt_nodes
}  // namespace localization
}  // namespace autoware
