// Copyright 2020 Apex.AI, Inc.
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
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <memory>

#include "lidar_utils/point_cloud_utils.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using autoware::common::types::float32_t;
using autoware::common::lidar_utils::create_custom_pcl;

class RayGroundPclValidationTester : public rclcpp::Node
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

public:
  RayGroundPclValidationTester()
  : Node{"pcl_listener"},
    m_sub_nonground_points{create_subscription<PointCloud2>("nonground_cloud", rclcpp::QoS{50},
        [this](const PointCloud2::SharedPtr msg) {
          m_nonground_points.emplace_back(*msg);
        })},
    m_sub_ground_points{create_subscription<PointCloud2>("ground_cloud", rclcpp::QoS{50},
      [this](const PointCloud2::SharedPtr msg) {
        m_ground_points.emplace_back(*msg);
      })},
  m_pub_raw_points{create_publisher<PointCloud2>("raw_cloud", rclcpp::QoS{50})}
  {
  }

  bool8_t receive_correct_ground_pcls(uint32_t expected_gounrd_pcl_size, uint32_t expected_num)
  {
    bool8_t ret = true;
    if (m_ground_points.size() != expected_num) {
      std::cout << "expected num of pcl not matched" << std::endl;
      std::cout << "actual num = " << m_ground_points.size() << std::endl;
      std::cout << "expected num = " << expected_num << std::endl;
      return false;
    }
    for (std::size_t i = 0; i < m_ground_points.size(); i++) {
      ret = (m_ground_points[i].data.size() == expected_gounrd_pcl_size) && ret;
      std::cout << "ground pc actual size = " << m_ground_points[i].data.size() << std::endl;
    }
    return ret;
  }

  bool8_t receive_correct_nonground_pcls(uint32_t expected_nongnd_pcl_size, uint32_t expected_num)
  {
    bool8_t ret = true;
    if (m_nonground_points.size() != expected_num) {
      std::cout << "expected num of pcl not matched" << std::endl;
      std::cout << "actual num = " << m_nonground_points.size() << std::endl;
      std::cout << "expected num = " << expected_num << std::endl;
      return false;
    }
    for (std::size_t i = 0; i < m_nonground_points.size(); i++) {
      ret = (m_nonground_points[i].data.size() == expected_nongnd_pcl_size) && ret;
      std::cout << "nonground pc actual size = " << m_nonground_points[i].data.size() << std::endl;
    }
    return ret;
  }

  std::vector<PointCloud2> m_nonground_points;
  std::vector<PointCloud2> m_ground_points;
  std::shared_ptr<rclcpp::Subscription<PointCloud2>> m_sub_nonground_points;
  std::shared_ptr<rclcpp::Subscription<PointCloud2>> m_sub_ground_points;
  std::shared_ptr<rclcpp::Publisher<PointCloud2>> m_pub_raw_points;
};

TEST(ray_ground_classifier_pcl_validation, filter_test)
{
  rclcpp::init(0, nullptr);

  using autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode;
  std::shared_ptr<RayGroundClassifierCloudNode> ray_gnd_ptr;
  std::shared_ptr<RayGroundPclValidationTester> ray_gnd_validation_tester;
  const std::string raw_pcl_topic{"raw_cloud"};
  const std::string ground_pcl_topic{"ground_cloud"};
  const std::string nonground_pcl_topic{"nonground_cloud"};
  const uint32_t mini_cloud_size = 10U;

  const int32_t cloud_size{55000U};
  const char8_t * const frame_id{"base_link"};

  std::vector<rclcpp::Parameter> params;

  params.emplace_back("__node", "ray_ground_classifier_cloud_node");

  params.emplace_back("frame_id", frame_id);

  params.emplace_back("cloud_timeout_ms", 110);

  params.emplace_back("pcl_size", cloud_size);

  params.emplace_back("raw_topic", raw_pcl_topic);
  params.emplace_back("ground_topic", ground_pcl_topic);
  params.emplace_back("nonground_topic", nonground_pcl_topic);

  params.emplace_back("classifier.sensor_height_m", 0.0);
  params.emplace_back("classifier.max_local_slope_deg", 20.0);
  params.emplace_back("classifier.max_global_slope_deg", 7.0);
  params.emplace_back("classifier.nonground_retro_thresh_deg", 70.0);
  params.emplace_back("classifier.min_height_thresh_m", 0.05);
  params.emplace_back("classifier.max_global_height_thresh_m", 3.3);
  params.emplace_back("classifier.max_last_local_ground_thresh_m", 3.6);
  params.emplace_back("classifier.max_provisional_ground_distance_m", 5.0);
  params.emplace_back("classifier.min_height_m", -2.5);
  params.emplace_back("classifier.max_height_m", 3.5);

  params.emplace_back("aggregator.min_ray_angle_rad", -3.14159);
  params.emplace_back("aggregator.max_ray_angle_rad", 3.14159);
  params.emplace_back("aggregator.ray_width_rad", 0.01);
  params.emplace_back("aggregator.max_ray_points", 512);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  ray_gnd_ptr = std::make_shared<RayGroundClassifierCloudNode>(
    node_options);

  ray_gnd_validation_tester = std::make_shared<RayGroundPclValidationTester>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(ray_gnd_ptr->get_node_base_interface());
  exec.add_node(ray_gnd_validation_tester);

  std::vector<std::string> five_field_names{"x", "y", "z", "intensity", "timestamp"};
  std::vector<std::string> three_field_names{"x", "y", "z"};
  const auto three_fields_pc = create_custom_pcl<float32_t>(three_field_names, mini_cloud_size);
  const auto five_fields_pc = create_custom_pcl<float32_t>(five_field_names, mini_cloud_size);

  // expected size = 4 bytes * 4 fields * cloud_size
  uint32_t expected_gnd_pcl_size = 4U * 4U * mini_cloud_size;
  uint32_t expected_nongnd_pcl_size = 0U;  // no points will be classified as nonground
  uint32_t expected_num_of_pcl = 2U;

  while (ray_gnd_validation_tester->m_pub_raw_points->get_subscription_count() < 1) {
    std::this_thread::sleep_for(std::chrono::milliseconds{1LL});
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500LL));

  while (ray_gnd_validation_tester->m_nonground_points.size() < (expected_num_of_pcl - 1) &&
    ray_gnd_validation_tester->m_ground_points.size() < (expected_num_of_pcl - 1))
  {
    ray_gnd_validation_tester->m_pub_raw_points->publish(*five_fields_pc);
    // wait for ray_gnd_filter to process 1st pc and publish data
    std::this_thread::sleep_for(std::chrono::milliseconds(500LL));
    exec.spin_some();  // for tester to collect data
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500LL));

  while (ray_gnd_validation_tester->m_nonground_points.size() < expected_num_of_pcl &&
    ray_gnd_validation_tester->m_ground_points.size() < expected_num_of_pcl)
  {
    ray_gnd_validation_tester->m_pub_raw_points->publish(*three_fields_pc);
    // wait for ray_gnd_filter to process 2nd pc and publish data
    std::this_thread::sleep_for(std::chrono::milliseconds(500LL));
    exec.spin_some();  // for tester to collect data
  }

  // Check all published nonground / ground pointclouds have the expected sizes
  EXPECT_TRUE(ray_gnd_validation_tester->receive_correct_ground_pcls(
      expected_gnd_pcl_size, expected_num_of_pcl));
  EXPECT_TRUE(ray_gnd_validation_tester->receive_correct_nonground_pcls(
      expected_nongnd_pcl_size, expected_num_of_pcl));
}
