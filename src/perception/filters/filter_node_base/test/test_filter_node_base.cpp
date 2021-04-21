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


#include <memory>
#include <vector>

#include "filter_node_base/filter_node_base.hpp"
#include "gtest/gtest.h"
#include "lidar_utils/point_cloud_utils.hpp"


using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::perception::filters::filter_node_base::FilterNodeBase;

class TestPCF : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    rclcpp::shutdown();
  }
};

/* \class TestFilter
 * \brief This class implements the FilterNodeBase to test for correct inheritence
 */
class TestFilter : public FilterNodeBase
{
protected:
  // Implementation of the filter child method
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
  {
    // Set parameter to true
    test_parameter_ = true;
  }

private:
  bool8_t test_parameter_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit TestFilter(const rclcpp::NodeOptions & options)
  : FilterNodeBase("test_filter_node", options), test_parameter_(false) {}

  bool8_t testSetParameter() {return test_parameter_;}
};


TEST_F(TestPCF, test_inheritance) {
  // Generate parameters
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("max_queue_size", 5);
  params.emplace_back("use_indices", false);
  params.emplace_back("approximate_sync", false);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  // Create instance of the TestFilter child class
  auto node = std::make_shared<TestFilter>(node_options);

  //// Set up
  // Create dummy pointcloud
  sensor_msgs::msg::PointCloud2 cloud;
  std::vector<float32_t> seeds = {0.0, 0.0, 0.0};
  autoware::common::lidar_utils::init_pcl_msg(cloud, "base_link", seeds.size());

  uint32_t pidx = 0;
  for (auto seed : seeds) {
    autoware::common::types::PointXYZIF pt;
    pt.x = seed;
    pt.y = seed;
    pt.z = seed;
    pt.intensity = seed;
    autoware::common::lidar_utils::add_point_to_cloud(cloud, pt, pidx);
  }

  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input",
    rclcpp::QoS(10));

  auto start_time = std::chrono::system_clock::now();
  auto max_test_dur = std::chrono::seconds(1);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    cloud_pub->publish(cloud);
    if (std::chrono::system_clock::now() - start_time > max_test_dur) {
      break;
    }
  }
  EXPECT_EQ(node->testSetParameter(), true);
}
