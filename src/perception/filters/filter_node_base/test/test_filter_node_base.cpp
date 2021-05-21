// Copyright 2021 Tier IV, Inc.
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
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "lidar_utils/point_cloud_utils.hpp"


using bool8_t = autoware::common::types::bool8_t;
using float32_t = autoware::common::types::float32_t;
using FilterNodeBase = autoware::perception::filters::filter_node_base::FilterNodeBase;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

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
  void filter(
    const PointCloud2ConstPtr & input, PointCloud2 & output) override
  {
    // Copy the pointcloud so input = output
    output = *input;
    // Set parameter as true
    test_parameter_ = true;
  }

  // Implementation of the get_node_parameters child method
  rcl_interfaces::msg::SetParametersResult get_node_parameters(
    const std::vector<rclcpp::Parameter> &) override
  {
    rcl_interfaces::msg::SetParametersResult result{};
    result.successful = true;
    result.reason = "parameter received";
    return result;
  }

private:
  bool8_t test_parameter_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit TestFilter(const rclcpp::NodeOptions & options)
  : FilterNodeBase("test_filter_node", options), test_parameter_(false) {}

  bool8_t testSetParameter() {return test_parameter_;}
};


void check_pcl_eq(sensor_msgs::msg::PointCloud2 & msg1, sensor_msgs::msg::PointCloud2 & msg2)
{
  EXPECT_EQ(msg1.width, msg2.width);
  EXPECT_EQ(msg1.header.frame_id, msg2.header.frame_id);
  EXPECT_EQ(msg1.header.stamp, msg2.header.stamp);

  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it_1(msg1, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it_1(msg1, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it_1(msg1, "z");
  sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it_1(msg1, "intensity");

  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it_2(msg2, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it_2(msg2, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it_2(msg2, "z");
  sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it_2(msg2, "intensity");

  while (x_it_1 != x_it_1.end() &&
    y_it_1 != y_it_1.end() &&
    z_it_1 != z_it_1.end() &&
    intensity_it_1 != intensity_it_1.end() &&
    x_it_2 != x_it_2.end() &&
    y_it_2 != y_it_2.end() &&
    z_it_2 != z_it_2.end() &&
    intensity_it_2 != intensity_it_2.end()
  )
  {
    EXPECT_FLOAT_EQ(*x_it_1, *x_it_2);
    EXPECT_FLOAT_EQ(*y_it_1, *y_it_2);
    EXPECT_FLOAT_EQ(*z_it_1, *z_it_2);
    EXPECT_FLOAT_EQ(*intensity_it_1, *intensity_it_2);

    ++x_it_1;
    ++y_it_1;
    ++z_it_1;
    ++intensity_it_1;

    ++x_it_2;
    ++y_it_2;
    ++z_it_2;
    ++intensity_it_2;
  }
}


TEST_F(TestPCF, test_inheritance) {
  // Generate parameters
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("max_queue_size", 5);

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

  // Create callback for cloud message
  bool8_t received_cloud = false;
  auto handle_cloud_output =
    [&cloud, &received_cloud](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    -> void {
      check_pcl_eq(*msg, cloud);
      received_cloud = true;
    };

  auto sub_ptr = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "output",
    rclcpp::SensorDataQoS().keep_last(10), handle_cloud_output);

  while (rclcpp::ok() && !received_cloud) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    cloud_pub->publish(cloud);
  }
  EXPECT_EQ(node->testSetParameter(), true);
}
