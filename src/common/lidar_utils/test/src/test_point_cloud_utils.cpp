// Copyright 2017-2020 the Autoware Foundation, Arm Limited
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

#include <string>
#include <vector>

#include "common/types.hpp"
#include "lidar_utils/lidar_utils.hpp"

using autoware::common::types::float32_t;

TEST(point_cloud_utils, has_intensity_and_throw_if_no_xyz_test)
{
  using autoware::common::lidar_utils::create_custom_pcl;

  const uint32_t mini_cloud_size = 10U;

  std::vector<std::string> right_field_names{"x", "y", "z", "intensity"};
  std::vector<std::string> not_intensity_field_names{"x", "y", "z", "not_intensity"};
  std::vector<std::string> three_field_names{"x", "y", "z"};
  std::vector<std::string> five_field_names{"x", "y", "z", "intensity", "timestamp"};
  std::vector<std::string> invalid_field_names{"x", "y"};
  std::vector<std::string> wrong_x_field_names{"h", "y", "z"};
  std::vector<std::string> wrong_y_field_names{"x", "h", "z"};
  std::vector<std::string> wrong_z_field_names{"x", "y", "h"};
  const auto correct_pc = create_custom_pcl<float32_t>(right_field_names, mini_cloud_size);
  const auto not_intensity_pc = create_custom_pcl<float32_t>(
    not_intensity_field_names,
    mini_cloud_size);
  const auto three_fields_pc = create_custom_pcl<float32_t>(three_field_names, mini_cloud_size);
  const auto five_fields_pc = create_custom_pcl<float32_t>(five_field_names, mini_cloud_size);
  const auto invalid_pc = create_custom_pcl<float32_t>(invalid_field_names, mini_cloud_size);
  const auto no_x_pc = create_custom_pcl<float32_t>(wrong_x_field_names, mini_cloud_size);
  const auto no_y_pc = create_custom_pcl<float32_t>(wrong_y_field_names, mini_cloud_size);
  const auto no_z_pc = create_custom_pcl<float32_t>(wrong_z_field_names, mini_cloud_size);

  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(invalid_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_x_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_y_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_z_pc), std::runtime_error);
  EXPECT_FALSE(has_intensity_and_throw_if_no_xyz(not_intensity_pc));
  EXPECT_FALSE(has_intensity_and_throw_if_no_xyz(three_fields_pc));
  EXPECT_TRUE(has_intensity_and_throw_if_no_xyz(correct_pc));
  EXPECT_TRUE(has_intensity_and_throw_if_no_xyz(five_fields_pc));
}

TEST(point_cloud_utils, add_point_to_cloud_raw_is_correct)
{
  // this test if add_point_to_cloud_raw's results are the same than the safer add_point_to_cloud
  const uint32_t max_cloud_size = 10;

  sensor_msgs::msg::PointCloud2 reference_msg;
  sensor_msgs::msg::PointCloud2 test_msg;
  init_pcl_msg(reference_msg, "dummy_frame", max_cloud_size);
  init_pcl_msg(test_msg, "dummy_frame", max_cloud_size);

  // fill the reference cloud
  PointCloudIts reference_msg_it;
  reference_msg_it.reset(reference_msg, 0, current_cloud_size);
  for (uint32_t i = 0; i < max_cloud_size; i++) {
    autoware::common::types::PointXYZIF pt{i, 256 * i, 0};
    add_point_to_cloud(reference_msg_it, pt, current_cloud_size);
  }

  ASSERT_EQ(test_msg.data.size(), 10U);
  // fill the tested cloud
  for (uint32_t i = 0; i < max_cloud_size; i++) {
    autoware::common::types::PointXYZIF pt{i, 256 * i, 0};
    add_point_to_cloud_raw(test_msg, pt, current_cloud_size);
  }

  EXPECT_EQ(reference_msg.data, test_msg.data);
  autoware::common::types::PointXYZIF pt{0, 0, 0};
  EXPECT_FALSE(add_point_to_cloud_raw(test_msg, pt, current_cloud_size)) <<
    "add_point_to_cloud_raw wrote a point outsize of the cloud's data bounds";
}

TEST(fast_atan2, max_error)
{
  float32_t max_error = 0;
  for (float32_t f = 0; f < autoware::common::types::TAU; f += 0.00001f) {
    float32_t x = cos(f);
    float32_t y = sin(f);
    max_error = ::std::max(
      max_error,
      fabsf(atan2f(y, x) - autoware::common::lidar_utils::fast_atan2(y, x)));
  }

  ASSERT_TRUE(max_error < FAST_ATAN2_MAX_ERROR);

  ASSERT_TRUE(
    fabsf(
      autoware::common::lidar_utils::fast_atan2(0.0f, 0.0f) -
      atan2f(0.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(
    fabsf(
      autoware::common::lidar_utils::fast_atan2(1.0f, 0.0f) -
      atan2f(1.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(
    fabsf(
      autoware::common::lidar_utils::fast_atan2(-1.0f, 0.0f) -
      atan2f(-1.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(
    fabsf(
      autoware::common::lidar_utils::fast_atan2(0.0f, 1.0f) -
      atan2f(0.0f, 1.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(
    fabsf(
      autoware::common::lidar_utils::fast_atan2(0.0f, -1.0f) -
      atan2f(0.0f, -1.0f)) < FAST_ATAN2_MAX_ERROR);
}
