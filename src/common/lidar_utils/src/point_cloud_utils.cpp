// Copyright 2017-2018 Apex.AI, Inc.
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

//lint -e537 pclint vs cpplint NOLINT
#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "common/types.hpp"
#include "lidar_utils/point_cloud_utils.hpp"

namespace autoware
{
namespace common
{
namespace lidar_utils
{

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;
using autoware::common::types::float32_t;

// Check the pointcloud msg has x, y, z fields, otherwise throw an exception; check
// the pointcloud msg has intensity field, otherwise return false
bool8_t has_intensity_and_throw_if_no_xyz(
  const PointCloud2::SharedPtr & cloud)
{
  return has_intensity_and_throw_if_no_xyz(*cloud);
}

bool8_t has_intensity_and_throw_if_no_xyz(
  const PointCloud2 & cloud)
{
  bool8_t ret = true;
  // Validate point step
  if (cloud.fields.size() < 3U) {
    throw std::runtime_error("Invalid PointCloud msg");
  }

  const auto check_field = [](
    const sensor_msgs::msg::PointField & field,
    const char8_t * const name,
    const uint32_t offset) -> bool8_t {
      bool8_t res = true;
      if ((name != field.name) || (offset != field.offset) ||
        (sensor_msgs::msg::PointField::FLOAT32 != field.datatype) || (1U != field.count))
      {
        res = false;
      }
      return res;
    };

  if (!check_field(cloud.fields[0U], "x", 0U)) {
    throw std::runtime_error("PointCloud doesn't have correct x field");
  } else if (!check_field(cloud.fields[1U], "y", 4U)) {
    throw std::runtime_error("PointCloud doesn't have correct y field");
  } else if (!check_field(cloud.fields[2U], "z", 8U)) {
    throw std::runtime_error("PointCloud doesn't have correct z field");
  } else {
    // do nothing
  }
  if (cloud.fields.size() >= 4U) {
    if (!check_field(cloud.fields[3U], "intensity", 12U)) {
      ret = false;
    }
  } else {
    ret = false;
  }
  return ret;
}

PointCloudIts::PointCloudIts() {m_its.reserve(4);}

void PointCloudIts::reset(sensor_msgs::msg::PointCloud2 & cloud, uint32_t idx)
{
  // Destroy the old iterators
  m_its.clear();

  // Create new iterators
  m_its.emplace_back(cloud, "x");
  m_its.emplace_back(cloud, "y");
  m_its.emplace_back(cloud, "z");
  m_its.emplace_back(cloud, "intensity");

  // Advance iterators to given index
  x_it() += idx;
  y_it() += idx;
  z_it() += idx;
  intensity_it() += idx;
}


////
void init_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & frame_id,
  const std::size_t size)
{
  init_pcl_msg(msg, frame_id, size, 4U,
    "x", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1U, sensor_msgs::msg::PointField::FLOAT32);
}

////////////////////////////////////////////////////////////////////////////////
bool8_t add_point_to_cloud(
  PointCloudIts & cloud_its,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t & point_cloud_idx)
{
  bool8_t ret = false;

  auto & x_it = cloud_its.x_it();
  auto & y_it = cloud_its.y_it();
  auto & z_it = cloud_its.z_it();
  auto & intensity_it = cloud_its.intensity_it();

  // Actual size is 20 due to padding by compilers for the memory alignment boundary.
  // This check is to make sure that when we do a insert of 16 bytes, we will not stride
  // past the bounds of the structure.
  static_assert(
    sizeof(autoware::common::types::PointXYZIF) >= ((4U * sizeof(float32_t)) + sizeof(uint16_t)),
    "PointXYZIF is not expected size: ");

  if (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end() &&
    intensity_it != intensity_it.end())
  {
    // add the point data
    *x_it = pt.x;
    *y_it = pt.y;
    *z_it = pt.z;
    *intensity_it = pt.intensity;

    // increment the index to keep track of the pointcloud's size
    x_it += 1;
    y_it += 1;
    z_it += 1;
    intensity_it += 1;
    ++point_cloud_idx;

    ret = true;
  }
  return ret;
}

bool8_t add_point_to_cloud(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t & point_cloud_idx)
{
  bool8_t ret = false;

  sensor_msgs::PointCloud2Iterator<float32_t> x_it(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float32_t> y_it(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float32_t> z_it(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float32_t> intensity_it(cloud, "intensity");

  x_it += point_cloud_idx;
  y_it += point_cloud_idx;
  z_it += point_cloud_idx;
  intensity_it += point_cloud_idx;

  // Actual size is 20 due to padding by compilers for the memory alignment boundary.
  // This check is to make sure that when we do a insert of 16 bytes, we will not stride
  // past the bounds of the structure.
  static_assert(
    sizeof(autoware::common::types::PointXYZIF) >= ((4U * sizeof(float32_t)) + sizeof(uint16_t)),
    "PointXYZIF is not expected size: ");

  if (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end() &&
    intensity_it != intensity_it.end())
  {
    // add the point data
    *x_it = pt.x;
    *y_it = pt.y;
    *z_it = pt.z;
    *intensity_it = pt.intensity;

    // increment the index to keep track of the pointcloud's size
    ++point_cloud_idx;
    ret = true;
  }
  return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////

void reset_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::size_t size,
  uint32_t & point_cloud_idx)
{
  sensor_msgs::PointCloud2Modifier pc_modifier(msg);
  pc_modifier.clear();
  point_cloud_idx = 0;
  pc_modifier.resize(size);
}

/////////////////////////////////////////////////////////////////////////////////////////

void resize_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::size_t new_size)
{
  sensor_msgs::PointCloud2Modifier pc_modifier(msg);
  pc_modifier.resize(new_size);
}

}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware
