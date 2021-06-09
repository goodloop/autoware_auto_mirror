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

#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "pcl_conversions/pcl_conversions.h"

#include "geometry_msgs/msg/point32.hpp"
#include "outlier_filter/radius_search_2d_filter.hpp"
#include "lidar_utils/point_cloud_utils.hpp"

using autoware::common::types::float32_t;
using PointXYZ = geometry_msgs::msg::Point32;
using RadiusSearch2DFilter =
  autoware::perception::filters::outlier_filter::radius_search_2d_filter::RadiusSearch2DFilter;

// Helper methods
// FIXME(jilada): this function is based on a function from test_point_cloud_fusion.hpp
pcl::PointCloud<pcl::PointXYZ> make_pc(
  std::vector<pcl::PointXYZ> points,
  builtin_interfaces::msg::Time stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  autoware::common::lidar_utils::init_pcl_msg(msg, "base_link", points.size());

  uint32_t pidx = 0;
  for (auto point : points) {
    autoware::common::types::PointXYZIF pt;
    pt.x = point.x;
    pt.y = point.y;
    pt.z = point.z;
    pt.intensity = 1.0;
    autoware::common::lidar_utils::add_point_to_cloud(msg, pt, pidx);
  }

  msg.header.stamp = stamp;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::moveFromROSMsg(msg, pcl_cloud);

  return pcl_cloud;
}

pcl::PointXYZ make_point(float x, float y, float z)
{
  pcl::PointXYZ p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// FIXME(jilada): this function is copied from test_point_cloud_fusion.hpp
builtin_interfaces::msg::Time to_msg_time(
  const std::chrono::system_clock::time_point time_point)
{
  const auto tse = time_point.time_since_epoch();
  if (tse < std::chrono::nanoseconds(0LL)) {
    throw std::invalid_argument("ROS 2 builtin interfaces time does not support negative a epoch.");
  }
  builtin_interfaces::msg::Time result;

  result.sec = static_cast<decltype(result.sec)>(
    std::chrono::duration_cast<std::chrono::seconds>(tse).count());

  result.nanosec = static_cast<decltype(result.nanosec)>((
      tse - std::chrono::duration_cast<std::chrono::seconds>(tse)).count());

  return result;
}

void check_pc(
  std::vector<pcl::PointXYZ> new_points,
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc)
{
  // Check that points are of equal size
  EXPECT_EQ(new_points.size(), filtered_pc->points.size());

  // Compare points
  auto pc_it = filtered_pc->begin();
  auto p_it = new_points.begin();

  while (pc_it != filtered_pc->end() && p_it != new_points.end()) {
    // Check values
    ASSERT_FLOAT_EQ(pc_it->x, p_it->x);
    ASSERT_FLOAT_EQ(pc_it->y, p_it->y);
    ASSERT_FLOAT_EQ(pc_it->z, p_it->z);

    // Update iterators
    pc_it++;
    p_it++;
  }

  // Check that both iterators have reach the end
  ASSERT_EQ(pc_it, filtered_pc->end());
  ASSERT_EQ(p_it, new_points.end());
}


// TEST METHODS
/* TEST 1: A single point pointcloud
 *
 *   x -> removed
 *
 */
TEST(RadiusSearch2DFilter, test_single_point) {
  auto filter = std::make_shared<RadiusSearch2DFilter>(1.0, 5);
  std::vector<pcl::PointXYZ> points = {
    make_point(0.0f, 0.0f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc(new pcl::PointCloud<pcl::PointXYZ>(input));
  filter->filter(pc, output);

  // Perform checks on the output pointcloud
  // For this test the single pointcloud is considered an outlier and will be removed
  check_pc({}, output);
}

/* TEST 2: A simple radial pointcloud
 *   x        x
 * x x x -> x x x
 *   x        x
 */
TEST(RadiusSearch2DFilter, test_simple_cloud) {
  auto filter = std::make_shared<RadiusSearch2DFilter>(1.0, 5);
  std::vector<pcl::PointXYZ> points = {
    make_point(0.0f, 0.0f, 0.0f),
    make_point(0.2f, 0.0f, 0.0f),
    make_point(0.0f, 0.2f, 0.0f),
    make_point(-0.2f, 0.0f, 0.0f),
    make_point(0.0f, -0.2f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc(new pcl::PointCloud<pcl::PointXYZ>(input));
  filter->filter(pc, output);

  // Perform checks on the output pointcloud
  // For this test the pointcloud should remain the same
  check_pc(points, output);
}

/* TEST 3: A simple radial pointcloud with one outlier
 *   x       x        x
 * x x x         -> x x x
 *   x                x
 */
TEST(RadiusSearch2DFilter, test_outlier_point) {
  auto filter = std::make_shared<RadiusSearch2DFilter>(0.5, 5);
  std::vector<pcl::PointXYZ> points = {
    make_point(0.0f, 0.0f, 0.0f),
    make_point(0.2f, 0.0f, 0.0f),
    make_point(0.0f, 0.2f, 0.0f),
    make_point(-0.2f, 0.0f, 0.0f),
    make_point(0.0f, -0.2f, 0.0f),
    make_point(0.8f, 0.2f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc(new pcl::PointCloud<pcl::PointXYZ>(input));
  filter->filter(pc, output);

  // Perform checks on the output pointcloud
  // For this test the single pointcloud is considered an outlier and will be removed
  // Pop the last element since it is the outlier
  points.pop_back();
  check_pc(points, output);
}

/* TEST 4: A simple radial pointcloud, minimum number of neighbours increased
 *   x
 * x x x -> all removed
 *   x
 */
TEST(RadiusSearch2DFilter, test_increase_min_neighbours) {
  auto filter = std::make_shared<RadiusSearch2DFilter>(1.0, 10);
  std::vector<pcl::PointXYZ> points = {
    make_point(0.0f, 0.0f, 0.0f),
    make_point(0.2f, 0.0f, 0.0f),
    make_point(0.0f, 0.2f, 0.0f),
    make_point(-0.2f, 0.0f, 0.0f),
    make_point(0.0f, -0.2f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc(new pcl::PointCloud<pcl::PointXYZ>(input));
  filter->filter(pc, output);

  // Perform checks on the output pointcloud
  // Min neighbours increased, not enough neighbours all points should fail checks
  check_pc({}, output);
}

/* TEST 5: A simple radial pointcloud, minimum radius decreased
 *   x
 * x x x -> all removed
 *   x
 */
TEST(RadiusSearch2DFilter, test_decrease_search_radius) {
  auto filter = std::make_shared<RadiusSearch2DFilter>(0.1, 5);
  std::vector<pcl::PointXYZ> points = {
    make_point(0.0f, 0.0f, 0.0f),
    make_point(0.2f, 0.0f, 0.0f),
    make_point(0.0f, 0.2f, 0.0f),
    make_point(-0.2f, 0.0f, 0.0f),
    make_point(0.0f, -0.2f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc(new pcl::PointCloud<pcl::PointXYZ>(input));
  filter->filter(pc, output);

  // Perform checks on the output pointcloud
  // Min neighbours increased, not enough neighbours all points should fail checks
  check_pc({}, output);
}
