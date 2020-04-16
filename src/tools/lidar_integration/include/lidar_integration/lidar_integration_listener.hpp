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

#ifndef LIDAR_INTEGRATION__LIDAR_INTEGRATION_LISTENER_HPP_
#define LIDAR_INTEGRATION__LIDAR_INTEGRATION_LISTENER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <lidar_integration/visibility_control.hpp>
#include <string>

namespace lidar_integration
{

using sensor_msgs::msg::PointCloud2;
using autoware_auto_msgs::msg::BoundingBoxArray;

class LIDAR_INTEGRATION_PUBLIC LidarIntegrationListener : public rclcpp::Node
{
  struct Statistics
  {
    bool success;
    bool last_failed;
    std::chrono::steady_clock::time_point first_pub_time;
    std::chrono::steady_clock::time_point last_pub_time;
    uint32_t total_size;
    uint32_t count;
  };  // struct Statistics

  static void init_statistics(Statistics & stats);

public:
  LidarIntegrationListener(
    const std::string & name,
    const float expected_period_ms,
    const uint32_t expected_size,
    const float relative_tolerance_period,
    const float relative_tolerance_size);

  virtual ~LidarIntegrationListener();

  virtual bool is_success() const = 0;

protected:
  // Update the statistics
  void callback(const uint32_t size);

  bool is_success(
    const rclcpp::SubscriptionBase * const sub_ptr,
    const char * const src) const;

private:
  bool has_valid_period(const Statistics & stats) const;

  bool has_valid_size(const Statistics & stats, const uint32_t expected_size) const;

  bool is_success(
    const rclcpp::SubscriptionBase * const sub_ptr,
    const uint32_t expected_size,
    const char * const src) const;

  void console_statistics(const Statistics & stat, const char * src) const;

  const float m_expected_period_us;
  const float m_relative_tolerance;
  const float m_relative_size_tolerance;
  const uint32_t m_expected_size;
  Statistics m_stats;
};  // LidarIntegrationListener

/// Specialization of the listener for point clouds
class LIDAR_INTEGRATION_PUBLIC LidarIntegrationPclListener : public LidarIntegrationListener
{
public:
  LidarIntegrationPclListener(
    const std::string & topic,
    const float expected_period_ms,
    const uint32_t expected_size,
    const float relative_tolerance_period,
    const float relative_tolerance_size,
    const std::string & name = "lidar_integration_listener");

  bool is_success() const override;

private:
  rclcpp::Subscription<PointCloud2>::SharedPtr m_sub_ptr;
};  // class LidarIntegrationPclListener

/// Specialization of the listener for bounding boxes
class LIDAR_INTEGRATION_PUBLIC LidarIntegrationBoxListener : public LidarIntegrationListener
{
public:
  LidarIntegrationBoxListener(
    const std::string & topic,
    const float expected_period_ms,
    const uint32_t expected_size,
    const float relative_tolerance_period,
    const float relative_tolerance_size,
    const std::string & name = "lidar_integration_listener");

  bool is_success() const override;

private:
  rclcpp::Subscription<BoundingBoxArray>::SharedPtr m_sub_ptr;
};  // class LidarIntegrationBoxListener

}  // namespace lidar_integration

#endif  // LIDAR_INTEGRATION__LIDAR_INTEGRATION_LISTENER_HPP_
