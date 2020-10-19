// Copyright 2020 The Autoware Foundation
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

#ifndef RAY_GROUND_CLASSIFIER_NODES__CONTRACT_HPP_
#define RAY_GROUND_CLASSIFIER_NODES__CONTRACT_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "autoware_auto_contracts/scalar_flicker.hpp"

using autoware::common::contracts::ScalarFlicker;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier_nodes
{
namespace contract
{
namespace preconditions
{

/// @brief Enforce pre-conditions on point cloud input.
contracts_lite::ReturnStatus callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  ScalarFlicker<size_t, 3> & input_flicker_detector,
  const std::string & frame_id);

}  // namespace preconditions

namespace postconditions
{

/// @brief Enforce the post-conditions on point cloud output.
contracts_lite::ReturnStatus callback(
  const sensor_msgs::msg::PointCloud2 & ground_msg,
  const sensor_msgs::msg::PointCloud2 & nonground_msg,
  ScalarFlicker<size_t, 3> & output_ground_flicker_detector,
  ScalarFlicker<size_t, 3> & output_nonground_flicker_detector);

}  // namespace postconditions

}  // namespace contract
}  // namespace ray_ground_classifier_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // RAY_GROUND_CLASSIFIER_NODES__CONTRACT_HPP_
