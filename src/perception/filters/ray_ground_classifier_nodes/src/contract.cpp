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

#include "ray_ground_classifier_nodes/contract.hpp"

#include <string>
#include <utility>

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

//------------------------------------------------------------------------------

contracts_lite::ReturnStatus callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  ScalarFlicker<size_t, 3> & input_flicker_detector,
  const std::string & frame_id)
{
  // Verify header
  auto header_status_comment = CONTRACT_COMMENT(
    "",
    "RayGroundClassifierCloudNode: message frame '" + msg->header.frame_id +
    "' must be '" + frame_id + "'");
  auto header_status = contracts_lite::ReturnStatus(std::move(header_status_comment)
      ,
      (msg->header.frame_id == frame_id));

  // Verify the consistency of PointCloud msg
  auto data_size_valid_comment = CONTRACT_COMMENT(
    "",
    "RayGroundClassifierCloudNode: PointCloud2 data size must be valid; it must hold that "
    "msg->data.size() == msg->row_step (" + std::to_string(
      msg->data.size()) + " == " + std::to_string(msg->row_step) + ")");
  auto data_size_valid = contracts_lite::ReturnStatus(std::move(data_size_valid_comment)
      ,
      (msg->data.size() == msg->row_step));

  const auto num_points = msg->width * msg->height;
  auto flicker_valid = input_flicker_detector.no_flicker(num_points);

  const auto data_length = num_points * msg->point_step;
  auto data_length_comment = CONTRACT_COMMENT(
    "",
    "RayGroundClassifierCloudNode: PointCloud2 data length must be valid; it must hold that "
    "data_length == msg->row_step (" + std::to_string(
      data_length) + " == " +
    std::to_string(msg->row_step) + ")");
  auto data_length_valid = contracts_lite::ReturnStatus(std::move(data_length_comment),
      (data_length == msg->row_step));

  return header_status && data_size_valid && data_length_valid && flicker_valid;
}

//------------------------------------------------------------------------------

}  // namespace preconditions

namespace postconditions
{

//------------------------------------------------------------------------------

contracts_lite::ReturnStatus callback(
  const sensor_msgs::msg::PointCloud2 & ground_msg,
  const sensor_msgs::msg::PointCloud2 & nonground_msg,
  ScalarFlicker<size_t, 3> & output_ground_flicker_detector,
  ScalarFlicker<size_t, 3> & output_nonground_flicker_detector)
{
  const auto ground_points = ground_msg.width * ground_msg.height;
  auto ground_flicker_valid = output_ground_flicker_detector.no_flicker(ground_points);

  const auto nonground_points = nonground_msg.width * nonground_msg.height;
  auto nonground_flicker_valid = output_nonground_flicker_detector.no_flicker(nonground_points);

  return ground_flicker_valid && nonground_flicker_valid;
}

//------------------------------------------------------------------------------

}  // namespace postconditions

}  // namespace contract
}  // namespace ray_ground_classifier_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
