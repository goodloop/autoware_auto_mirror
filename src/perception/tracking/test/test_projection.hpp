// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TEST_PROJECTION_HPP_
#define TEST_PROJECTION_HPP_

#include <tracking/projection.hpp>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <vector>

class PrismProjectionTest : public testing::Test
{
public:
  PrismProjectionTest();

  float32_t half_length{1.0F};
  float32_t half_width{1.0F};
  float32_t height{4.0F};

  float32_t distance_from_origin{8.0F};
  autoware_auto_msgs::msg::Shape rectangular_prism;
};

std::vector<geometry_msgs::msg::Point32> expand_shape_to_vector(
  const autoware_auto_msgs::msg::Shape & shape);

void compare_shapes(
  const geometry_msgs::msg::Polygon & prism_face,
  const autoware::perception::tracking::Projection & projection,
  const autoware::perception::tracking::CameraIntrinsics & intrinsics);

#endif  // TEST_PROJECTION_HPP_
