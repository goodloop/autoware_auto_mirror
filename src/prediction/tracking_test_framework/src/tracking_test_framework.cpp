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

#include "tracking_test_framework/tracking_test_framework.hpp"

#include <iostream>
#include <vector>
#include <memory>

namespace autoware
{

namespace tracking_test_framework
{

Line::Line(const Eigen::Vector2f & start, const Eigen::Vector2f & end)
{
  this->m_start = start;
  this->m_end = end;
  this->m_line_length = (end - start).norm();
  this->m_line_direction = (end - start) / this->m_line_length;
}

// p_1 = \hat{p_1} + t_1 \cdot d_1 \\ p_2 = \hat{p_2} + t_2 \cdot d_2,
std::vector<Eigen::Vector2f> Line::intersect_with_lidar(
  std::unique_ptr<Shape> & shape, const bool
  closest_point_only = true)
{
  (void)closest_point_only;
  std::unique_ptr<Line>
  line(static_cast<Line *>(shape.release()));

  Eigen::Vector2f p_delta = this->m_start - line->m_start;
  float denominator = cross_2d(this->m_line_direction, line->m_line_direction);
  float_t t1 = cross_2d(line->m_line_direction, p_delta) / denominator;
  float_t t2 = cross_2d(this->m_line_direction, p_delta) / denominator;
  // If the scalar factors are not within the line length and non zero
  if (t1 < 0 || t1 > this->m_line_length || t2 < 0 || t2 > line->m_line_length) {
    return std::vector<Eigen::Vector2f>();
  }
  return std::vector<Eigen::Vector2f>{this->m_start + (t1 * this->m_line_direction)};
}

// Test function
int32_t print_hello()
{
  Eigen::Vector2f start1(-1, -1);
  Eigen::Vector2f end1(2, 0.5);
  Eigen::Vector2f start2(1, 0);
  Eigen::Vector2f end2(1, 2);
  std::unique_ptr<Shape> l1 = std::make_unique<Line>(start1, end1);
  std::unique_ptr<Shape> l2 = std::make_unique<Line>(start2, end2);
  std::vector<Eigen::Vector2f> intersection = l1->intersect_with_lidar(l2, true);
  std::cout << "Intersection pts: " << intersection[0] << std::endl;
  return 0;
}


}  // namespace tracking_test_framework
}  // namespace autoware
