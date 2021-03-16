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

namespace autoware
{

Line::Line(const Point start, const Point end);

// p_1 = \hat{p_1} + t_1 \cdot d_1 \\ p_2 = \hat{p_2} + t_2 \cdot d_2,
Line::intersect_with_lidar(const Line l , const bool closest_point_only)
{

    Point p_delta = minus_2d(this->m_start , l.m_start);
    Point denominator = cross_2d(this->m_direction, l.direction);
    Point t1 = cross_2d(l.direction, p_delta) / denominator; // need to see about division for
    // Point type
    Point t2 = cross_2d(this->m_direction, p_delta) / denominator; // need to see about division
    // for Point type
    if (t1.x < 0 && t1.y<0 || t1.x > this->length or t2 < 0 or t2 > line.length)
//         return []
//    return [self.start + t1 * self.direction]

}


int32_t tracking_test_framework::print_hello()
{
  std::cout << "Hello World" << std::endl;


  return 0;
}

}  // namespace autoware
