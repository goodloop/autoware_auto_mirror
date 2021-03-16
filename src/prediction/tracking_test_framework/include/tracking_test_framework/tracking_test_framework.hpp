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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the tracking_test_framework class.

#ifndef TRACKING_TEST_FRAMEWORK__TRACKING_TEST_FRAMEWORK_HPP_
#define TRACKING_TEST_FRAMEWORK__TRACKING_TEST_FRAMEWORK_HPP_

#include <tracking_test_framework/visibility_control.hpp>

#include <cstdint>

using Point = geometry_msgs::msg::Point32;

namespace autoware
{
/// \brief TODO(divya.aggarwal): Document namespaces!
namespace tracking_test_framework
{

/// \brief TODO(divya.aggarwal): Document your functions
int32_t TRACKING_TEST_FRAMEWORK_PUBLIC print_hello();

class Shape {

public:
  virtual std::vector<PointClusters> intersect_with_lidar(Line l , bool closest_point_only)=0;
};

class Line : Shape{

public :
  Line(const Point start, const Point end);
  std::vector<PointClusters> intersect_with_lidar(const Line l , const bool closest_point_only);
private :
  Point m_start;
  Point m_end;
  uint32_t m_line_length;
  Point m_line_direction;

  Point get_point(const Point start);

};

class Rectangle : Shape{

public :
  Rectangle(const Point center, const uint32_t size, const float orientation_degrees);
  std::vector<PointClusters> intersect_with_lidar(const Line l , const bool closest_point_only);

private :
  Point m_center;
  Point m_size;
  float m_orientation_degrees;

};

class Circle : Shape{

public :
  Circle(const Point center, const float radius);
  std::vector<PointClusters> intersect_with_lidar(const Line l , const bool closest_point_only);

private :
  Point m_center;
  float m_radius;

};

}  // namespace tracking_test_framework
}  // namespace autoware

#endif  // TRACKING_TEST_FRAMEWORK__TRACKING_TEST_FRAMEWORK_HPP_
