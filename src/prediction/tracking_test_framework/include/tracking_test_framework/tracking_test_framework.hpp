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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tracking_test_framework/visibility_control.hpp>

#include <cstdint>
#include <vector>


#include <memory>

namespace autoware
{
namespace tracking_test_framework
{

int32_t TRACKING_TEST_FRAMEWORK_PUBLIC print_hello();

inline float cross_2d(const Eigen::Vector2f & vec1, const Eigen::Vector2f & vec2)
{
  return (vec1.x() * vec2.y()) - (vec1.y() * vec2.x());
}

class TRACKING_TEST_FRAMEWORK_PUBLIC Shape
{
public:
  virtual std::vector<Eigen::Vector2f> intersect_with_lidar(
    std::unique_ptr<Shape> & shape,
    const bool closest_point_only) = 0;
};

class TRACKING_TEST_FRAMEWORK_PUBLIC Line : public Shape
{
public:
  Line(const Eigen::Vector2f & start, const Eigen::Vector2f & end);

  std::vector<Eigen::Vector2f> intersect_with_lidar(
    std::unique_ptr<Shape> & shape,
    const bool closest_point_only);
  Eigen::Vector2f get_point(const Eigen::Vector2f & start);

private:
  Eigen::Vector2f m_start;
  Eigen::Vector2f m_end;
  float_t m_line_length;
  Eigen::Vector2f m_line_direction;


};
//class Rectangle : Shape
//{
//public:
//  Rectangle(const Eigen::Vector2f center, const uint32_t size, const float_t orientation_degrees);
//  std::vector<Eigen::Vector2f> intersect_with_lidar(std::unique_ptr<Shape> shape,const bool
//  closest_point_only);
//
//private:
//  Eigen::Vector2f m_center;
//  Eigen::Vector2f m_size;
//  float_t m_orientation_degrees;
//};
//
//class Circle : Shape
//{
//public:
//  Circle(const Eigen::Vector2f center, const float_t radius);
//  std::vector<Eigen::Vector2f> intersect_with_lidar(
//    std::unique_ptr<Shape> shape,
//    const bool closest_point_only);
//
//private:
//  Eigen::Vector2f m_center;
//  float_t m_radius;
//};

}  // namespace tracking_test_framework
}  // namespace autoware

#endif  // TRACKING_TEST_FRAMEWORK__TRACKING_TEST_FRAMEWORK_HPP_
