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
class TRACKING_TEST_FRAMEWORK_PUBLIC Line;
class TRACKING_TEST_FRAMEWORK_PUBLIC Shape
{
public:
  virtual std::vector<Eigen::Vector2f> intersect_with_lidar(
    const Line & line,
    const bool closest_point_only) const = 0;
  virtual ~Shape() = default;
};

class TRACKING_TEST_FRAMEWORK_PUBLIC Line : public Shape
{
public:
  Line(const Eigen::Vector2f & start, const Eigen::Vector2f & end);

  std::vector<Eigen::Vector2f> intersect_with_lidar(
    const Line & line,
    const bool closest_point_only) const override;
  Eigen::Vector2f get_point(const float_t scale) const;

  inline const Eigen::Vector2f & get_start_pt() const
  {
    return m_start;
  }
  inline const Eigen::Vector2f & get_end_pt() const
  {
    return m_end;
  }
  inline const Eigen::Vector2f & get_line_dir() const
  {
    return m_line_direction;
  }
  inline const float_t & get_line_length() const
  {
    return m_line_length;
  }
private:
  Eigen::Vector2f m_start;
  Eigen::Vector2f m_end;
  float_t m_line_length;
  Eigen::Vector2f m_line_direction;
};
class TRACKING_TEST_FRAMEWORK_PUBLIC Rectangle : public Shape
{
public:
  Rectangle(
    const Eigen::Vector2f & center, const Eigen::Vector2f & size,
    const float_t orientation_degrees);
  std::vector<Eigen::Vector2f> intersect_with_lidar(
    const Line & line, const bool
    closest_point_only) const override;

private:
  Eigen::Vector2f m_center;
  Eigen::Vector2f m_size;
  float_t m_orientation_degrees;
  std::array<Line, 4> m_borders {Line(Eigen::Vector2f(), Eigen::Vector2f()), Line(
      Eigen::Vector2f(),
      Eigen::Vector2f()), Line(
      Eigen::Vector2f(),
      Eigen::Vector2f()), Line(Eigen::Vector2f(), Eigen::Vector2f())};
  std::array<Eigen::Vector2f, 4> m_corners;
};

class TRACKING_TEST_FRAMEWORK_PUBLIC Circle : public Shape
{
 public:
  Circle(const Eigen::Vector2f& center, const float_t radius);
  std::vector<Eigen::Vector2f> intersect_with_lidar(
      const Line & line, const bool
  closest_point_only) const override;

private:
  Eigen::Vector2f m_center;
  float_t m_radius;
};

}  // namespace tracking_test_framework
}  // namespace autoware

#endif  // TRACKING_TEST_FRAMEWORK__TRACKING_TEST_FRAMEWORK_HPP_
