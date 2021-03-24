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

#include <algorithm>
#include <memory>
#include <vector>

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

/// If we represent two lines in the form of:
/// 𝑝1= 𝑝1^+𝑡1⋅𝑑1   𝑝2=𝑝2^+𝑡2⋅𝑑2,
/// where  𝑝1  and  𝑝2  are arbitrary points on the given lines,
/// 𝑝1^  and  𝑝2^  are the starting points of the lines,
/// 𝑡1  and  𝑡2  are scalar scaling parameters that scale the corresponding direction unit
/// vectors  𝑑1  and  𝑑2 , thus defining any point on the line.
std::vector<Eigen::Vector2f> Line::intersect_with_lidar(
  const Line & line, const bool) const
{
  const Eigen::Vector2f p_delta = this->m_start - line.m_start;
  float_t t1, t2;
  float denominator = cross_2d(this->m_line_direction, line.m_line_direction);

  // To prevent divide by zero error check if denominator is non-zero
  if (denominator != 0.0F) {
    t1 = cross_2d(line.m_line_direction, p_delta) / denominator;
    t2 = cross_2d(this->m_line_direction, p_delta) / denominator;
  } else {
    throw std::runtime_error("Attempt to divide by zero");
  }
  // If the scalar factors are not within the line length and non zero return empty vec
  if (t1 < 0 || t1 > this->m_line_length || t2 < 0 || t2 > line.m_line_length) {
    return std::vector<Eigen::Vector2f>();
  }
  return std::vector<Eigen::Vector2f>{this->m_start + (t1 * this->m_line_direction)};
}

Eigen::Vector2f Line::get_point(const float_t scale) const
{
  return this->m_start + scale * this->m_line_direction;
}

Rectangle::Rectangle(
  const Eigen::Vector2f & center, const Eigen::Vector2f & size, const float_t
  orientation_degrees)
{
  constexpr auto degrees = 180.0F;
  this->m_center = center;
  this->m_size = size;
  this->m_orientation_degrees = orientation_degrees *
    (static_cast<float_t>(EIGEN_PI) / degrees);
  const Eigen::Rotation2D<float_t> rotation(m_orientation_degrees);
  const Eigen::Matrix2f rotation_matrix = rotation.toRotationMatrix();

  m_corners = {m_center + rotation_matrix * ((Eigen::Vector2f{-m_size[0], -m_size[1]} *0.5)),
    m_center + rotation_matrix * ((Eigen::Vector2f{m_size[0], -m_size[1]} *0.5)),
    m_center + rotation_matrix * ((Eigen::Vector2f{m_size[0], m_size[1]} *0.5)),
    m_center + rotation_matrix * ((Eigen::Vector2f{-m_size[0], m_size[1]} *0.5))};

  m_borders = {
    Line{m_corners[0], m_corners[1]},
    Line{m_corners[1], m_corners[2]},
    Line{m_corners[2], m_corners[3]},
    Line{m_corners[3], m_corners[0]}};
}

///  We can pass the rectangle borders to the Line::intersect with lidar function
/// one by one to see if the lidar is intersecting any of the 4 sides of the rectangle and get
/// the intersection points if any from there.
std::vector<Eigen::Vector2f> Rectangle::intersect_with_lidar(
  const Line & line, const
  bool
  closest_point_only) const
{
  std::vector<Eigen::Vector2f> intersections{};
  for (const auto & border : this->m_borders) {
    const auto & current_intersection_vec = border.intersect_with_lidar(line, true);
    if (!current_intersection_vec.empty()) {
      intersections.emplace_back(current_intersection_vec[0]);
    }
  }
  if (intersections.empty()) {
    return std::vector<Eigen::Vector2f>();
  }
  /// Case to return closest intersection only
  if (closest_point_only) {
    sort(
      intersections.begin(), intersections.end(), [&](const
      Eigen::Vector2f & point_a,
      const Eigen::Vector2f &
      point_b) {
        return (point_a - line.get_start_pt()).norm() < (point_b -
        line.get_start_pt())
        .norm();
      });
    return intersections;
  }
  /// Else return all intersections
  return intersections;
}

Circle::Circle(const Eigen::Vector2f & center, const float_t radius)
{
  m_center = center;
  m_radius = radius;
}

/// Given a line in the form of  𝑝=𝑝0+𝑡⋅𝑑 , where  𝑝  is a point on a line, 𝑝0 is the 2D
/// starting point of the line,
/// 𝑡  is a scale parameter and  𝑑  is the normalized direction vector and a circle in the form
/// of  (𝑥−𝑥c)^2+(𝑦−𝑦c)^2 = 𝑟^2 ,
/// where  𝑥,𝑦  are coordinates of a point on a circle,  𝑥c,𝑦c  are the coordinates of the center
/// of the circle and 𝑟  is the radius of this circle, we can form a single equation from which
/// we look for such values of 𝑡 that the resulting point lies both on the point and on the
/// circle.
std::vector<Eigen::Vector2f> Circle::intersect_with_lidar(
  const Line & line, const
  bool
  closest_point_only) const
{
  const Eigen::Vector2f delta = m_center - line.get_start_pt();
  const float_t root_part = powf(m_radius, 2) - powf(cross_2d(delta, line.get_line_dir()), 2);
  if (root_part < 0) {
    return std::vector<Eigen::Vector2f>();
  }
  const float_t prefix_part = line.get_line_dir().transpose() * delta;
  std::vector<float_t> distances {};
  if (static_cast<int>(root_part) == 0) {
    distances.emplace_back(prefix_part);
  } else {
    distances.emplace_back(prefix_part + powf(root_part, 2));
    distances.emplace_back(prefix_part - powf(root_part, 2));
  }
  std::sort(distances.begin(), distances.end());
  if (distances[0] < 0 || distances[0] > line.get_line_length()) {
    return std::vector<Eigen::Vector2f>();
  }

  std::vector<Eigen::Vector2f> intersections{};
  if (closest_point_only) {
    intersections.emplace_back(line.get_point(distances[0]));
  } else {
    for (const auto & scale  : distances) {
      intersections.emplace_back(line.get_point(scale));
    }
  }
  return intersections;
}


}  // namespace tracking_test_framework
}  // namespace autoware
