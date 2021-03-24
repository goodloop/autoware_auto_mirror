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
#include <utility>

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
  const Line & line, const bool) const
{
  const Eigen::Vector2f p_delta = this->m_start - line.m_start;
  float_t t1, t2;
  float denominator = cross_2d(this->m_line_direction, line.m_line_direction);
  if (denominator != 0.0F) {
    t1 = cross_2d(line.m_line_direction, p_delta) / denominator;
    t2 = cross_2d(this->m_line_direction, p_delta) / denominator;
  } else {
    throw std::runtime_error("Attempt to divide by zero");
  }

  // If the scalar factors are not within the line length and non zero
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
  return intersections;
}

Circle::Circle(const Eigen::Vector2f & center, const float_t radius)
{
  m_center = center;
  m_radius = radius;
}

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

// Test function
int32_t print_hello()
{
  // Test for line intersection with line
  Line l1 {Eigen::Vector2f{-1,-1},Eigen::Vector2f{2,0.5} };
  Line l2 {Eigen::Vector2f{1,0},Eigen::Vector2f{1,2}};
  std::vector<Eigen::Vector2f> intersection = l1.intersect_with_lidar(l2, true);
  std::cout << "Intersection pts of line with line: " << intersection[0] << std::endl;

  // Test for rectangle intersection with line
  Rectangle rect {Eigen::Vector2f{0, 0}, Eigen::Vector2f{2, 3}, 20.0F};
  Line l3 {Eigen::Vector2f{-2, 0.5}, Eigen::Vector2f{2, -0.5}};
  std::vector<Eigen::Vector2f> intersections = rect.intersect_with_lidar(l3, true);
  std::cout << "Intersection pts of rectangle with line: " << intersections[0] << std::endl;

  // Test for circle intersection with line
  Line vertical_line {Eigen::Vector2f{0, 0}, Eigen::Vector2f{0, 2}};
  Line horizontal_line {Eigen::Vector2f{0, 0}, Eigen::Vector2f{2, 0}};
  Line diagonal_line {Eigen::Vector2f{0, 0}, Eigen::Vector2f{2, 2}};

  Circle circle {Eigen::Vector2f {1, 1}, 1};

  std::vector<Eigen::Vector2f> intersections_hor = circle.intersect_with_lidar(horizontal_line,
                                                                               true);
  std::cout << "Intersection pts of circle with hor line: " << intersections_hor[0] << std::endl;
  std::vector<Eigen::Vector2f> intersections_vert = circle.intersect_with_lidar(vertical_line,
                                                                                true);
  std::cout << "Intersection pts of circle with vert line: " << intersections_vert[0] << std::endl;
  std::vector<Eigen::Vector2f> intersections_diag = circle.intersect_with_lidar(diagonal_line,
                                                                                true);
  std::cout << "Intersection pts of circle with diag line: " << intersections_diag[0] << std::endl;
  return 0;
}


}  // namespace tracking_test_framework
}  // namespace autoware
