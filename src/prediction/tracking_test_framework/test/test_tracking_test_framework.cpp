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

#include "gtest/gtest.h"
#include "tracking_test_framework/tracking_test_framework.hpp"

#include <vector>

// Test function
int32_t test_intersection()
{
  // Test for line intersection with line
  autoware::tracking_test_framework::Line l1 {Eigen::Vector2f{-1, -1}, Eigen::Vector2f{2, 0.5}};
  autoware::tracking_test_framework::Line l2 {Eigen::Vector2f{1, 0}, Eigen::Vector2f{1, 2}};
  std::vector<Eigen::Vector2f> intersection = l1.intersect_with_lidar(l2, true);
  std::cout << "Intersection pts of line with line: " << intersection[0] << std::endl;

  // Test for rectangle intersection with line
  autoware::tracking_test_framework::Rectangle rect {Eigen::Vector2f{0, 0},
    Eigen::Vector2f{2, 3}, 20.0F};
  autoware::tracking_test_framework::Line l3 {Eigen::Vector2f{-2, 0.5}, Eigen::Vector2f{2, -0.5}};
  std::vector<Eigen::Vector2f> intersections = rect.intersect_with_lidar(l3, true);
  std::cout << "Intersection pts of rectangle with line: " << intersections[0] << std::endl;

  // Test for circle intersection with line
  autoware::tracking_test_framework::Line vertical_line {Eigen::Vector2f{0, 0},
    Eigen::Vector2f{0, 2}};
  autoware::tracking_test_framework::Line horizontal_line {Eigen::Vector2f{0, 0},
    Eigen::Vector2f{2, 0}};
  autoware::tracking_test_framework::Line diagonal_line {Eigen::Vector2f{0, 0},
    Eigen::Vector2f{2, 2}};

  autoware::tracking_test_framework::Circle circle {Eigen::Vector2f {1, 1}, 1};

  std::vector<Eigen::Vector2f> intersections_hor = circle.intersect_with_lidar(
    horizontal_line,
    true);
  std::cout << "Intersection pts of circle with hor line: " << intersections_hor[0] << std::endl;
  std::vector<Eigen::Vector2f> intersections_vert = circle.intersect_with_lidar(
    vertical_line,
    true);
  std::cout << "Intersection pts of circle with vert line: " << intersections_vert[0] << std::endl;
  std::vector<Eigen::Vector2f> intersections_diag = circle.intersect_with_lidar(
    diagonal_line,
    true);
  std::cout << "Intersection pts of circle with diag line: " << intersections_diag[0] << std::endl;
  return 0;
}

TEST(test_tracking_test_framework, test_lidar_intersection) {
  EXPECT_EQ(test_intersection(), 0);
}
