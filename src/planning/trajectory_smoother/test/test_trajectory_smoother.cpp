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

#include <motion_testing/motion_testing.hpp>
#include <cmath>

#include "gtest/gtest.h"
#include "trajectory_smoother/trajectory_smoother.hpp"

#define MINIMAL_ERROR_THRESHOLD 0.001

using motion::motion_testing::constant_velocity_trajectory;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectorySmoother = motion::planning::trajectory_smoother::TrajectorySmoother;
using TrajectorySmootherConfig = motion::planning::trajectory_smoother::TrajectorySmootherConfig;

void introduce_error(Trajectory::SharedPtr in_trajectory, const std::size_t &index, const float & epsilon) {
  // Create random small difference in trajectory at element defined by index
  in_trajectory->points[index].longitudinal_velocity_mps += epsilon;
  // in_trajectory->points[index].lateral_velocity_mps += epsilon;
}

Trajectory generate_trajectory(const std::size_t trajectory_length,
                               const float &x0, const float &y0,
                               const float &heading, const float &v0) {
  // Produce fake trajectory and obstacles
  const std::chrono::milliseconds dt(100);
  auto trajectory = constant_velocity_trajectory(
      x0, y0, heading, v0,
      std::chrono::duration_cast<std::chrono::nanoseconds>(dt));
  trajectory.points.resize(trajectory_length);

  return trajectory;
}

void assert_sanity_checks(const Trajectory::SharedPtr in_trajectory, const Trajectory::SharedPtr err_trajectory) {
  // Assert that the trajectories are of equal length
  ASSERT_EQ(in_trajectory->points.size(), err_trajectory->points.size());

  // TODO: Assert other checks to ensure the trajectories are properly formed
}

void assert_trajectory_acceleration(const Trajectory::SharedPtr in_trajectory, const Trajectory::SharedPtr err_trajectory) {
  // Assert acceleration is within an acceptable range
  std::cout << in_trajectory->points.size() << std::endl;
  for (std::size_t i = 0; i < err_trajectory->points.size(); i++) {
    ASSERT_NEAR(in_trajectory->points[i].acceleration_mps2,
                err_trajectory->points[i].acceleration_mps2,
                MINIMAL_ERROR_THRESHOLD)
    // std::cout << in_trajectory->points[i].longitudinal_velocity_mps << " : " << err_trajectory->points[i].longitudinal_velocity_mps << std::endl;
    // std::cout << in_trajectory->points[i].lateral_velocity_mps << " : " << err_trajectory->points[i].lateral_velocity_mps << std::endl;
    // std::cout << in_trajectory->points[i].acceleration_mps2 << " : " << err_trajectory->points[i].acceleration_mps2 << std::endl;
  }
}

void assert_trajectory_jitter(const Trajectory::SharedPtr in_trajectory, const Trajectory::SharedPtr err_trajectory) {
  // Assert jitter is within an acceptable range
}

TEST(trajectory_smoother, single_error) {
  // Generate 2 trajectories
  auto trajectory = std::make_shared<Trajectory>(generate_trajectory(10, 0, 0, 1, 10));
  auto err_trajectory = std::make_shared<Trajectory>(generate_trajectory(10, 0, 0, 1, 10));

  // Generate TrajectorySmoother
  const TrajectorySmootherConfig config{5.0, 2};
  TrajectorySmoother smoother(config);

  // Send through smoother
  smoother.Filter(*err_trajectory);

  // Assert checks for trajectories
  assert_sanity_checks(trajectory, err_trajectory);
  assert_trajectory_acceleration(trajectory, err_trajectory);
  assert_trajectory_jitter(trajectory, err_trajectory);
}
