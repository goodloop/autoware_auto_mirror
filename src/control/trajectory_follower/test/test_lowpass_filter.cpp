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

#include <vector>

#include "gtest/gtest.h"
#include "trajectory_follower/lowpass_filter.hpp"

TEST(test_lowpass_filter, MoveAverageFilter) {
  namespace MoveAverageFilter = autoware::motion::control::trajectory_follower::MoveAverageFilter;

  {  // Fail case: window size higher than the vector size
    const int window_size = 5;
    std::vector<double> vec = {1.0, 2.0, 3.0, 4.0};
    EXPECT_FALSE(MoveAverageFilter::filt_vector(window_size, vec));
  }
  {
    const int window_size = 0;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    for (size_t i = 0; i < filtered_vec.size(); ++i) {
      EXPECT_EQ(filtered_vec[i], original_vec[i]);
    }
  }
  {
    const int window_size = 1;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]);
    EXPECT_EQ(filtered_vec[1], 8.0 / 3);
    EXPECT_EQ(filtered_vec[2], 13.0 / 3);
    EXPECT_EQ(filtered_vec[3], original_vec[3]);
  }
  {
    const int window_size = 2;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0, 7.0, 10.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]);
    EXPECT_EQ(filtered_vec[1], 8.0 / 3);
    EXPECT_EQ(filtered_vec[2], 21.0 / 5);
    EXPECT_EQ(filtered_vec[3], 30.0 / 5);
    EXPECT_EQ(filtered_vec[4], 23.0 / 3);
    EXPECT_EQ(filtered_vec[5], original_vec[5]);
  }
}
TEST(test_lowpass_filter, Butterworth2dFilter) {
  using autoware::motion::control::trajectory_follower::Butterworth2dFilter;
  const double dt = 1.0;
  const double cutoff_hz = 1.0;
  Butterworth2dFilter filter(dt, cutoff_hz);
  for (double i = 1.0; i < 10.0; ++i) {
    EXPECT_LT(filter.filter(i), i);
  }

  const std::vector<double> original_vec = {1.0, 2.0, 3.0, 4.0};
  std::vector<double> filtered_vec;
  filter.filt_vector(original_vec, filtered_vec);
  ASSERT_EQ(filtered_vec.size(), original_vec.size());
  EXPECT_EQ(filtered_vec[0], original_vec[0]);
  for (size_t i = 1; i < filtered_vec.size(); ++i) {
    EXPECT_LT(filtered_vec[i], original_vec[i]);
  }

  filtered_vec.clear();
  filter.filtfilt_vector(original_vec, filtered_vec);
  ASSERT_EQ(filtered_vec.size(), original_vec.size());

  std::vector<double> coefficients;
  filter.getCoefficients(coefficients);
  EXPECT_EQ(coefficients.size(), size_t(6));
}
