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

#include "trajectory_follower/interpolate.hpp"

#include <algorithm>
#include <limits>
#include <vector>

/*
 * linear interpolation
 */

namespace motion
{
namespace control
{
namespace trajectory_follower
{
namespace
{
bool isIncrease(const std::vector<double> & x)
{
  for (size_t i = 0; i < x.size() - 1; ++i) {
    if (x.at(i) > x.at(i + 1)) {return false;}
  }
  return true;
}

bool isValidInput(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index)
{
  if (base_index.empty() || base_value.empty() || return_index.empty()) {
    std::cerr << "mpc bad index : some vector is empty. base_index: " << base_index.size() <<
      ", base_value: " << base_value.size() << ", return_index: " << return_index.size() <<
      std::endl;
    return false;
  }
  if (!isIncrease(base_index)) {
    std::cerr << "mpc bad index : base_index is not monotonically increasing. base_index = [" <<
      base_index.front() << ", " << base_index.back() << "]" << std::endl;
    return false;
  }
  if (!isIncrease(return_index)) {
    std::cerr << "mpc bad index : base_index is not monotonically increasing. return_index = [" <<
      return_index.front() << ", " << return_index.back() << "]" << std::endl;
    return false;
  }
  if (return_index.front() < base_index.front()) {
    std::cerr << "mpc bad index : return_index.front() < base_index.front()" << std::endl;
    return false;
  }
  if (base_index.back() < return_index.back()) {
    std::cerr << "mpc bad index : base_index.back() < return_index.back()" << std::endl;
    return false;
  }
  if (base_index.size() != base_value.size()) {
    std::cerr << "mpc bad index : base_index.size() != base_value.size()" << std::endl;
    return false;
  }

  return true;
}
}  // namespace

bool linearInterpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value)
{
  // check if inputs are valid
  if (!isValidInput(base_index, base_value, return_index)) {
    std::cerr << "[mpc linear interpolate] invalid input. interpolation failed." << std::endl;
    return false;
  }

  // calculate linear interpolation
  size_t i = 0;
  const size_t base_size = base_index.size();
  for (const auto idx : return_index) {
    if (base_index.at(i) == idx) {
      return_value.push_back(base_value.at(i));
      continue;
    }
    while (base_index.at(i) < idx) {
      ++i;
      if (i <= 0 || base_size - 1 < i) {break;}
    }

    if (i <= 0 || base_size - 1 < i) {
      std::cerr << "mpc LinearInterpolate : undesired condition. skip index. (i = " << i <<
        ", base_size = " << base_size << "), idx = " << idx <<
        ", base_index.at(i)  = " << base_index.at(i) << std::endl;
      continue;
    }

    const double dist_base_idx = base_index.at(i) - base_index.at(i - 1);
    const double dist_to_forward = base_index.at(i) - idx;
    const double dist_to_backward = idx - base_index.at(i - 1);

    const double value =
      (dist_to_backward * base_value.at(i) + dist_to_forward * base_value.at(i - 1)) /
      dist_base_idx;
    return_value.push_back(value);
  }
  return true;
}

bool linearInterpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const double & return_index, double & return_value)
{
  std::vector<double> return_index_v;
  return_index_v.push_back(return_index);

  std::vector<double> return_value_v;
  if (!linearInterpolate(base_index, base_value, return_index_v, return_value_v)) {
    return false;
  }
  return_value = return_value_v.at(0);
  return true;
}
/*
 * spline interpolation
 */

SplineInterpolate::SplineInterpolate() {}
SplineInterpolate::SplineInterpolate(const std::vector<double> & x) {generateSpline(x);}
SplineInterpolate::~SplineInterpolate() {}
void SplineInterpolate::generateSpline(const std::vector<double> & x)
{
  const size_t N = x.size();

  a_.clear();
  b_.clear();
  c_.clear();
  d_.clear();

  a_ = x;

  c_.push_back(0.0);
  for (size_t i = 1; i < N - 1; i++) {
    c_.push_back(3.0 * (a_.at(i - 1) - 2.0 * a_.at(i) + a_.at(i + 1)));
  }
  c_.push_back(0.0);

  std::vector<double> w_;
  w_.push_back(0.0);

  double tmp;
  for (size_t i = 1; i < N - 1; i++) {
    tmp = 1.0 / (4.0 - w_.at(i - 1));
    c_.at(i) = (c_.at(i) - c_.at(i - 1)) * tmp;
    w_.push_back(tmp);
  }

  for (size_t i = N - 2; i > 0; i--) {
    c_.at(i) = c_.at(i) - c_.at(i + 1) * w_.at(i);
  }

  for (size_t i = 0; i < N - 1; i++) {
    d_.push_back((c_.at(i + 1) - c_.at(i)) / 3.0);
    b_.push_back(a_.at(i + 1) - a_.at(i) - c_.at(i) - d_.at(i));
  }
  d_.push_back(0.0);
  b_.push_back(0.0);

  initialized_ = true;
}

double SplineInterpolate::getValue(const double & s)
{
  if (!initialized_) {return 0.0;}

  size_t j = std::max(std::min(static_cast<size_t>(std::floor(s)), a_.size() - 1), size_t(0));
  const double ds = s - static_cast<double>(j);
  return a_.at(j) + (b_.at(j) + (c_.at(j) + d_.at(j) * ds) * ds) * ds;
}

void SplineInterpolate::getValueVector(
  const std::vector<double> & s_v, std::vector<double> & value_v)
{
  if (!initialized_) {return;}
  value_v.clear();
  for (const auto & sv : s_v) {
    value_v.push_back(getValue(sv));
  }
}

bool SplineInterpolate::interpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value)
{
  // check if inputs are valid
  if (!isValidInput(base_index, base_value, return_index)) {
    std::cerr << "[mpc spline interpolate] invalid input. interpolation failed." << std::endl;
    return false;
  }

  std::vector<double> normalized_idx;

  // calculate normalized index
  size_t i = 0;
  const size_t base_size = base_index.size();

  for (const auto idx : return_index) {
    if (base_index.at(i) == idx) {
      normalized_idx.push_back(static_cast<double>(i));
      continue;
    }
    while (base_index.at(i) < idx) {
      ++i;
      if (i <= 0 || base_size - 1 < i) {break;}
    }

    if (i <= 0 || base_size - 1 < i) {
      std::cerr << "mpc SplineInterpolate : undesired condition. skip index. (i = " << i <<
        ", base_size = " << base_size << "), idx = " << idx <<
        ", base_index.at(i)  = " << base_index.at(i) << std::endl;
      continue;
    }

    const double dist_base_idx = base_index.at(i) - base_index.at(i - 1);
    const double dist_to_forward = base_index.at(i) - idx;
    const double dist_to_backward = idx - base_index.at(i - 1);
    const double i_double = static_cast<double>(i);
    const double value =
      (dist_to_backward * i_double + dist_to_forward * (i_double - 1)) / std::max(
      dist_base_idx,
      std::numeric_limits<double>::epsilon());
    normalized_idx.push_back(value);
  }

  // calculate spline coefficients
  generateSpline(base_value);

  // interpolate by spline with normalized index
  for (const auto n_idx : normalized_idx) {
    return_value.push_back(getValue(n_idx));
  }
  return true;
}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
