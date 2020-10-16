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

#include "ray_ground_classifier_nodes/scoped_timer.hpp"

#include <limits>
#include <cmath>
#include <algorithm>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"

static constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();
static constexpr auto INF = std::numeric_limits<double>::infinity();

//------------------------------------------------------------------------------

Timing_Statistics::Timing_Statistics()
: sample_count(0), M2(0.0), mean(NaN), min_value(INF), max_value(-INF) {}

//------------------------------------------------------------------------------

Scoped_Timer Timing_Statistics::get_timer(std::string prefix)
{
  return Scoped_Timer(std::move(prefix), this);
}

//------------------------------------------------------------------------------

Timing_Statistics::Stats::Stats(
  double mean, double variance, double sample_variance, double min_value, double max_value,
  size_t sample_count)
: mean(mean), variance(variance), sample_variance(sample_variance), min_value(min_value), max_value(
    max_value), sample_count(sample_count) {}

//------------------------------------------------------------------------------

std::string Timing_Statistics::Stats::string() const
{
  const auto m = mean * 1e-6;
  const auto v = variance * 1e-12;
  const auto sv = sample_variance * 1e-12;
  const auto min = min_value * 1e-6;
  const auto max = max_value * 1e-6;
  return "{mean: " + std::to_string(m) + "ms, variance: " + std::to_string(v) +
         ", sample_variance: " + std::to_string(sv) + ", min_value: " + std::to_string(min) +
         ", max_value: " + std::to_string(max) + ", sample_count: " + std::to_string(
    sample_count) + "}";
}

//------------------------------------------------------------------------------

void Timing_Statistics::update_stats(double val)
{
  ++sample_count;
  min_value = std::min(min_value, val);
  max_value = std::max(max_value, val);
  if (sample_count == 1) {
    mean = val;
    return;
  }

  const auto delta = (val - mean);
  mean += delta / static_cast<double>(sample_count);
  const auto delta2 = (val - mean);
  M2 += (delta * delta2);
}

//------------------------------------------------------------------------------

bool Timing_Statistics::has_stats() const
{
  return sample_count > 1;
}

//------------------------------------------------------------------------------

Timing_Statistics::Stats Timing_Statistics::get_stats() const
{
  if (!has_stats()) {
    return Stats{NaN, NaN, NaN, NaN, NaN, 0};
  }

  return Stats{mean, M2 / static_cast<double>(sample_count),
    M2 / static_cast<double>(sample_count - 1), min_value, max_value, sample_count};
}

//------------------------------------------------------------------------------

Scoped_Timer::Scoped_Timer(std::string prefix, Timing_Statistics * ptr)
: prefix(std::move(prefix)), ptr(ptr)
{
  t1 = std::chrono::steady_clock::now();
}

//------------------------------------------------------------------------------

Scoped_Timer::Scoped_Timer(std::string prefix)
: Scoped_Timer(std::move(prefix), nullptr) {}

//------------------------------------------------------------------------------

Scoped_Timer::Scoped_Timer()
: Scoped_Timer("") {}

//------------------------------------------------------------------------------

Scoped_Timer::~Scoped_Timer()
{
  const auto t2 = std::chrono::steady_clock::now();
  const auto nano_count =
    std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
  const auto nano = static_cast<double>(nano_count);
  if (ptr) {ptr->update_stats(nano);}
  const auto ms = nano * 1e-6;
  RCLCPP_FATAL(rclcpp::get_logger(prefix), std::to_string(ms) + "ms");
}

//------------------------------------------------------------------------------
