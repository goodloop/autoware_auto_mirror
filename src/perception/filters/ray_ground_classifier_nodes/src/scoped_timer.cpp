// MIT License
//
// Copyright (c) 2020 Mapless AI, Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

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
