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

#ifndef RAY_GROUND_CLASSIFIER_NODES__SCOPED_TIMER_HPP_
#define RAY_GROUND_CLASSIFIER_NODES__SCOPED_TIMER_HPP_

#include <chrono>
#include <string>

class Scoped_Timer;

class Timing_Statistics
{
public:
  struct Stats
  {
    double mean;
    double variance;
    double sample_variance;
    double min_value;
    double max_value;
    size_t sample_count;

    Stats() = delete;
    Stats(
      double mean, double variance, double sample_variance, double min_value, double max_value,
      size_t sample_count);
    std::string string() const;
  };

  Timing_Statistics();
  Scoped_Timer get_timer(std::string prefix);
  void update_stats(double val);
  bool has_stats() const;
  Stats get_stats() const;

private:
  size_t sample_count;
  double M2;
  double mean;
  double min_value;
  double max_value;
};

class Scoped_Timer
{
public:
  Scoped_Timer(std::string prefix, Timing_Statistics * ptr);
  explicit Scoped_Timer(std::string prefix);
  Scoped_Timer();
  ~Scoped_Timer();

private:
  std::string prefix;
  Timing_Statistics * ptr;
  std::chrono::steady_clock::time_point t1;
};


#endif  // RAY_GROUND_CLASSIFIER_NODES__SCOPED_TIMER_HPP_
