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
