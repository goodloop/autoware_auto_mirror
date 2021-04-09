#pragma once

#include <chrono>
#include <utility>
#include <vector>
#include <string>
#include <iostream>

class TimeKeeperSequental {
 public:
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = Clock::time_point;

  explicit TimeKeeperSequental(std::string name) : name_(std::move(name)) {

  };

  static double MillisPassedBetween(const TimePoint &t1, const TimePoint &t2) {
    return std::chrono::duration<double>(t2 - t1).count() * 1000.0;
  };

  static double MillisPassedSince(const TimePoint &t) {
    return std::chrono::duration<double>(Clock::now() - t).count() * 1000.0;
  };

  void AddTimePoint(const std::string &name) {
    vec_timepoints_.emplace_back(TimeKeeperSequental::Clock::now());
    vec_timepoint_names_.emplace_back(name);
  };

  void PrintTimes() {

    for (size_t i = 1; i < vec_timepoints_.size(); ++i) {
      std::cout << name_ << " - " << vec_timepoint_names_[i] << " took "
                << MillisPassedBetween(vec_timepoints_[i - 1], vec_timepoints_[i])
                << " ms." << std::endl;
    }

    std::cout << name_ << " took TOTAL "
              << MillisPassedBetween(vec_timepoints_.front(), vec_timepoints_.back())
              << " ms." << std::endl;

  };

 private:
  std::string name_;
  std::vector<TimePoint> vec_timepoints_;
  std::vector<std::string> vec_timepoint_names_;

};