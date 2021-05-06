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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <prediction_nodes/rule_based/lonely_world_behavior.hpp>

#include <time_utils/time_utils.hpp>

namespace autoware {

namespace prediction {

void predict_stationary(autoware_auto_msgs::msg::PredictedObject &predicted_object) {
  using namespace std::chrono_literals;

  // TODO parameters to be read from launch file
//  builtin_interfaces::msg::Duration time_horizon{rosidl_runtime_cpp::MessageInitialization::ZERO};
//  time_horizon.sec = 7U;
//  builtin_interfaces::msg::Duration time_step{rosidl_runtime_cpp::MessageInitialization::ZERO};
//  time_step.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(100ms).count();
  const std::chrono::milliseconds time_horizon = 7s;
  const std::chrono::milliseconds time_step = 100ms;

  // TODO requires checking that `time_*` values > 0 before
  // need an extra state in the end if the division has remainder
  auto n_steps = static_cast<std::size_t>(time_horizon.count() / time_step.count());
  if (time_horizon.count() % time_step.count()) {
    ++n_steps;
  }

  // TODO issue: predicted path only has one pose, not multiple for each shape

  // TODO is path is predefined to a size of 100, does it contain 100 default-initialized
  //  poses or none if PredictedPath is initialized with strategy ZERO?
  autoware_auto_msgs::msg::PredictedPath predicted_path;
  predicted_path.path = decltype(predicted_path.path){n_steps,
                                                      predicted_object.kinematics.initial_pose.pose};
  predicted_path.confidence = 1.0;
  predicted_path.time_step = time_utils::to_message(time_step);

  predicted_object.kinematics.predicted_paths.emplace_back(predicted_path);
}

void predict_all_stationary(autoware_auto_msgs::msg::PredictedObjects &predicted_objects) {
  std::for_each(predicted_objects.objects.begin(), predicted_objects.objects.end(),
                predict_stationary);
}

}
}