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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the prediction_nodes class.

#ifndef PREDICTION_NODES__LONELY_WORLD_BEHAVIOR_HPP_
#define PREDICTION_NODES__LONELY_WORLD_BEHAVIOR_HPP_

#include <prediction_nodes/visibility_control.hpp>

#include <autoware_auto_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>

#include <cstdint>

namespace autoware
{
/// \brief TODO(opensource): Document namespaces!
namespace prediction
{

/// \brief TODO(opensource): Document your functions
void predict_stationary(autoware_auto_msgs::msg::PredictedObject & predicted_object);
void predict_all_stationary(autoware_auto_msgs::msg::PredictedObjects & predicted_objects);

}  // namespace prediction_nodes
}  // namespace autoware

#endif  // PREDICTION_NODES__LONELY_WORLD_BEHAVIOR_HPP_
