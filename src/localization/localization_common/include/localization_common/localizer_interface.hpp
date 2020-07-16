// Copyright 2019 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef LOCALIZATION_COMMON__LOCALIZER_INTERFACE_HPP_
#define LOCALIZATION_COMMON__LOCALIZER_INTERFACE_HPP_

#include <localization_common/visibility_control.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <optimization/optimizer_options.hpp>

namespace autoware
{
namespace localization
{
namespace localization_common
{
/// An interface that all registration classes must follow.
///
/// It keeps it open to its children to decide what the measurements can be registered against.
/// \tparam InputMsgT Message type that will be registered.
template<typename InputMsgT, typename MapT>
class LOCALIZATION_COMMON_PUBLIC LocalizerInterface
{
public:
  /// Registers a measurement to the current map and returns the
  /// estimated pose of the vehicle and its validity.
  /// \param[in] msg Measurement message to register.
  /// \param[in] transform_initial Initial guess of the pose to initialize the localizer with
  /// in iterative processes like solving optimization problems.
  /// \param[out] pose_out Reference to the resulting pose estimate after registration.
  virtual common::optimization::OptimizationSummary register_measurement(
    const InputMsgT & msg,
    const MapT& map,
    const geometry_msgs::msg::TransformStamped & transform_initial,
    geometry_msgs::msg::PoseWithCovarianceStamped & pose_out) = 0;
};
}  // namespace localization_common
}  // namespace localization
}  // namespace autoware

#endif  // LOCALIZATION_COMMON__LOCALIZER_INTERFACE_HPP_
