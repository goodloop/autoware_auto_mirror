// Copyright 2018 the Autoware Foundation
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

/// \copyright Copyright 2018 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief This file defines the constant velocity motion model
#ifndef MOTION_MODEL__CONSTANT_VELOCITY_NEW_HPP_
#define MOTION_MODEL__CONSTANT_VELOCITY_NEW_HPP_

#include <common/types.hpp>
#include <motion_model/visibility_control.hpp>

#include <Eigen/Core>

#include <chrono>

using autoware::common::types::float32_t;

namespace autoware
{
namespace motion
{
namespace motion_model
{
class MOTION_MODEL_PUBLIC ConstantVelocityNew
{
public:
  using Matrix = Eigen::Matrix<float32_t, 4U, 4U>;

  struct States
  {
    static const Eigen::Index POSE_X = 0U;  ///< index of x position
    static const Eigen::Index POSE_Y = 1U;  ///< index of y position
    static const Eigen::Index VELOCITY_X = 2U;  ///< index of x velocity
    static const Eigen::Index VELOCITY_Y = 3U;  ///< index of y velocity
  };

  Matrix compute_jacobian(const std::chrono::nanoseconds & dt);
  Matrix compute_jacobian_in_place(const std::chrono::nanoseconds & dt);

private:
  Matrix m_jacobian{};
};

}  // namespace motion_model
}  // namespace motion
}  // namespace autoware

#endif  // MOTION_MODEL__CONSTANT_VELOCITY_NEW_HPP_
