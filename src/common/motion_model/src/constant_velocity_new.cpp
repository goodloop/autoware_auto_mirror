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

#include <common/types.hpp>
#include <motion_model/constant_velocity_new.hpp>

using autoware::common::types::float32_t;

namespace autoware
{
namespace motion
{
namespace motion_model
{
ConstantVelocityNew::Matrix ConstantVelocityNew::compute_jacobian(
  const std::chrono::nanoseconds & dt)
{
  const float32_t dt_s = static_cast<float32_t>(dt.count()) / 1000000000LL;
  m_jacobian.setIdentity();
  // only nonzero elements are ones along diagonal + constant terms for velocity
  m_jacobian(States::POSE_X, States::VELOCITY_X) = dt_s;
  m_jacobian(States::POSE_Y, States::VELOCITY_Y) = dt_s;
  return m_jacobian;
}

ConstantVelocityNew::Matrix ConstantVelocityNew::compute_jacobian_in_place(
  const std::chrono::nanoseconds & dt)
{
  const float32_t dt_s = static_cast<float32_t>(dt.count()) / 1000000000LL;
  return (Matrix{} <<
    1.0F, 0.0F, dt_s, 0.0F,
    0.0F, 1.0F, 0.0F, dt_s,
    0.0F, 0.0F, 1.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F).finished();
}
}  // namespace motion_model
}  // namespace motion
}  // namespace autoware
