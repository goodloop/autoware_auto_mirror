// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#ifndef MEASUREMENT_CONVERSION__MEASUREMENT_TYPEDEFS_HPP_
#define MEASUREMENT_CONVERSION__MEASUREMENT_TYPEDEFS_HPP_

#include <common/types.hpp>
#include <state_estimation/measurement/linear_measurement.hpp>
#include <state_vector/common_variables.hpp>

namespace autoware
{
namespace common
{
namespace state_estimation
{

template<typename MeasurementT>
struct Stamped
{
  std::chrono::system_clock::time_point timestamp;
  MeasurementT measurement;

  template<typename NewScalarT>
  auto cast() const noexcept
  {
    using NewMeasurementT = decltype(measurement.template cast<NewScalarT>());
    return Stamped<NewMeasurementT> {
      timestamp,
      measurement.template cast<NewScalarT>()
    };
  }
};

template<typename ScalarT>
using MeasurementXYPos = LinearMeasurement<state_vector::GenericState<ScalarT,
  state_vector::variable::X, state_vector::variable::Y>>;
using MeasurementXYPos32 = MeasurementXYPos<common::types::float32_t>;
using MeasurementXYPos64 = MeasurementXYPos<common::types::float64_t>;

template<typename ScalarT>
using MeasurementXYZPos = LinearMeasurement<state_vector::GenericState<ScalarT,
  state_vector::variable::X, state_vector::variable::Y, state_vector::variable::Z>>;
using MeasurementXYZPos32 = MeasurementXYZPos<common::types::float32_t>;
using MeasurementXYZPos64 = MeasurementXYZPos<common::types::float64_t>;

template<typename ScalarT>
using MeasurementXYZRPYPos = LinearMeasurement<state_vector::GenericState<ScalarT,
  state_vector::variable::X, state_vector::variable::Y, state_vector::variable::Z,
  state_vector::variable::ROLL, state_vector::variable::PITCH, state_vector::variable::YAW>>;
using MeasurementXYZRPYPos32 = MeasurementXYZRPYPos<common::types::float32_t>;
using MeasurementXYZRPYPos64 = MeasurementXYZRPYPos<common::types::float64_t>;

template<typename ScalarT>
using MeasurementXYSpeed = LinearMeasurement<state_vector::GenericState<ScalarT,
  state_vector::variable::X_VELOCITY, state_vector::variable::Y_VELOCITY>>;
using MeasurementXYSpeed32 = MeasurementXYSpeed<common::types::float32_t>;
using MeasurementXYSpeed64 = MeasurementXYSpeed<common::types::float64_t>;

template<typename ScalarT>
using MeasurementXYZSpeed = LinearMeasurement<state_vector::GenericState<ScalarT,
  state_vector::variable::X_VELOCITY,
  state_vector::variable::Y_VELOCITY,
  state_vector::variable::Z_VELOCITY>>;
using MeasurementXYZSpeed32 = MeasurementXYZSpeed<common::types::float32_t>;
using MeasurementXYZSpeed64 = MeasurementXYZSpeed<common::types::float64_t>;

template<typename ScalarT>
using MeasurementXYZRPYSpeed = LinearMeasurement<state_vector::GenericState<ScalarT,
  state_vector::variable::X_VELOCITY,
  state_vector::variable::Y_VELOCITY,
  state_vector::variable::Z_VELOCITY,
  state_vector::variable::ROLL_CHANGE_RATE,
  state_vector::variable::PITCH_CHANGE_RATE,
  state_vector::variable::YAW_CHANGE_RATE>>;
using MeasurementXYZRPYSpeed32 = MeasurementXYZRPYSpeed<common::types::float32_t>;
using MeasurementXYZRPYSpeed64 = MeasurementXYZRPYSpeed<common::types::float64_t>;

template<typename ScalarT>
using MeasurementXYPosAndSpeed = LinearMeasurement<
  state_vector::GenericState<ScalarT,
  state_vector::variable::X, state_vector::variable::Y,
  state_vector::variable::X_VELOCITY, state_vector::variable::Y_VELOCITY>>;
using MeasurementXYPosAndSpeed32 = MeasurementXYPosAndSpeed<common::types::float32_t>;
using MeasurementXYPosAndSpeed64 = MeasurementXYPosAndSpeed<common::types::float64_t>;

template<typename ScalarT>
using MeasurementXYZPosAndSpeed = LinearMeasurement<
  state_vector::GenericState<ScalarT,
  state_vector::variable::X, state_vector::variable::Y, state_vector::variable::Z,
  state_vector::variable::X_VELOCITY,
  state_vector::variable::Y_VELOCITY,
  state_vector::variable::Z_VELOCITY>>;
using MeasurementXYZPosAndSpeed32 = MeasurementXYZPosAndSpeed<common::types::float32_t>;
using MeasurementXYZPosAndSpeed64 = MeasurementXYZPosAndSpeed<common::types::float64_t>;

template<typename ScalarT>
using MeasurementXYZRPYPosAndSpeed = LinearMeasurement<state_vector::GenericState<ScalarT,
  state_vector::variable::X, state_vector::variable::Y, state_vector::variable::Z,
  state_vector::variable::ROLL, state_vector::variable::PITCH, state_vector::variable::YAW,
  state_vector::variable::X_VELOCITY,
  state_vector::variable::Y_VELOCITY,
  state_vector::variable::Z_VELOCITY,
  state_vector::variable::ROLL_CHANGE_RATE,
  state_vector::variable::PITCH_CHANGE_RATE,
  state_vector::variable::YAW_CHANGE_RATE>>;
using MeasurementXYZRPYPosAndSpeed32 = MeasurementXYZRPYPosAndSpeed<common::types::float32_t>;
using MeasurementXYZRPYPosAndSpeed64 = MeasurementXYZRPYPosAndSpeed<common::types::float64_t>;


}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // MEASUREMENT_CONVERSION__MEASUREMENT_TYPEDEFS_HPP_
