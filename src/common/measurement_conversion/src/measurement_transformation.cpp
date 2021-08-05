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

#include <measurement_conversion/measurement_transformation.hpp>

namespace autoware
{
namespace common
{
namespace state_estimation
{

template<>
MeasurementXYSpeed64 transform_measurement(
  const MeasurementXYSpeed64 & measurement,
  const Eigen::Isometry3d & tf__world__frame_id)
{
  const auto converted_tf__world__frame_id = downscale_isometry<2>(tf__world__frame_id);
  return MeasurementXYSpeed64{
    converted_tf__world__frame_id.rotation() * measurement.state().vector(),
    Eigen::Matrix2d{converted_tf__world__frame_id.rotation() * measurement.covariance().matrix() *
      converted_tf__world__frame_id.rotation().transpose()}};
}

template<>
MeasurementXYPos64 transform_measurement(
  const MeasurementXYPos64 & measurement,
  const Eigen::Isometry3d & tf__world__frame_id)
{
  const auto converted_tf__world__frame_id = downscale_isometry<2>(tf__world__frame_id);
  return MeasurementXYPos64{
    converted_tf__world__frame_id * measurement.state().vector(),
    Eigen::Matrix2d
    {
      converted_tf__world__frame_id.rotation() *
        measurement.covariance().matrix() *
        converted_tf__world__frame_id.rotation().transpose()
    }
  };
}

template<>
Stamped<MeasurementXYSpeed64> transform_measurement(
  const Stamped<MeasurementXYSpeed64> & measurement,
  const Eigen::Isometry3d & tf__world__frame_id)
{
  return Stamped<MeasurementXYSpeed64> {
    measurement.timestamp,
    transform_measurement(measurement.measurement, tf__world__frame_id)};
}

template<>
Stamped<MeasurementXYPos64> transform_measurement(
  const Stamped<MeasurementXYPos64> & measurement,
  const Eigen::Isometry3d & tf__world__frame_id)
{
  return Stamped<MeasurementXYPos64> {
    measurement.timestamp,
    transform_measurement(measurement.measurement, tf__world__frame_id)};
}


template<>
Stamped<MeasurementXYPosAndSpeed64> transform_measurement(
  const Stamped<MeasurementXYPosAndSpeed64> & measurement,
  const Eigen::Isometry3d & tf__world__frame_id)
{
  const auto converted_tf__world__frame_id = downscale_isometry<2>(tf__world__frame_id);
  const auto & state = measurement.measurement.state().vector();
  const Eigen::Vector2d pos_state = converted_tf__world__frame_id *
    Eigen::Vector2d{state(0), state(1)};
  const Eigen::Vector2d speed_state = converted_tf__world__frame_id.rotation() *
    Eigen::Vector2d{state(2), state(3)};

  return Stamped<MeasurementXYPosAndSpeed64>{
    measurement.timestamp,
    MeasurementXYPosAndSpeed64{
      (Eigen::Vector4d{} << pos_state, speed_state).finished(),
      measurement.measurement.covariance()}
  };
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
