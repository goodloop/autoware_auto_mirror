// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTERPOLATION__LINEAR_INTERPOLATION_HPP_
#define INTERPOLATION__LINEAR_INTERPOLATION_HPP_

#include <vector>

#include "interpolation/interpolation_utils.hpp"

namespace interpolation
{
/// \brief Calculate linear interpolation from given source, destination and ration value
/// \param src_val The source value
/// \param dst_val The destination value
/// \param ratio Interpolation ratio
/// \return The interpolated value
double lerp(const double src_val, const double dst_val, const double ratio);

/// \brief Calculate linear interpolation from given values
/// \param base_keys Set of base keys, coordinates of the base point
/// \param base_values Set of base values
/// \param query_keys Set of query keys, coordinates of the query point
/// \return The linear interpolated values
std::vector<double> lerp(
  const std::vector<double> & base_keys,
  const std::vector<double> & base_values,
  const std::vector<double> & query_keys);
}  // namespace interpolation

#endif  // INTERPOLATION__LINEAR_INTERPOLATION_HPP_
