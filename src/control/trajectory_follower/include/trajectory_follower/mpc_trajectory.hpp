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

#ifndef TRAJECTORY_FOLLOWER__MPC_TRAJECTORY_HPP_
#define TRAJECTORY_FOLLOWER__MPC_TRAJECTORY_HPP_

#include <iostream>
#include <vector>

#include "trajectory_follower/visibility_control.hpp"

namespace motion
{
namespace control
{
namespace trajectory_follower
{
/**
 * @class trajectory class for mpc follower
 * @brief calculate control command to follow reference waypoints
 */
class TRAJECTORY_FOLLOWER_PUBLIC MPCTrajectory
{
public:
  std::vector<double> x;              //!< @brief x position x vector
  std::vector<double> y;              //!< @brief y position y vector
  std::vector<double> z;              //!< @brief z position z vector
  std::vector<double> yaw;            //!< @brief yaw pose yaw vector
  std::vector<double> vx;             //!< @brief vx velocity vx vector
  std::vector<double> k;              //!< @brief k curvature k vector
  std::vector<double> smooth_k;       //!< @brief k smoothed-curvature k vector
  std::vector<double> relative_time;  //!< @brief relative_time duration time from start point

  /**
   * @brief push_back for all values
   */
  void push_back(
    const double & xp, const double & yp, const double & zp, const double & yawp,
    const double & vxp, const double & kp, const double & smooth_kp, const double & tp);
  /**
   * @brief clear for all values
   */
  void clear();

  /**
   * @brief check size of MPCTrajectory
   * @return size, or 0 if the size for each components are inconsistent
   */
  size_t size() const;
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
#endif  // TRAJECTORY_FOLLOWER__MPC_TRAJECTORY_HPP_
